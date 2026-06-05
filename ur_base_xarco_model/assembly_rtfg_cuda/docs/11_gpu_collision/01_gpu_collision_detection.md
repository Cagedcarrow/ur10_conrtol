# GPU 碰撞检测 (GPU Collision Detection)

## 概述

碰撞检测是机器人轨迹规划中的关键安全约束。传统的 CPU FCL（Flexible Collision Library）方案在处理大量轨迹帧时面临性能瓶颈。本功能包实现了 GPU 加速的解析几何碰撞检测，通过分析性距离公式（box-box、box-sphere、box-cylinder）在 GPU 上并行计算数千个碰撞对的间隙。

## 设计哲学：混合 CPU-GPU 碰撞检测

```
复杂网格碰撞 (mesh-mesh, mesh-primitive) → CPU FCL（精度优先）
简单几何对 (box-box, box-sphere, box-cylinder) → GPU 批处理（速度优先）
```

GPU 路径处理可解析的几何对（机器人连杆 AABB vs 环境障碍物），CPU FCL 保留处理复杂三角网格碰撞。这种混合策略兼顾了 GPU 的批量并行优势与 FCL 的几何精度。

## 实现架构

### 线程映射

```
Grid:  (N_frames, 1, 1)       — 每个轨迹帧一个 Block
Block: (N_pairs_per_frame, 1, 1) — 每个碰撞对一个 Thread
```

每帧内，threads 通过 striding 遍历所有 (robot_box, env_object) 对，使用 `atomicMin_double`（基于 `atomicCAS` 循环）计算每帧的最小间隙。

### 碰撞对类型

| 类型 | 编码 | 参数 (8 doubles) | 检测算法 |
|------|------|-----------------|---------|
| Box-Box | `0.0` | (0, cx, cy, cz, hx, hy, hz, 0) | 轴对齐包围盒间隙：max(‖Δx‖-hx₁-hx₂, ‖Δy‖-hy₁-hy₂, ‖Δz‖-hz₁-hz₂) |
| Box-Sphere | `1.0` | (1, cx, cy, cz, r, 0, 0, 0) | 盒上最近点到球心距离 - 半径 |
| Box-Cylinder | `2.0` | (2, cx, cy, cz, r, h, 0, 0) | XY 平面盒-圆柱间隙 × Z 向间隙，取复合最小值 |

所有算法均为 O(1) 解析计算，无迭代。

## 核心实现

### 文件结构

| 文件 | 作用 |
|------|------|
| `src/cuda/cuda_collision.cu` | GPU 碰撞检测 kernel (`collision_check_batch`) |
| `include/assembly_rtfg_cuda/cuda_collision.h` | Host launch 接口 |

### Kernel 签名

```cuda
__global__ void collision_check_batch(
    const double* __restrict__ d_robot_boxes,   // [N_frames, N_boxes, 6] (xyz + half-extents)
    const double* __restrict__ d_env_objects,    // [N_objects, 8] (type + params)
    double* __restrict__ d_clearances,           // [N_frames] output min clearance
    double* __restrict__ d_colliding,            // [N_frames] output: 1.0 if collision
    int N_frames, int N_boxes, int N_objects);
```

### 碰撞基元（device functions）

**Box-Box 间隙**:

```cuda
__device__ double box_box_clearance(
    double bx, double by, double bz, double bhx, double bhy, double bhz,
    double ox, double oy, double oz, double ohx, double ohy, double ohz) {
  double dx = fabs(bx - ox) - (bhx + ohx);
  double dy = fabs(by - oy) - (bhy + ohy);
  double dz = fabs(bz - oz) - (bhz + ohz);
  return fmax(dx, fmax(dy, dz));  // > 0: 分离, < 0: 穿透
}
```

**Box-Sphere 间隙**:

```cuda
__device__ double box_sphere_clearance(
    double bx, double by, double bz, double bhx, double bhy, double bhz,
    double sx, double sy, double sz, double sr) {
  // 盒上距离球心最近的点
  double cx = cuda_clamp(sx, bx - bhx, bx + bhx);
  double cy = cuda_clamp(sy, by - bhy, by + bhy);
  double cz = cuda_clamp(sz, bz - bhz, bz + bhz);
  double dx = sx - cx, dy = sy - cy, dz = sz - cz;
  return sqrt(dx*dx + dy*dy + dz*dz) - sr;
}
```

**Box-Cylinder 间隙** (圆柱轴假定为垂直/Z向):

```cuda
__device__ double box_cylinder_clearance(
    double bx, double by, double bz, double bhx, double bhy, double bhz,
    double cx, double cy, double cz, double cr, double ch) {
  // XY 平面：盒 vs 无限圆柱
  double closest_x = cuda_clamp(cx, bx - bhx, bx + bhx);
  double closest_y = cuda_clamp(cy, by - bhy, by + bhy);
  double h_dist = sqrt((closest_x-cx)*(closest_x-cx) +
                       (closest_y-cy)*(closest_y-cy)) - cr;

  // Z 向间隙
  double z_dist = fabs(bz - cz) - (bhz + ch);

  // 复合判断
  if (h_dist < 0 && z_dist < 0) return fmax(h_dist, z_dist); // 双向穿透
  if (h_dist < 0) return z_dist;
  if (z_dist < 0) return h_dist;
  return sqrt(h_dist*h_dist + z_dist*z_dist); // 3D 分离距离
}
```

### Double 原子最小值

CUDA 不原生支持 double 类型的 `atomicMin`。使用 `atomicCAS` 循环实现：

```cuda
__device__ void atomicMin_double(double* addr, double val) {
  unsigned long long* addr_as_ull = reinterpret_cast<unsigned long long*>(addr);
  unsigned long long old = *addr_as_ull;
  unsigned long long assumed;
  do {
    assumed = old;
    if (val >= __longlong_as_double(assumed)) break;
    old = atomicCAS(addr_as_ull, assumed, __double_as_longlong(val));
  } while (assumed != old);
}
```

该实现对每帧只更新 1 次的场景完全安全（N_boxes × N_objects 个线程竞争）。

## C++ 集成接口

`CudaBatchIK` 类提供三个 GPU 碰撞检测方法：

```cpp
// 上传环境物体（调用一次，环境变化时重新调用）
void uploadEnvironment(const std::vector<double>& env_objects);

// 更新每帧机器人包围盒
void updateRobotBoxesGPU(const std::vector<double>& robot_boxes,
                         int N_frames, int N_boxes);

// 执行 GPU 碰撞检测，返回 (clearances, colliding) 向量
std::pair<std::vector<double>, std::vector<double>> checkCollisionsGPU(
    int N_frames, int N_boxes);
```

内部管理 4 个设备缓冲区：`d_env_objects_`, `d_robot_boxes_`, `d_clearances_`, `d_colliding_`。

## 性能数据

在 NVIDIA GeForce RTX 4060 Laptop GPU 上的实测结果：

| 帧数 | Boxes | Objects | Pairs/帧 | 时间 (ms) | Pairs/ms |
|------|-------|---------|---------|-----------|----------|
| 100 | 8 | 5 | 40 | 0.010 | 396,071 |
| 1,000 | 8 | 5 | 40 | 0.022 | 1,799,330 |
| 10,000 | 8 | 5 | 40 | 0.069 | 2,646,000 |
| 1,000 | 8 | 20 | 160 | 0.069 | 2,303,000 |

**编译指标**: 36 寄存器、0 溢出、0 字节栈帧、0 字节共享内存。

## 局限性

1. **轴对齐包围盒 (AABB)**: 当前实现使用轴对齐包围盒，不支持旋转盒。对于旋转连杆，需先在主机端计算 OBB 的 AABB 外包。
2. **简单几何体**: 不支持 mesh-mesh 或 mesh-primitive 碰撞（保留给 CPU FCL）。
3. **圆柱方向**: box-cylinder 基元假定圆柱轴为 Z 向（垂直）。倾斜圆柱需预处理旋转。
