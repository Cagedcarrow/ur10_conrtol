# 铲斗运动学链 (Shovel Kinematics Chain)

## 概述

UR10 料斗挖掘作业中，末端执行器为 **sensor_shovel**（传感铲斗）组件，其运动学链比标准 UR10 TCP 更长：

```
UR10基座 → ... → wrist_3 → ur10-sensor_shovel → sensor_shovel_tcp_fixed
                                                       ↑
                                                 铲斗尖端 (实际作业点)
```

铲斗尖端（`sensor_shovel_tcp_fixed`）是实际的轨迹拟合目标点，需要独立的运动学误差分析。

## 工具链变换矩阵

### ur10-sensor_shovel 固定关节

连接 wrist_3 到 sensor_shovel 本体的固定偏移：

| 参数 | 值 |
|------|-----|
| RPY | (-1.5707963, 0, 0) |
| XYZ | (0, 0.09, 0) |

### sensor_shovel_tcp_fixed 固定关节

铲斗本体到作业尖端的固定偏移：

| 参数 | 值 |
|------|-----|
| RPY | (-1.5708, 1.5708, -0.61087) |
| XYZ | (-0.47377, 0.077109, 0.0733) |

### 复合变换

```
T_wrist3_to_tcp = T_wrist3_to_sensor_shovel × T_sensor_shovel_to_tcp
```

预计算值（16 个 double，行优先 4×4 齐次矩阵）：

```
[ -3.00890e-06  -8.19151e-01   5.73577e-01  -4.73770e-01 ]
[ -9.99999e-01   3.68857e-06   2.19626e-08   1.63300e-01 ]
[ -2.13367e-06  -5.73577e-01  -8.19151e-01  -7.71090e-02 ]
[   0.0          0.0           0.0           1.0          ]
```

## GPU 实现

### 铲斗 TCP 变换

`cuda_utilities.cuh:279-282` — 从 wrist_3 变换到铲斗尖端：

```cuda
__device__ __forceinline__ void shovel_tcp_transform(
    const double* T_wrist3, const double* T_wrist3_to_tcp, double* T_tcp) {
  mat44_mul(T_wrist3, T_wrist3_to_tcp, T_tcp);
}
```

### 铲斗位姿误差

`cuda_utilities.cuh:289-315` — 计算铲斗尖端的位置误差（欧氏距离）和姿态误差（SO(3) 测地距离）：

```cuda
__device__ __forceinline__ void shovel_pose_error(
    const double* T_tcp_cur, const double* T_tcp_tgt,
    double* pos_err, double* rot_err) {
  // 位置误差：铲斗尖端世界坐标欧氏距离
  double dx = T_tcp_tgt[3]  - T_tcp_cur[3];
  double dy = T_tcp_tgt[7]  - T_tcp_cur[7];
  double dz = T_tcp_tgt[11] - T_tcp_cur[11];
  *pos_err = sqrt(dx * dx + dy * dy + dz * dz);

  // 姿态误差：SO(3) 测地距离 θ = arccos((trace(R_err) - 1) / 2)
  // R_err = R_cur^T × R_tgt（仅计算对角线）
  double e00 = r00*t00 + r10*t10 + r20*t20;
  double e11 = r01*t01 + r11*t11 + r21*t21;
  double e22 = r02*t02 + r12*t12 + r22*t22;
  *rot_err = acos(cuda_clamp((e00+e11+e22 - 1.0) * 0.5, -1.0, 1.0));
}
```

### 铲斗尖端位置提取

`cuda_utilities.cuh:319-324`：

```cuda
__device__ __forceinline__ void shovel_tcp_position(
    const double* T_tcp, double* px, double* py, double* pz) {
  *px = T_tcp[3]; *py = T_tcp[7]; *pz = T_tcp[11];
}
```

## Kernel 集成

`cuda_kernels.cu` 中的 `ik_batch_solve` kernel 在 Phase 3 结果阶段调用铲斗运动学：

1. **共享内存分配**: `s_T_tcp[16]`（当前 TCP 变换）、`s_T_tcp_tgt[16]`（目标 TCP 变换）
2. **FK 后计算**: 调用 `shovel_tcp_transform(T_wrist3, c_T_wrist3_to_tcp, s_T_tcp)` 和 `shovel_tcp_transform(T_tgt, c_T_wrist3_to_tcp, s_T_tcp_tgt)`
3. **误差输出**: 调用 `shovel_pose_error(s_T_tcp, s_T_tcp_tgt, &pos_err, &rot_err)`，结果写入 `d_shovel_errors[2*blockIdx.x + 0/1]`

## C++ 接口

`CandidateInfo` 结构体（`types.h`）新增字段：

```cpp
struct CandidateInfo {
  // ... 原有字段 ...
  double shovel_pos_err;  // 铲斗 TCP 位置误差 (m)
  double shovel_rot_err;  // 铲斗 TCP 姿态误差 (rad)
};
```

`CudaBatchIK` 类自动管理 `d_shovel_errors_` 设备缓冲区，`flush()` 方法在 kernel 启动后读回铲斗误差并填充 `CandidateInfo`。

## 设计意义

铲斗运动学链的独立分析允许：

1. **腕部 vs 尖端精度对比**: 同时获得 wrist_3 误差和 shovel_tcp 误差，评估工具链杠杆放大效应
2. **轨迹质量评估**: 铲斗尖端位置误差是实际作业质量的关键指标
3. **零额外计算开销**: 铲斗变换复用 FK 中间结果（T_wrist3），仅增加一次 4×4 矩阵乘法和 SO(3) 测地距离计算
