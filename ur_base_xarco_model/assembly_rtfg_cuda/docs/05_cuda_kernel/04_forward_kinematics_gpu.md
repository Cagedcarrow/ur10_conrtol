# GPU 正运动学：Rodrigues 公式 + URDF 约定

## 概述

正运动学 (Forward Kinematics, FK) 是逆运动学求解的基础操作。本包在 GPU 上使用 **Rodrigues 旋转公式**实现 URDF 约定的正运动学计算，支持任意轴旋转（对于 UR10 的混合旋转轴：Z/Y/Y/Y/-Z/Y 至关重要）。

**源码位置**: `cuda_utilities.cuh:106-189`

## FK 算法

### URDF 运动学约定

URDF 描述的运动学链与标准 DH 参数不同。URDF 使用**固定段原点 (origin) + 旋转轴 (axis)** 的表述：

```
对每个可动关节:
    T = T × origin(seg)          # 固定段变换 (平移 + 旋转)
    T = T × Rodrigues(axis, q)   # 关节旋转 (绕 axis 转 q 弧度)
最后:
    T = T × T_wrist3_to_tcp      # 工具末端偏移
```

### 为什么不用 DH 参数？

UR10 的 6 个关节轴为：Z / Y / Y / Y / -Z / Y

标准 DH 假设所有旋转轴为 Z 轴。强行套用 DH 需要额外的 Rx(α) 变换将 Z 扭到 Y，推导复杂且容易出错。GPU 直接使用 URDF 原始参数可以保证与 CPU 端输出完全一致。

## 实现详解

### 1. Rodrigues 旋转公式

**公式** (Rodrigues' Rotation Formula):

```
R = I + sin(θ)·[a]× + (1-cos(θ))·[a]×²

其中 [a]× 是单位轴 a = (ax, ay, az) 的反对称矩阵:

        [ 0   -az   ay]
[a]× =  [ az   0   -ax]
        [-ay   ax   0 ]
```

**展开为 4×4 行主序矩阵** (`cuda_utilities.cuh:106-135`):

```cpp
__device__ __forceinline__ void build_rotation_matrix(
    double ax, double ay, double az, double angle, double* R) {
    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;  // "versine"

    // Row 0
    R[0]  = t * ax * ax + c;
    R[1]  = t * ax * ay + s * az;
    R[2]  = t * ax * az - s * ay;
    R[3]  = 0.0;

    // Row 1
    R[4]  = t * ax * ay - s * az;
    R[5]  = t * ay * ay + c;
    R[6]  = t * ay * az + s * ax;
    R[7]  = 0.0;

    // Row 2
    R[8]  = t * ax * az + s * ay;
    R[9]  = t * ay * az - s * ax;
    R[10] = t * az * az + c;
    R[11] = 0.0;

    // Row 3
    R[12] = 0.0;
    R[13] = 0.0;
    R[14] = 0.0;
    R[15] = 1.0;
}
```

**注意**：行主序存储，R[row*4+col] —— 与 CPU 端 Eigen 的列主序不同。在 GPU 上使用行主序是因为 UV 坐标等习惯。

### 2. 4×4 矩阵乘法

```cpp
// cuda_utilities.cuh:140-151
__device__ __forceinline__ void mat44_mul(
    const double* A, const double* B, double* C) {
    for (int r = 0; r < 4; ++r) {
        double a0 = A[r * 4 + 0];
        double a1 = A[r * 4 + 1];
        double a2 = A[r * 4 + 2];
        double a3 = A[r * 4 + 3];
        C[r * 4 + 0] = a0*B[0] + a1*B[4] + a2*B[8]  + a3*B[12];
        C[r * 4 + 1] = a0*B[1] + a1*B[5] + a2*B[9]  + a3*B[13];
        C[r * 4 + 2] = a0*B[2] + a1*B[6] + a2*B[10] + a3*B[14];
        C[r * 4 + 3] = a0*B[3] + a1*B[7] + a2*B[11] + a3*B[15];
    }
}
```

优化技巧：将 `A` 的一行 4 个元素预加载到寄存器 `a0-a3`，减少共享内存读取次数。

### 3. 完整 FK 函数

```cpp
// cuda_utilities.cuh:165-189
__device__ __forceinline__ void forward_kinematics(const double* q, double* T_tip) {
    // 初始化 4×4 单位矩阵
    for (int i = 0; i < 16; ++i)
        T_tip[i] = (i % 5 == 0) ? 1.0 : 0.0;

    double T_tmp[16], R[16];

    for (int seg = 0; seg < 6; ++seg) {
        // Step 1: T = T × origin(seg)
        mat44_mul(T_tip, &c_segment_origins[seg * 16], T_tmp);
        for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];

        // Step 2: T = T × Rodrigues(axis(seg), q[index])
        double theta = q[c_q_index[seg]];
        build_rotation_matrix(
            c_segment_axes[seg * 3 + 0],
            c_segment_axes[seg * 3 + 1],
            c_segment_axes[seg * 3 + 2],
            theta, R);
        mat44_mul(T_tip, R, T_tmp);
        for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];
    }

    // Step 3: T = T × T_wrist3_to_tcp (工具变换)
    mat44_mul(T_tip, c_T_wrist3_to_tcp, T_tmp);
    for (int i = 0; i < 16; ++i) T_tip[i] = T_tmp[i];
}
```

## 数学表示

### 完整 FK 链

$$T_{base}^{tcp}(q) = \prod_{i=1}^{6} \left[ T_{origin}^{(i)} \cdot R_{Rodrigues}(a^{(i)}, q^{(i)}) \right] \cdot T_{wrist3}^{tcp}$$

其中：
- $T_{origin}^{(i)}$ — 第 i 个关节的固定段变换（4×4 齐次矩阵）
- $R_{Rodrigues}(a, \theta)$ — Rodrigues 旋转矩阵
- $q^{(i)}$ — 第 i 个关节角
- $T_{wrist3}^{tcp}$ — 工具末端偏移

### Rodrigues 展开

$$R(a, \theta) = \begin{bmatrix}
t a_x^2 + c & t a_x a_y - s a_z & t a_x a_z + s a_y & 0 \\
t a_x a_y + s a_z & t a_y^2 + c & t a_y a_z - s a_x & 0 \\
t a_x a_z - s a_y & t a_y a_z + s a_x & t a_z^2 + c & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

其中 $c = \cos\theta$, $s = \sin\theta$, $t = 1 - \cos\theta$

## UR10 关节参数

### 6 个关节的 URDF 参数

| 段 | 关节名 | 原点 xyz (m) | 原点 RPY (rad) | 旋转轴 | q 索引 |
|----|--------|-------------|---------------|--------|-------|
| 0 | shoulder_pan | (0, 0, 0.1273) | (0, 0, 0) | (0, 0, 1) | 0 |
| 1 | shoulder_lift | (0, 0.220941, 0) | (0, 1.57079, 0) | (0, 1, 0) | 1 |
| 2 | elbow | (-3.9e-6, -0.1719, 0.612) | (0, 0, 0) | (0, 1, 0) | 2 |
| 3 | wrist_1 | (-3.6e-6, 0, 0.5723) | (0, 1.5707895, 2.7e-6) | (0, 1, 0) | 3 |
| 4 | wrist_2 | (3e-7, 0.1149, 3e-7) | (0, 0, 0) | (0, 0, -1) | 4 |
| 5 | wrist_3 | (0, -3e-7, 0.1157) | (0, 0, 0) | (0, 1, 0) | 5 |

**注意 wrist_2 的旋转轴是 -Z 方向**！这是 DH 参数难以表达的地方，但 Rodrigues 公式可以自然处理。

## 位姿误差计算

```cpp
// cuda_utilities.cuh:196-236
__device__ __forceinline__ void pose_error(
    const double* T_cur, const double* T_tgt, double* err) {
    // 位置误差 (直接相减)
    err[0] = T_tgt[3]  - T_cur[3];
    err[1] = T_tgt[7]  - T_cur[7];
    err[2] = T_tgt[11] - T_cur[11];

    // 旋转误差: R_err = R_cur^T * R_tgt → axis-angle
    double e00 = r00*t00 + r10*t10 + r20*t20;  // R_cur^T * R_tgt
    // ...
    double trace = e00 + e11 + e22;
    double angle = acos(clamp((trace - 1.0) * 0.5, -1.0, 1.0));

    // 轴角 → 世界坐标系旋转误差
    double s = 0.5 * angle / sin(angle);
    double wx = s * (e21 - e12);
    // ...
    err[3] = T_cur[0]*wx + T_cur[1]*wy + T_cur[2]*wz;  // 转换到世界
}
```

## 性能特征

| 指标 | 值 |
|------|-----|
| 单次 FK 计算量 | ~250 FP64 FLOP (12 次 mat44_mul + 1 次 Rodrigues) |
| 单次 FK 时间 (GPU) | ~0.5 μs |
| Jacobian 中 FK 调用次数 | 12 次/迭代 (6 列 × +eps/-eps) |
| 迭代中 FK 总计算量 | ~3,000 FP64 FLOP |
| 273 目标 × 7.9 迭代 × FK | ~6.5M FP64 FLOP |

## 相关代码行号

| 功能 | 文件 | 行号 |
|------|------|------|
| Rodrigues 旋转矩阵 | `cuda_utilities.cuh` | 106-135 |
| 4×4 矩阵乘法 | `cuda_utilities.cuh` | 140-151 |
| 完整 FK | `cuda_utilities.cuh` | 165-189 |
| 位姿误差 | `cuda_utilities.cuh` | 196-236 |
| 常量内存参数 | `cuda_utilities.cuh` | 80-86 |
| CPU 端 FK (测试参考) | `test/test_cuda_kernel.cu` | 132-173 |
| CPU FK vs GPU FK 验证 | `test/test_cuda_kernel.cu` | 636-712 |
