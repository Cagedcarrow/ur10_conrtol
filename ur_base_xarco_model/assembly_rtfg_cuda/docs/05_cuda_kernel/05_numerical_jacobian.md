# 数值雅可比矩阵

## 概述

雅可比矩阵 **J ∈ ℝ^(6×6)** 将关节空间的速度映射到任务空间 (TCP) 的速度。本包使用**中心差分法**在 GPU 上数值计算雅可比矩阵，6 列并行执行。

**源码位置**: `cuda_kernels.cu:131-173`

## 数学定义

雅可比矩阵的每一列对应一个关节的微分运动：

$$J_{:,j} = \frac{\partial f(q)}{\partial q_j} \approx \frac{f(q + \epsilon e_j) - f(q - \epsilon e_j)}{2\epsilon}$$

其中：
- $f(q)$ — 正运动学函数 (FK)，输出 TCP 位姿 (6-DOF: 位置 + 旋转)
- $\epsilon$ — 微小扰动量 (1e-6 rad)
- $e_j$ — 第 j 个单位向量

雅可比矩阵结构：
```
J = [∂p/∂q₁  ∂p/∂q₂  ...  ∂p/∂q₆]   位置部分 (3×6)
    [∂r/∂q₁  ∂r/∂q₂  ...  ∂r/∂q₆]   旋转部分 (3×6)
```

## GPU 实现

### 6 列并行计算

```cpp
// cuda_kernels.cu:133-173
if (threadIdx.x < 6) {
    int j = threadIdx.x;        // 列索引 0-5
    const double eps = 1e-6;

    // 关节角扰动
    double q_plus[6], q_minus[6], T_p[16], T_m[16];
    for (int i = 0; i < 6; ++i) {
        q_plus[i] = s_q[i];
        q_minus[i] = s_q[i];
    }
    q_plus[j]  += eps;
    q_minus[j] -= eps;

    // 计算两个 FK
    forward_kinematics(q_plus, T_p);
    forward_kinematics(q_minus, T_m);

    double inv_2eps = 0.5 / eps;

    // ── 位置部分 (3行) ──
    s_J[0 * 8 + j] = (T_p[3]  - T_m[3])  * inv_2eps;  // ∂x/∂qⱼ
    s_J[1 * 8 + j] = (T_p[7]  - T_m[7])  * inv_2eps;  // ∂y/∂qⱼ
    s_J[2 * 8 + j] = (T_p[11] - T_m[11]) * inv_2eps;  // ∂z/∂qⱼ

    // ── 旋转部分 (3行) ──
    // R_cur 的旋转矩阵
    double r00 = s_T[0], r01 = s_T[1], r02 = s_T[2];
    double r10 = s_T[4], r11 = s_T[5], r12 = s_T[6];
    double r20 = s_T[8], r21 = s_T[9], r22 = s_T[10];

    // R_diff = R_cur^T · R_{q±eps}
    double dR[9];
    dR[0] = (r00*T_p[0] + r10*T_p[4] + r20*T_p[8])
          - (r00*T_m[0] + r10*T_m[4] + r20*T_m[8]);
    // ... dR[1]~dR[8] 同理 ...

    // 从 R_diff 提取角速度
    s_J[3 * 8 + j] = (dR[7] - dR[5]) * 0.5 * inv_2eps;  // ωx
    s_J[4 * 8 + j] = (dR[2] - dR[6]) * 0.5 * inv_2eps;  // ωy
    s_J[5 * 8 + j] = (dR[3] - dR[1]) * 0.5 * inv_2eps;  // ωz
}
```

### 旋转部分的计算原理

在 `pose_error` 中，旋转误差被表示为 `R_cur^T · R_tgt` 的轴角形式。类似地，数值雅可比的旋转部分通过计算 `R_cur^T · R(q±eps)` 的差分来获得角速度：

1. **正向扰动**: `R_plus = R_cur^T · R(q + eps·eⱼ)`
2. **负向扰动**: `R_minus = R_cur^T · R(q - eps·eⱼ)`
3. **中心差分**: `dR = R_plus - R_minus`
4. **角速度提取**: `ω = 0.5 × [dR₃₂-dR₂₃, dR₁₃-dR₃₁, dR₂₁-dR₁₂]`

### 共享内存存储 (8 列 padding)

```cpp
s_J[row * 8 + col] = value;  // 6×8 padded layout
```

- `s_J[0*8+j]` 到 `s_J[5*8+j]` 共 6 个连续存储
- 每行 8 列 = 64 bytes = 16 Banks → **无 Bank 冲突**

## 每列的计算量

| 操作 | FP64 FLOP |
|------|-----------|
| FK (+eps) | ~250 |
| FK (-eps) | ~250 |
| 位置差分 (3 行) | ~10 |
| 旋转差分 (dR 矩阵) | ~108 |
| 角速度提取 | ~20 |
| **每列总计** | **~638** |

6 列并行总计算量: ~638 FLOP (每列独立, 同时执行)

## 扰动步长选择

```cpp
const double eps = 1e-6;  // 1 微弧度
```

选择依据：
- **太大**: 截断误差增大 (O(ε²))
- **太小**: 舍入误差增大 (浮点数精度)
- **1e-6**: 对于双精度和典型的关节角度 (~1 rad)，是最优平衡点

## 与其他方案的对比

| 方案 | 优势 | 劣势 |
|------|------|------|
| **数值雅可比 (当前)** | 通用，不依赖 FK 解析形式 | 12 次 FK 调用/迭代 |
| 解析雅可比 | 精确，计算量少 | 需要推导 36 个偏导公式 |
| 自动微分 | 精确 | 需要 AD 库支持 |

对于 UR10 这种混合轴机械臂，解析雅可比推导繁琐。数值方法虽然多调用 12 次 FK，但在 GPU 上 6 列并行执行，性能开销在可接受范围内。

## 相关代码行号

| 功能 | 文件 | 行号 |
|------|------|------|
| 数值雅可比实现 | `cuda_kernels.cu` | 131-173 |
| 共享内存声明 (s_J) | `cuda_kernels.cu` | 54 |
| FK 函数 | `cuda_utilities.cuh` | 165-189 |
| 位姿误差函数 | `cuda_utilities.cuh` | 196-236 |
| CPU 参考 Jacobian | `test/test_cuda_kernel.cu` | 215-247 |
