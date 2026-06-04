# LDL^T 6×6 求解器

## 概述

LDL^T 分解是 Cholesky 分解的一种变体，用于求解对称正定 (SPD) 线性系统。与标准 Cholesky ($LL^T$) 相比，LDL^T 避免了平方根运算，在双精度计算中更稳定。

**源码位置**: `cuda_utilities.cuh:243-291`

## 求解问题

$$(J^T W^2 J + \lambda I) \cdot dq = J^T W^2 e$$

等效于：

$$H \cdot dq = g$$

其中 $H$ 是 6×6 对称正定矩阵，$g$ 是 6 维梯度向量。

## 算法原理

### LDL^T 分解

对称正定矩阵 $H$ 可分解为：

$$H = L \cdot D \cdot L^T$$

其中：
- $L$ — 单位下三角矩阵 (对角线元素为 1)
- $D$ — 对角矩阵

### 求解步骤

```
Step 1: H = L·D·L^T    (分解, ~57 标量运算)
Step 2: L·y = g         (前代, ~15 标量运算)
Step 3: z = y ./ D      (对角缩放, 6 标量运算)
Step 4: L^T·dq = z      (回代, ~15 标量运算)
```

总运算量约 **63 FP64 FLOP**，在 GPU 上耗时约 **0.1 μs**。

## GPU 实现

### 完整代码

```cpp
// cuda_utilities.cuh:243-291
__device__ __forceinline__ void ldlt_solve_6x6(
    const double* H, const double* g, double* dq) {
    
    double L[6][6] = {{0}};  // 单位下三角
    double D[6] = {0};       // 对角矩阵
    double y[6] = {0};       // 中间变量
    double x[6] = {0};       // 解向量

    // 复制到本地 (padded → dense)
    double A[6][6];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            A[i][j] = H[i * 6 + j];

    // ── Step 1: LDL^T 分解 ──
    for (int j = 0; j < 6; ++j) {
        // D[j] = A[j][j] - Σ_k L[j][k]²·D[k]
        double d = A[j][j];
        for (int k = 0; k < j; ++k)
            d -= L[j][k] * L[j][k] * D[k];
        D[j] = d;

        // L[i][j] = (A[i][j] - Σ_k L[i][k]·L[j][k]·D[k]) / D[j]
        for (int i = j + 1; i < 6; ++i) {
            double sum = A[i][j];
            for (int k = 0; k < j; ++k)
                sum -= L[i][k] * L[j][k] * D[k];
            L[i][j] = sum / D[j];
        }
        L[j][j] = 1.0;
    }

    // ── Step 2: 前代 L·y = g ──
    for (int i = 0; i < 6; ++i) {
        double sum = g[i];
        for (int k = 0; k < i; ++k)
            sum -= L[i][k] * y[k];
        y[i] = sum;
    }

    // ── Step 3: 对角缩放 z = y / D ──
    for (int i = 0; i < 6; ++i)
        y[i] = y[i] / D[i];

    // ── Step 4: 回代 L^T·x = z ──
    for (int i = 5; i >= 0; --i) {
        double sum = y[i];
        for (int k = i + 1; k < 6; ++k)
            sum -= L[k][i] * x[k];
        x[i] = sum;
    }

    // 输出结果
    for (int i = 0; i < 6; ++i) dq[i] = x[i];
}
```

### 为什么不用 cuBLAS？

对于 6×6 矩阵，`cublasDgemm` 或 `cublasDgetrf` 的启动开销 (~5-10 μs) 远超计算本身 (~0.1 μs)。因此选择内联 `__device__` 函数实现手写求解器。

### 数值稳定性

```cpp
// 正定性检查（隐式）
// 如果 D[j] ≤ 0，矩阵非正定，LDL^T 失败
// 但由于 λ > 0，H = J^T·J + λI 始终正定
```

## 运算分解

### LDL^T 分解 (核心循环)

```
j=0: D[0]=A[0][0], L[1..5][0] = A[1..5][0]/D[0]
j=1: D[1]=A[1][1]-L[1][0]²D[0], L[2..5][1] = (A[2..5][1]-L[2..5][0]L[1][0]D[0])/D[1]
j=2: D[2]=A[2][2]-L[2][0]²D[0]-L[2][1]²D[1], ...
...
```

### 标量运算计数

| 阶段 | 乘加 | 除法 | 总计 |
|------|------|------|------|
| 分解 | 50 | 6 | 57 |
| 前代 | 15 | 0 | 15 |
| 缩放 | 0 | 6 | 6 |
| 回代 | 15 | 0 | 15 |
| **总计** | **80** | **12** | **~93** |

实际 FP64 运算量约 **93 FLOP**（计入乘加为 2 个 FLOP）。

## 调用上下文

```cpp
// cuda_kernels.cu:226-236
if (threadIdx.x == 0) {           // 仅 Lane 0 (Warp 3) 执行
    double H_dense[36], g_dense[6];
    // 从 padded 共享内存 → 密集本地数组
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c)
            H_dense[r * 6 + c] = s_H[r * 8 + c];
        g_dense[r] = s_g[r];
    }
    ldlt_solve_6x6(H_dense, g_dense, s_dq);  // 结果写入共享内存
}
```

## 性能特征

| 指标 | 值 |
|------|-----|
| 单次求解时间 | ~0.1 μs |
| FP64 运算量 | ~93 FLOP |
| 寄存器使用 | ~18 (6×6 矩阵 + 中间变量) |
| 串行路径 | 是 (Lane 0 独占) |
| 是否可并行 | 否 (数据依赖链) |

## 矩阵规模适应

对于 6×6 的固定规模，手写展开循环可以进一步优化：

```cpp
// 理论优化: 完全展开 (避免循环开销)
// j=0
D[0] = A[0][0];
L10 = A[1][0] / D[0];
L20 = A[2][0] / D[0];
// ...
```

当前编译器 (`-O3`) 可能已经执行此优化。

## 相关代码行号

| 功能 | 文件 | 行号 |
|------|------|------|
| LDL^T 函数完整实现 | `cuda_utilities.cuh` | 243-291 |
| 函数调用 | `cuda_kernels.cu` | 235 |
| H_dense 打包 (padded→dense) | `cuda_kernels.cu` | 228-234 |
| CPU 参考 LDL^T (测试用) | `test/test_cuda_kernel.cu` | 292-361 |
