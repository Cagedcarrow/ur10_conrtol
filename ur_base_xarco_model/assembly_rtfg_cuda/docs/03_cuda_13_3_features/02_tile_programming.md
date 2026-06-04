# C++ Tile Programming (cuda::tile)

## 概述

CUDA 13.3 引入了基于 CUTLASS 3.x 的 C++ Tile 编程接口，通过 `cuda::tile` 模板类型在 CUDA Kernel 中表达分块矩阵操作。编译器自动推导共享内存布局、bank 冲突回避策略，以及最优的 memory op 指令选择。

## 基本原理

传统 CUDA Kernel 中，开发者手动管理共享内存布局：

```cuda
// 传统方式：手动 8 列填充避免 bank 冲突
__shared__ double s_J[48];  // 6×8 padding
s_J[row * 8 + col] = value; // 手动索引
```

使用 `cuda::tile`，编译器自动处理：

```cuda
// CUDA 13.3 Tile 方式
cuda::tile<double, 6, 6> s_J;  // 编译器自动 padding
s_J(row, col) = value;           // 自然 2D 索引
```

## 应用场景：6×6 双精度矩阵

### 共享内存声明对比

| 特性 | 传统方式 | cuda::tile 方式 |
|------|---------|----------------|
| 声明 | `double s_H[48]` | `cuda::tile<double,6,6> s_H` |
| 索引 | `s_H[row*8+col]` | `s_H(row, col)` |
| Bank 冲突 | 需手动 padding | 编译器自动最优布局 |
| 内存布局 | 6×8 = 48 doubles | 编译器决定 |
| 冗余空间 | 12 doubles (25%) | 最优 (可能 0) |

### 对本功能包的潜在价值

IK Kernel 中有三个 6×6 矩阵需要共享内存存储：

1. **Jacobian J** (6×6) — `s_J[48]` (6×8 padding)
2. **Hessian H** (6×6) — `s_H[48]` (6×8 padding)

如果迁移到 `cuda::tile`:

```cuda
#include <cuda/tile.hpp>

__shared__ cuda::tile<double, 6, 6> s_J;
__shared__ cuda::tile<double, 6, 6> s_H;

// Jacobian 赋值
s_J(row, col) = value;

// Hessian 计算
for (int k = 0; k < 6; ++k) {
    double w2 = w_k * w_k;
    sum += s_J(k, row) * w2 * s_J(k, col);
}
s_H(row, col) = sum;
```

## 优势

| 优势 | 说明 |
|------|------|
| 代码简洁性 | 自然 2D 索引，无需手动计算偏移 |
| 自动优化 | 编译器选择最优共享内存布局 |
| 可移植性 | 在不同 GPU 架构间自动适配 |
| 减少错误 | 消除手工索引错误 |

## 局限

- 需要 CUDA 13.3+ 和 sm_80+ (Ampere 及以上)
- 当前本功能包未使用，需逐步迁移
- 对 6×6 小矩阵，手动管理性能已接近最优

## 结论

`cuda::tile` 是 CUDA 13.3 的重要新特性，可简化共享内存管理。但对于本功能包的 6×6 固定大小矩阵，手动 8 列填充已实现 0 bank 冲突，迁移收益主要在代码可维护性而非性能。
