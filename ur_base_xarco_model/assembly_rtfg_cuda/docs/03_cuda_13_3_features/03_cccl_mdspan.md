# CCCL 3.3 mdspan

## 概述

CCCL (CUDA C++ Core Libraries) 3.3 集成了 Thrust、CUB、libcu++，并新增 `cuda::std::mdspan` — 一个非拥有式的多维数组视图，用于在 Kernel 内以自然多维语法访问数据。

## 关键特性

```cuda
#include <cuda/std/mdspan>

namespace ck = cuda::std;

// 在 Kernel 中创建 6×6 矩阵视图
__global__ void my_kernel(double* data) {
    ck::mdspan<double, ck::extents<6, 6>> A(data);
    // 现在可以用 A(i,j) 访问
    double val = A(3, 2);  // 第 3 行第 2 列
    A(0, 0) = 1.0;
}
```

## 对本功能包的潜在应用

### 1. 全局内存矩阵访问

当前 IK Kernel 中，目标位姿矩阵通过一维数组访问：

```cuda
// 当前方式
double t00 = d_targets[tid * 16 + 0];  // 行优先索引
double t01 = d_targets[tid * 16 + 1];
```

使用 mdspan:

```cuda
// mdspan 方式
ck::mdspan<const double, ck::extents<ck::dynamic_extent, 16>> 
    targets(d_targets + tid * 16, 16);
double t00 = targets(0, 0);
```

### 2. 结果数组访问

```cuda
// 当前方式
d_results[tid * 6 + j] = s_q[j];

// mdspan 方式
ck::mdspan<double, ck::extents<ck::dynamic_extent, 6>>
    results(d_results, N, 6);
results(tid, j) = s_q[j];
```

### 3. 共享内存矩阵

与 `cuda::tile` 结合使用：

```cuda
__shared__ double s_J_raw[48];  // 6×8 padding
ck::mdspan<double, ck::extents<6, 6>> s_J(s_J_raw);
// 注意：mdspan 不能表达 padding 偏移，需要 strides
ck::mdspan<double, ck::extents<6, 6>, 
           ck::layout_stride<ck::extents<6, 6>>>
    s_J_padded(s_J_raw, {8, 1});  // stride: row=8, col=1
```

## DLPack 互操作

CCCL 3.3 的 mdspan 支持 DLPack 格式的零拷贝互操作，允许在 PyTorch、CuPy 和 CUDA C++ 之间共享张量数据，无需序列化。

## 当前使用情况

本功能包**未使用** CCCL 3.3 或 mdspan。所有数据通过 CPU 端 `std::vector` 打包，使用 CUDA Runtime API 进行 H2D/D2H 传输。

## 迁移建议

| 组件 | 迁移优先级 | 说明 |
|------|-----------|------|
| d_targets_ 访问 | 低 | 仅 kernel 入口处使用一次，收益有限 |
| d_results_ 写入 | 低 | 同样单次使用 |
| 共享内存矩阵 | 中 | 结合 cuda::tile 使用收益更大 |
| LDL^T 求解器 | 低 | 所有数据在寄存器中，无需 mdspan |
