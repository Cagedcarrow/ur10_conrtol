# CUDA 13.3 新特性总览

## 版本信息

| 属性 | 值 |
|------|-----|
| CUDA 版本 | 13.3 |
| 驱动版本 | 570.86.10 |
| 支持 GPU 架构 | Kepler ~ Blackwell (sm_35 ~ sm_100) |
| 本功能包目标架构 | sm_89 (Ada Lovelace) |

## CUDA 13.3 主要新特性

### 1. C++ Tile Programming (cuda::tile)

CUDA 13.3 引入了基于 CUTLASS 3.x 的 C++ Tile 编程接口，允许开发者使用 `cuda::tile` 模板类型定义分块矩阵操作，编译器自动处理共享内存布局和 bank 冲突避免。

> **本功能包应用**: 当前未使用 `cuda::tile`。IK Kernel 使用手动共享内存管理（6×8 填充）。未来可迁移到 `cuda::tile<double,6,6>` 简化代码。

### 2. CCCL 3.3 (CUDA C++ Core Libraries)

CCCL 3.3 整合了 Thrust、CUB、libcu++ 三大库，新增了 `cuda::std::mdspan` 用于多维数组视图。

> **本功能包应用**: 当前未使用 CCCL 3.3。使用的 CUDA 特性（DeviceBuffer、cudaMemcpyAsync 等）均为 CUDA Runtime API 中的基础功能。

### 3. CompileIQ AI 编译器

NVCC 集成了 AI 驱动的编译器，可以自动分析循环/分支/内存模式并选择最优编译策略。

> **本功能包应用**: 当前通过 `-O3 -lineinfo --ptxas-options=-v` 使用传统优化。CUDA 13.3 的 CompileIQ 可自动优化 Kernel 性能。

### 4. CUDA Python 1.0 (cuda.core)

正式发布 `cuda.core` Python 包，提供设备枚举、流创建、内存分配、Kernel Launch 的 Python API。

> **本功能包应用**: 当前为 C++ 实现，未使用 CUDA Python。

### 5. CUDA Graphs 增强

CUDA 13.3 改进了 CUDA Graphs 对动态形状的支持，可更灵活地捕获和重放 Kernel 序列。

> **本功能包应用**: 当前未使用 CUDA Graphs。单次 IK 批处理求解使用单一 Stream + 直接 Kernel Launch。

## Ada Lovelace (sm_89) 架构特性

| 特性 | 规格 | 说明 |
|------|------|------|
| SM 数量 | 24 | RTX 4060 Laptop |
| CUDA Cores/SM | 128 | 每个 SM 128 个 FP32 Core |
| FP64 Core/SM | 4 | FP64:FP32 = 1:32 |
| Shared Memory/SM | 48 KB | 可配置 |
| Register File/SM | 65,536 | 32-bit registers |
| Warp Size | 32 | 标准 |
| Max Threads/SM | 1,024 | 32 Warps |
| Max Blocks/SM | 32 | Ada Lovelace |
| L1 Cache | 128 KB | 与 Shared Memory 统一 |
| L2 Cache | 32 MB | Ada Lovelace |
| Memory Bus | 128-bit | GDDR6 |
| Memory Bandwidth | 256 GB/s | 理论峰值 |

## 本功能包实际使用的 CUDA 特性

| 特性 | 是否使用 | 说明 |
|------|---------|------|
| CUDA Runtime API | ✅ | cudaMalloc/Free/MemcpyAsync/DeviceSynchronize |
| __constant__ Memory | ✅ | 7 个常量数组 (1,384 B) |
| Shared Memory | ✅ | 8 列填充手动管理 |
| Registers | ✅ | 98 regs/thread |
| cudaStream_t | ✅ | 单非阻塞流 |
| cudaMemcpyAsync | ✅ | 异步 H2D/D2H |
| Cooperative Groups | ❌ | 未使用 thread_block_tile |
| CUDA Graphs | ❌ | 未使用 |
| Tensor Cores | ❌ | 无需 FP16/INT8 运算 |
| Dynamic Parallelism | ❌ | 无需 |
| NVTX | ❌ | 未添加性能标记 |
| cuBLAS/cuSOLVER | ❌ | 手写 LDL^T 求解器 |
