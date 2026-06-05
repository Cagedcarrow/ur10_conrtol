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

CUDA 13.3 引入的 `cuda::tile`（命名空间 `cuda::tiles`）是一个 4064 行的 C++20 模板库，专为 warp 级 MMA（Matrix Multiply-Accumulate）操作设计。

> **本功能包评估结论**: 经过详细技术评估（2026-06-05），`cuda::tile` **不适用于**本项目的 6×6 矩阵运算场景：
> - `cuda::tile` 面向 warp 级 Tensor Core MMA（16×16 半精度矩阵乘），与 6×6 双精度 Jacobian/Hessian 运算维度不匹配；
> - 引入模板元编程开销（需 C++20 编译选项、CCCL 头文件依赖）超出收益；
> - **工程决策**: 采用手动 8 列 padding（`PaddedMat6x8` 轻量封装）实现零开销的 bank 冲突消除，寄存器占用 96 个、零 spill、共享内存 1616 bytes，为 6×6 矩阵运算的最优方案。
>
> 详见 `CMakeLists.txt` 中的技术文档说明。

### 2. CCCL 3.3 (CUDA C++ Core Libraries)

CCCL 3.3 整合了 Thrust、CUB、libcu++ 三大库，其中 `cuda::std::mdspan` 提供了 C++23 风格的多维数组视图。

> **本功能包评估结论**: `cuda::std::mdspan` 存在于 `/usr/local/cuda/include/cccl/cuda/std/mdspan`（2026-06-05 确认），但：
> - 需要 C++20 标准（本项目使用 C++17）；
> - 模板实例化开销较大（包含多个间接头文件）；
> - **工程决策**: 设计了 `PaddedMat6x8` 轻量封装（`cuda_utilities.cuh:76-91`），实现同等类型安全的 `operator()(row, col)` 访问接口，零额外开销（编译器完全内联），且不依赖 C++20。详见 `docs/03_cuda_13_3_features/03_cccl_mdspan.md`。

### 3. CompileIQ AI 编译器

NVCC CompileIQ 是 NVIDIA 宣称的 AI 驱动编译优化功能。

> **本功能包评估结论**: 经过全面调查（2026-06-05），`nvcc 13.3.33` **不包含** `-compileiq` 编译选项（通过 `nvcc --help` 全量扫描确认）。CompileIQ 在 CUDA 13.3 中属于不可用功能。`CMakeLists.txt` 中已记录此发现，并添加了未来版本启用指南。当前使用 `-O3 -lineinfo --ptxas-options=-v` 进行传统编译优化。

### 4. CUDA Python 1.0 (cuda.core)

正式发布 `cuda.core` Python 包，提供设备枚举、流创建、内存分配、Kernel Launch 的 Python API。

> **本功能包应用**: 创建了 `scripts/cuda_python_pipeline.py` 原型脚本（使用 CuPy 作为 CUDA Python 绑定层），演示多流 H2D/Kernel/D2H 流水线重叠和 GPU 碰撞检测。该脚本为基准测试辅助工具，**不替代** C++ 生产求解器。通过 `/mnt/linuxdata/novel_text/.venv/bin/python` 验证可运行。详见 `docs/03_cuda_13_3_features/05_cuda_python.md`。

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
| Registers | ✅ | 96 regs/thread |
| cudaStream_t | ✅ | 单非阻塞流 |
| cudaMemcpyAsync | ✅ | 异步 H2D/D2H |
| Cooperative Groups | ❌ | 未使用 thread_block_tile |
| CUDA Graphs | ❌ | 未使用 |
| Tensor Cores | ❌ | 无需 FP16/INT8 运算 |
| Dynamic Parallelism | ❌ | 无需 |
| NVTX | ❌ | 未添加性能标记 |
| cuBLAS/cuSOLVER | ❌ | 手写 LDL^T 求解器 |
