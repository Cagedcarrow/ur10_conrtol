# CUDA 内存管理 — 文档索引

本目录详细分析 `assembly_rtfg_cuda` 功能包中的 CUDA 内存管理策略，涵盖 GPU 内存层次结构、RAII 封装、常量内存广播、共享内存 Bank 冲突避免和寄存器使用分析。

| 文件 | 内容 | 核心代码引用 |
|------|------|-------------|
| [01_memory_hierarchy.md](01_memory_hierarchy.md) | GPU 内存层次总览（全局/常量/共享/寄存器/L1/L2） | `cuda_kernels.cu`, `cuda_utilities.cuh` |
| [02_device_buffer.md](02_device_buffer.md) | DeviceBuffer RAII 封装详解（cudaMalloc/cudaFree/toDevice/toHost） | `cuda_memory.h:14-106` |
| [03_constant_memory.md](03_constant_memory.md) | `__constant__` 常量内存广播（1,384 bytes 运动学参数） | `cuda_utilities.cuh:74-86`, `cuda_ik_solver.cu:261-282` |
| [04_shared_memory.md](04_shared_memory.md) | 共享内存 Bank 冲突避免（8列 padding 策略） | `cuda_kernels.cu:51-65`, ncu Bank 冲突 = 0 |
| [05_register_usage.md](05_register_usage.md) | 寄存器使用分析（98 regs/thread, 零溢出） | ncu 实测, PTX 汇编器报告 |
| [06_memory_lifecycle.md](06_memory_lifecycle.md) | 完整内存生命周期（分配→使用→释放） | `cuda_ik_solver.cu`, `cuda_memory.h` |
