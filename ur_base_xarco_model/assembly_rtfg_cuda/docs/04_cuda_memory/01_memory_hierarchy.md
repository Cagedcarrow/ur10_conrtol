# GPU 内存层次总览

## 概述

CUDA 的 GPU 内存层次结构是本功能包实现高性能逆运动学求解的基石。`assembly_rtfg_cuda` 充分利用了 GPU 内存层次中的每一级缓存/存储，将 DLS 迭代完全保持在片上执行，实现了 168× 超过 Roofline Ridge Point 的算术强度。

## CUDA GPU 内存层次结构

```
┌──────────────────────────────────────────────────────────────┐
│                    全局内存 (Global Memory)                    │
│                    ~8 GB GDDR6, ~256 GB/s                    │
│                       512-bit 宽                             │
├──────────────────────────────────────────────────────────────┤
│                      L2 缓存 (24 MB)                        │
├──────────┬───────────────────────────────────┬───────────────┤
│ 常量内存  │         L1 / 共享内存             │   寄存器文件   │
│ 64 KB    │    每个 SM: 128 KB (可配置)       │  每个 SM:     │
│ + 缓存    │    本包配置: 48 KB 共享 + 80 KB L1│  65,536 × 32b │
├──────────┴───────────────────────────────────┴───────────────┤
│                        GPU 线程 (CUDA Cores)                  │
│                    3,072 CUDA Cores @ 2.37 GHz               │
└──────────────────────────────────────────────────────────────┘
```

## 各层次在本包中的使用

### 1. 全局内存 (Global Memory)

**特点**: 容量最大 (8 GB)、延迟最高 (~400-800 cycles)、所有线程可读写

**本包用途**: 
- 输入缓冲区：273 个目标位姿变换矩阵 (`d_targets_`, `cuda_ik_solver.cu:69-70`)
- 输入缓冲区：273 个种子关节角 (`d_seeds_`, `cuda_ik_solver.cu:71-72`)
- 输出缓冲区：273 × 6 个求解结果 (`d_results_`, `cuda_ik_solver.cu:73`)
- 输出缓冲区：273 × 2 个误差值 (`d_errors_`, `cuda_ik_solver.cu:74`)

**总 DRAM 流量** (273 目标 批处理):
- DRAM 读: `273 × (16 + 6) × 8 = 48,048 bytes`（目标位姿 + 种子）
- DRAM 写: `273 × (6 + 2 + 1) × 8 = 19,656 bytes`（结果 + 误差 + 迭代次数）
- **总流量: ~68 KB** → 仅占 256 GB/s 带宽的 0.16%

### 2. 常量内存 (Constant Memory) — [详细分析](03_constant_memory.md)

**特点**: 64 KB, 只读, 同一 warp 内所有线程读同一地址时仅 1 周期延迟

**本包用量**: **1,384 bytes** (`cuda_utilities.cuh:80-86`)
- `c_segment_origins[96]` — 6 段 × 16 个 double = 768 bytes
- `c_segment_axes[18]` — 6 段 × 3 个 double = 144 bytes
- `c_q_index[6]` — 6 个 int = 24 bytes
- `c_T_wrist3_to_tcp[16]` — 1 个 4×4 矩阵 = 128 bytes
- `c_joint_limits[12]` — 6×2 限位 = 96 bytes
- `c_weight_schedule[24]` — 4 级 × 6 权重 = 192 bytes
- `c_lambda_params[4]` — 4 个阻尼参数 = 32 bytes

### 3. 共享内存 (Shared Memory) — [详细分析](04_shared_memory.md)

**特点**: 每个 Block 私有, 低延迟 (~30 cycles), 可编程, 32-bank 架构

**本包用量**: **1,616 bytes/block** (`cuda_kernels.cu:51-65`)

| 变量 | 类型 | 有效大小 | 实际分配 | 说明 |
|------|------|---------|---------|------|
| `s_q[8]` | double | 6 | 8 (6+2 padding) | 当前关节角 |
| `s_T[16]` | double | 16 | 16 | 当前 FK 结果 4×4 |
| `s_T_tgt[16]` | double | 16 | 16 | 目标位姿 4×4 |
| `s_T_tcp[16]` | double | 16 | 16 | 铲斗 TCP 变换 4×4 |
| `s_T_tcp_tgt[16]` | double | 16 | 16 | 铲斗 TCP 目标 4×4 |
| `s_J[48]` | double | 36 | 48 (6×8) | 雅可比矩阵 (padding) |
| `s_H[48]` | double | 36 | 48 (6×8) | 海森矩阵 (padding) |
| `s_err[6]` | double | 6 | 6 | 位姿误差 |
| `s_g[6]` | double | 6 | 6 | 梯度向量 |
| `s_dq[6]` | double | 6 | 6 | 关节步长 |
| `s_q_ref[6]` | double | 6 | 6 | 参考关节角 |
| `s_q_best[6]` | double | 6 | 6 | 最佳关节角 |
| 标量变量 | int/double | 5 | 5 | converged, iter_count, lambda, etc. |
| **总计** | | | **~1,616 bytes** | |

### 4. 寄存器文件 (Register File) — [详细分析](05_register_usage.md)

**特点**: 最快存储 (0 周期), 每个线程私有, 64 KB/SM

**本包用量**: **96 registers/thread** (ncu 实测)
- 128 threads/block × 96 regs = 12,288 regs/block
- 6 blocks/SM (floor(65,536 / 12,288))
- Ada 物理 Occupancy: 6 × 4 warps / 48 warps.max = **50.0%**
- CUDA Occupancy API 报告值 (maxWarpsPerSM=32): **18.75%**
- 零寄存器溢出 (spill) — PTX 汇编器确认

### 5. L1 缓存

**特点**: 每个 SM 私有, 与共享内存共享 128 KB

**本包配置**: 80 KB L1 + 48 KB 共享内存 (未显式配置, 使用默认分配)
- L1 缓存对本包的主要作用是缓存常量和全局内存访问

### 6. L2 缓存 (24 MB)

**特点**: 所有 SM 共享, 缓存全局/常量内存访问

**本包实测效果**: 
- DRAM Throughput: 0.16% (ncu 实测, `cuda_kernels.cu`)
- 核函数输出数据完全缓存于 L2, DRAM 写 = 0 bytes

## 性能关键发现

| 指标 | 实测值 | 意义 |
|------|--------|------|
| DRAM Throughput | **0.16%** | DLS 迭代完全在片上执行 |
| Compute Throughput (FP64) | **2.20%** | 消费级 GPU FP64 硬件受限 |
| Bank 冲突 | **0** | 8列 padding 策略完全有效 |
| 寄存器溢出 | **0 (零溢出)** | 寄存器使用充分但不超额 |
| L1/TEX Throughput | 0.75% | 低缓存压力 |
| L2 Throughput | 0.24% | 低缓存压力 |

## 相关文件

- [DeviceBuffer RAII 封装](02_device_buffer.md)
- [常量内存广播](03_constant_memory.md)
- [共享内存 Bank 冲突避免](04_shared_memory.md)
- [寄存器使用分析](05_register_usage.md)
- [完整内存生命周期](06_memory_lifecycle.md)
