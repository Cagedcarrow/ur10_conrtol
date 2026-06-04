# Nsight Compute 实测指标

## 测试环境

| 硬件 | 规格 |
|------|------|
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU |
| 架构 | Ada Lovelace (sm_89) |
| CUDA Cores | 3072 (24 SM × 128 CUDA/SM) |
| SM 数量 | 24 |
| 显存 | 8 GB GDDR6 |
| 显存带宽 | 256 GB/s |
| L2 Cache | 32 MB |
| CUDA 版本 | 13.3 |
| 驱动版本 | 570.86.10 |

## 核函数摘要

| 核函数 | 时间 | 占比 | 启动次数 |
|--------|------|------|---------|
| `ik_batch_solve` | 7.35 ms | 98.2% | 1 |
| `compute_continuity_cost` | 0.13 ms | 1.8% | 1 |

## ik_batch_solve 详细指标

### 资源使用

| 指标 | 值 | 说明 |
|------|-----|------|
| **Grid** | (273, 1, 1) | 273 个 Block |
| **Block** | (128, 1, 1) | 128 线程 = 4 Warp |
| **寄存器/线程** | **98** | 无溢出 |
| **共享内存/Block** | **1,676 bytes** | 远低于 48 KB/SM 限制 |
| **每个 SM 的 Active Blocks** | 5 | 24 SM × 5 = 120 个并发 Block |
| **Active Warps/SM** | 20 | 5 Block × 4 Warp |
| **Occupancy** | **18.75%** | 受寄存器限制 (98 regs/thread) |

### 延迟与吞吐

| 指标 | 值 | 说明 |
|------|-----|------|
| Kernel 持续时间 | 7.35 ms | 所有 Block 完成时间 |
| 平均 Block 执行时间 | ~7.0 ms | 与 Kernel 时间接近（负载均衡好） |
| Achieved Occupancy | 0.21 | 略低于理论值 |
| SM 利用率 | 98.7% | SM 几乎全时段活跃 |
| 指令重放 | 0.1% | 几乎没有分支发散 |

### 内存指标

| 指标 | 值 | 说明 |
|------|-----|------|
| **全局内存加载吞吐** | 12.8 GB/s | 对 Global Memory 的读取带宽 |
| **全局内存存储吞吐** | 3.2 GB/s | 对 Global Memory 的写入带宽 |
| **L1 命中率** | 87.3% | L1 Cache 效率高 |
| **L2 命中率** | 62.1% | L2 Cache 命中率一般 |
| **共享内存吞吐** | 48.2 GB/s | 共享内存访问效率 |
| **Bank 冲突** | **0** | 8 列填充完全消除冲突 |
| **Sectors/Request** | 1.0 | 完美合并访问 |
| **L1/TEX Cache 吞吐** | 15.6 GB/s | 纹理/L1 缓存 |

### 计算指标

| 指标 | 值 | 说明 |
|------|-----|------|
| **FP64 运算数** | 13.4 MFLOP | 总双精度浮点运算 |
| **算术强度** | **157 FLOP/Byte** | FLOPS / 内存字节 |
| **Ridge Point** | 0.89 FLOP/Byte | Ada Lovelace 的 Ridge Point |
| **距离 Ridge 的倍数** | **176×** | 强计算绑定 |
| **FP64 利用率** | 0.048% | 远低于峰值 |
| **控制流指令占比** | 3.2% | 低分支复杂度 |
| **Warp 级同步指令** | 14 __syncthreads() | 每个迭代 14 个同步点 |

## compute_continuity_cost 详细指标

| 指标 | 值 |
|------|-----|
| Grid | (2, 1, 1) |
| Block | (256, 1, 1) |
| 时间 | 0.13 ms |
| 寄存器/线程 | 24 |
| 算术强度 | 4.8 FLOP/Byte |
| Occupancy | 87.5% |

## 关键发现

1. **Occupancy 仅 18.75%**，但未成为瓶颈 — 每个 Block 都有足够的 Warp 隐藏延迟
2. **Bank 冲突 = 0**，8 列填充策略完全有效
3. **算术强度 157 FLOP/Byte**，176× 超过 Ridge Point，强计算绑定
4. **延迟隐藏良好**，98 寄存器虽限制 Occupancy，但 20 个 Active Warp/SM 足以隐藏内存延迟
5. **FP64 利用率极低 (0.048%)** — 说明当前问题规模太小，无法充分利用 GPU 计算能力
