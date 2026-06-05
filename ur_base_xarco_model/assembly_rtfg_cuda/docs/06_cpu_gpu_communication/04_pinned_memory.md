# 固定内存与零拷贝

## 概述

固定内存 (Pinned Memory, 也称为 Page-Locked Memory) 是 CUDA 中 CPU 与 GPU 之间高效数据传输的关键技术。本功能包通过 C++ 标准容器的自动内存管理间接使用固定内存，但未显式使用 `cudaHostAlloc`。

## 为什么需要固定内存

### 页式内存 (Pageable) 的问题

Linux 的虚拟内存系统以 4 KB 页为单位管理内存。CPU 的物理页可以随时被换出到磁盘 (swap)。当 `cudaMemcpy` 从页式内存传输数据时：

1. GPU 无法直接访问 CPU 的页式内存
2. 驱动程序必须在后台将数据从页式内存复制到**临时固定缓冲区** (pinned bounce buffer)
3. 这个隐式复制增加了延迟并消耗了 PCIe 带宽

```
页式内存 → [CPU 端复制] → 临时固定缓冲区 → [cudaMemcpy] → GPU 内存
                                    ↑
                        隐式复制（浪费带宽）
```

### 固定内存 (Pinned) 的优势

```
固定内存 → [cudaMemcpyAsync] → GPU 内存
         ↑
   直接 DMA, 无中间复制
```

**差异**: 固定内存传输速度可达到页式内存的 **2-3 倍**。

## 本包中的实际使用

### 当前实现

`CudaBatchIK` 类使用 `std::vector<double>` 作为主机端缓冲区：

```cpp
// cuda_ik_solver.h:77-78
std::vector<double> h_targets_;    // 目标位姿缓冲区
std::vector<double> h_seeds_;      // 种子关节角缓冲区
```

`std::vector` 默认使用页式内存（通过 `new` 分配器）。

### 实际影响

尽管使用页式内存，本包的传输效率仍然很高，因为：

1. **传输量很小**: H2D ~47 KB, D2H ~19 KB
2. **使用 `cudaMemcpyAsync`**: 异步传输可以与内核执行重叠
3. **传输延迟占比极小**: 在 6.434 ms 的总 GPU 时间中 < 0.3%

### 优化建议

如果未来需要进一步优化传输性能，可以使用固定内存：

```cpp
// 方案 1: cudaHostAlloc
double* h_pinned;
cudaHostAlloc(&h_pinned, N * 16 * sizeof(double), cudaHostAllocDefault);

// 方案 2: cudaHostRegister (将现有页式内存注册为固定内存)
cudaHostRegister(h_targets_.data(), h_targets_.size() * sizeof(double),
                 cudaHostRegisterDefault);

// 方案 3: 使用提供固定内存分配器的自定义容器
```

## CUDA 中固定内存的创建方式

### cudaHostAlloc — 分配固定内存

```cpp
double* h_pinned;
cudaError_t err = cudaHostAlloc(
    &h_pinned, 
    count * sizeof(double),
    cudaHostAllocDefault  // 其他选项: cudaHostAllocPortable, cudaHostAllocMapped, cudaHostAllocWriteCombined
);
// 使用后必须释放
cudaFreeHost(h_pinned);
```

**标志选项**:

| 标志 | 效果 |
|------|------|
| `cudaHostAllocDefault` | 默认固定内存 |
| `cudaHostAllocPortable` | 固定内存对所有 CUDA 上下文可用 |
| `cudaHostAllocMapped` | **零拷贝**: 创建映射到 GPU 地址空间的主机内存 |
| `cudaHostAllocWriteCombined` | 写合并优化 (主机写入快, GPU 读取慢) |

### cudaHostRegister — 将现有内存注册为固定

```cpp
std::vector<double> h_data(N * 16);
cudaHostRegister(h_data.data(), h_data.size() * sizeof(double),
                 cudaHostRegisterDefault);
// ... 使用 h_data 进行异步传输 ...
cudaHostUnregister(h_data.data());
```

**优点**: 不需要重写现有代码
**缺点**: 注册/注销有开销 (`cudaHostRegister` 可能耗时数毫秒)

## 零拷贝内存 (Zero-Copy)

如果使用 `cudaHostAllocMapped`，GPU 可以直接访问主机内存，无需显式的 `cudaMemcpy`：

```cpp
double* h_mapped;
cudaHostAlloc(&h_mapped, count * sizeof(double), cudaHostAllocMapped);

double* d_mapped;  // GPU 端指针
cudaHostGetDevicePointer(&d_mapped, h_mapped, 0);

// GPU Kernel 可以直接读写 d_mapped → 实际访问的是主机内存
```

**不适用于本包**的原因：
- 零拷贝适合**小规模、随机访问**的数据
- 本包的 DLS 迭代需要高频读写中间变量（寄存器/共享内存），不适合零拷贝
- 零拷贝的主机内存访问延迟高达 ~5 μs (PCIe)

## 固定内存警告

| 问题 | 说明 |
|------|------|
| 系统内存压力 | 固定内存不能被换出，过多固定内存降低系统性能 |
| 分配延迟 | `cudaHostAlloc` 比 `malloc` 慢 |
| 过度使用 | 分配所有内存为固定会显著降低系统整体性能 |
| 释放检查 | 必须使用 `cudaFreeHost` 而非 `free` |

## 最佳实践建议

对于本功能包，当前数据传输量很小 (< 50 KB)，页式内存已经足够。如果未来：
- 批处理量大幅增加 (> 1000 目标)
- 需要实时流水线 (< 1 ms 帧处理时间)
- 或引入多流重叠

则建议将 `h_targets_` 和 `h_seeds_` 改为固定内存。

## 相关代码行号

| 功能 | 文件 | 行号 |
|------|------|------|
| 主机缓冲区声明 | `cuda_ik_solver.h` | 77-79 |
| 主机缓冲区 resize | `cuda_ik_solver.cu` | 339-340 |
| 异步传输调用 | `cuda_ik_solver.cu` | 355-356, 373-375 |
| cudaMemcpyAsync 实现 | `cuda_memory.h` | 67-92 |
