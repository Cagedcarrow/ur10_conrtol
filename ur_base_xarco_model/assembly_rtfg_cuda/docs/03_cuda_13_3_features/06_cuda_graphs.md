# CUDA Graphs 与多流流水线

## 概述

CUDA Graphs 允许将一系列 GPU 操作（Kernel Launch、cudaMemcpy、事件等）捕获为静态图，然后一次性提交。CUDA 13.3 增强了对动态形状的支持。

## 原理

```mermaid
graph TB
    subgraph Traditional [传统方式: 每次重复提交]
        A1[cudaMemcpyAsync H2D] --> B1[Kernel Launch]
        B1 --> C1[cudaMemcpyAsync D2H]
        C1 --> D1[cudaDeviceSynchronize]
    end
    
    subgraph CUDA_Graph [CUDA Graph: 捕获→重放]
        CAP[捕获阶段] --> GRAPH[静态图]
        GRAPH -->|重放 N 次| PLAY[图重放]
    end
```

## 对本功能包的优化应用

### 当前问题

当前单次 IK 批处理求解流程：

```
H2D (47KB) → Kernel Launch → Continuity → D2H (19KB) → Sync
```

每次 `flush()` 调用都重复提交这些操作，存在以下开销：

| 操作 | 开销 | 说明 |
|------|------|------|
| Kernel Launch | ~20 μs | CUDA runtime 调度 |
| cudaMemcpyAsync | ~5 μs | 每次拷贝的 API 调用 |
| Stream 同步 | ~2 μs | Stream 操作 |

对于 273 目标的单次求解，这些开销占比很小 (< 1%)。但在**多次**调用场景（如轨迹拟合中调用数十次）中，这些开销会累积。

### CUDA Graph 优化方案

```mermaid
sequenceDiagram
    participant Host as CPU
    participant Graph as CUDA Graph
    
    Note over Host,Graph: 捕获阶段 (首次)
    Host->>Graph: cudaStreamBeginCapture(stream)
    Host->>Graph: cudaMemcpyAsync (H2D)
    Host->>Graph: launch_ik_batch_solve
    Host->>Graph: launch_continuity_cost
    Host->>Graph: cudaMemcpyAsync (D2H)
    Host->>Graph: cudaStreamEndCapture → cudaGraph_t
    Graph->>Graph: cudaGraphInstantiate → cudaGraphExec_t
    
    Note over Host,Graph: 重放阶段 (后续)
    loop 每帧
        Host->>Graph: cudaGraphLaunch(graphExec, stream)
        Host->>Host: cudaDeviceSynchronize
        Note over Host: 无需逐个提交 API 调用
    end
```

## 多流流水线设计

### 三级流水线 (H2D/Kernel/D2H)

```mermaid
sequenceDiagram
    participant Stream1 as Stream 1 (H2D)
    participant Stream2 as Stream 2 (Kernel)
    participant Stream3 as Stream 3 (D2H)
    
    Note over Stream1,Stream3: 帧 0
    Stream1->>Stream1: H2D: targets[0]
    Note over Stream2: Stream1→Stream2 事件依赖
    Stream2->>Stream2: Kernel: solve targets[0]
    Note over Stream3: Stream2→Stream3 事件依赖
    Stream3->>Stream3: D2H: results[0]
    
    Note over Stream1,Stream3: 帧 1 (重叠)
    Stream1->>Stream1: H2D: targets[1]
    Stream2->>Stream2: Kernel: solve targets[1]
    Stream3->>Stream3: D2H: results[1]
    
    Note over Stream1,Stream3: 帧 2 (全流水)
    Stream1->>Stream1: H2D: targets[2]
    Stream2->>Stream2: Kernel: solve targets[2]
    Stream3->>Stream3: D2H: results[2]
```

### 预期收益

| 优化 | 当前 | 优化后 | 收益 |
|------|------|--------|------|
| CUDA Graph | 无 | 图重放 | 减少 ~20% Launch 开销 |
| 多流流水线 | 单流串行 | 三级流水线 | 吞吐量提升 ~3× |
| 固定内存 | 分页内存 | 固定内存 | H2D 提速 ~2× |

## 实现注意事项

1. **数据更新**: 每次重放前需更新 H2D 数据（使用可写内存或 `cudaGraphUpdate`）
2. **同步**: 多流需要 `cudaEvent_t` 事件同步
3. **内存**: 需预分配所有 DeviceBuffer
4. **错误处理**: 图捕获失败需要回退到传统模式

## 当前使用情况

本功能包**未使用** CUDA Graphs 和多流流水线。当前实现：
- 单 Stream (non-blocking)
- 每次 `flush()` 单独提交所有操作
- `cudaDeviceSynchronize()` 阻塞等待
