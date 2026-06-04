# CUDA Python 1.0 (cuda.core)

## 概述

CUDA 13.3 正式发布了 CUDA Python 1.0 (`cuda.core`)，提供 Python 原生的 GPU 编程 API，包括设备枚举、流管理、内存分配、Kernel Launch 等功能。

## 核心 API

```python
import cuda.core

# 设备枚举
devices = cuda.core.Device.all()
print(f"Found {len(devices)} devices")
dev = devices[0]  # RTX 4060
print(f"Using {dev.name}")

# 流创建
stream = dev.create_stream(non_blocking=True)

# 内存分配
d_data = dev.alloc(4096, kind="device")  # 4KB device memory
h_data = dev.alloc(4096, kind="pinned")  # 4KB pinned host memory

# Kernel Launch (自动推导网格/块大小)
kernel = dev.compile("""
__global__ void my_kernel(float* data) {
    int idx = threadIdx.x;
    data[idx] *= 2.0f;
}
""")
kernel.launch(data=d_data, grid=128, block=256, stream=stream)

# 同步
stream.sync()
```

## 多流流水线示例

```python
import cuda.core
import numpy as np

dev = cuda.core.Device(0)
stream_h2d = dev.create_stream()
stream_kernel = dev.create_stream()
stream_d2h = dev.create_stream()

# 流水线操作
for batch in range(10):
    with stream_h2d:
        h2d_event = stream_h2d.record_event()
        d_input.copy_from_host(h_input[batch], async=True)
    
    with stream_kernel:
        stream_kernel.wait_for(h2d_event)
        kernel_event = stream_kernel.record_event()
        kernel.launch(grid=273, block=128, args=[d_input, d_output])
    
    with stream_d2h:
        stream_d2h.wait_for(kernel_event)
        d_output.copy_to_host(h_output[batch], async=True)

dev.sync()
```

## 对本功能包的潜在价值

| 特性 | 应用场景 | 说明 |
|------|---------|------|
| 简化原型验证 | 快速测试 IK 算法变体 | Python 迭代更快 |
| 自动网格推导 | 避免手动计算 Grid/Block | 减少 Launch 配置错误 |
| 固定内存支持 | `kind="pinned"` 简化分配 | 当前使用 pageable memory |
| NVTX 集成 | 自动性能分析标记 | 当前无 NVTX 埋点 |

## 局限性

- 当前本功能包为 ROS2 C++ 节点，无 Python 接口
- CUDA Python 引入额外依赖
- Kernel 性能与纯 C++ 一致（底层使用同一 PTX/SASS）

## 当前使用情况

本功能包**未使用** CUDA Python。所有 CUDA 操作在 C++ 层通过 CUDA Runtime API 完成。
