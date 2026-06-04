# CPU-GPU 通信与同步 — 文档索引

本目录详细分析 `assembly_rtfg_cuda` 功能包中 CPU 端与 GPU 端之间的数据传输、异步流执行、同步机制和固定内存使用。

| 文件 | 内容 | 核心代码引用 |
|------|------|-------------|
| [01_data_transfer.md](01_data_transfer.md) | H2D/D2H 数据传输详解（cudaMemcpyAsync 异步传输） | `cuda_memory.h:67-92`, `cuda_ik_solver.cu:355-375` |
| [02_cuda_stream.md](02_cuda_stream.md) | cudaStream_t 异步流并发执行 | `cuda_ik_solver.cu:139-145`, `cuda_ik_solver.cu:359-365` |
| [03_synchronization.md](03_synchronization.md) | 同步机制（cudaDeviceSynchronize / Event / 默认流阻塞） | `cuda_ik_solver.cu:367`, `cuda_memory.h:150-156` |
| [04_pinned_memory.md](04_pinned_memory.md) | 固定内存与零拷贝（cudaHostAlloc / cudaMemHostAlloc） | `cuda_ik_solver.cu:77-79` |
