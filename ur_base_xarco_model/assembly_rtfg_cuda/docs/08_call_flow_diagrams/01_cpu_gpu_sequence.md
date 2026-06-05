# CPU-GPU 完整时序图

## 单次 IK 批处理求解完整时序

```mermaid
sequenceDiagram
    participant ROSNode as rtfg_solver_node (CPU)
    participant CudaIK as CudaBatchIK (CPU Wrapper)
    participant HostBuf as Host Buffer (std::vector)
    participant GPUGlobal as GPU Global Memory
    participant GPUConst as GPU __constant__
    participant GPUShared as GPU Shared Memory
    participant GPUReg as GPU Registers

    ROSNode->>CudaIK: solveBatch(targets, seeds)
    
    Note over CudaIK: ===== Phase 1: 初始化 (首次调用) =====
    
    CudaIK->>CudaIK: initialize(robot, cfg)
    CudaIK->>GPUConst: cudaMemcpyToSymbol(c_segment_origins, 96×double)
    CudaIK->>GPUConst: cudaMemcpyToSymbol(c_segment_axes, 18×double)
    CudaIK->>GPUConst: cudaMemcpyToSymbol(c_q_index, 6×int)
    CudaIK->>GPUConst: cudaMemcpyToSymbol(c_T_wrist3_to_tcp, 16×double)
    CudaIK->>GPUConst: cudaMemcpyToSymbol(c_joint_limits, 12×double)
    CudaIK->>GPUConst: cudaMemcpyToSymbol(c_weight_schedule, 24×double)
    CudaIK->>GPUConst: cudaMemcpyToSymbol(c_lambda_params, 4×double)
    Note over GPUConst: 常量内存总计: 1,384 bytes
    
    CudaIK->>CudaIK: cudaStreamCreate(&stream_)
    
    Note over CudaIK: ===== Phase 2: 数据打包 =====
    
    CudaIK->>HostBuf: h_targets_.resize(N × 16)
    CudaIK->>HostBuf: h_seeds_.resize(N × 6)
    Note over HostBuf: AoS(Mat4) → SoA(逐元素)
    
    Note over CudaIK: ===== Phase 3: H2D 传输 =====
    
    CudaIK->>GPUGlobal: d_targets_->toDevice(h_targets_.data())
    Note over GPUGlobal: cudaMemcpyAsync(host→dev, ~47 KB, stream)
    
    CudaIK->>GPUGlobal: d_seeds_->toDevice(h_seeds_.data())
    
    Note over CudaIK: ===== Phase 4: Kernel Launch =====
    
    CudaIK->>GPUGlobal: launch_ik_batch_solve(...)
    
    Note over GPUGlobal: Grid(273,1) Block(128,1)
    
    GPUGlobal->>GPUShared: coalesced load d_targets → s_T_tgt[16]
    GPUGlobal->>GPUShared: coalesced load d_seeds → s_q[8]
    GPUShared->>GPUShared: s_q_ref[6] ← s_q[6] (备份种子)

    Note over GPUShared: ===== 迭代循环 (6.7 avg) =====
    
    loop for iter < max_iter AND !s_converged
        GPUShared->>GPUReg: s_q → registers (FK)
        GPUReg->>GPUReg: forward_kinematics() 
        Note over GPUReg: Rodrigues × 6 seg<br/>Warp 0 串行
        GPUReg->>GPUShared: s_T[16] ← FK result
        
        GPUShared->>GPUReg: s_T, s_T_tgt → pose_error()
        GPUReg->>GPUShared: s_err[6] ← pos/rot error
        
        GPUShared->>GPUReg: s_q ± eps → J 列计算
        Note over GPUReg: 线程 0-5 各计算一列<br/>Warp 1 每列 2×FK
        GPUReg->>GPUShared: s_J[48] ← Jacobian
        
        GPUShared->>GPUReg: s_J → H = J^T·W²·J
        Note over GPUReg: 线程 0-35 各算一个元素<br/>Warp 2 并行
        GPUReg->>GPUShared: s_H[48] ← Hessian
        
        GPUShared->>GPUReg: s_H, s_g → ldlt_solve_6x6()
        Note over GPUReg: Warp 3 串行 ~0.1μs
        GPUReg->>GPUShared: s_dq[6] ← step
        
        GPUReg->>GPUShared: s_q clamped + aligned
    end
    
    GPUShared->>GPUGlobal: coalesced store → d_results[6N]
    GPUShared->>GPUGlobal: store → d_errors[2N]
    GPUShared->>GPUGlobal: store → d_iterations[N]
    
    Note over CudaIK: ===== Phase 5: 连续性代价 =====
    
    CudaIK->>GPUGlobal: launch_compute_continuity_cost(...)
    Note over GPUGlobal: Grid(2,1) Block(256,1)
    
    Note over CudaIK: ===== Phase 6: D2H + 同步 =====
    
    CudaIK->>GPUGlobal: d_results_->toHost(h_results.data())
    Note over GPUGlobal: cudaMemcpyAsync(dev→host, ~19 KB, stream)
    
    CudaIK->>CudaIK: cudaDeviceSynchronize()
    
    Note over CudaIK: ===== Phase 7: 结果解析 =====
    
    CudaIK->>CudaIK: 遍历 results → CandidateInfo
    CudaIK->>ROSNode: 返回 vector<CandidateInfo>
    
    ROSNode->>ROSNode: RollingPlanner::solve()
    Note over ROSNode: 对每个 Candidate 计算连续性代价<br/>选择最优路径
```

## 同步点标识

```mermaid
sequenceDiagram
    participant CPU as CPU Thread
    participant GPU as GPU Stream
    
    CPU->>GPU: cudaMemcpyAsync(H2D, 47KB)
    Note over GPU: ▌异步，CPU 立即返回
    
    CPU->>GPU: launch_ik_batch_solve<<<273,128>>>
    Note over GPU: ▌异步，CPU 立即返回
    
    CPU->>GPU: launch_continuity<<<2,256>>>
    Note over GPU: ▌异步，CPU 立即返回
    
    CPU->>GPU: cudaMemcpyAsync(D2H, 19KB)
    Note over GPU: ▌异步，CPU 立即返回
    
    Note over CPU: CPU 可在此做其他工作<br/>(当前版本无重叠)
    
    CPU->>GPU: cudaDeviceSynchronize()
    Note over CPU,GPU: ██ 阻塞同步点 ██
    Note over GPU: 所有流操作完成
    Note over CPU: 数据可用
    
    Note over CPU,GPU: ────── 时序关键路径 ──────
    Note over CPU,GPU: H2D → Kernel → Continuity → D2H → Sync
    Note over CPU,GPU: 总 GPU 耗时: ~7.85 ms
```

## 关键时序指标

| 阶段 | 时间 (μs) | 操作 |
|------|-----------|------|
| H2D 传输 | 30 | cudaMemcpyAsync (47 KB) |
| Kernel Launch | 20 | CUDA runtime 开销 |
| ik_batch_solve | 6,434 | 273 Block × 6.7 iter |
| continuity_cost | 130 | 273 目标的连续性代价 |
| D2H 回读 | 20 | cudaMemcpyAsync (19 KB) |
| DeviceSync | 300 | 阻塞等待 |
| **总计** | **~7,850** | **< 8 ms** |
