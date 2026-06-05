# IK 求解数据流图

## 完整数据流图

```mermaid
graph TB
    subgraph CPU [CPU 端]
        HOST_TARGETS[h_targets: vector<double> N×16<br/>行优先 4×4 矩阵]
        HOST_SEEDS[h_seeds: vector<double> N×6<br/>初始关节角]
        HOST_RESULTS[h_results: vector<double> N×6<br/>求解结果 q]
        HOST_ERRORS[h_errors: vector<double> N×2<br/>pos_err, rot_err]
        HOST_ITERS[h_iters: vector<double> N<br/>迭代次数]
        
        CFG[SolverConfig<br/>max_iter, tol 等]
        
        HOST_TARGETS -->|AoS→SoA| D_TARGETS
        HOST_SEEDS -->|打包| D_SEEDS
        CFG -->|参数值| LAUNCH
    end
    
    subgraph Transfer1 [H2D 传输 ~47 KB]
        D_TARGETS[cuda::DeviceBuffer<br/>d_targets_ [N×16]]
        D_SEEDS[cuda::DeviceBuffer<br/>d_seeds_ [N×6]]
        D_RESULTS[cuda::DeviceBuffer<br/>d_results_ [N×6]]
        D_ERRORS[cuda::DeviceBuffer<br/>d_errors_ [N×2]]
        D_ITERS[cuda::DeviceBuffer<br/>d_iterations_ [N]]
    end
    
    subgraph GPU [GPU 端]
        subgraph Constant [__constant__ 1,384 B]
            C_SEG[c_segment_origins 96×8=768B]
            C_AX[c_segment_axes 18×8=144B]
            C_QI[c_q_index 6×4=24B]
            C_TCP[c_T_wrist3_to_tcp 16×8=128B]
            C_LIM[c_joint_limits 12×8=96B]
            C_W[c_weight_schedule 24×8=192B]
            C_L[c_lambda_params 4×8=32B]
        end
        
        subgraph Kernel [ik_batch_solve Kernel]
            SHARED[Shared Memory 1,616 B<br/>s_q[8] s_T[16] s_T_tgt[16]<br/>s_T_tcp[16] s_T_tcp_tgt[16]<br/>s_J[48] s_H[48] s_err[6]<br/>s_g[6] s_dq[6] s_q_ref[6]<br/>s_q_best[6] s_lambda ...]
            
            SHARED -->|LDL^T| REG[Register 96 regs/thread<br/>~12,288 regs/block]
            REG -->|结果| SHARED
        end
        
        D_TARGETS -->|coalesced load| SHARED
        D_SEEDS -->|coalesced load| SHARED
        Constant -->|broadcast| Kernel
        
        SHARED -->|coalesced store| D_RESULTS
        SHARED -->|coalesced store| D_ERRORS
        SHARED -->|coalesced store| D_ITERS
    end
    
    subgraph Transfer2 [D2H 传输 ~19 KB]
        D_RESULTS -->|cudaMemcpyAsync| HOST_RESULTS
        D_ERRORS -->|cudaMemcpyAsync| HOST_ERRORS
        D_ITERS -->|cudaMemcpyAsync| HOST_ITERS
    end
    
    subgraph Post [CPU 后处理]
        HOST_RESULTS -->|构建| CAND[CandidateInfo<br/>q, pos_err, rot_err]
        HOST_ERRORS --> CAND
        HOST_ITERS --> CAND
    end
```

## 内存布局细节

### 全局内存 SoA 布局 (合并访问)

```
d_targets_ (N×16):
  ┌─ Target[0] ─┬─ Target[1] ─┬─ ... ─┬─ Target[272] ─┐
  │T₀₀..T₀₁₅   │T₁₀..T₁₁₅   │       │T₂₇₂₀..T₂₇₂₁₅  │
  └─────────────┴─────────────┴───────┴────────────────┘
  线程 0 访问: d_targets[0]      → 连续加载
  线程 1 访问: d_targets[16]     → 相邻地址
  线程 2 访问: d_targets[32]     → 合并访问

d_results_ (N×6):
  ┌─ q₀[0..5] ─┬─ q₁[0..5] ─┬─ ... ─┬─ q₂₇₂[0..5] ─┐
  线程 0 访问: d_results[0..5]   → 连续存储
```

### 共享内存 8 列填充 (Bank Conflict Free)

```
s_J[6×8] = 6 rows × 8 columns = 48 doubles:
  ┌──────────┬──────────┬──────┬──────────┬──────────┐
  │ col 0    │ col 1    │ ...  │ col 5    │ col 6(pad)│ col 7(pad)│
  │ J[0*8+0] │ J[0*8+1] │ ...  │ J[0*8+5] │ unused    │ unused    │
  ├──────────┼──────────┼──────┼──────────┼──────────┤
  │ J[1*8+0] │ J[1*8+1] │ ...  │ J[1*8+5] │ unused    │ unused    │
  ├──────────┼──────────┼──────┼──────────┼──────────┤
  │ ...      │ ...      │ ...  │ ...      │ ...      │
  ├──────────┼──────────┼──────┼──────────┼──────────┤
  │ J[5*8+0] │ J[5*8+1] │ ...  │ J[5*8+5] │ unused    │ unused    │
  └──────────┴──────────┴──────┴──────────┴──────────┘
  每行 64 bytes = 16 banks → 10 字节/银行 → 0 冲突
```

## 迭代数据流

```mermaid
graph LR
    subgraph Iteration [单次迭代数据流]
        direction LR
        SEED[s_q[6]] --> FK[FK<br/>Warp 0]
        FK --> ERR[Pose Error<br/>lane 0]
        ERR --> CONV{Converged?}
        CONV -->|No| JAC[Jacobian<br/>Warp 1]
        CONV -->|Yes| WRITE[Write Results]
        JAC --> DAMP[Adaptive Damping<br/>lane 0]
        DAMP --> HESS[Hessian J^T·W²·J<br/>Warp 2]
        HESS --> GRAD[Gradient g<br/>Warp 0]
        GRAD --> LDLT[LDL^T Solve<br/>Warp 3]
        LDLT --> CLAMP[Step Clamping<br/>lane 0]
        CLAMP --> UPDATE[s_q ← s_q + dq<br/>Warp 0-3]
        UPDATE --> FK
    end
```

## 数据大小统计

| 数据结构 | 元素 | 字节 | 存储位置 |
|---------|------|------|---------|
| `d_targets_` | N × 16 | N × 128 | Global Memory |
| `d_seeds_` | N × 6 | N × 48 | Global Memory |
| `d_results_` | N × 6 | N × 48 | Global Memory |
| `d_errors_` | N × 2 | N × 16 | Global Memory |
| `d_iterations_` | N | N × 8 | Global Memory |
| `s_q[8]` | 8 (6+2 padding) | 64 | Shared Memory |
| `s_T[16]` | 16 | 128 | Shared Memory |
| `s_J[48]` | 48 (6×8 padding) | 384 | Shared Memory |
| `s_H[48]` | 48 (6×8 padding) | 384 | Shared Memory |
| Constant Memory | 7 arrays | 1,384 | __constant__ |
| Registers | 96/thread | 12,288/block | Register File |

> N = 273 时: H2D = 47 KB, D2H = 19 KB, 共享内存/Block = 1,616 B
