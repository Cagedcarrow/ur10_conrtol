# assembly_rtfg_cuda 文档

> CUDA 13.3 加速的 UR10 轨迹拟合 — GPU 批处理 IK 求解器

## 目录结构

```
docs/
├── README.md                                    ← 本文档 (导航)
│
├── 01_package_overview/                         # 功能包总体分析
│   ├── README.md
│   ├── 01_summary.md                            # 总览与核心指标
│   ├── 02_file_inventory.md                     # 16 个文件清册
│   ├── 03_build_system.md                       # CMake + package.xml
│   └── 04_node_architecture.md                  # ROS2 节点架构
│
├── 02_config_and_launch/                        # 配置与启动
│   ├── README.md
│   ├── 01_launch_file.md                        # 启动文件详解
│   ├── 02_runtime_config.md                     # 运行时配置
│   └── 03_solver_params.md                      # 求解器参数
│
├── 03_cuda_13_3_features/                       # CUDA 13.3 新特性
│   ├── README.md
│   ├── 01_cuda_13_3_overview.md                 # 新特性总览
│   ├── 02_tile_programming.md                   # C++ Tile 编程
│   ├── 03_cccl_mdspan.md                        # CCCL 3.3 mdspan
│   ├── 04_compileiq.md                          # CompileIQ AI 编译器
│   ├── 05_cuda_python.md                        # CUDA Python 1.0
│   └── 06_cuda_graphs.md                        # CUDA Graphs
│
├── 04_cuda_memory/                              # ★ CUDA 内存深度分析
│   ├── README.md
│   ├── 01_memory_hierarchy.md                   # GPU 内存层次
│   ├── 02_device_buffer.md                      # DeviceBuffer RAII
│   ├── 03_constant_memory.md                    # 常量内存广播
│   ├── 04_shared_memory.md                      # 共享内存 Bank 冲突
│   ├── 05_register_usage.md                     # 寄存器使用
│   └── 06_memory_lifecycle.md                   # 内存生命周期
│
├── 05_cuda_kernel/                              # ★ CUDA 核函数深度分析
│   ├── README.md
│   ├── 01_kernel_execution_model.md             # Grid/Block/Warp
│   ├── 02_ik_batch_solve_kernel.md              # Kernel 逐行详解
│   ├── 03_warp_assignment.md                    # 4 Warp 分工
│   ├── 04_forward_kinematics_gpu.md             # GPU FK
│   ├── 05_numerical_jacobian.md                 # 数值雅可比
│   ├── 06_hessian_gradient_dls.md               # Hessian + DLS
│   ├── 07_ldlt_solve.md                         # LDL^T 求解器
│   ├── 08_adaptive_damping.md                   # 自适应阻尼
│   ├── 09_convergence_stagnation.md             # 收敛检测
│   └── 10_continuity_cost_kernel.md             # 连续性代价
│
├── 06_cpu_gpu_communication/                    # ★ CPU-GPU 通讯
│   ├── README.md
│   ├── 01_data_transfer.md                      # H2D/D2H 传输
│   ├── 02_cuda_stream.md                        # cudaStream_t
│   ├── 03_synchronization.md                    # 同步机制
│   └── 04_pinned_memory.md                      # 固定内存
│
├── 07_performance/                              # 性能分析
│   ├── README.md
│   ├── 01_speedup_analysis.md                   # 843× 加速比
│   ├── 02_ncu_metrics.md                        # Nsight Compute 指标
│   ├── 03_roofline_model.md                     # Roofline 模型
│   ├── 04_three_versions_comparison.md          # 三版本对比
│   └── 05_amdal_law.md                          # Amdahl 定律
│
├── 08_call_flow_diagrams/                       # 调用流程图
│   ├── README.md
│   ├── 01_cpu_gpu_sequence.md                   # CPU-GPU 时序图
│   ├── 02_ik_solve_dataflow.md                  # IK 数据流图
│   └── 03_initialization_flow.md                # 初始化流程
│
└── 09_appendix/                                 # 附录
    ├── README.md
    ├── 01_source_file_reference.md              # 源码行号速查
    ├── 02_test_file_analysis.md                 # 测试分析
    └── 03_terminology.md                        # 术语表
```

## 文档地图

### 按角色阅读

| 角色 | 推荐阅读 |
|------|---------|
| **新开发者** | 01_package_overview/ → 09_appendix/03_terminology.md |
| **CUDA 开发者** | 04_cuda_memory/ → 05_cuda_kernel/ → 06_cpu_gpu_communication/ |
| **性能调优** | 07_performance/ → 04_cuda_memory/05_register_usage.md → 05_cuda_kernel/03_warp_assignment.md |
| **ROS2 集成** | 01_package_overview/04_node_architecture.md → 02_config_and_launch/ |
| **论文对照** | 07_performance/01_speedup_analysis.md → 07_performance/03_roofline_model.md |
| **系统理解** | 08_call_flow_diagrams/ → 全文 |

### 按难度排序

| 级别 | 目录 |
|------|------|
| ⭐ 入门 | 01_package_overview/, 09_appendix/ |
| ⭐⭐ 基础 | 02_config_and_launch/, 03_cuda_13_3_features/ |
| ⭐⭐⭐ 进阶 | 04_cuda_memory/, 06_cpu_gpu_communication/ |
| ⭐⭐⭐⭐ 深度 | 05_cuda_kernel/, 08_call_flow_diagrams/ |
| ⭐⭐⭐⭐⭐ 专家 | 07_performance/ |

## 核心指标速览

| 指标 | 值 |
|------|-----|
| GPU | RTX 4060 Laptop (Ada Lovelace, sm_89) |
| CUDA 版本 | 13.3 |
| 批处理 IK 时间 (273 目标) | **7.35 ms** |
| 加速比 (vs CPU KDL) | **843×** |
| 寄存器/线程 | 98 (零溢出) |
| 共享内存/Block | 1,676 B |
| Bank 冲突 | 0 |
| FP64 算术强度 | 157 FLOP/Byte |

## 论文引用

本文档对应论文 "cuda加速算法设计.pdf"，涵盖以下核心贡献：

1. **CUDA 批处理 IK 求解器**: 273 个独立 IK 任务并行化，843× 加速比
2. **Warp 级分工设计**: 4 Warp 分别负责 FK/Jacobian/Hessian/LDL^T
3. **内存层次优化**: 常量内存广播、共享内存 8 列填充、寄存器零溢出
4. **自适应 DLS 算法**: 加权阻尼 + 停滞恢复 + 发散检测

## 相关资源

- [assembly_rtfg_cpp 文档](../assembly_rtfg_cpp/docs/README.md) — CPU 版本参考
- [CUDA 13.3 官方文档](https://docs.nvidia.com/cuda/) — CUDA Runtime API
- [Nsight Compute 文档](https://docs.nvidia.com/nsight-compute/) — GPU Profiling
