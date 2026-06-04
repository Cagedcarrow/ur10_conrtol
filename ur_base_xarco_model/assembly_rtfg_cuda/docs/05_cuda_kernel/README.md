# CUDA 核函数深度分析 — 文档索引

本目录对 `assembly_rtfg_cuda` 功能包中的 CUDA 核函数进行逐行深度分析，涵盖执行模型、Warp 分工、正运动学、数值雅可比、加权海森矩阵、LDL^T 求解器、自适应阻尼和收敛检测等核心主题。

| 文件 | 内容 | 核心代码引用 |
|------|------|-------------|
| [01_kernel_execution_model.md](01_kernel_execution_model.md) | 核函数执行模型（Grid/Block/Warp/Thread 四层映射） | `cuda_kernels.cu:36-46`, `cuda_kernels.cu:345-346` |
| [02_ik_batch_solve_kernel.md](02_ik_batch_solve_kernel.md) | ik_batch_solve 逐行详解（完整结构） | `cuda_kernels.cu:36-288` |
| [03_warp_assignment.md](03_warp_assignment.md) | 4 个 Warp 的分工设计（FK/Jacobian/Hessian/LDL^T） | `cuda_kernels.cu:13-19` |
| [04_forward_kinematics_gpu.md](04_forward_kinematics_gpu.md) | GPU 正运动学：Rodrigues 公式 + URDF 约定 | `cuda_utilities.cuh:106-189` |
| [05_numerical_jacobian.md](05_numerical_jacobian.md) | 数值雅可比矩阵（6 列并行计算） | `cuda_kernels.cu:131-173` |
| [06_hessian_gradient_dls.md](06_hessian_gradient_dls.md) | 加权海森矩阵 H = J^T·W²·J + λI 与梯度 g = J^T·W²·e | `cuda_kernels.cu:195-223` |
| [07_ldlt_solve.md](07_ldlt_solve.md) | LDL^T 6×6 串行求解器 | `cuda_utilities.cuh:243-291` |
| [08_adaptive_damping.md](08_adaptive_damping.md) | 自适应阻尼策略（λ 动态调整 + 停滞提升） | `cuda_kernels.cu:176-192` |
| [09_convergence_stagnation.md](09_convergence_stagnation.md) | 收敛检测与停滞恢复（best-q 回退机制） | `cuda_kernels.cu:104-129` |
| [10_continuity_cost_kernel.md](10_continuity_cost_kernel.md) | compute_continuity_cost 连续性代价核函数 | `cuda_kernels.cu:290-333` |
