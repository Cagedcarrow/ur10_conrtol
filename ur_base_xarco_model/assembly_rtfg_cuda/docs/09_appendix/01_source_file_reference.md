# 源码文件快速参考

## CUDA 源文件

| 文件 | 路径 | 行数 | 核心功能 |
|------|------|------|---------|
| `cuda_kernels.cu` | `src/cuda/cuda_kernels.cu` | 372 | `ik_batch_solve` + `compute_continuity_cost` 核函数 |
| `cuda_ik_solver.cu` | `src/cuda/cuda_ik_solver.cu` | 465 | `CudaBatchIK` 类实现 (initialize/enqueue/flush) |
| `cuda_utilities.cuh` | `src/cuda/cuda_utilities.cuh` | 295 | 设备端工具 (FK, LDL^T, pose_error, 常量内存) |
| `cuda_memory.cu` | `src/cuda/cuda_memory.cu` | 19 | DeviceBuffer 模板实例化 |

### cuda_kernels.cu 关键行号索引

| 行号 | 功能 | 说明 |
|------|------|------|
| 36-47 | `ik_batch_solve` 函数签名 | Kernel 参数列表 |
| 50-66 | 共享内存变量声明 | 13 个 `__shared__` 变量 |
| 67-75 | Phase 1: 加载 seed + target | 合并访问全局内存 |
| 77-84 | 初始化收敛跟踪 | s_converged, s_stagnation |
| 87 | 主迭代循环 | `for (iter < max_iter && !s_converged)` |
| 91-93 | FK 计算 (Warp 0) | `forward_kinematics(s_q, s_T)` |
| 98-101 | 位姿误差计算 | `pose_error(s_T, s_T_tgt, s_err)` |
| 104-118 | 收敛检测 + 最佳跟踪 | 双判据收敛 + stagnation 计数 |
| 122-129 | 发散/振荡检测 | stagnation > 25 时退出 |
| 133-173 | 数值雅可比 (Warp 1) | 6 列并行，中心差分 |
| 177-192 | 自适应阻尼 | 远近程双分支 + 停滞提升 |
| 197-211 | Hessian H (Warp 2) | J^T·W²·J + λI |
| 214-223 | 梯度 g | J^T·W²·e |
| 227-236 | LDL^T 求解 (Warp 3) | 串行 6×6 求解 |
| 240-250 | 步长夹紧 | 限制 0.25 rad |
| 253-259 | 关节限位 + 步进 | clamp(q + dq, lo, hi) |
| 262-269 | 分支对准 | J5/J6 π-wrap 防止 |
| 273-287 | Phase 3: 结果写出 | 合并存储到全局内存 |
| 293-333 | `compute_continuity_cost` | 连续性代价核函数 |
| 339-367 | Kernel 启动封装 | `launch_ik_batch_solve`, `launch_continuity_cost` |

### cuda_ik_solver.cu 关键行号索引

| 行号 | 功能 | 说明 |
|------|------|------|
| 40-131 | URDF 原点矩阵 (硬编码) | 6 段 + 工具偏移 |
| 139-145 | 构造函数 | cudaStreamCreate |
| 147-151 | 析构函数 | cudaStreamDestroy |
| 160-286 | `initialize()` | 常量内存上传 (7 次 cudaMemcpyToSymbol) |
| 268-281 | cudaMemcpyToSymbol 调用 | 所有 7 个常量数组 |
| 284 | 同步 | cudaDeviceSynchronize |
| 292-308 | `ensureCapacity()` | DeviceBuffer 大小保障 |
| 314-323 | `enqueue()` | 添加待处理目标 |
| 329-397 | `flush()` | 完整批处理流程 |
| 355-356 | H2D 传输 | d_targets_->toDevice(), d_seeds_->toDevice() |
| 359-366 | Kernel Launch | `launch_ik_batch_solve(..., stream_)` |
| 367 | 同步 | cudaDeviceSynchronize |
| 370-375 | D2H 回读 | d_results_->toHost() 等 |

### cuda_utilities.cuh 关键行号索引

| 行号 | 功能 | 说明 |
|------|------|------|
| 15-34 | CUDA_CHECK 宏 | 错误检查 |
| 44-64 | 设备端数学工具 | wrap, clamp, norm6 |
| 69-88 | 常量内存声明 | EXTERN_CONSTANT 模式 |
| 80-86 | 7 个常量数组声明 | `c_segment_origins` ~ `c_lambda_params` |
| 106-135 | Rodrigues 旋转矩阵 | `build_rotation_matrix` |
| 140-151 | 4×4 矩阵乘 | `mat44_mul` |
| 165-189 | 正运动学 | `forward_kinematics` (URDF 约定) |
| 196-236 | 位姿误差 | `pose_error` (6-DOF) |
| 243-291 | LDL^T 6×6 求解 | `ldlt_solve_6x6` |

## C++ 源文件

| 文件 | 路径 | 行数 | 核心功能 |
|------|------|------|---------|
| `rtfg_solver_node.cpp` | `src/rtfg_solver_node.cpp` | 778 | ROS2 节点 (服务/话题/动作) |
| `ik_solver.cpp` | `src/ik_solver.cpp` | - | IK 求解器工厂 (从 cpp 包复制) |
| `trajectory_solver.cpp` | `src/trajectory_solver.cpp` | - | 轨迹求解器 (从 cpp 包复制) |

## 头文件

| 文件 | 路径 | 核心类 |
|------|------|--------|
| `cuda_ik_solver.h` | `include/assembly_rtfg_cuda/cuda_ik_solver.h` | `class CudaBatchIK` |
| `cuda_kernels.h` | `include/assembly_rtfg_cuda/cuda_kernels.h` | `launch_ik_batch_solve()` |
| `cuda_memory.h` | `include/assembly_rtfg_cuda/cuda_memory.h` | `DeviceBuffer<T>`, `ConstantMemory<T>` |

## ROS2 相关

| 文件 | 用途 |
|------|------|
| `launch/rtfg_sim.launch.py` | 启动文件 (208 行) |
| `test/test_cuda_kernel.cu` | GPU Kernel 测试 |

## 配置

| 文件 | 用途 |
|------|------|
| `CMakeLists.txt` | 构建配置 |
| `package.xml` | 包描述 |
| `config/environment_runtime_config.yaml` | 运行时配置 |
