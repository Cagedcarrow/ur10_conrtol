# 术语表

## CUDA 术语

| 术语 | 缩写 | 含义 |
|------|------|------|
| Kernel | - | 在 GPU 上执行的函数，用 `__global__` 声明 |
| Grid | - | 所有 Block 的集合 |
| Block | - | 一组同时执行的线程，通过 `__shared__` 共享内存 |
| Thread | - | 最小的执行单元 |
| Warp | - | 32 个线程为一组，是 GPU 调度基本单位 |
| SM | Streaming Multiprocessor | GPU 的计算单元，包含 CUDA Cores、共享内存、寄存器 |
| Occupancy | - | 每个 SM 上活跃 Warp 数与最大 Warp 数的比例 |
| Bank Conflict | - | 多个线程同时访问共享内存同一 Bank 导致的串行化 |
| Coalescing | - | 相邻线程访问相邻内存地址，合并为单次传输 |
| Arithmetic Intensity | AI | FLOP / Byte，计算的密集程度 |
| Ridge Point | - | Roofline 模型中的拐点，区分计算绑定和内存绑定 |
| Roofline | - | 性能分析模型，对比算术强度和峰值性能 |
| __constant__ | - | GPU 常量内存，对所有线程只读广播 |
| __shared__ | - | Block 内共享内存，低延迟可读写 |
| DeviceBuffer | - | RAII 封装 CUDA 设备内存的 C++ 模板类 |
| cudaMemcpyAsync | - | 异步内存拷贝，通过 Stream 实现重叠 |
| cudaStream_t | - | CUDA 流，GPU 操作队列 |
| cudaDeviceSynchronize | - | 阻塞 CPU 直到 GPU 所有操作完成 |
| LDL^T | - | 一种 Cholesky 分解变体，A = L·D·L^T |
| DLS | Damped Least Squares | IK 求解中用于处理奇异位姿的阻尼最小二乘法 |
| TFLOPS | - | 万亿次浮点运算/秒 |
| H2D | Host-to-Device | CPU 到 GPU 的数据传输 |
| D2H | Device-to-Host | GPU 到 CPU 的数据传输 |

## UR10 / URDF 术语

| 术语 | 含义 |
|------|------|
| URDF | Unified Robot Description Format，统一机器人描述格式 |
| SRDF | Semantic Robot Description Format，语义机器人描述格式 |
| FK | Forward Kinematics，正运动学 (关节角 → 末端位姿) |
| IK | Inverse Kinematics，逆运动学 (末端位姿 → 关节角) |
| Jacobian | J，关节速度到末端速度的映射矩阵 (6×6) |
| Hessian | H，在 IK 中指 J^T·W²·J + λI |
| TCP | Tool Center Point，工具中心点 |
| Rodrigues | 罗德里格斯旋转公式，任意轴旋转 |
| D-H | Denavit-Hartenberg 参数，传统机器人建模方法 |
| DOF | Degrees of Freedom，自由度 (UR10 为 6-DOF) |
| Revolute Joint | 旋转关节，UR10 所有关节类型 |
| Axis | URDF 中定义关节旋转轴的 `<axis>` 元素 |
| Origin | URDF 中定义关节原点位姿的 `<origin>` 元素 |
| Clearance | 间隙距离，机器人各部件到环境的最小距离 |
| DLS | Damped Least Squares，阻尼最小二乘 (IK 求解算法) |

## ROS2 术语

| 术语 | 含义 |
|------|------|
| Node | ROS2 节点，可通信的最小单元 |
| Service | 同步远程过程调用 (请求-响应) |
| Topic | 异步发布-订阅通信 |
| Action | 带有反馈的长时间运行任务 |
| Publisher | 话题发布者 |
| Subscriber | 话题订阅者 |
| Launch File | 启动文件，定义节点和参数 |
| Parameter | 节点可动态配置的参数 |
| TF | Transform，坐标系变换树 |
| RViz2 | ROS2 可视化工具 |
| MoveIt2 | 运动规划框架 |
| RSP | Robot State Publisher，机器人状态发布者 |
| JTC | Joint Trajectory Controller，关节轨迹控制器 |
| JSB | Joint State Broadcaster，关节状态广播器 |
| FCL | Flexible Collision Library，碰撞检测库 |

## 本功能包专有术语

| 术语 | 含义 |
|------|------|
| RTFG | Real-Time Trajectory Fitting，实时轨迹拟合 |
| CudaBatchIK | 基于 CUDA 的批处理 IK 求解器类 |
| RollingPlanner | 滚动规划器，分段求解连续轨迹 |
| ContinuousTrajectorySolver | 连续轨迹求解器 |
| BasinBox | 碰撞箱（环境中的碰撞体） |
| TargetPlan | 目标轨迹规划（含位姿序列和分段信息） |
| CandidateInfo | IK 求解结果候选（含误差/代价/间隙） |
| FitPreview | 轨迹拟合预览服务 |
| ExecuteCached | 执行缓存轨迹的服务 |
| Weighted DLS | 加权阻尼最小二乘法 |
| Adaptive Damping | 自适应阻尼策略 |
| Stagnation | 停滞/不收敛状态 |
| Continuity Cost | 连续性代价（衡量关节运动的平滑性） |
| Playback | 插值后的平滑回放轨迹 |
| 入泥点 | Entry Point，铲刀切入泥面的起始点 |
| 入泥角 | Theta，铲刀切入泥面的角度 |
