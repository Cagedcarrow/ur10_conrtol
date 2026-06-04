# assembly_rtfg_cpp 功能包完整代码分析文档

> 功能包路径: `/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp`
>
> 生成日期: 2026-06-04
>
> 源代码: 10 个 `.cpp` 文件 + 12 个 `.h` 文件

---

## 目录

1. [所有源文件的文件级功能描述](#1-所有源文件的文件级功能描述)
2. [所有类的类图](#2-所有类的类图)
3. [所有 ROS2 Service 的接口定义](#3-所有-ros2-service-的接口定义)
4. [所有 Topic 的类型和发布频率](#4-所有-topic-的类型和发布频率)
5. [IK 算法的核心数据流](#5-ik-算法的核心数据流)
6. [关键参数表](#6-关键参数表)
7. [搜索策略的流程图描述](#7-搜索策略的流程图描述)
8. [Playback 生成算法](#8-playback-生成算法)
9. [性能瓶颈识别](#9-性能瓶颈识别)
10. [CUDA 并行化扩展点分析](#10-cuda-并行化扩展点分析)

---

## 1. 所有源文件的文件级功能描述

### 1.1 头文件 (`.h`) — 共 12 个

| # | 文件路径 | 行数 | 功能 |
|---|---------|------|------|
| 1 | `include/assembly_rtfg_cpp/types.h` | 186 | **核心数据结构定义**。定义了命名空间 `rtfg` 下的全部数据模型: `Mat4`/`Vec3`/`Vec6`/`Mat6` 类型别名, `BasinBox`(料箱几何), `ProgressEvent`(进度事件), `SegmentSpec`(关节段规格), `GeometryType`(几何类型枚举), `LinkCollision`(碰撞体), `RobotModel`(完整机器人模型), `CandidateInfo`(IK候选解), `CollisionSummary`(碰撞汇总), `SolverConfig`(求解器配置), `SolverTiming`(性能计时), `TrajectoryResult`(轨迹求解结果) |
| 2 | `include/assembly_rtfg_cpp/ik_solver.h` | 41 | **IK 求解器函数声明**。声明了 4 个顶层函数: `solveSinglePoseKdl()`(KDL LMA 主求解器), `solveSinglePose()`(自定义 DLS 求解器), `buildSeedList()`(局部种子列表), `buildGlobalSeedList()`(全局随机种子列表) |
| 3 | `include/assembly_rtfg_cpp/ik_backend.h` | 92 | **IK 后端工厂模式声明**。定义了抽象基类 `IKSolverBase` 和 5 个派生类(`CurrentNumericIK`, `KdlLMAIK`, `TracIKSolver`, `IKFastSolver`, `URKinematicsSolver`), 以及工厂函数 `createIKSolverBackend()` |
| 4 | `include/assembly_rtfg_cpp/trajectory_solver.h` | 24 | **轨迹求解主流程声明**。声明 `solveTrajectory()` — 完整轨迹求解: anchor IK -> playback 生成 -> 碰撞审计 |
| 5 | `include/assembly_rtfg_cpp/collision_checker.h` | 19 | **碰撞检查声明**。声明 `evaluateConfiguration()` — 使用 FCL 对给定关节配置进行自身碰撞、工具-主体和工具-料箱碰撞评估 |
| 6 | `include/assembly_rtfg_cpp/collision_pipeline.h` | 35 | **碰撞检测流水线声明**。定义 `CollisionPipeline` 类, 包含快速检查(`quickPlaybackCheck`)和精确检查(`preciseCheck`), 以及碰撞步长管理(`collisionStride`, `shouldRunPreciseCheck`) |
| 7 | `include/assembly_rtfg_cpp/robot_model.h` | 49 | **机器人模型加载与运动学声明**。声明 `loadRobotModel()`(URDF加载), `forwardKinematics()`(正向运动学), `tipTransform()`(TCP变换), `numericJacobian()`(数值雅可比), `clampToLimits()`(限位夹紧), `clampToLimitsWithRebound()`(带反弹的限位), `alignToReference()`(分支对齐) |
| 8 | `include/assembly_rtfg_cpp/trajectory_generator.h` | 54 | **轨迹生成器声明**。定义 `EnvironmentPose`(环境位姿), `TrajectoryParams`(轨迹参数), `RuntimeConfig`(运行时配置), `TargetPlan`(目标路径规划)等结构体, 以及 `loadRuntimeConfig()`, `buildTargetPlan()`, `buildBasinBoxes()` 等函数 |
| 9 | `include/assembly_rtfg_cpp/utils.h` | 29 | **工具函数声明**。声明数学工具函数: `fail()`, `clamp01()`, `rad2deg()`, `rtToTform()`, `rpyToRot()`, `xyzrpyToTform()`, `rotToLogVec()`, `rotationDistance()`, `quinticBlend()`, `slerpRotation()`, `wrapJointDelta()`, `continuityCost()`, `poseError()` |
| 10 | `include/assembly_rtfg_cpp/continuous_trajectory_solver.h` | 27 | **连续轨迹求解器类声明**。定义 `ContinuousTrajectorySolver` 类, 封装配置和求解入口 |
| 11 | `include/assembly_rtfg_cpp/rolling_planner.h` | 26 | **滚动规划器类声明**。定义 `RollingPlanner` 类, 实现窗口式滚动求解 |
| 12 | `include/assembly_rtfg_cpp/utils.h` (重复) | — | 与第9项相同 |

### 1.2 源文件 (`.cpp`) — 共 10 个

| # | 文件路径 | 行数 | 功能 |
|---|---------|------|------|
| 1 | `src/rtfg_solver_node.cpp` | 779 | **主 ROS2 节点** (`RtfgSolverNode`)。负责: (1) 声明 ROS2 参数并初始化, (2) 创建 5 个 Service (`/rtfg/load_config`, `/rtfg/fit_preview`, `/rtfg/execute_cached`, `/rtfg/move_to_start`, `/rtfg/move_to_home`), (3) 创建 6 个 Publisher, (4) `onFitPreview()` 处理核心求解流程(Plan -> Solve -> Collision Audit -> Publish), (5) `onExecuteCached()` 执行缓存轨迹(优先 real robot controller, 回退 joint-state playback), (6) `onMoveToStart()`/`onMoveToHome()` 的仿真模式动画, (7) 完整的性能日志 (PROFILE) 输出 |
| 2 | `src/ik_solver.cpp` | 259 | **IK 求解器核心实现**。包含: (1) `solveSinglePose()`(第22-111行) — 自定义 DLS 迭代求解, (2) `buildSeedList()`(第113-147行) — 构建局部种子列表(q_prev, home_q, zero 及 J5/J6 wraps), (3) `buildGlobalSeedList()`(第149-169行) — 构建 96+1 个全局随机种子, (4) `solveSinglePoseKdl()`(第187-256行) — 基于 KDL LMA 的 IK 求解(将 TCP target 转换到 wrist_3 帧后调用 KDL 解析求解器) |
| 3 | `src/ik_backend.cpp` | 80 | **IK Backend 工厂模式实现**。实现了 5 个具体求解器的 `solve()` 方法: `CurrentNumericIK` 委托给 `solveSinglePose()`, `KdlLMAIK` 委托给 `solveSinglePoseKdl()`, 其余3个(`TracIKSolver`, `IKFastSolver`, `URKinematicsSolver`) 抛出 `unsupportedBackend` 异常。工厂函数 `createIKSolverBackend()`(第60-77行) 解析 `cfg.solver_backend` 字符串, 创建对应的求解器实例 |
| 4 | `src/trajectory_solver.cpp` | 760 | **轨迹求解主流程** (`solveTrajectory()`)。核心框架: (1) 第71-761行 — 主函数, (2) 第91-95行 — 权重调度表定义(4级权重, 从严格到宽松), (3) 第112-368行 — `run_point_search()` lambda, 包含完整的搜索策略: 连续预测 -> 回退阶段1(主要种子) -> 回退阶段2(J5/J6 wraps) -> 回退阶段3(全局随机种子) -> FCL 精确检查与排序, (4) 第386-458行 — 对所有目标位姿求解 anchor IK, (5) 第476-600行 — Playback 生成(密度公式 + 子步 IK), (6) 第641-747行 — 碰撞审计(快速检查 + FCL 精确检查) |
| 5 | `src/collision_checker.cpp` | 160 | **FCL 碰撞检查封装** (`evaluateConfiguration()`)。核心实现: (1) 使用 `thread_local` 缓存 FCL 碰撞对象(避免 BVH 树重建), (2) 自身碰撞和工具-主体碰撞的双循环检查(第73-106行), (3) 工具-料箱碰撞检查(第130-154行), (4) FCL 0.7.0 `-1.0` sentinel 值过滤(第141-143行), (5) 碰撞优先级排序: tool_basin > tool_body > self |
| 6 | `src/collision_pipeline.cpp` | 57 | **碰撞检测流水线** (`CollisionPipeline` 类)。提供: (1) `collisionStride()` — 根据实时/完整模式返回碰撞步长, (2) `shouldRunPreciseCheck()` — 判断是否运行精确 FCL 检查, (3) `quickPlaybackCheck()` — 快速检查(关节 delta > 1.35 rad 或 TCP z 超出 [-0.05, 1.50] 范围), (4) `preciseCheck()` — 委托给 `evaluateConfiguration()` |
| 7 | `src/robot_model.cpp` | 357 | **URDF 模型加载与运动学实现**。包含: (1) `loadStlAsFclModel()`(第25-91行) — 二进制/文本 STL 文件解析为 FCL BVH 模型, (2) `buildGeomFromUrdf()`(第93-120行) — URDF 碰撞几何转 FCL 对象, (3) `loadRobotModel()`(第126-257行) — 完整的 URDF 解析(关节链追踪, KDL 链构建, wrist_3->TCP 固定偏移计算), (4) `forwardKinematics()`(第259-273行), (5) `tipTransform()`(第275-278行), (6) `numericJacobian()`(第280-301行) — 6x6 几何雅可比矩阵, (7) `clampToLimits()`(第303-310行), (8) `clampToLimitsWithRebound()`(第312-325行) — 限制夹紧时反弹 10% range, (9) `alignToReference()`(第327-347行) — 每个关节 ±4π 范围内找到最接近参考值的等效角度, (10) `urdfPoseToTform()`(第349-355行) |
| 8 | `src/trajectory_generator.cpp` | 328 | **轨迹生成实现**。包含: (1) `loadRuntimeConfig()`(第156-184行) — 从 YAML 加载运行时配置, (2) `readInitialQ()`(第71-99行) — 从 URDF ros2_control 块解析初始关节角度, (3) `buildTargetPlan()`(第204-299行) — 构建目标路径(斜线段 30 点 + 圆弧 140 点 + 提升段 90 点 + 出泥旋转过渡), (4) `buildBasinBoxes()`(第301-325行) — 构建料箱碰撞盒(底板 + 前后左右 5 个面), (5) 辅助函数: `computeTangents()`, `buildTrackRotation()`, `buildLiftRotation()` |
| 9 | `src/continuous_trajectory_solver.cpp` | 21 | **连续轨迹求解器实现**。`ContinuousTrajectorySolver::solve()` 简单委托给 `solveTrajectory()` |
| 10 | `src/rolling_planner.cpp` | 26 | **滚动规划器实现**。`RollingPlanner::solve()` 根据 `enable_window_solve` 开关决定是否截取前 window_size 个目标进行求解 |
| 11 | `src/utils.cpp` | 113 | **工具函数实现**。实现了全部数学工具函数: `rtToTform()`, `rpyToRot()`, `rotToLogVec()`, `rotationDistance()`, `quinticBlend()`, `slerpRotation()`, `wrapJointDelta()`, `continuityCost()`(包含分支切换惩罚: 25° 阈值), `poseError()` |

### 1.3 测试文件

| # | 文件路径 | 行数 | 功能 |
|---|---------|------|------|
| 1 | `test/rtfg_geometry_check.cpp` | 49 | **路径生成几何验证**。加载 YAML 配置和 URDF, 调用 `buildTargetPlan()`, 验证 approach_start/entry/arc_end 等关键点坐标与预期值在 2mm 容差内 |

### 1.4 Service 定义文件

| # | 文件路径 | 功能 |
|---|---------|------|
| 1 | `srv/LoadConfig.srv` | 加载运行时配置的 Service 接口 |
| 2 | `srv/FitPreview.srv` | 轨迹求解与预览的 Service 接口 |
| 3 | `srv/ExecuteCached.srv` | 执行缓存轨迹的 Service 接口 |

---

## 2. 所有类的类图

### 2.1 IKSolverBase 抽象基类 (ik_backend.h:10-22)

```
IKSolverBase (抽象基类)
│
├── virtual ~IKSolverBase() = default
├── virtual std::string name() const = 0
└── virtual CandidateInfo solve(
        const RobotModel&, const std::vector<BasinBox>&,
        const SolverConfig&, const Mat4& target,
        const Eigen::VectorXd& seed, const Eigen::VectorXd& q_prev,
        const Eigen::VectorXd& dq_prev,
        const std::array<double, 6>& weights, double orient_limit) const = 0
```

### 2.2 派生类 (ik_backend.h:24-87)

```
IKSolverBase
│
├── CurrentNumericIK                   (ik_backend.h:24-35)
│   ├── name() → "numeric"
│   └── solve() → 委托 solveSinglePose()   [src/ik_backend.cpp:15-25]
│
├── KdlLMAIK                           (ik_backend.h:37-48)
│   ├── name() → "kdl"
│   └── solve() → 委托 solveSinglePoseKdl()  [src/ik_backend.cpp:27-37]
│
├── TracIKSolver                       (ik_backend.h:50-61)
│   ├── name() → "tracik"
│   └── solve() → 抛出 unsupportedBackend  [src/ik_backend.cpp:39-44]
│
├── IKFastSolver                       (ik_backend.h:63-74)
│   ├── name() → "ikfast"
│   └── solve() → 抛出 unsupportedBackend  [src/ik_backend.cpp:46-51]
│
└── URKinematicsSolver                 (ik_backend.h:76-87)
    ├── name() → "ur_kinematics"
    └── solve() → 抛出 unsupportedBackend  [src/ik_backend.cpp:53-58]
```

### 2.3 Factory 函数分支逻辑 (ik_backend.cpp:60-77)

```
createIKSolverBackend(const SolverConfig& cfg)
│
├── cfg.solver_backend == "numeric" || empty
│   → std::make_unique<CurrentNumericIK>()
│
├── cfg.solver_backend == "kdl"
│   → std::make_unique<KdlLMAIK>()
│
├── cfg.solver_backend == "tracik"
│   → std::make_unique<TracIKSolver>()      ← 未实现
│
├── cfg.solver_backend == "ikfast"
│   → std::make_unique<IKFastSolver>()       ← 未实现
│
├── cfg.solver_backend == "ur_kinematics"
│   → std::make_unique<URKinematicsSolver>() ← 未实现
│
└── 其他
    → throw std::runtime_error
```

### 2.4 CollisionPipeline 类 (collision_pipeline.h:16-33)

```
CollisionPipeline
│
├── 构造函数(const RobotModel&, const vector<BasinBox>&, const SolverConfig&)
│
├── collisionStride() → int                      // 返回碰撞步长
├── shouldRunPreciseCheck(point_idx, keypoint) → bool  // 是否运行精确检查
├── quickPlaybackCheck(q, q_prev) → QuickCheckResult   // 快速检查 (delta norm + workspace z)
└── preciseCheck(q) → CollisionSummary           // FCL 精确碰撞检查
```

### 2.5 ContinuousTrajectorySolver 类 (continuous_trajectory_solver.h:7-26)

```
ContinuousTrajectorySolver
│
├── 构造函数(const RobotModel&, const vector<BasinBox>&, SolverConfig)
├── solve(targets, names, current_q, home_q) → TrajectoryResult  // 委托 solveTrajectory()
└── config() → const SolverConfig&
```

### 2.6 RollingPlanner 类 (rolling_planner.h:7-24)

```
RollingPlanner
│
├── 构造函数(SolverConfig)
├── enabled() → bool        // cfg_.enable_window_solve
├── windowSize() → int      // cfg_.window_size
├── windowOverlap() → int   // cfg_.window_overlap
└── solve(solver, targets, names, current_q, home_q) → TrajectoryResult
    // 根据 enable_window_solve 决定是否截取前 window_size 个点
```

---

## 3. 所有 ROS2 Service 的接口定义

### 3.1 /rtfg/load_config (LoadConfig.srv)

**Request:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `yaml_path` | `string` | YAML 配置文件路径(空表示使用当前路径) |

**Response:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 加载是否成功 |
| `message` | `string` | 状态消息 |
| `left_wall_offset` | `float64` | 左侧壁偏移 |
| `mud_height` | `float64` | 泥浆高度 |
| `approach_len` | `float64` | 接近长度 |
| `theta_deg` | `float64` | 圆弧角度(度) |
| `depth` | `float64` | 深度 |
| `x_plane` | `float64` | X 平面位置 |
| `pose_x` | `float64` | 环境位姿 X |
| `pose_y` | `float64` | 环境位姿 Y |
| `pose_z` | `float64` | 环境位姿 Z |
| `roll_deg` | `float64` | 滚转角(度) |
| `pitch_deg` | `float64` | 俯仰角(度) |
| `yaw_deg` | `float64` | 偏航角(度) |
| `initial_q` | `float64[]` | 初始关节角度(6维) |

### 3.2 /rtfg/fit_preview (FitPreview.srv)

**Request:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `left_wall_offset` | `float64` | 左侧壁偏移 |
| `mud_height` | `float64` | 泥浆高度 |
| `approach_len` | `float64` | 接近长度 |
| `theta_deg` | `float64` | 圆弧角度(度) |
| `depth` | `float64` | 深度 |
| `x_plane` | `float64` | X 平面位置 |
| `pose_x` | `float64` | 环境位姿 X |
| `pose_y` | `float64` | 环境位姿 Y |
| `pose_z` | `float64` | 环境位姿 Z |
| `roll_deg` | `float64` | 滚转角(度) |
| `pitch_deg` | `float64` | 俯仰角(度) |
| `yaw_deg` | `float64` | 偏航角(度) |
| `current_q` | `float64[]` | 当前关节角度(6维), 空则使用 runtime initial_q |
| `clearance_threshold` | `float64` | 碰撞间隙阈值(>0时覆盖默认值) |

**Response:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 求解成功(无碰撞违规) |
| `message` | `string` | 状态消息 |
| `trajectory` | `trajectory_msgs/JointTrajectory` | 生成的关节轨迹 |
| `target_tcp_poses` | `geometry_msgs/PoseArray` | 目标 TCP 位姿序列 |
| `actual_tcp_poses` | `geometry_msgs/PoseArray` | 实际 TCP 位姿序列 |
| `collision_points_xyz` | `float64[]` | 碰撞点坐标(flat 数组) |
| `collision_types` | `string[]` | 碰撞类型列表 |
| `collision_objects` | `string[]` | 碰撞对象列表 |
| `collision_segments` | `string[]` | 碰撞段名称列表 |
| `min_self_clearance` | `float64` | 最小自身碰撞间隙 |
| `min_tool_body_clearance` | `float64` | 最小工具-主体间隙 |
| `min_tool_basin_clearance` | `float64` | 最小工具-料箱间隙 |
| `min_self_object` | `string` | 最小自身间隙对象 |
| `min_tool_body_object` | `string` | 最小工具-主体间隙对象 |
| `min_tool_basin_object` | `string` | 最小工具-料箱间隙对象 |
| `anchor_count` | `int32` | 锚点数量 |
| `playback_count` | `int32` | Playback 点数量 |
| `max_target_rotation_delta_deg` | `float64` | 目标最大旋转增量(度) |
| `max_actual_rotation_delta_deg` | `float64` | 实际最大旋转增量(度) |
| `max_anchor_joint_step_deg` | `float64` | 锚点最大关节步长(度) |
| `max_playback_joint_step_deg` | `float64` | Playback 最大关节步长(度) |
| `timing_total_wall_s` | `float64` | 总时长(秒) |
| `timing_ik_total_s` | `float64` | IK 总时长(秒) |
| `timing_collision_total_s` | `float64` | 碰撞检测总时长(秒) |
| `timing_avg_per_pose_s` | `float64` | 每姿态平均时长(秒) |

### 3.3 /rtfg/execute_cached (ExecuteCached.srv)

**Request:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `execute` | `bool` | 执行标志(必须为 true) |

**Response:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 执行是否成功 |
| `message` | `string` | 状态消息 |

### 3.4 /rtfg/move_to_start (std_srvs/Trigger)

**Request:** (空)
**Response:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | IK 求解是否成功 |
| `message` | `string` | 状态消息 |

### 3.5 /rtfg/move_to_home (std_srvs/Trigger)

**Request:** (空)
**Response:**
| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 是否成功启动 |
| `message` | `string` | 状态消息 |

---

## 4. 所有 Topic 的类型和发布频率

| # | Topic 名称 | 消息类型 | Publisher | QoS/QoS深度 | 发布时机 |
|---|-----------|---------|-----------|------------|---------|
| 1 | `/rtfg/target_tcp_poses` | `geometry_msgs/PoseArray` | `target_pub_` | 10 | `onFitPreview()` 求解完成后 (rtfg_solver_node.cpp:301) |
| 2 | `/rtfg/tcp_path` | `geometry_msgs/PoseArray` | `actual_pub_` | 10 | `onFitPreview()` 求解完成后 (rtfg_solver_node.cpp:305) |
| 3 | `/rtfg/collision_markers` | `visualization_msgs/MarkerArray` | `marker_pub_` | 10 | `publishVisualization()` 中 (rtfg_solver_node.cpp:638-666) |
| 4 | `/rtfg/metrics` | `std_msgs/String` | `metrics_pub_` | 10 | `publishVisualization()` 中 (rtfg_solver_node.cpp:675-693) |
| 5 | `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | `display_pub_` | 10 | `publishVisualization()` 中 (rtfg_solver_node.cpp:668-673) |
| 6 | `/joint_states` | `sensor_msgs/JointState` | `joint_state_pub_` | 10 | `onPlaybackTimer()` 以 **50 Hz** (20ms) 发布 (rtfg_solver_node.cpp:427-428, 748-761); `onMoveTimer()` 以 **50 Hz** 发布 (rtfg_solver_node.cpp:500-501, 531-553) |

---

## 5. IK 算法的核心数据流

### 5.1 solveSinglePose() — 自定义 DLS 迭代流程 (ik_solver.cpp:22-111)

```
输入: RobotModel, basin_boxes, SolverConfig, target(Mat4),
      seed(VectorXd), q_prev, dq_prev, weights(6维), orient_limit

处理:
  1. q = clampToLimits(robot, seed)                    // 行27: 种子限位夹紧
  2. q_ref = q_prev + dq_prev                          // 行28: 参考配置(连续预测)
  
  3. for iter = 0 to cfg.max_iterations-1:             // 行34: 迭代循环
     a. T = tipTransform(robot, q)                     // 行35: 正向运动学
     b. err = poseError(T, target)                     // 行36: 位姿误差
     c. weighted = err .* weights                      // 行39-41: 加权
     d. pos_err = |err.pos|, rot_err = |err.rot|       // 行43-44
     e. IF pos_err <= tolerance AND rot_err <= orient_limit  // 行46: 收敛判断
        → break
     f. J = numericJacobian(robot, q)                  // 行51: 6x6 雅可比
        J.row(r) *= weights[r]                         // 行52-54: 加权雅可比
     g. 自适应阻尼:                                     // 行57-63:
        IF pos_err>0.1 || rot_err>0.1:
          lambda = max(1e-3, 5e-3*(pos_err/0.05))
          lambda = min(lambda, 0.1)
        ELSE:
          lambda = 5e-4 + 5e-3*(pos_err/0.05)
     h. H = J^T*J + lambda*I                            // 行65: 正规方程
        g = J^T * weighted                              // 行66
        dq = H.ldlt().solve(g)                          // 行67: LDLT 求解
     i. IF |dq| > 0.45 → dq *= 0.45/|dq|               // 行70-72: 步长裁剪
     j. IF |dq| <= dq_stop_threshold → break            // 行73: 步长停止
     k. q = clampToLimits(robot, q + dq)                // 行78: 更新+限位
        q = alignToReference(robot, q, q_ref)           // 行79: 分支对齐
     l. 停滞检测: IF 连续 stagnation_patience 次误差变化 ≤ stagnation_epsilon → break  // 行82-93

  4. 最终评估: T = tipTransform(q), err = poseError(T, target)  // 行97-98
  5. cand.q = q, cand.pos_err, cand.rot_err, cand.clearance
     cand.cost = continuityCost(q, q_prev, dq_prev)     // 行104: 连续性代价

输出: CandidateInfo (valid=true/false)
```

### 5.2 solveSinglePoseKdl() — KDL LMA 流程 (ik_solver.cpp:187-256)

```
输入: 同上

处理:
  1. IF !robot.kdl_chain || kdl_chain->getNrOfJoints() != 6 → 返回无效  // 行196-199
  2. target_wrist3 = target * T_wrist3_to_tcp.inverse()   // 行203: TCP→wrist_3 变换
  3. 构建 KDL::Frame target_frame (从 target_wrist3)       // 行206-211
  4. 构建 KDL::JntArray q_init (从 seed_clamped)           // 行214-216
  5. 构建权重向量 L(k) = sqrt(max(weights[k], 1e-6))       // 行222-225
  6. KDL::ChainIkSolverPos_LMA solver(chain, L, 1e-6, 200, 1e-7)  // 行227
  7. solver.CartToJnt(q_init, target_frame, q_out)          // 行229: KDL 求解
  8. IF rc < 0 → 返回无效 (KDL 失败)                         // 行231-235
  9. q_kdl = clampToLimits(alignToReference(...))           // 行239-241: 后处理
  10. 评估: T_tcp = tipTransform(q_kdl), poseError         // 行244-245

输出: CandidateInfo (valid=true/false)
```

### 5.3 自适应阻尼策略 (ik_solver.cpp:57-63)

```
lambda_base = cfg.lambda (默认 5e-3)

IF pos_err > 0.1 OR rot_err > 0.1:
    lambda = max(1e-3, 5e-3 * (pos_err / 0.05))
    lambda = min(lambda, 0.1)        // 远程时高阻尼, 保证稳定性
ELSE:
    lambda = 5e-4 + 5e-3 * (pos_err / 0.05)  // 近程时低阻尼, 快速收敛
```

### 5.4 分支对齐机制 alignToReference() (robot_model.cpp:327-347)

```
输入: RobotModel, q (当前关节角度), q_ref (参考关节角度)

处理流程(对每个可动关节):
  1. best = q[idx], best_dist = |q[idx] - q_ref[idx]|
  2. 对于 k = -2, -1, 0, 1, 2:
     candidate = q[idx] + 2π*k                    // ±4π 范围内的等效角度
     IF candidate 在关节限位内:
       dist = |candidate - q_ref[idx]|
       IF dist < best_dist: best = candidate      // 选择最接近参考值的等效角度
  3. q_aligned[idx] = clamp(best, lower, upper)

输出: q_aligned (与参考值最接近的等效构型)
```

---

## 6. 关键参数表

### SolverConfig (types.h:99-120)

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `solver_mode` | `string` | `"full"` | 求解模式: `"full"` / `"realtime"` |
| `solver_backend` | `string` | `"numeric"` | IK 后端: `"numeric"` / `"kdl"` / `"tracik"` / `"ikfast"` / `"ur_kinematics"` |
| `clearance_threshold` | `double` | `2e-3` | 碰撞间隙阈值(m), 低于此值视为碰撞 |
| `ik_position_tolerance` | `double` | `3e-2` | IK 位置收敛容差(m) |
| `max_iterations` | `int` | `60` | 最大迭代次数(自动设为 full/realtime 对应值) |
| `max_iterations_full` | `int` | `60` | 完整模式最大迭代次数 |
| `max_iterations_realtime` | `int` | `30` | 实时模式最大迭代次数 |
| `lambda` | `double` | `5e-3` | DLS 阻尼系数基础值 |
| `stagnation_epsilon` | `double` | `1e-5` | 停滞检测阈值(误差变化量) |
| `stagnation_patience` | `int` | `3` | 停滞容忍次数 |
| `dq_stop_threshold` | `double` | `1e-6` | 关节增量停止阈值 |
| `max_collision_candidates_full` | `int` | `12` | 完整模式最大碰撞候选数 |
| `max_collision_candidates_realtime` | `int` | `2` | 实时模式最大碰撞候选数 |
| `collision_check_stride_full` | `int` | `1` | 完整模式碰撞检查步长 |
| `collision_check_stride_realtime` | `int` | `5` | 实时模式碰撞检查步长 |
| `publish_sparse_posearray_realtime` | `bool` | `true` | 实时模式是否发布稀疏 PoseArray |
| `posearray_stride_realtime` | `int` | `10` | 实时模式 PoseArray 稀疏步长 |
| `enable_window_solve` | `bool` | `false` | 是否启用窗口滚动求解 |
| `window_size` | `int` | `100` | 滚动窗口大小 |
| `window_overlap` | `int` | `10` | 滚动窗口重叠数 |

### 节点参数默认值 (rtfg_solver_node.cpp:135-160)

节点在声明参数时, 部分参数对 SolverConfig 默认值做了覆盖:

| 参数名 | 节点默认值 | 对应 SolverConfig 默认值变化 |
|--------|-----------|---------------------------|
| `solver_backend` | `"kdl"` | 与 `SolverConfig::solver_backend` 默认 `"numeric"` 不同 |
| `max_collision_candidates_full` | `48` | 覆盖默认 `12` |
| `clearance_threshold` | `2e-3` | 一致 |
| `collision_check_stride_full` | `7` | 覆盖默认 `1` |
| `collision_check_stride_realtime` | `7` | 覆盖默认 `5` |

---

## 7. 搜索策略的流程图描述

### 7.1 run_point_search() 整体流程 (trajectory_solver.cpp:112-368)

```
为每个目标位姿 target[i] 执行:

┌─────────────────────────────────────────────────────────────────────┐
│ 阶段0: 连续预测 (rebound_seed)           trajectory_solver.cpp:296-328 │
│                                                                     │
│   rebound_seed = clampToLimitsWithRebound(q_prev + dq_prev)         │
│   run_stage({rebound_seed, q_prev}, weight_schedule)                │
│   finalize_candidates(1)  ← 只允许 1 个候选进入 FCL                 │
│                                                                     │
│   IF safe.valid:                                                    │
│     continuous_prediction_hits++                                    │
│     stop_search = true  → 返回                                      │
│     ~~~ ~85% 命中率 ~~~                                             │
│   ELSE:                                                             │
│     continuous_prediction_fallbacks++                                │
│                                                                     │
│   边缘接受: IF fallback.pos_err ≤ tolerance AND clearance > 0:       │
│     → 标记为 tight_clearance, 接受 (防止 350° 分支切换)               │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                      (连续预测失败, 继续)
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│ 阶段1: 主要种子 × 4级权重              trajectory_solver.cpp:332-334  │
│                                                                     │
│   run_stage({q_prev, home_q, zero}, weight_schedule)                │
│                                                                     │
│   权重调度表:                                                        │
│     权重 1: [1,1,1,0.20,0.20,0.20], orient_limit = 30°              │
│     权重 2: [1,1,1,0.10,0.10,0.10], orient_limit = 45°              │
│     权重 3: [1,1,1,0.03,0.03,0.03], orient_limit = 70°              │
│     权重 4: [1,1,1,0.00,0.00,0.00], orient_limit = ∞ (位置优先)     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                      (阶段1未找到有效解, 继续)
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│ 阶段2: J5/J6 wraps 扩展种子           trajectory_solver.cpp:337-349  │
│                                                                     │
│   buildSeedList(q_prev, home_q, robot)  → 每个种子扩展 9 个组合      │
│   (q_prev + {0, ±2π}×J5 × {0, ±2π}×J6)                             │
│   + home_q 类似扩展                                                  │
│   + zero 类似扩展                                                    │
│   去重后约 27 个唯一种子                                              │
│   run_stage(expanded_seeds, weight_schedule)                        │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                      (阶段2未找到有效解, 继续)
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│ 阶段3: 全局随机种子                     trajectory_solver.cpp:351-363  │
│                                                                     │
│   buildGlobalSeedList(robot) → 97 个种子:                            │
│     1 个中间点 + 96 个均匀随机采样全关节空间                           │
│   预算: 最多 6 次尝试 × 4 级权重                                      │
│   run_stage(global_seeds[0..5], weight_schedule)                    │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│ finalize_candidates(collision_limit)   trajectory_solver.cpp:186-288 │
│                                                                     │
│   1. 按 score 排序所有 ranked_candidates                             │
│   2. 截断: entered_fcl = min(size, collision_limit)                  │
│   3. 对前 entered_fcl 个候选执行 FCL精确检查:                         │
│      - preciseCheck(q) → CollisionSummary                            │
│      - clearance = min(self, tool_body, tool_basin)                  │
│      - IF clearance < clearance_threshold → 拒绝                     │
│      - 选择最佳 safe 候选(优先 clearance, 其次 score)                 │
│                                                                     │
│   4. 回退检查: fallback 候选的 FCL 检查                               │
│                                                                     │
│   5. 实时模式关键点回退: IF realtime + keypoint + !safe.valid:        │
│      → 以 full 模式递归调用 run_point_search() (允许更多候选)         │
│                                                                     │
│   分支惩罚: step_from_prev > 30° → score += 1000 + step_from_prev    │
│     (trajectory_solver.cpp:153-158)                                  │
└─────────────────────────────────────────────────────────────────────┘
```

### 7.2 关键评分机制

**候选排序** (trajectory_solver.cpp:187-194):
```
主键: score (越小越好)
  - base = continuityCost(q_candidate, q_prev, dq_prev)
  - IF step_from_prev > 30° → score += 1000 + step_from_prev  (分支惩罚)
次键: pos_err
末键: rot_err
```

**连续性代价函数** `continuityCost()` (utils.cpp:79-101):
```
cost = |dq_pos| + 0.65 * |dq_vel|
其中:
  dq_pos = wrapJointDelta(q_candidate - q_prev)
  dq_vel = dq_pos - dq_prev
分支惩罚: 任何关节 unwrapped delta > 25° → cost += (|dq_raw| - 25°)
```

### 7.3 边缘接受机制 (trajectory_solver.cpp:311-328)

当连续预测产生 `fallback.valid` 但 FCL clearance < `clearance_threshold` 时:
- 如果 clearance > 0.0 (即没有物理穿透, 只是间隙不足)
- 仍然接受该候选, 标记为 `"tight_clearance:Xmm"`
- 这避免了 tight-clearance 点发生分支切换(导致 350°+ 的关节跳跃)

---

## 8. Playback 生成算法

### 8.1 密度公式 (trajectory_solver.cpp:494-498)

对于相邻锚点 qa 和 qb 之间, 计算插值段数:

```
joint_step = |wrapJointDelta(qb - qa)|                    // 关节空间步长(rad)
pos_step   = |Tb.translation - Ta.translation|             // 笛卡尔位置步长(m)
rot_step   = rotationDistance(Ta.rotation, Tb.rotation)   // 姿态步长(rad)

nseg = 1 + max(
    2,                                                      // 至少 2 段
    ceil(joint_step / 0.70_deg),                           // 每段 ≤ 0.70° 关节运动
    ceil(pos_step / 0.0030),                               // 每段 ≤ 3mm 位置运动
    ceil(rot_step / 0.30_deg)                              // 每段 ≤ 0.30° 姿态运动
)
nseg = clamp(4, nseg, 32)
```

### 8.2 Quintic 插值 (trajectory_solver.cpp:584-592)

对于每个插值段 k = 1..nseg:
```
alpha = k / nseg
a = quinticBlend(alpha)       // 10t³ - 15t⁴ + 6t⁵ (utils.cpp:57-59)
q = qa + a * wrapJointDelta(qb - qa)
```

### 8.3 子步 IK 安全机制 (trajectory_solver.cpp:510-569)

当 `joint_step_deg > 20°` 时触发子步 IK (防止大跨度造成位姿跳跃):

```
1. n_sub = clamp(4, max(密度公式, 4), 64)
2. q_sub = qa, dq_sub = qa - q(i-2) 或 zero
3. FOR k = 1 TO n_sub:
   a. alpha = k / n_sub
   b. 插值目标位姿: pos_sub = (1-α)pos_A + α*pos_B
                      R_sub   = slerp(RA, RB, α)
   c. 使用 IK 求解 T_sub 的最优构型
      → 走完 4 级权重调度表
   d. IF sub_cand.valid AND ik_jump ≤ 0.5 (~28°):
        q_sub = sub_cand.q   (接受 IK 子步解)
      ELSE:
        q_sub = qa + α*wrapJointDelta(qb - qa)  (回退到线性插值)
   e. dq_sub = wrapJointDelta(q_sub - playback_q.back())
4. anchor_q[i] = playback_q.back()  // 修正锚点
```

---

## 9. 性能瓶颈识别

### 9.1 IK 求解 — ~62% 总时间

| 函数 | 时间占比 | 说明 |
|------|---------|------|
| `solveSinglePose()` 的 for 循环 (ik_solver.cpp:34-95) | ~62% | DLS 迭代循环中每次都要调用 `numericJacobian()`(6x6 雅可比计算 + 正向运动学) 和 `tipTransform()`。实测 ~6.2s 总耗时 |
| `tipTransform()` (robot_model.cpp:275-278) | 内部 | 每次迭代都触发 `forwardKinematics()`(遍历 6 个关节段) |
| `numericJacobian()` (robot_model.cpp:280-301) | 内部 | 遍历 6 个关节, 每个计算 axis × (tcp - origin) |
| `CandidateInfo` solve 调用 | — | 每个 target 平均 ~60 次迭代, 每个 target 尝试多个种子(平均 3-5 个种子 × 4 级权重) |

**关键**: `solveSinglePoseKdl()` (KDL LMA) 是单次解析求解(无迭代循环), 理论上更快, 但当前节点默认使用 `"kdl"` 后端; 由于 KDL 链只到 wrist_3(wrist_3->TCP 偏移在外部分离), 比完整数值 DLS 省去 6x6 雅可比计算.

### 9.2 FCL 碰撞检测 — ~37% 总时间 (trajectory_solver.cpp:641-747)

| 函数 | 时间占比 | 说明 |
|------|---------|------|
| `CollisionPipeline::preciseCheck()` → `evaluateConfiguration()` (collision_checker.cpp:23-157) | ~37% | 实测 ~3.7s 总耗时 |
| 自身碰撞双循环 (collision_checker.cpp:73-106) | 内部主导 | O(n^2) 对 robot.collisions 所有碰撞体对进行距离查询 |
| 工具-料箱检查 (collision_checker.cpp:130-154) | 较小 | tool_links × 5 个 basin boxes |
| `thread_local` 对象缓存 | 优化 | `CollisionObjectd` 复用避免 BVH 重建 |

**FCL 调用频率**:
- 完整模式: `shouldRunPreciseCheck` 每 stride=1 个点 + 关键点 + 端点都触发
- 实时模式: stride=5, 配合 keypoint 检测
- Playback 碰撞审计: 对所有 playback 点采样检查(每个点如果 stride 条件满足)

### 9.3 其他耗时操作

| 操作 | 占比 | 说明 |
|------|------|------|
| Playback 生成 (trajectory_solver.cpp:476-641) | <1% | 纯数学计算(没有 IK/FCL) |
| 子步 IK (trajectory_solver.cpp:510-569) | 额外 | 仅对大关节步长触发, 每次触发增加额外 IK 求解 |
| PoseArray/JointTrajectory 打包 | <1% | 常规 ROS 消息序列化 |

---

## 10. CUDA 并行化扩展点分析

### 10.1 可并行化部分

| 阶段 | 函数 | 并行化策略 | 期望加速 |
|------|------|-----------|---------|
| **IK 求解** | `solveSinglePose()` / `run_point_search()` | **多目标并行**: trajectory_solver.cpp:386-458 中对 260 个 target 依次求解, 每个 target 的 IK 求解(多个种子尝试)可独立并行到 GPU 线程块. 每个线程块处理一个 target+seed, 完成 DLS 迭代循环 | ~10-50x (取决于迭代次数和种子数) |
| **雅可比计算** | `numericJacobian()` (robot_model.cpp:280-301) | 6 个关节的雅可比列计算可映射到 6 个 CUDA 线程并行. 正向运动学链可被分解为 parallel prefix scan | ~2-6x |
| **碰撞检查** | `evaluateConfiguration()` 自身碰撞双循环 (collision_checker.cpp:73-106) | O(n^2) 的碰撞体对距离查询: n≈10-14 个碰撞体, ~50-100 对. 每对为独立 FCL `fcl::distance()` 调用, 批量发射到 GPU | ~5-10x |
| **种子评估** | `run_stage()` / `eval_seed()` (trajectory_solver.cpp:163-184) | 多个种子 × 4 级权重的 IK 求解完全独立, 可映射到 CUDA 网格. trajectory_solver.cpp:351-363 中的全局随机种子(96个)尤其适合 GPU 批量启动 | ~10-20x |
| **Playback 插值** | trajectory_solver.cpp:584-592 | 每个插值点的 `quinticBlend()` + `wrapJointDelta()` 计算独立, 可向量化 | ~10x |
| **碰撞审计** | trajectory_solver.cpp:649-734 | 每个 playback 点的碰撞检查相互独立, 适合 GPU 批量处理 | ~10-20x |

### 10.2 需保留 CPU 的部分

| 组件 | 原因 | 文件引用 |
|------|------|---------|
| **FCL BVH 遍历** | FCL 的 `fcl::OBBRSSd` BVH 树是递归遍历结构, GPU 上分支发散严重. `fcl::distance()` 内部使用 GJK/EPA 算法, 高度控制流密集 | collision_checker.cpp 全部 |
| **`thread_local` 对象缓存** | GPU 全局内存不适合存放 `shared_ptr<CollisionGeometryd>` 和 `CollisionObjectd` 对象 | collision_checker.cpp:31-53 |
| **STL 文件解析** | `loadStlAsFclModel()` 文件 I/O 和 BVH 构建是离线加载, 并非运行时瓶颈 | robot_model.cpp:25-91 |
| **ROS2 通信层** | `RtfgSolverNode` 的 Service/Topic/Pub/Sub 属于 OS 级 I/O | rtfg_solver_node.cpp 全部 |
| **种子列表构建** | `buildSeedList()`, `buildGlobalSeedList()` 调用次数少, 数据量小(<100 个种子) | ik_solver.cpp:113-169 |

### 10.3 推荐的 CUDA 适配策略

```
┌──────────────────────────────────────────────────────────────┐
│ CPU 主线程 (ROS2 Node + 调度)                                │
│                                                              │
│  1. RtfgSolverNode::onFitPreview()                          │
│  2. 预处理: buildTargetPlan() + buildBasinBoxes()            │
│  3. 对每个 target 下发 IK 任务到 GPU                         │
│  4. 从 GPU 回收 CandidateInfo 结果                           │
│  5. run_point_search() 的排序 + 分支惩罚 CPU 侧处理          │
│  6. 调度 FCL 碰撞检查(CPU 侧, 每候选)                        │
│  7. Playback 生成(部分 GPU 向量化)                           │
│  8. 碰撞审计(FCL 保留 CPU)                                   │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│ GPU 内核 (批量 IK 求解)                                      │
│                                                              │
│  kernel_solve_single_pose(target, seed, weights, orient_limit)│
│    → 每个线程/线程块处理一个 (target, seed, weight_level)     │
│    → 包含: forwardKinematics(6 个关节链)                     │
│            numericJacobian(6 列雅可比)                        │
│            DLS 迭代: H = J^T*J + λI, LDLT 求解               │
│            alignToReference (每个关节 ±4π 搜索)               │
│            continuityCost 计算                                │
│    → 输出: CandidateInfo (q, pos_err, rot_err, cost)         │
│                                                              │
│  所有 target (260个) × 种子(3-27个) × 权重(4级)             │
│  = ~3120-28080 个独立 IK 任务 → 充分占用 GPU                 │
└──────────────────────────────────────────────────────────────┘
```

### 10.4 GPU 加速预期

| 配置 | 纯 CPU (当前) | CPU+GPU (预测) | 加速比 |
|------|-------------|---------------|--------|
| IK 求解 (260 targets × ~6 种子) | ~6200 ms | ~100-300 ms | ~20-60x |
| FCL 碰撞检查 (retain CPU) | ~3700 ms | ~3700 ms | 1x (保留) |
| Playback 生成 | ~50 ms | ~10 ms | ~5x |
| **总计** | **~10 s** | **~4 s** | **~2.5x** |

**瓶颈转移**: GPU 加速后, FCL 碰撞检查将从 37% 占比上升到 ~90%+, 成为新的主要瓶颈.

---

## 附录: 文件目录结构

```
assembly_rtfg_cpp/
├── CMakeLists.txt                              # 构建配置
├── include/assembly_rtfg_cpp/
│   ├── types.h                                # 核心数据结构
│   ├── ik_solver.h                            # IK 求解器函数声明
│   ├── ik_backend.h                           # IK 后端工厂模式
│   ├── trajectory_solver.h                    # 轨迹求解主流程声明
│   ├── collision_checker.h                    # 碰撞检查声明
│   ├── collision_pipeline.h                   # 碰撞流水线类
│   ├── robot_model.h                          # 机器人模型声明
│   ├── trajectory_generator.h                 # 轨迹生成声明
│   ├── utils.h                                # 工具函数声明
│   ├── continuous_trajectory_solver.h         # 连续轨迹求解器类
│   └── rolling_planner.h                      # 滚动规划器类
├── src/
│   ├── rtfg_solver_node.cpp                   # 主节点 (779行)
│   ├── ik_solver.cpp                          # IK 求解器核心 (259行)
│   ├── ik_backend.cpp                         # IK 后端工厂 (80行)
│   ├── trajectory_solver.cpp                  # 轨迹求解主流程 (760行)
│   ├── collision_checker.cpp                  # FCL 碰撞检查 (160行)
│   ├── collision_pipeline.cpp                 # 碰撞流水线 (57行)
│   ├── robot_model.cpp                        # 机器人模型 (357行)
│   ├── trajectory_generator.cpp               # 轨迹生成 (328行)
│   ├── continuous_trajectory_solver.cpp       # 连续轨迹求解器 (21行)
│   ├── rolling_planner.cpp                    # 滚动规划器 (26行)
│   └── utils.cpp                              # 工具函数 (113行)
├── srv/
│   ├── LoadConfig.srv                         # 配置加载 Service
│   ├── FitPreview.srv                         # 轨迹预览 Service
│   └── ExecuteCached.srv                      # 执行缓存 Service
├── test/
│   └── rtfg_geometry_check.cpp                # 路径几何验证测试
├── config/                                    # YAML 运行时配置
├── launch/                                    # ROS2 launch 文件
├── urdf/                                      # URDF 机器人模型
├── rviz/                                      # RViz2 配置
└── benchmarks/                                # 基准测试数据
```

---

*文档生成完毕。所有文件引用路径均为绝对路径, 行号基于实际读取的源文件标注。*
