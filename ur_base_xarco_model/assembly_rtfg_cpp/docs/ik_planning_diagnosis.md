# assembly_rtfg_cpp 逆运动学与轨迹规划诊断报告

## 1. 结论先行

当前 `assembly_rtfg_cpp` 的主耗时链路**不是**每个点调用 `move_group.plan()`、`computeCartesianPath()` 或 `setFromIK()`。

代码证据显示，当前主路径是：

`轨迹点生成 -> 自研 IK（KDL LMA / 数值 DLS）-> FCL 碰撞检测 -> 回放轨迹生成 -> ROS2 message 包装 -> FollowJointTrajectory action 发送`

换句话说，当前 1300 点约 13 秒的核心开销，主要来自：

1. 每个目标点的 IK 迭代；
2. 每个候选的碰撞距离检测；
3. 回放轨迹生成与 `JointTrajectory` / `PoseArray` 的逐点打包；
4. 如果启用 `launch_moveit:=true`，`move_group` 会启动，但当前 solver 节点并不调用它做逐点规划。

---

## 2. 搜索结果汇总

### 2.1 明确搜到的相关点

- `trajectory_msgs/JointTrajectory`：在 `rtfg_solver_node.cpp` 中构造并返回/发送。
- `FollowJointTrajectory` action：在 `rtfg_solver_node.cpp::onExecuteCached()` 中发送。
- `KDL`：在 `ik_solver.cpp`、`types.h`、`robot_model.cpp` 中使用。
- `CartToJnt`：在 `ik_solver.cpp::solveSinglePoseKdl()` 中调用。
- `collision`：在 `collision_checker.cpp`、`trajectory_solver.cpp` 中做 FCL 距离检测。
- `lookupTransform`：在 `assembly_rtfg_cpp` 内**未找到**。
- `setFromIK` / `computeCartesianPath` / `getPositionIK` / `plan(` / `move_group.plan` / `PlanningScene` / `RobotModelLoader` / `setPlanningTime` / `setNumPlanningAttempts`：在 `assembly_rtfg_cpp` 内**未找到**。

### 2.2 需要注意的“存在但不在主耗时链”的点

- `launch/rtfg_sim.launch.py` 会在 `launch_moveit:=true` 时拉起 `move_group`。
- `assembly_rtfg_cpp/package.xml` 依赖 `moveit_ros_move_group`，但这属于启动/展示依赖，不代表 solver 在运行时调用 MoveIt 规划 API。
- `assembly_rtfg_cpp/docs/rtfg_ros2_cpp_技术文档/05_Cpp性能基准与优化记录.html` 已明确写过：MoveIt 只是在展示和兼容层出现，第一版主链是自研 IK + FCL。

---

## 3. 真实调用链

### 3.1 总链路

`轨迹点生成 -> 位姿转换 -> IK 求解 -> 碰撞检测 -> 轨迹规划/回放 -> 时间参数化 -> 发送控制器`

### 3.2 对应文件和函数

#### 轨迹点生成

- 文件：`assembly_rtfg_cpp/src/trajectory_generator.cpp`
- 函数：`buildTargetPlan()`
- 是否在 1300 点循环内：是，生成了约 273 个目标锚点，再扩展为更密的回放轨迹
- 是否重复初始化对象：否，纯函数式生成
- 是否调用 ROS service/action/topic：否
- 是否可能成为耗时瓶颈：中等，通常不是主瓶颈，但会随着点数增加线性增长

#### 位姿转换

- 文件：`assembly_rtfg_cpp/src/trajectory_generator.cpp`
- 函数：`poseTform()`、`transformPoint()`、`buildTrackRotation()`、`buildLiftRotation()`
- 是否在 1300 点循环内：是，和轨迹生成一起执行
- 是否重复初始化对象：否
- 是否调用 ROS service/action/topic：否
- 是否可能成为耗时瓶颈：低到中等

#### IK 求解

- 文件：`assembly_rtfg_cpp/src/trajectory_solver.cpp`
- 函数：`solveTrajectory()`
- 单点求解实现文件：`assembly_rtfg_cpp/src/ik_solver.cpp`
- 默认单点 IK 函数：`solveSinglePose()`
- 备选后端：`solveSinglePoseKdl()`，内部调用 `KDL::ChainIkSolverPos_LMA::CartToJnt()`
- 是否在 1300 点循环内：是，`solveTrajectory()` 会逐目标点求解
- 是否重复初始化对象：部分是。当前每个目标点都要构造候选、权重阶段、seed 阶段；如果启用 KDL 备选后端，会在每次调用中构造 KDL solver
- 是否调用 ROS service/action/topic：否
- 是否可能成为耗时瓶颈：高，是当前最主要的耗时来源之一

#### 碰撞检测

- 文件：`assembly_rtfg_cpp/src/collision_checker.cpp`
- 函数：`evaluateConfiguration()`
- 是否在 1300 点循环内：是。`solveTrajectory()` 在每个候选上做延迟碰撞检查，在回放轨迹上做稀疏二次审计
- 是否重复初始化对象：不是每次完全重建。`thread_local` 缓存了机器人与料斗的 FCL object，避免重复 BVH 构建，但每次仍要更新 transform 和做 distance
- 是否调用 ROS service/action/topic：否
- 是否可能成为耗时瓶颈：高，尤其是 pairwise distance 和 tool-basin 检测

#### 轨迹规划 / 回放

- 文件：`assembly_rtfg_cpp/src/trajectory_solver.cpp`
- 函数：`solveTrajectory()`
- 具体行为：把 anchor 间的轨迹用 quintic blend 展开成 playback 轨迹
- 是否在 1300 点循环内：是，回放点比锚点更多
- 是否重复初始化对象：会分配 `playback_q_list`、`playback_segment_names`
- 是否调用 ROS service/action/topic：否
- 是否可能成为耗时瓶颈：中等，通常低于 IK 和碰撞，但在点数很大时仍可见

#### 时间参数化

- 文件：`assembly_rtfg_cpp/src/rtfg_solver_node.cpp`
- 函数：`makeJointTrajectory()`
- 具体行为：固定 `dt = 0.02`，为每个点写入 `time_from_start`
- 是否在 1300 点循环内：是
- 是否重复初始化对象：会构造 `trajectory_msgs::msg::JointTrajectory`
- 是否调用 ROS service/action/topic：否
- 是否可能成为耗时瓶颈：低到中等，属于轻量固定步长参数化，不是 MoveIt TOTG

#### 发送控制器

- 文件：`assembly_rtfg_cpp/src/rtfg_solver_node.cpp`
- 函数：`onExecuteCached()`
- 具体行为：向 `/rtfg/joint_trajectory_controller/follow_joint_trajectory` 发送 `FollowJointTrajectory` action goal
- 是否在 1300 点循环内：否，只发送一次
- 是否重复初始化对象：每次执行会构造一个 goal
- 是否调用 ROS service/action/topic：是，action
- 是否可能成为耗时瓶颈：低。它会有等待 action server 的开销，但不是 1300 点总耗时的主因

---

## 4. 错误用法检查

### 4.1 是否 1300 个点每个点都调用 `move_group.plan()`

结论：**否**

证据：

- `assembly_rtfg_cpp` 内未搜索到 `move_group.plan`、`plan(`、`computeCartesianPath()`、`setFromIK()`、`getPositionIK()`。
- 当前 solver 节点没有通过 MoveIt 规划 API 逐点规划。

### 4.2 是否 1300 个点每个点都调用 `execute()`

结论：**否**

证据：

- `execute_cached` 只在最近一次 `fit_preview success=true` 后调用一次。
- 发送的是一个 `FollowJointTrajectory` action goal，不是每点一次 execute。

### 4.3 是否在循环内反复创建 `MoveGroupInterface`、`RobotModelLoader`、`PlanningScene`、IK solver

结论：**MoveIt 相关对象未使用；KDL solver 可能在备选后端里按调用构造，但默认主链不是 MoveIt**

证据：

- `MoveGroupInterface`、`RobotModelLoader`、`PlanningScene` 在 `assembly_rtfg_cpp` 中未找到。
- `RobotModel` 在 `rtfg_solver_node` 构造时加载一次并复用。
- 默认 IK 主链是 `solveSinglePose()` 的数值迭代；`solveSinglePoseKdl()` 目前是备选后端，不是当前主路径。

### 4.4 是否每个点都 `lookupTransform()`

结论：**否**

证据：

- `assembly_rtfg_cpp` 中未找到 `lookupTransform`。
- 位姿变换是通过 Eigen / 自定义矩阵函数完成。

### 4.5 是否每个点都进行碰撞检测

结论：**是，但方式是“候选级延迟碰撞 + 回放稀疏审计”**

证据：

- `solveTrajectory()` 对每个 IK 候选在满足误差门限后调用 `evaluateConfiguration()`。
- 回放阶段还会按 stride 做稀疏碰撞复检。

### 4.6 是否每个点都大量 `RCLCPP_INFO`

结论：**不是**

证据：

- 现有 `rtfg_solver_node.cpp` 原本只有启动时一条 `RCLCPP_INFO`。
- 本次诊断新增的是每次 service 调用结束后的汇总 profile 行，不是每个点输出。

### 4.7 是否 Debug 编译而不是 Release 编译

结论：**当前构建缓存显示为 Release**

证据：

- `build/assembly_rtfg_cpp/CMakeCache.txt` 中 `CMAKE_BUILD_TYPE:STRING=Release`
- `CMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG`

### 4.8 是否 IK timeout 或 planning time 设置过大

结论：**MoveIt planning time 不是当前主问题，因为主链路没有调用 MoveIt plan**

证据：

- `assembly_rtfg_cpp` 内未找到 `setPlanningTime()` / `setNumPlanningAttempts()`
- 当前 solver 的超时/迭代控制在 `SolverConfig::max_iterations` 和数值迭代逻辑内

### 4.9 是否没有使用上一点关节角作为下一点 IK 初值

结论：**没有问题，当前明确使用了上一点关节角**

证据：

- `trajectory_solver.cpp` 中，`q_prev` 会在每个成功点后更新为当前 `best_safe.q`
- 下一点的候选搜索从 `q_prev`、`home_q`、`zero`、wrap seed、global seed 逐级展开

---

## 5. 当前代码里真正可能慢的地方

### 5.1 主慢点 1：逐点 IK 迭代

`solveSinglePose()` 在每个目标点内做多轮数值迭代：

- 正向运动学
- 数值雅可比
- DLS 解线性方程
- clamp / align / stagnation 判断

这部分是最典型的 1300 点累计耗时来源。

### 5.2 主慢点 2：候选级 FCL 距离检测

`evaluateConfiguration()` 会：

- 遍历 robot collision link pair
- 遍历 tool 与 basin box
- 每次都做 FCL distance

即使做了 `thread_local` 复用，pairwise distance 本身仍然贵。

### 5.3 主慢点 3：回放轨迹与消息打包

`makeActualPoseArray()` 会对每个回放点做一次 FK，再包装成 `PoseArray`。  
这不是 TF lookup，但它是“1300 点级别的逐点 FK 包装”，容易被误认为是外部规划慢。

### 5.4 主慢点 4：MoveIt 只在 launch 中出现，不在主求解中出现

因此：

- `move_group` 不是当前 13 秒的主因
- `computeCartesianPath()` / `plan()` / `setFromIK()` 也不是当前实现路径

---

## 6. 诊断结论

### 6.1 当前 13 秒主要耗时在哪里

主要在 **自研 IK 求解 + 碰撞检测**，其次是 **回放轨迹生成和消息包装**。  
不是 MoveIt 的 `plan()` 链路。

### 6.2 当前到底是“纯 IK 慢”还是“MoveIt 规划链路慢”

结论：**当前主要是纯 IK + 碰撞慢，不是 MoveIt 规划慢。**

### 6.3 是否适合改成“笛卡尔轨迹点 → 批量 IK → JointTrajectory → 一次性发送 scaled_joint_trajectory_controller”

结论：**适合，而且当前架构本质上已经接近这个模式。**

说明：

- 当前已经是“轨迹点生成 -> 批量 IK -> `JointTrajectory` -> action 发送”
- 真正可以进一步优化的是：
  - 降低每点 IK 候选数量
  - 降低碰撞距离调用次数
  - 减少 `PoseArray` / `JointTrajectory` 的包装成本
  - 必要时增加摘要模式，避免把所有诊断数据都在主路径里构造出来

### 6.4 是否建议改用 `ur_kinematics` / `IKFast` / `TRAC-IK`

结论：

- **如果目标是继续明显提速**：优先考虑 `IKFast` 或 `ur_kinematics` 这类解析 IK
- **如果目标是更稳的替换**：`TRAC-IK` 可以作为兼容性更强的替代，但不一定比解析 IK 更快
- 当前默认路径是数值 IK，继续压缩时，解析 IK 的收益通常更大

### 6.5 最小修改方案

第一阶段不要重构全部工程，建议只做三件事：

1. 保留当前算法路径，只把 profile 输出补齐
2. 先看 profile 报告，确认主耗时是否真在 IK / collision / packaging
3. 再决定是否替换 IK 后端或缩减 collision 审计

---

## 7. 当前已加入的计时点

本次代码已加入如下 `[PROFILE]` 汇总输出：

- `轨迹点生成`
- `TF查询`
- `主求解器`
- `单点IK平均`
- `1300点IK总耗时`
- `碰撞检测`
- `MoveIt plan`
- `时间参数化`
- `ROS消息打包`
- `总耗时`
- `发送JointTrajectory`

这些日志是诊断层输出，不改变主流程。

---

## 8. 参考文件

- [rtfg_solver_node.cpp](/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp/src/rtfg_solver_node.cpp)
- [trajectory_solver.cpp](/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp/src/trajectory_solver.cpp)
- [ik_solver.cpp](/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp/src/ik_solver.cpp)
- [collision_checker.cpp](/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp/src/collision_checker.cpp)
- [trajectory_generator.cpp](/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp/src/trajectory_generator.cpp)
- [rtfg_sim.launch.py](/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp/launch/rtfg_sim.launch.py)

