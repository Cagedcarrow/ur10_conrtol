# assembly_rtfg_cpp 功能包完整文档

## 1. 整体介绍

`assembly_rtfg_cpp` 是 UR10 机械臂在 **ROS2 Humble** 环境下的**实时轨迹生成与碰撞检测**核心功能包。

### 背景

本功能包源自 MATLAB 原型项目，路径：
```
/home/liuxiaopeng/公共的/MATLAB_ur10_control/UR10_control_Matlab/realtime_trajectory_fit_gui/
```

该 MATLAB 原型（`main_realtime_trajectory_fit_gui.m`）的轨迹求解采用**双层架构**：

```
trackTrajectory (轨迹求解入口)
    │
    ├── 优先: MEX C++ 插件 (rtfg_solver_mex.mexa64)
    │   └── 自定义 DLS IK，C++ 实现，~9 秒完成 273 个目标点
    │
    └── 回退: MATLAB 纯 IK (MEX 不可用时)
        └── MATLAB built-in IK，降采样锚点 + 全目标 IK 重解，~317 秒
```

正常运行时走 MEX 路径，**约 9 秒即可完成求解且无姿态跳变**。只有 MEX 二进制缺失或崩溃时才会回退到纯 MATLAB 路径。

本功能包（`assembly_rtfg_cpp`）将 MEX C++ 的核心算法移植为独立的 ROS2 C++ 节点，并在 MEX 基础上增加了**分支跳变惩罚**、**边缘接受**、**IK 子步细分**、**连续预测**等改进。

**三种求解路径对比**：

| 层次 | IK 引擎 | 典型耗时 | 姿态跳变 | 碰撞 | 运行条件 |
|------|---------|:--------:|:--------:|:----:|---------|
| **MATLAB MEX**（默认） | 自定义 DLS C++ | **~9 s** | 无 | 无 | 需要 MEX 二进制 |
| **MATLAB 纯 IK**（回退） | MATLAB built-in IK | **~317 s** | **有** | **有** | MEX 不可用时 |
| **ROS2 C++**（本功能包） | 自定义 DLS C++ | **~10 s** | 无 | 无 | 独立部署 |

### 核心功能

- **轨迹求解**：接收目标位姿序列（approach → dipping → retract），通过 DLS IK 求解器为每个目标点计算机器人关节角
- **碰撞检测**：基于 FCL 0.7.0 库，对 self-collision、tool-body、tool-basin 三类碰撞进行精确距离计算
- **Playback 生成**：quintic 插值 + IK 子步（>20° 间隙触发），生成平滑连续轨迹
- **连续预测（Continuous Prediction）**：利用 `q_prev + dq_prev` 作为预测种子，大幅减少穷举搜索（约 85% 命中率）
- **分支跳变惩罚**：对距离 q_prev >30° 的候选施加 1000+ 惩罚分，防止 IK 求解切换到不同运动学分支
- **边缘接受（MARGINAL）**：在 clearance > 0mm 但 < 2mm 阈值时接受连续预测结果，防止因微小碰撞裕量不足导致分支跳变
- **GUI 控制**：提供 PyQt5 图形界面，支持一键环境准备、参数配置、轨迹求解与执行、可视化
- **仿真模式**：默认以仿真模式运行，无需真实机械臂；joint-state playback 在 RViz2 中显示运动

### 架构概览

```
ROS2 Service API
     │
     ▼
rtfg_solver_node (主节点，5 个 Service)
     │
     ├──► trajectory_solver.cpp   (轨迹求解主流程)
     │       ├── run_point_search  (单点 IK + FCL 筛选)
     │       ├── 连续预测 + 分阶段搜索
     │       └── playback 插值 + IK 子步
     │
     ├──► continuous_trajectory_solver.cpp (连续轨迹求解器)
     ├──► rolling_planner.cpp              (滚动规划器)
     ├──► ik_solver.cpp           (DLS IK 求解器)
     ├──► ik_backend.cpp          (IK 后端抽象层)
     ├──► collision_pipeline.cpp  (FCL 碰撞检测流水线)
     ├──► collision_checker.cpp   (FCL 碰撞检查器)
     ├──► robot_model.cpp         (URDF 模型加载与运动学)
     ├──► trajectory_generator.cpp (轨迹生成器)
     └──► utils.cpp               (continuityCost 等工具函数)
     
GUI (Python / PyQt5)
     │
     ├──► rtfg_launcher.py     → 5 按钮主界面 (MATLAB 风格流程)
     └──► ros_client.py        → ROS2 Service 通信封装
```

---

## 2. 节点信息图

### 节点列表

| 节点名称 | 类型 | 功能 |
|---------|------|------|
| `/rtfg_solver_node` | C++ (rclcpp) | 核心求解节点，提供 **5 个 Service** |
| `/rtfg_gui` (可选) | Python (PyQt5) | 图形界面控制节点 |

### Service API（5 个 Service）

| Service | 类型 | 功能 |
|---------|------|------|
| `/rtfg/load_config` | `LoadConfig.srv` | 加载 YAML 配置文件，返回场景参数和初始关节角 |
| `/rtfg/fit_preview` | `FitPreview.srv` | 执行完整轨迹求解（IK + FCL + Playback），缓存轨迹 |
| `/rtfg/execute_cached` | `ExecuteCached.srv` | 发布缓存的轨迹到 controller（真实模式）或启动 joint-state playback（仿真模式） |
| `/rtfg/move_to_start` | `std_srvs/Trigger` | 求解第一个目标位姿的 IK 并将机械臂移动到轨迹起始点（入泥点） |
| `/rtfg/move_to_home` | `std_srvs/Trigger` | 将机械臂平滑移动回 URDF 初始位姿（initial_q） |

### Topic 发布

| Topic | 类型 | 说明 |
|-------|------|------|
| `/rtfg/target_tcp_poses` | `geometry_msgs/PoseArray` | 目标 TCP 路径可视化（稀疏采样） |
| `/rtfg/tcp_path` | `geometry_msgs/PoseArray` | 实际 TCP 路径可视化（稀疏采样） |
| `/rtfg/collision_markers` | `visualization_msgs/MarkerArray` | 碰撞点球体标记（红色，半径 2.5cm） |
| `/rtfg/metrics` | `std_msgs/String` | 求解质量指标文本（锚点数、播放点数、clearance 等） |
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | MoveIt 兼容的规划路径显示 |
| `/joint_states` | `sensor_msgs/JointState` | 仿真模式下的关节状态发布（用于 RViz2 动画） |

### 数据流

```
GUI (Python)
   │
   ├── LoadConfig (yaml_path)
   │      │
   │      ▼
   │   Config loaded → 场景参数 + 初始关节角
   │
   ├── FitPreview (场景参数)
   │      │
   │      ▼
   │   1. 生成 273 个目标位姿 (approach + dipping + retract)
   │   2. 为每个目标求解 IK (连续预测 + 分阶段搜索 + 分支惩罚)
   │   3. 构建 anchor_q (所有目标点全解)
   │   4. Playback 插值 (quintic + >20° IK 子步)
   │   5. FCL 碰撞复检 (关键点强制 + 跳步策略)
   │   6. 返回轨迹 + 质量指标
   │
   ├── ExecuteCached
   │      │
   │      ├── [真实硬件] → FollowJointTrajectory action → robot controller
   │      │
   │      └── [仿真模式] → joint_state_pub_ (~50 Hz) → RViz2 动画
   │
   ├── MoveToStart
   │      │
   │      ▼
   │   求解第一个目标位姿的 IK → move_timer_ (~50 Hz quintic blend) → RViz2 动画
   │
   └── MoveToHome
          │
          ▼
       平滑插值回到 initial_q → move_timer_ (~50 Hz quintic blend) → RViz2 动画
```

---

## 3. 运动学求解算法

### 3.1 IK 求解器：Damped Least Squares (DLS)

采用基于数值雅可比矩阵的阻尼最小二乘法，在 `ik_solver.cpp` 中实现。与 MEX C++ 版本算法一致，但增加了 `dq_stop_threshold` 提前终止条件：

```cpp
for (int iter = 0; iter < max_iterations; ++iter) {
    Mat4 T = tipTransform(robot, q);           // 正向运动学
    Vec6 err = poseError(T, target);            // 位姿误差
    Vec6 weighted = err .* weights;             // 加权
    Mat6 J = numericJacobian(robot, q);         // 数值雅可比
    double lambda = adaptiveDamping(err);        // 自适应阻尼
    VectorXd dq = (J^T J + λI)^{-1} J^T * weighted;
    if (step_norm <= dq_stop_threshold) break;  // 提前终止（ROS2 特有）
    q = clampToLimits(robot, q + dq);           // 关节限位钳制
    q = alignToReference(robot, q, q_ref);      // 分支对齐
}
```

**关键参数对比**：

| 参数 | MEX C++ | ROS2 C++ |
|------|---------|---------|
| `max_iterations` | 60 | 60 |
| `position_tolerance` | 0.03 m | 0.03 m |
| `lambda` (自适应) | 相同 | 相同 |
| `stagnation_patience` | **8 次** | **3 次** |
| `dq_stop_threshold` | **无** | **1e-6**（有） |
| `stagnation_epsilon` | **无** | **1e-5**（有） |

### 3.2 权重调度（Weight Schedule）

4 级松弛策略，与 MATLAB 版本完全一致：

| 等级 | 位置权重 | 姿态权重 | 姿态容差 | 用途 |
|------|---------|---------|---------|------|
| Strict | [1,1,1] | [0.20,0.20,0.20] | 30° | 主搜索 |
| Relaxed | [1,1,1] | [0.10,0.10,0.10] | 45° | 第一后备 |
| Very Relaxed | [1,1,1] | [0.03,0.03,0.03] | 70° | 第二后备 |
| Position Only | [1,1,1] | [0,0,0] | inf | 最后一搏 |

### 3.3 候选排序与碰撞筛选

**通用流程（MEX / ROS2 一致）**：

1. **排序**：按 `cost = continuityCost` 升序排列
2. **FCL 检查**：对前 N 个候选执行 FCL 精确距离计算（全模式 max_collision_candidates_full=48，实时模式 max_collision_candidates_realtime=2）
3. **安全筛选**：只接受 clearance ≥ 2mm 的候选

**ROS2 C++ 特有增强**：

```
分支惩罚:
  if (step_from_prev > 30°) score += 1000.0 + step_from_prev
  
边缘接受:
  if (clearance > 0mm 但 < 2mm) → 接受（标记为 "tight_clearance"）
```

### 3.4 连续预测（Continuous Prediction）

核心优化策略，大幅减少计算时间：

```
对每个目标点:
  1. 预测种子: rebound_seed = clamp(q_prev + dq_prev)
  2. 尝试 {rebound_seed, q_prev} × 全部 4 级权重
  3. 如果 top-1 候选通过 FCL → 接受（连续预测命中，约 85% 的点）
  4. 如果 clearance < 2mm 但 > 0mm → 边缘接受
  5. 如果 clearance = 0 → 穷举搜索（多阶段后备）
```

**后备搜索阶段**：
1. **阶段 1**：`{q_prev, home_q, zero}` × 完整权重调度
2. **阶段 2**：J5/J6 wraps 扩展种子列表
3. **阶段 3**：全局随机种子（极少需要，最多 6 个）

### 3.5 Playback 插值

| 特性 | MATLAB MEX | ROS2 C++ |
|------|-----------|---------|
| 标准插值 | quintic blend (10t³-15t⁴+6t⁵) | 相同 |
| 最大段数 | 32 | **32（标准）/ 64（子步）** |
| 子步触发 | 无 | **joint_step > 20°** |
| 子步 IK | 无 | **slerp 位姿插值 + IK 求解** |
| 分支拒绝 | 无 | **IK 跳变 > 0.5 rad → 退回线性插值** |
| 锚点修正 | 无 | **子步终点回写 anchor_q** |
| 密度公式 | `ceil(joint_step / 0.70°)` | **`max(0.70°/seg + 3mm/seg + 0.30°/seg)`** |

> **密度公式说明**：ROS2 C++ 的 playback 密度公式已调整为与 MEX 一致，使用 `max(joint_step/0.70°, pos_step/3mm, rot_step/0.30°)` 三元复合准则，确保播放步长与 MEX 相当（从 9.91° 降至 ~4.81°）。

---

## 4. 相比 MATLAB 版本的优化之处

> **重要说明**：MATLAB 主程序 `main_realtime_trajectory_fit_gui.m` 默认调用的是 **MEX C++ 插件**（约 9 秒完成，无姿态跳变）。下面的 "MATLAB 纯 IK" 路径是仅当 MEX 不可用时的**回退路径**，非正常用户体验。ROS2 C++ 的对比基准是 **MATLAB MEX**。

### 4.1 MEX vs ROS2 C++ 性能对比（实测数据）

测试场景：273 个目标位姿，相同 UR10 模型和场景参数（left_wall_offset=0.195, depth=0.052, theta=-30°）。

| 指标 | MATLAB MEX（默认） | ROS2 C++ | 差异说明 |
|------|:----------------:|:--------:|---------|
| **总耗时** | **8.89 s** | **~10.0 s** | MEX 因无 ROS2 开销快 15% |
| IK 求解 | 5.61 s | 6.18 s | MEX 无子步 IK，略快 |
| 碰撞检测 | 2.36 s | 4.01 s | MEX stride=10 更激进 |
| 锚点数量 | 272 | **273** | 接近 |
| Playback 点数 | **1258** | **1357** | ROS2 子步增加点数 |
| **最大锚点步长** | 42.56° | **~4.81°** ✅ | **ROS2 分支惩罚 + 子步修正有效** |
| **最大播放步长** | **2.49°** ✅ | **~4.81°** | 密度公式匹配后已接近 |
| TCP 姿态跳变 | **0.735°** | 1.334° | MEX 播放更平滑 |
| **最小 clearance** | **2.640 mm** | **~0.49 mm** | ROS2 边缘接受导致裕量较小，但无碰撞 |
| **有无碰撞** | **无** ✅ | **无** ✅ | 均无碰撞 |
| **可实际运行** | ✅ | ✅ | 均可 |

### 4.2 关键架构差异

#### 4.2.1 IK 求解器（MEX vs ROS2）

两者使用相同的自定义 DLS C++ 算法，差异在于：

| 特性 | MATLAB MEX | ROS2 C++ |
|------|:---------:|:--------:|
| IK 引擎 | 自定义 DLS C++ | 自定义 DLS C++ |
| 分支对齐 | `alignToReference` | `alignToReference` |
| 代价函数 | `norm(dq) + 0.65*norm(dq - dq_prev)` | 相同 + **分支惩罚** |
| 停滞检测 | 8 次连续 | **3 次连续**（更早退出） |
| 提前终止 | 无 | **有**（dq_step ≤ 1e-6） |

> MATLAB 纯 IK 使用 MATLAB built-in `inverseKinematics`，与上两者完全不同。

#### 4.2.2 锚点求解策略

| 特性 | MATLAB MEX | ROS2 C++ |
|------|:---------:|:--------:|
| 锚点策略 | **全解（272 个）** | **全解（273 个）** |
| 降采样 | 无（MATLAB 侧不降采样直接送入 MEX） | 无 |
| 分支惩罚 | 无 | **有**（1000+ 惩罚分） |
| 连续预测 | 无（每次都穷举） | **有**（q_prev + dq_prev 预测） |

> **关于降采样**：MATLAB 纯 IK 路径为了减少 IK 调用次数而将 273 个目标降采样到 ~141 个锚点，但这导致相邻锚点间隔变大，IK 求解器在不同目标点之间切换运动学分支，产生 261° 的关节跳变。MEX 解决了这个问题——它不降采样，直接全解 272 个点。ROS2 C++ 延续了全解策略，并增加了分支惩罚进一步抑制跳变。

#### 4.2.3 Playback 生成

| 特性 | MATLAB MEX | ROS2 C++ |
|------|:---------:|:--------:|
| 方法 | quintic 插值 | quintic 插值 + **IK 子步** |
| 最大段数 | 32 | 32（标准）/ **64（子步）** |
| 子步触发 | 无 | **joint_step > 20°** |
| 子步 IK | 无 | slerp 位姿插值 + IK 求解 |
| 分支拒绝 | 无 | **IK 跳变 > 0.5 rad → 退回线性插值** |
| 锚点修正 | 无 | **子步终点回写 anchor_q** |

#### 4.2.4 碰撞检测

| 特性 | MATLAB MEX | ROS2 C++ |
|------|:---------:|:--------:|
| 锚点预筛选 | 延迟检查（deferred） | 延迟检查（deferred） |
| 复检策略 | **固定 stride=10** | **智能跳步**（关键点强制 + stride 可变） |
| 碰撞类型 | self, tool-body, tool-basin | 相同 |
| 边缘接受 | 无 | **有**（clearance > 0mm 接受） |

#### 4.2.5 代价函数

MEX 和 ROS2 使用相同的 MATLAB 兼容 `continuityCost`：

```
cost = norm(q - q_prev) + 0.65 * norm((q - q_prev) - dq_prev)
```

ROS2 C++ 额外增加了**分支惩罚**：

```
if (step_from_prev_deg > 30.0) score += 1000.0 + step_from_prev_deg
```

---

## 5. 本功能包 GUI 程序的功能

GUI 程序位于 `gui/scripts/rtfg_launcher.py`，基于 PyQt5 实现，版本 v3.0。

### 5.1 5 按钮 MATLAB 风格流程

GUI 提供了 5 个按钮，对应 MATLAB `main_realtime_trajectory_fit_gui.m` 的操作流程：

| 步骤 | 按钮 | 功能 | 对应 MATLAB 操作 |
|------|------|------|-----------------|
| **①** | 环境准备与验证 | 清理旧进程 → 启动求解器节点 → 检查服务 → 加载配置 | 初始化 + 参数加载 |
| **②** | RViz2 启动 | 拉起 RViz2 可视化窗口（加载 `rtfg_display.rviz` 配置） | 可视化显示 |
| **③** | 移动到入泥姿态点 | 求解第一个目标位姿的 IK，将机械臂平滑移动至轨迹起始点 | "移动到轨迹起始点" |
| **④** | 开始拟合并播放 | 运行轨迹拟合求解，完成后**自动播放**轨迹 | "尖端轨迹拟合" + "开始运行" |
| **⑤** | 返回初始姿态 | 平滑插值回到 URDF 初始位姿 | 复位到初始角度 |

**关键设计**：
- 所有按钮在流程中保持**可持续点击**，非一次性
- 步骤指示器实时显示当前完成状态（绿色=完成，蓝色=进行中，红色=失败）
- 异步 worker 线程避免阻塞 GUI

### 5.2 GUI 通信层

`gui/rtfg_gui/ros_client.py` 封装了 ROS2 Service 调用：

```python
class RosClient:
    def load_config(self, yaml_path="") → dict       # /rtfg/load_config
    def fit_preview(self, params, timeout_s=600.0) → dict  # /rtfg/fit_preview
    def execute_cached(self) → dict                   # /rtfg/execute_cached
    def move_to_start(self) → dict                    # /rtfg/move_to_start (Trigger)
    def move_to_home(self) → dict                     # /rtfg/move_to_home (Trigger)
    def get_topics(self) → list                       # 查询 /rtfg/ topic 列表
    def destroy(self)                                 # 清理节点
```

支持 `--sim` 仿真模式（默认）和 `--real` 真实硬件模式。

### 5.3 ROS2 环境初始化（Shell-launcher 机制）

`rtfg_launcher.py` 采用**shell-launcher 机制**确保 ROS2 环境正确加载：

```
main()
  │
  ├── _check_ros2_sourced()  → 检查当前进程是否已 source ROS2 humble
  │     │
  │     ├── 已 source → 直接启动 Qt 应用
  │     │
  │     └── 未 source → _launch_via_ros2_shell()
  │           │
  │           ├── 写入 .ros2_launcher.sh（source ROS2 + exec Python）
  │           ├── 通过 subprocess.Popen 启动子进程（新进程组）
  │           └── 当前进程 os._exit(0)（干净退出，避免 Qt/X11 状态损坏）
  │
  └── QApplication → RtfgControlPanel → 用户交互
```

**为什么不用 `os.execve()`**：`os.execve()` 替换进程映像时不调用 Qt 的 X11 清理函数，会导致 Qt/X11 通信状态不一致。Shell-launcher 方式通过干净的进程启动避免了此问题。

### 5.4 仿真模式

默认以仿真模式运行（`--sim`），无需连接真实机械臂：

- **move_to_start/move_to_home**：通过 `move_timer_`（50 Hz quintic blend）发布 `/joint_states`，在 RViz2 中显示平滑运动
- **execute_cached**：当真实机器人 controller 不可用时，自动回退到 `playback_timer_`（50 Hz）逐点发布 `/joint_states`，通过 `robot_state_publisher` + RViz2 动画显示

使用 `--real` 切换到真实硬件模式。

### 5.5 启动方式

```bash
# 启动核心求解节点
ros2 run assembly_rtfg_cpp rtfg_solver_node

# 启动 GUI（另一终端，默认仿真模式）
python3 assembly_rtfg_cpp/gui/scripts/rtfg_launcher.py

# 真实硬件模式
python3 assembly_rtfg_cpp/gui/scripts/rtfg_launcher.py --real
```

---

## 6. 耗时最多的地方，是如何用 C++ 优化的

### 6.1 性能瓶颈分析（ROS2 C++，273 目标点全解）

| 阶段 | 耗时 | 占比 |
|------|:----:|:----:|
| **IK 求解** | ~6.2 s | **~62%** |
| **碰撞检测（FCL）** | ~3.7 s | **~37%** |
| Playback 生成 | <0.1 s | <1% |

### 6.2 IK 求解优化

**瓶颈**：每个目标点需要尝试多个种子 × 4 级权重调度，每次 DLS 迭代需要计算雅可比矩阵和正运动学。

**优化手段**：

1. **连续预测（Continuous Prediction）** — 最关键优化（约 4× 加速）
   - 利用 `q_prev + dq_prev` 作为预测种子
   - 预测命中时只需 1 次 IK 求解（+ 1 次 FCL 检查）
   - 约 85%+ 的点通过连续预测命中
   - 避免了对 96 个随机种子 + J5/J6 wraps 的穷举搜索

2. **数值雅可比的高效计算**
   - 复用 `tipTransform` 的中间结果
   - 使用 Eigen 的模板化表达式，避免临时变量分配

3. **自适应阻尼**
   - 远距离（pos_err > 0.1）使用大阻尼（最大 0.1），快速收敛
   - 近距离使用小阻尼（0.0005），精确定位
   - 相比固定阻尼，迭代次数减少约 40%

4. **停滞检测 + 提前终止**
   - `stagnation_patience = 3`：连续 3 次无改善时提前停止
   - `dq_stop_threshold = 1e-6`：步长极小时提前终止
   - 相比 MEX 的 8 次停滞耐心，更早退出无效迭代

### 6.3 碰撞检测优化

**瓶颈**：FCL 距离计算涉及 BVH 遍历，每次调用约 0.1-0.5ms。

**优化手段**：

1. **延迟碰撞检查（Deferred Collision Check）**
   - 只在 IK 达到位置容差后才检查碰撞
   - 避免对发散或无解候选的 FCL 调用
   - 约减少 60% 的 FCL 调用

2. **智能跳步策略**
   - `shouldRunPreciseCheck()` 根据关键点/非关键点决定是否检查
   - 段边界点强制检查
   - 非关键点跳步（全模式 stride=7，实时模式 stride=5）
   - 约跳过 0-50% 的 playback 点（取决于模式）

3. **QuickCheck 轻量级检测**
   - 对跳过的 playback 点执行快速检查
   - 比较前后帧关节变化量
   - 无需 FCL 调用即可发现明显的轨迹异常

### 6.4 与 MATLAB（MEX）的耗时对比

> MATLAB 主程序默认使用 MEX C++ 插件，此处对比的是 ROS2 C++ 与 MATLAB MEX 的性能差异。下方也列出了 MATLAB 纯 IK 回退路径的数据作为参考——正常用户不会遇到此路径。

| 阶段 | MATLAB MEX | ROS2 C++ | 差异 |
|------|:---------:|:--------:|:----:|
| **总耗时** | **8.89 s** | **~10.0 s** | ROS2 慢 15%（ROS2 开销 + 更多功能）|
| IK 求解 | 5.61 s | 6.18 s | ROS2 有连续预测（快）+ 子步 IK（慢） |
| 碰撞检测 | 2.36 s | 4.01 s | ROS2 智能跳步检查更多点 |
| Playback 生成 | <0.1 s | <0.1 s | 接近 |

作为参考，MATLAB 纯 IK 回退路径（非正常路径）耗时 **316.72 s**，是 MEX 和 ROS2 的 **30-35 倍**。主要原因：
1. 使用 MATLAB built-in IK（解释执行，比 C++ DLS 慢 ~20 倍）
2. Playback 阶段对每个目标点执行完整 IK 求解（而非插值）
3. 碰撞检测覆盖每个 playback 点（而非跳步）

---

## 7. 性能对比（MATLAB MEX vs ROS2 C++）

### 7.1 速度对比

| 指标 | MATLAB MEX | ROS2 C++ | 差异 |
|------|:---------:|:--------:|:----:|
| **总耗时** | **8.89 s** | **~10.0 s** | MEX 快 15% |
| IK 求解 | 5.61 s | 6.18 s | MEX 快 10% |
| 碰撞检测 | 2.36 s | 4.01 s | MEX 快 70% |
| 求解点数 | 272 | 273 | 接近 |
| Playback 点数 | 1258 | 1357 | ROS2 子步更多 |

**为什么 MEX 比 ROS2 快**：

1. **零 ROS2 开销**：MEX 在 MATLAB 进程内直接调用 C++ 函数，无需 ROS2 Service 通信、序列化/反序列化
2. **碰撞跳步更激进**：MEX 使用固定 stride=10，ROS2 使用智能策略（关键点强制检查），ROS2 检查点更多
3. **无 Playback 子步**：MEX 只有 quintic 插值（无 IK 子步），ROS2 对 >20° 间隙执行额外 IK 求解
4. **停滞检测更宽松**：MEX 的 `stagnation_patience = 8` 意味着迭代更多次才退出；但 ROS2 多了 `dq_stop_threshold` 提前终止可在某些点上更快

### 7.2 质量对比

| 指标 | MATLAB MEX | ROS2 C++ | 更优 |
|------|:---------:|:--------:|:----:|
| 最大锚点步长 | **42.56°** | **~4.81°** | **ROS2**（惩罚 + 子步修正有效）|
| 最大播放步长 | **2.49°** | **~4.81°** | MEX（但仍平滑无跳变）|
| TCP 姿态跳变 | **0.735°** | 1.334° | MEX |
| 最小 clearance | **2.640 mm** | **~0.49 mm** | MEX（裕量更大）|
| 有无碰撞 | 无 | 无 | 平手 |
| 分支跳变惩罚 | 无 | **有** | ROS2 |
| 边缘接受 | 无 | **有** | ROS2 |
| IK 子步 | 无 | **有** | ROS2 |
| 锚点修正 | 无 | **有** | ROS2 |

### 7.3 关键差异解读

**分支稳定性**：ROS2 的 `max_anchor_step = ~4.81°` 显著优于 MEX 的 `42.56°`，这是因为 ROS2 的分支跳变惩罚（1000+）有效阻止了 IK 求解器在 tight-clearance 点切换到不同分支。子步 IK 求解后的锚点修正进一步压缩了有效步长。

**Playback 平滑度**：MEX 的 `max_playback_step = 2.49°` 优于 ROS2 的 `~4.81°`，但差距已通过密度公式匹配大幅缩小（从 9.91° 降至 4.81°）。ROS2 的 4.81° 步长仍然满足平滑运动要求。

**Clearance 差异**：MEX 的 `min_tool_basin = 2.64mm` 高于 ROS2 的 `~0.49mm`，因为 MEX 在 clearance < 2mm 时强制切换分支寻找更优解，而 ROS2 的边缘接受机制接受 tight-clearance 结果以保持分支稳定。

### 7.4 工程部署对比

| 方面 | MATLAB MEX | ROS2 C++ |
|------|:---------:|:--------:|
| 运行环境 | 需要 MATLAB + MEX 编译 | **纯 C++，独立部署** |
| ROS 集成 | 需额外适配层 | **原生 ROS2 节点** |
| 跨语言开销 | MATLAB ↔ C++ 数据编组 | **无（纯 C++）** |
| 可维护性 | MEX 调试困难 | **C++ 源码可读性强** |
| 许可限制 | 需要 MATLAB 商业许可 | **开源无限制** |
| 实时性 | 受 MATLAB GUI 线程影响 | **独立进程，资源可控** |

---

## 8. 可以用 CUDA 13.3 进行重构改写的地方

### 8.1 IK 求解并行化（P0，推荐优先实现）

**当前状态**：273 个目标点串行求解，每个点用 DLS 迭代。

```cpp
// 当前（串行）：
for (int i = 0; i < solve_points; ++i) {
    PointSearchResult search = run_point_search(targets[i], ...);
    anchor_q.row(i) = search.safe.q;
}
```

**CUDA 重构方案**：

```
每个目标点 → 一个 CUDA block
每个 block 内：
  - 每个线程计算雅可比矩阵的一行
  - warp-level 约简求解 H = J^T J + λI
  - shared memory 中存储 q、J、H
  - 所有线程同步迭代 DLS 步骤
```

**预期加速**：**5-10×**。理论加速比 30-50×，但 DLS 迭代内部有数据依赖，每步迭代仍需串行同步。

**关键挑战**：
- Eigen 在 CUDA 上支持有限，需要手写线性代数核
- LDLT 分解在 GPU 上实现复杂
- 分支对齐（`alignToReference`）需要条件分支逻辑
- 273 个目标点 × 6 DOF = 1638 个 float，数据量小，传输开销占比高

### 8.2 碰撞检测并行化（P1）

**当前状态**：约 4000 次 FCL 调用（1357 playback 点 × 3 类碰撞），每次调用遍历 BVH。

**CUDA 重构方案**：

```
每个 playback 点 → 一个 CUDA block
每个 block 内：
  - 为 robot 和 env 构建 GPU 友好的 SOA BVH
  - stackless BVH 遍历（避免递归）
  - 每个 warp 处理一个 primitive pair
  - atomicMin 记录全局最小距离
```

**预期加速**：**20-50×**。但 BVH 构建和上传到 GPU 有额外开销。

**关键挑战**：
- FCL 的 BVH 结构不是 GPU 友好的（多态虚函数、AABB 树）
- BVH 需要重构为 SOA 布局
- 机器人运动学更新（`tipTransform`）需在 GPU 上重新计算

### 8.3 CUDA Graph 捕获重复流程

CUDA 13.3 的 Graph API 可以捕获重复的 IK + FCL 流程：

```cpp
cudaGraphCreate(&graph, 0);
cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal);
for (int i = 0; i < solve_points; ++i) {
    ik_kernel<<<1, 32>>>(...);
    fcl_kernel<<<1, 256>>>(...);
}
cudaStreamEndCapture(stream, &graph);
cudaGraphInstantiate(&instance, graph);
// 对每帧轨迹可重复执行
cudaGraphLaunch(instance, stream);
```

### 8.4 推荐优先级

| 优先级 | 模块 | 加速比 | 难度 | 复杂度 | 建议 |
|--------|------|:------:|:----:|:------:|------|
| **P0** | IK 求解并行化 | 5-10× | 高 | 高 | 核心瓶颈，值得投入 |
| **P1** | 碰撞检测并行化 | 20-50× | 高 | 高 | 依赖 BVH GPU 化 |
| **P2** | 雅可比矩阵计算 | 2-3× | 中 | 中 | 可嵌入 IK 并行中 |
| **P3** | Playback 插值 | 微小 | 低 | 低 | 已 <0.1s，不需要 |
| **P3** | TCP 路径计算 | 微小 | 低 | 低 | 已 <0.1s，不需要 |

### 8.5 CUDA 13.3 具体工具建议

```cpp
// 1. 使用 cuda::std::mdspan 表达矩阵分片（C++23 风格）
#include <cuda/std/mdspan>

// 2. 使用 Cooperative Groups 实现 warp 级约简
#include <cooperative_groups.h>

// 3. 使用 cuBLAS batched DLS 求解
cublasDgetrfBatched(handle, n, Aarray, lda, ipiv, info, batchSize);

// 4. 使用 NVRTC 运行时编译 IK 核函数（参数可调）
nvrtcCreateProgram(&prog, src, "ik_solver.cu", 0, NULL, NULL);

// 5. 使用 CUDA Graph 捕获重复流程
cudaGraphCreate(&graph, 0);
cudaStreamBeginCapture(stream, cudaStreamCaptureModeGlobal);
// ... 执行 IK + FCL 核函数 ...
cudaStreamEndCapture(stream, &graph);
```

### 8.6 注意事项

1. **数据传输开销**：UR10 的 6-DOF 关节空间很小（6×273 = 1638 double），传输到 GPU 的开销远小于计算收益，关键是要在 GPU 上完成更多计算
2. **原子操作冲突**：多个 block 同时写入全局最小 clearance 时需使用 `atomicMin`
3. **动态并行**：CUDA 13.3 支持动态并行，可用于 IK 的层级收敛控制
4. **统一内存**：使用 `cudaMallocManaged` 简化 CPU-GPU 数据管理，但需注意页错误迁移开销
5. **调试工具**：`Nsight Compute` 对动态并行和 graph 的支持更加完善

---

## 附录 A：已知问题

### QTextCursor 元类型警告

**现象**：启动 GUI 时终端输出 `QMetaType::registerType: Cannot register type "QTextCursor"`，但 GUI 功能正常。

**原因**：Qt 的 accessibility bridge（QT_ACCESSIBILITY=1）和 IBus 输入法（QT_IM_MODULE=ibus）创建跨线程信号-槽连接时涉及 QTextCursor。由于 QTextCursor 未注册为 Qt 元类型（type id=0），Qt 无法在线程间编组，产生警告。

**状态**：这是 PyQt5/Qt 环境兼容性问题，非代码缺陷。警告仅出现在终端，不影响：
- 轨迹求解和机械臂运动
- GUI 按钮交互
- 日志输出（已移除 QTextCursor 用法，改用 `QTextEdit.append()`）

**建议**：如需消除警告，可设置环境变量 `QT_ACCESSIBILITY=0` 或使用 `QT_IM_MODULE=fcitx` 替代 ibus。

---

## 附录 B：文件结构

```
assembly_rtfg_cpp/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── environment_runtime_config.yaml      # 运行时配置（薄壁）
│   ├── environment_runtime_config_safe.yaml  # 安全配置（厚壁）
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── ompl_planning.yaml
│   └── ros2_controllers.yaml
├── launch/
│   └── rtfg_sim.launch.py
├── include/assembly_rtfg_cpp/
│   ├── types.h
│   ├── robot_model.h
│   ├── ik_solver.h
│   ├── ik_backend.h
│   ├── collision_checker.h
│   ├── collision_pipeline.h
│   ├── trajectory_solver.h
│   ├── trajectory_generator.h
│   ├── continuous_trajectory_solver.h
│   ├── rolling_planner.h
│   └── utils.h
├── src/
│   ├── rtfg_solver_node.cpp
│   ├── robot_model.cpp
│   ├── ik_solver.cpp
│   ├── ik_backend.cpp
│   ├── collision_checker.cpp
│   ├── collision_pipeline.cpp
│   ├── trajectory_solver.cpp
│   ├── trajectory_generator.cpp
│   ├── continuous_trajectory_solver.cpp
│   ├── rolling_planner.cpp
│   └── utils.cpp
├── gui/
│   ├── scripts/rtfg_launcher.py      # v3.0, 5 按钮 MATLAB 风格 GUI
│   ├── rtfg_gui/ros_client.py        # ROS2 Service 通信封装
│   └── setup.py
├── benchmarks/
│   ├── rtfg_benchmark.py             # 基准测试客户端
│   ├── rtfg_param_sweep.py           # 参数扫描脚本
│   ├── rtfg_ros2_cpp_benchmark_latest.json
│   ├── rtfg_ros2_cpp_benchmark_latest.csv
│   ├── rtfg_ros2_cpp_benchmark_safe.json
│   ├── rtfg_ros2_cpp_benchmark_safe.csv
│   ├── rtfg_ros2_cpp_benchmark_diagnostic.json
│   └── rtfg_ros2_cpp_benchmark_diagnostic.csv
├── urdf/
│   └── assembly_rtfg_solver.urdf
├── rviz/
│   └── rtfg_view.rviz
├── test/
│   └── rtfg_geometry_check.cpp
├── docs/
│   └── package_overview/
│       ├── README.md
│       └── assembly_rtfg_cpp_overview.md
└── srv/
    ├── LoadConfig.srv
    ├── FitPreview.srv
    └── ExecuteCached.srv
```
