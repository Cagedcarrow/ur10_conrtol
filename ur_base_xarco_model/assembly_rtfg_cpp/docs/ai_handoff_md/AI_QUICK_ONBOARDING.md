# assembly_rtfg_cpp：AI 快速接手手册（算法/构建/验证完整版）

> 目标：让新的 AI 或开发者在最短时间内理解本项目的算法结构、运行方式、验证口径和改进方向，并能直接继续开发。

---

## 1. 项目一句话定义

`assembly_rtfg_cpp` 是 MATLAB `realtime_trajectory_fit_gui` 的 ROS2 C++ 重构版本，提供“轨迹生成 + IK 拟合 + 碰撞审计 + 轨迹执行”全链路服务化能力。

---

## 2. 功能总览（输入→处理→输出）

### 2.1 输入

1. 轨迹参数：
   - `left_wall_offset`
   - `mud_height`
   - `approach_len`
   - `theta_deg`
   - `depth`
   - `x_plane`
2. 环境位姿参数：
   - `pose_x/pose_y/pose_z`
   - `roll_deg/pitch_deg/yaw_deg`
3. 机械臂当前关节：
   - `current_q`（可空，空则走默认初值）
4. 安全阈值：
   - `clearance_threshold`（默认 2mm）

### 2.2 处理

1. `buildTargetPlan` 生成目标 TCP 路径；
2. 对每个目标点执行多 seed IK + 连续性筛选；
3. 对可行 IK 候选执行碰撞门控；
4. 生成回放轨迹并做稀疏碰撞复检；
5. 发布指标与可视化数据，按规则缓存可执行轨迹。

### 2.3 输出

1. `JointTrajectory`（回放轨迹）
2. 目标 TCP 与实际 TCP 路径
3. 碰撞对象、碰撞段、碰撞点信息
4. 耗时指标（总耗时、IK、碰撞）
5. 最小 clearance 指标

---

## 3. 核心算法结构（必须理解）

## 3.1 轨迹生成层（Trajectory Generator）

文件：`src/trajectory_generator.cpp`

作用：把参数化输入变成离散目标位姿序列（TCP 目标轨迹），并附加段名（如 approach / arc / exit）。

关键检查项（与 MATLAB 对齐）：

- `approach_start`
- `entry`
- `arc_end`
- `radius`
- `vertical_penetration`

这些值通过 `rtfg_geometry_check` 做毫米级一致性核对。

## 3.2 拟合求解层（Trajectory Solver + IK Solver）

文件：

- `src/trajectory_solver.cpp`
- `src/ik_solver.cpp`

核心机制：

1. 多阶段权重调度（姿态逐步放松）；
2. progressive seed 搜索：
   - `q_prev`
   - `home`
   - `zero`
   - 局部 wrap seed
   - 全局随机 seed（最后兜底）
3. 以连续性代价选择候选（避免抖动）；
4. 仅对 IK 达标候选做碰撞计算（延迟碰撞）；
5. 对“足够好候选”提前收敛（减少无效迭代）。

## 3.3 碰撞层（Collision Checker）

文件：`src/collision_checker.cpp`

碰撞类型：

1. self
2. tool_body
3. tool_basin

实现特点：

- FCL collision object 复用（避免重复建模）；
- 仅更新 transform + AABB；
- tool-basin 使用包络 box 距离；
- 输出最小距离与首个违规对象。

## 3.4 回放层（Playback）

文件：`src/trajectory_solver.cpp`

实现：

- anchor 点之间使用五次 blend 插值；
- 按关节步长/位姿步长控制子步数；
- 稀疏采样做二次碰撞审计（不是每点全检）。

---

## 4. ROS2 对外接口与行为约束

### 4.1 Services（稳定接口，不要随意改名）

1. `/rtfg/load_config`
2. `/rtfg/fit_preview`
3. `/rtfg/execute_cached`

### 4.2 缓存执行规则（重要）

- 只有 `fit_preview success=true` 时才写入可执行缓存。
- `execute_cached` 只提交最近一次成功轨迹。

### 4.3 Topics（可视化/观测）

- `/rtfg/target_tcp_poses`
- `/rtfg/tcp_path`
- `/rtfg/collision_markers`
- `/rtfg/metrics`
- `/display_planned_path`

---

## 5. 构建与运行（标准步骤）

## 5.1 构建

```bash
cd /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select assembly_rtfg_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 5.2 启动

```bash
ros2 launch assembly_rtfg_cpp rtfg_sim.launch.py launch_rviz:=false launch_moveit:=false
```

如果主机有其他 ROS 图干扰，建议隔离：

```bash
export ROS_DOMAIN_ID=66
```

---

## 6. 验收口径（第二阶段对应）

## 6.1 几何一致性

```bash
ros2 run assembly_rtfg_cpp rtfg_geometry_check
```

核对 `approach_start/entry/arc_end/radius/vertical_penetration`。

## 6.2 无碰撞成功（safe）

- 使用 `config/environment_runtime_config_safe.yaml`
- 期望：`fit_preview success=true`
- 且 `execute_cached success=true`

## 6.3 失败诊断回归（diagnostic）

- 使用默认配置或近壁参数
- 期望：`fit_preview success=false`
- 返回碰撞对象、碰撞类型、清晰失败信息

## 6.4 benchmark 产物

脚本：

- `benchmarks/rtfg_benchmark.py`

输出：

- `rtfg_ros2_cpp_benchmark_safe.json/.csv`
- `rtfg_ros2_cpp_benchmark_diagnostic.json/.csv`

---

## 7. 当前已知问题与常见坑

1. 同机若同时运行其它 MoveIt/控制器 launch，可能导致服务图混杂，出现“服务偶发不可见”。
2. benchmark 数字受机器负载影响较大；单次结果不能作为唯一口径。
3. 当前版本强调“稳定复刻 + 可解释审计”，并非极限速度版本。

---

## 8. 下一步优化建议（优先级顺序）

1. IK 侧：进一步减少候选冗余，增强早停条件；
2. 碰撞侧：对 pair 进行更细粒度白名单/剪枝，降低 distance 调用次数；
3. 审计侧：按工况分级采样 stride（safe 更大、diagnostic 更细）；
4. 响应侧：保留 full debug，同时增加摘要返回模式减少序列化开销；
5. 建立多次重复 benchmark（N 次均值+方差）作为文档主数据源。

---

## 9. 关键文件清单（接手先读）

1. `src/rtfg_solver_node.cpp`
2. `src/trajectory_solver.cpp`
3. `src/ik_solver.cpp`
4. `src/collision_checker.cpp`
5. `src/trajectory_generator.cpp`
6. `config/environment_runtime_config.yaml`
7. `config/environment_runtime_config_safe.yaml`
8. `benchmarks/rtfg_benchmark.py`
9. `docs/rtfg_ros2_cpp_技术文档/05_Cpp性能基准与优化记录.html`

---

## 10. 给下一位 AI 的执行建议（直接可用）

1. 先在隔离 `ROS_DOMAIN_ID` 下跑一轮完整验收；
2. 保存 safe/diagnostic 两组 JSON；
3. 每做一轮代码优化，只改一个优化点并复测，保证可归因；
4. 文档更新时保持“三层口径”：
   - MATLAB 历史基线（主对照）
   - ROS2 当前实测
   - ROS2 优化后实测
5. 若结果波动大，先排查环境并发，不要先改算法。

