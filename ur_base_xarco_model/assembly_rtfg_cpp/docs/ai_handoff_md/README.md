# assembly_rtfg_cpp 项目说明与阶段成果总结

## 1. 功能包定位（这个包是做什么的）

`assembly_rtfg_cpp` 是把 MATLAB `realtime_trajectory_fit_gui` 主链路迁移到 ROS2 C++ 的独立功能包，目标是：

1. 复刻 MATLAB 版的轨迹生成与拟合求解能力；
2. 用 C++ 替代 MATLAB/MEX 运行时依赖，形成可服务化调用的实时求解节点；
3. 在 RViz/ROS2 体系内提供可视化、可执行、可审计的轨迹验证流程；
4. 为后续性能优化（IK、碰撞、响应开销）建立工程基线。

该包不替换现有 `assembly_rviz_tcp_control`，而是并行的“重构与性能演进”包。

---

## 2. 本次已完成工作（阶段性）

### 2.1 工程结构与模型迁移

- 新建了完整 ROS2 C++ 包结构（`src/include/srv/config/urdf/launch/benchmarks/docs`）。
- 使用 MATLAB 口径模型（包含 `sensor_shovel_tcp`、`base_jizuo` 体系）作为求解与碰撞基准。
- 服务接口、节点、配置和基准脚本都已落地。

### 2.2 核心算法迁移

- 轨迹生成：迁移 `left_wall_offset/mud_height/approach_len/theta_deg/depth/x_plane` 等参数驱动逻辑。
- 运动学拟合：迁移多阶段权重调度、多 seed 方案、连续性代价和等效关节对齐策略。
- 碰撞检测：迁移 tool-body / self / tool-basin 三类安全门控与审计。
- 回放生成：五次平滑插值 + 子步轨迹生成 + 稀疏碰撞复检。

### 2.3 服务化接口与执行链路

已稳定提供：

- `/rtfg/load_config`
- `/rtfg/fit_preview`
- `/rtfg/execute_cached`

并保证“仅在 `fit_preview success=true` 时更新可执行缓存”。

### 2.4 文档体系与可追溯性

- `docs/rtfg_ros2_cpp_技术文档` 已建立完整章节文档（当前为 HTML）。
- benchmark 可输出 JSON/CSV，支持 safe/diagnostic 双工况对比。
- 文档里已区分 MATLAB 历史基线、ROS2 当前实测、优化后实测与差异原因。

---

## 3. 当前能力边界（必须明确）

1. **无碰撞成功路径已具备**：safe 工况可 `success=true` 并执行缓存轨迹。
2. **失败诊断路径已具备**：近壁参数可返回 `success=false` 且给出碰撞对象/段信息。
3. **性能仍在持续优化中**：当前已具备优化框架与策略，但“全面超越 MATLAB MEX 历史最好值”仍需要下一轮深入优化。

---

## 4. 目录速览（建议先看）

- `src/rtfg_solver_node.cpp`：服务入口、缓存执行、消息发布
- `src/trajectory_solver.cpp`：主求解管线（IK→回放→审计）
- `src/ik_solver.cpp`：单点 IK 与 seed 策略
- `src/collision_checker.cpp`：FCL 碰撞/距离评估
- `src/trajectory_generator.cpp`：目标轨迹几何生成
- `config/environment_runtime_config*.yaml`：默认/安全参数
- `benchmarks/rtfg_benchmark.py`：基准脚本（输出 JSON/CSV）
- `launch/rtfg_sim.launch.py`：仿真启动入口

---

## 5. 这份 README 的用途

这份文件用于“先建立全局认知”：

- 让新接手者快速知道这个包解决什么问题；
- 明确已经完成到什么程度；
- 明确下一步工作应落在哪些文件和环节。

更细节的 AI 接手手册见：

- `docs/ai_handoff_md/AI_QUICK_ONBOARDING.md`

