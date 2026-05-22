# 本次任务总结

## 1. 任务目标
- 打通 UR10 参数化实验链路：`Validate -> Plan Preview -> Execute`。
- 新增 RViz2 GUI 面板，实时设置 `切入深度/速度/切入角` 并写回单一 YAML。
- 新增基于 `assembly_xacro` 的 FK 计算工具，输出 `^{base_jizuo}T_{sensor_shovel_shovel_tcp}` 矩阵与论文 LaTeX 公式。

## 2. 根因分析
- 原链路以 CSV 回放为主，缺少参数化服务接口与 GUI 门禁。
- 原系统没有统一参数入口文件，实验变量不可控且不易复现实验。
- FK 缺少从 `Act_q0..Act_q5` 直接批量计算 TCP 姿态矩阵的工具链。
- 启动链路中 Gazebo/MoveIt 时序抖动会导致后端在 MoveIt 未就绪时初始化失败。

## 3. 修改文件清单
- `assembly_parametric_motion/`（新包）
  - `package.xml`
  - `CMakeLists.txt`
  - `srv/ManageExperimentConfig.srv`
  - `srv/ValidatePlan.srv`
  - `srv/PreviewPlan.srv`
  - `srv/ExecuteCachedPlan.srv`
  - `src/parametric_motion_server.cpp`
- `assembly_rviz_param_panel/`（新包）
  - `package.xml`
  - `CMakeLists.txt`
  - `plugin_description.xml`
  - `include/assembly_rviz_param_panel/parametric_motion_panel.hpp`
  - `src/parametric_motion_panel.cpp`
  - `rviz/parametric_view.rviz`
- `assembly_description/`
  - `config/parametric_experiment.yaml`（新增）
  - `launch/parametric_gui.launch.py`（新增）
  - `launch/assembly_bringup.launch.py`（新增 `mode:=parametric_gui`）
  - `launch/gazebo_moveit.launch.py`（增加 `rviz_config` 透传）
  - `scripts/fk_tcp_matrix_from_xacro.py`（新增）
- `assembly_moveit_config/`
  - `launch/assembly_moveit.launch.py`（增加 `rviz_config` 参数）

## 4. 新增功能
- 参数化后端服务（`assembly_parametric_motion`）
  - `/assembly/config/load_save`
  - `/assembly/plan/validate`
  - `/assembly/plan/preview`
  - `/assembly/plan/execute`
- 单 YAML 配置落地：`parametric_experiment.yaml`。
- 规划门禁：
  - 未通过 Validate 禁止 Preview。
  - 未有成功 Preview 禁止 Execute。
  - Execute 仅执行最近一次成功预览缓存轨迹。
- RViz Panel（`assembly_rviz_param_panel/ParametricMotionPanel`）：
  - 参数输入、实验 ID 下拉、Load/Save/Validate/Preview/Execute、状态反馈。
- FK 工具输出：
  - `tcp_fk_matrices.csv`
  - `fk_vs_act_pose_error.csv`
  - `fk_summary.md`
  - `fk_formula.tex`

## 5. 核心实现逻辑
- 参数化轨迹在 `sensor_shovel_shovel_tcp` 局部坐标系生成分段路径：`approach -> contact -> penetrate -> cut -> lift+retreat`。
- `entry_angle_deg` 控制局部 XOY 切入方向；`penetration_depth_mm` 映射局部 `-Z`；`speed_setting` 映射速度/加速度缩放。
- `validate` 和 `preview` 共用规划计算逻辑，且 `preview` 仅在最近一次 `validate` 同参数通过后开放。
- 后端改为“延迟就绪”：
  - 节点启动后周期尝试同步 `/move_group` 参数并初始化 `MoveGroupInterface`，避免启动时序抖动直接退出。
- FK 工具解析 xacro 时对孤立非法字符 `+` 做只读过滤，不修改原模型语义；按关节链逐样本累乘齐次变换矩阵。

## 6. 执行命令
- 构建：
```bash
cd /root/ur10_ws
colcon build --symlink-install --packages-select assembly_description assembly_moveit_config assembly_parametric_motion assembly_rviz_param_panel
```
- 模型检查：
```bash
source /root/ur10_ws/install/setup.bash
xacro /root/ur10_ws/src/ur_base_xarco_model/assembly_description/urdf/assembly.urdf.xacro > /tmp/assembly_test.urdf
check_urdf /tmp/assembly_test.urdf
```
- FK 工具验证：
```bash
python3 /root/ur10_ws/src/ur_base_xarco_model/assembly_description/scripts/fk_tcp_matrix_from_xacro.py \
  --csv /root/ur10_ws/src/experiment_data_recorder/data/2026-05-01_19-57-41/ur10_ft300_realtime_data.csv \
  --xacro /root/ur10_ws/src/ur_base_xarco_model/assembly_xacro/assembly/assembly.urdf.xacro \
  --output-dir /tmp/fk_parametric_test
```
- 启动（参数化模式）：
```bash
ros2 launch assembly_description assembly_bringup.launch.py mode:=parametric_gui gazebo_gui:=false launch_rviz:=true
```

## 7. 测试结果
- ✅ `colcon build`：四包构建通过。
- ✅ `xacro + check_urdf`：`assembly_description/urdf/assembly.urdf.xacro` 可解析。
- ✅ FK 工具：四类输出文件生成成功，`det(R)` 与 `R*R^T` 指标正常。
- ✅ 在一次稳定启动中验证到：
  - `/assembly/config/load_save`
  - `/assembly/plan/validate`
  - `/assembly/plan/preview`
  - `/assembly/plan/execute`
  - `/joint_trajectory_controller/follow_joint_trajectory`
  - `joint_state_broadcaster` 与 `joint_trajectory_controller` 均为 `active`
- ⚠️ 环境存在 Gazebo 启动抖动（`spawn_entity`/`/spawn_entity` 偶发不可用），会导致 move_group 未拉起、后端保持等待状态。
- ⚠️ 目前在默认初始姿态下，`cartesian_fraction` 可能低于阈值（0.90），需根据现场姿态/环境调 `waypoint` 几何或阈值。

## 8. 剩余问题
- SRDF 中 end-effector parent group 有历史警告（不阻塞本次实现，但建议后续清理）。
- Gazebo Classic 进程在部分中断路径下退出不干净，建议统一启动前清理脚本。
- FK 与 `Act_X/Y/Z + Act_RX/RY/RZ` 的对比误差较大，当前更适合作为“模型链路矩阵导出工具”，误差物理语义需结合坐标定义再解释。

## 9. 下一步建议
- 在 `parametric_gui` 启动链路中加入更严格的时序门控（例如：检测 `/spawn_entity` 成功后再启 move_group 和后端）。
- 增加后端“分段 pose 规划回退模式”（当 Cartesian fraction 低于阈值时自动回退）。
- 在 RViz 面板增加 “加载当前 active 参数并自动 Validate” 快捷流程。
- 对 FK 工具补充“坐标系对齐开关”（例如 Act pose 的坐标系/单位归一化参数），用于论文误差分析段。
