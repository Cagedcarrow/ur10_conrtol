# 本次任务总结

## 1. 任务目标

将真实执行链收敛到 `ur_robot_driver`，解决 RViz `Plan` 成功但 `Execute` 失败问题。

## 2. 根因分析

1. `scaled_joint_trajectory_controller` action 不在线时，桥接节点必然返回 `Forward action not available`。
2. 真实执行链若同时依赖 TCP 姿态源与 driver 控制器，容易引入并发链路干扰与状态漂移。

## 3. 修改文件清单

- `ur10_real_pose_sync/launch/shovel_plan_execute.launch.py`

## 4. 新增功能/行为调整

1. `execution_mode:=real` 默认姿态源改为 `driver`。
2. `execution_mode:=real` 下 TCP 姿态节点不再默认可用，仅保留 `real_pose_source:=tcp_debug` 调试入口。
3. 保持 `allowed_start_tolerance=0.03`。
4. 保持桥接在线告警：forward action 离线时节流提示 Execute 必失败。

## 5. 核心实现逻辑

1. 真实执行主链：
   - `ur_robot_driver /joint_states` -> `ur10_joint_state_remap_node` -> `/assembly/joint_states`
   - MoveIt Execute -> `real_trajectory_bridge` -> `/scaled_joint_trajectory_controller/follow_joint_trajectory`
2. 默认不再启用 TCP30003 作为 real 执行姿态源，避免多链路并发。

## 6. 执行命令（标准两阶段）

```bash
# 阶段1：先起 driver（无头）
source /root/ur10_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10 \
  robot_ip:=10.160.9.21 \
  headless_mode:=true \
  launch_rviz:=false \
  initial_joint_controller:=scaled_joint_trajectory_controller

# 阶段2：确认 action 在线（必须有输出）
source /root/ur10_ws/install/setup.bash
ros2 action list | grep /scaled_joint_trajectory_controller/follow_joint_trajectory

# 阶段3：再起规划执行链
source /root/ur10_ws/install/setup.bash
ros2 launch ur10_real_pose_sync shovel_plan_execute.launch.py \
  execution_mode:=real \
  real_pose_source:=driver
```

## 7. 测试结果

1. `colcon build --packages-select ur10_real_pose_sync` 通过。
2. `execution_mode:=real real_pose_source:=driver` 启动冒烟通过：
   - `plan_exec_real_joint_state_remap_node` 启动；
   - `plan_exec_real_trajectory_bridge` 启动并输出 action 在线/离线告警。
3. 在 driver 未启动时，日志可明确报：
   - `No JointState received on /joint_states...`
   - `Forward action server is offline... Execute will fail...`

## 8. 剩余问题

1. 若 `ur_robot_driver` 本身报 `speed_slider_mask ... controlled by another RTDE client`，需先在机器人侧断开外部 RTDE 客户端。

## 9. 下一步建议

1. 先彻底解决 driver 的 RTDE 占用冲突，再进行 RViz `Plan+Execute` 验证。
2. 连续执行 10 次小位移 `Plan+Execute` 记录成功率。
