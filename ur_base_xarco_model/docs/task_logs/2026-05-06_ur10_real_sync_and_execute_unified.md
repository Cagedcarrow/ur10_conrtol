# 本次任务总结

## 1. 任务目标

统一实现以下三项能力：
1. RViz2 机械臂实时读取真机姿态。
2. 真机移动时，RViz2 机械臂同步联动。
3. RViz2 规划后 `Execute` 可下发真实 UR10 控制器执行。

## 2. 根因分析

历史版本问题在于：
- 仅有虚拟执行链（执行只驱动 RViz，不驱动真机）。
- 状态源存在分叉，导致有时起始姿态回到 xacro 默认角而非真机实时角。

## 3. 修改文件清单

- `ur10_real_pose_sync/launch/shovel_plan_execute.launch.py`
- `ur10_real_pose_sync/ur10_real_pose_sync/real_trajectory_bridge.py`（新增）
- `ur10_real_pose_sync/setup.py`

## 4. 新增功能

1. 单入口双模式：
   - `execution_mode:=virtual`（仅虚拟执行）
   - `execution_mode:=real`（真实执行）
2. 真机姿态双来源：
   - 默认 `real_pose_source:=tcp`（TCP30003 直读真机）
   - 备选 `real_pose_source:=driver`（ur_robot_driver /joint_states 映射）
3. 真机执行桥：
   - `real_trajectory_bridge` 接收 MoveIt 轨迹
   - `ur10_*` 关节名映射到 UR 控制器关节名
   - 转发到 `real_controller_action`（默认 `/scaled_joint_trajectory_controller/follow_joint_trajectory`）

## 5. 核心实现逻辑

1. 状态总线统一为 `/assembly/joint_states`，`robot_state_publisher` 与 `move_group` 都只消费该话题，避免多源污染。
2. 真机模式下默认启动 TCP 姿态节点，确保 RViz 起始姿态即真机当前姿态。
3. MoveIt 仍向 `/joint_trajectory_controller/follow_joint_trajectory` 发送执行请求；桥接节点负责转发到真实控制器 action。

## 6. 执行命令

```bash
cd /root/ur10_ws
colcon build --symlink-install --packages-select ur10_real_pose_sync
source install/setup.bash

# 真实执行模式（默认 TCP 姿态 + scaled 控制器）
ros2 launch ur10_real_pose_sync shovel_plan_execute.launch.py \
  execution_mode:=real real_pose_source:=tcp

# 备选：driver 姿态源
ros2 launch ur10_real_pose_sync shovel_plan_execute.launch.py \
  execution_mode:=real real_pose_source:=driver \
  real_joint_states_input_topic:=/joint_states
```

## 7. 测试结果

1. `colcon build --packages-select ur10_real_pose_sync`：通过。
2. `xacro + check_urdf`：通过。
3. `execution_mode:=real real_pose_source:=tcp` 启动冒烟通过：
   - `plan_exec_real_pose_tcp_node` 启动并连接 30003
   - `plan_exec_real_trajectory_bridge` 启动
   - `move_group` 启动并可规划

## 8. 剩余问题

1. 真机最终执行成功与否仍取决于 `ur_robot_driver` 侧控制器是否 active（尤其 scaled 控制器）。
2. 若控制器 action 名不一致，需要通过 `real_controller_action` 覆盖。

## 9. 下一步建议

1. 在真实环境先确认 action：
   - `ros2 action list | grep follow_joint_trajectory`
2. 确认 scaled 控制器 active 后再做 RViz `Plan + Execute` 联调。
3. 若 Execute 返回失败，抓取 `real_trajectory_bridge` 日志中的 `error_string` 与 `error_code` 定位。
