# 本次任务总结

## 1. 任务目标
修复 UR10 真机执行链，使 RViz2/MoveIt2 使用真实机械臂当前姿态作为初始状态，并确保 Execute 可通过真实控制器 action 下发。

## 2. 根因分析
1. 真机执行主阻塞是 `ur_robot_driver` 未成功接管，单独启动时稳定复现：
   - `Variable 'speed_slider_mask' is currently controlled by another RTDE client`
2. 在 driver 未接管时：
   - `/joint_states` 无发布
   - `/scaled_joint_trajectory_controller/follow_joint_trajectory` 不存在
   - MoveIt Execute 必然失败（桥接日志：Forward action server offline）
3. 启动默认值曾在 virtual 模式，容易让 RViz 初始姿态落到模型/虚拟链而非真机态。

## 3. 修改文件清单
1. `/root/ur10_ws/src/ur10_real_pose_sync/launch/shovel_plan_execute.launch.py`
2. `/root/ur10_ws/src/ur_base_xarco_model/assembly_moveit_config/launch/assembly_moveit.launch.py`
3. `/root/ur10_ws/scripts/diagnose_ur10_moveit_execution.sh`

## 4. 新增功能
1. `shovel_plan_execute` 默认切换为 `execution_mode:=real`，默认 `real_pose_source:=driver`。
2. 启动顺序拆分为两段：
   - 先起状态/执行链（remap/bridge/tcp_debug or virtual）
   - 后起 RSP + move_group + RViz
3. 诊断脚本 RTDE 判定改为检查最新日志目录，避免历史日志长期误判。

## 5. 核心实现逻辑
1. 真实模式只依赖 driver 的 `/joint_states` 进入 `/assembly/joint_states`。
2. MoveIt 仍对接 `joint_trajectory_controller`，由 `real_trajectory_bridge` 映射到真实控制器 action：
   `/scaled_joint_trajectory_controller/follow_joint_trajectory`。
3. `allowed_start_tolerance` 在 MoveIt 启动配置中统一到 `0.03` 以减轻轻微状态漂移导致的拒绝执行。

## 6. 执行命令
```bash
# 连通检查
ping -c 4 10.160.9.21
for p in 29999 30001 30002 30003; do nc -vz -w 2 10.160.9.21 $p; done

# 单独启动 driver（验证接管）
source /root/ur10_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10 robot_ip:=10.160.9.21 headless_mode:=true launch_rviz:=false \
  initial_joint_controller:=scaled_joint_trajectory_controller

# 一键诊断
bash /root/ur10_ws/scripts/diagnose_ur10_moveit_execution.sh 10.160.9.21 /root/ur10_ws
```

## 7. 测试结果
1. `colcon build --symlink-install --packages-select ur10_real_pose_sync` 通过。
2. `xacro + check_urdf` 通过。
3. `shovel_plan_execute.launch.py --show-args` 确认默认 real/driver。
4. 在 driver 未接管场景，`shovel_plan_execute` 能明确报：
   - remap 无 `/joint_states`
   - bridge forward action offline
5. 诊断脚本当前输出：`DRIVER_NOT_RUNNING`（未起 driver 时）。

## 8. 剩余问题
机器人端仍存在 RTDE 资源占用，需要先解除 `speed_slider_mask` 冲突，否则真实 Execute 无法成功。

## 9. 下一步建议
1. 先在示教器/其它上位机侧清理 RTDE 占用，再单独验证 driver 正常接管。
2. 确认 `scaled_joint_trajectory_controller` active 且 action 在线后，再启动 `shovel_plan_execute` 做小幅 Plan+Execute。
