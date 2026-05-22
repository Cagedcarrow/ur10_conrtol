# ur10_conrtol

这是一个基于 ROS 2 的 UR10 装配工作区，当前主要内容在 `ur_base_xarco_model/`，包含：

- `assembly_description`：机器人描述、URDF/Xacro、控制器与启动文件
- `assembly_moveit_config`：MoveIt2 规划配置
- `assembly_rviz_tcp_control`：RViz2 + MoveIt2 + fake hardware 的 TCP 交互控制入口

## 构建

```bash
cd /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model
source /opt/ros/humble/setup.bash
source /home/liuxiaopeng/ws_moveit2/install/setup.bash
colcon build --symlink-install
```

## 启动 TCP 交互控制

```bash
source /opt/ros/humble/setup.bash
source /home/liuxiaopeng/ws_moveit2/install/setup.bash
source /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/install/setup.bash
ros2 launch assembly_rviz_tcp_control assembly_rviz_tcp_control.launch.py
```

## 本次姿态跳动问题总结

“运动到目标点后，在目标姿态和初始姿态之间来回跳动”的根因不是规划失败，也不是控制器本体异常，而是 **`/joint_states` 被多个节点同时发布**。

当时同时存在：

- `joint_state_broadcaster`
- `joint_state_publisher_gui`
- 另一套旧的显示/控制 launch 残留进程

结果是 MoveIt 和 RViz 一边收到 fake hardware 的当前关节状态，一边又收到旧链路发出的初始关节状态，最终表现为姿态在两套状态之间反复覆盖。

最终修复方式：

1. 只保留一套 `assembly_rviz_tcp_control` 启动链
2. 关闭 `display.launch.py` 和 `joint_state_publisher_gui`
3. 确保 `/joint_states` 只有一个发布者：`joint_state_broadcaster`

## 备注

- `assembly_description/urdf/assembly.urdf.xacro` 已支持 `hardware_backend:=gazebo|fake`
- `assembly_moveit_config/srdf/assembly.srdf` 已按 TCP 末端进行规划配置
