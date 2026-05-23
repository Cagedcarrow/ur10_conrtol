# 通讯协议与 ROS 2 节点

## 通讯协议

- 物理链路：以太网
- ROS 2 控制层：`ur_robot_driver`
- 机器人底层协议：
  - RTDE
  - URScript
  - Dashboard

## 关键话题与 Action

- `/joint_states`
  - 来源：真实机械臂控制链
  - 用途：驱动 RViz2 / MoveIt2 当前姿态
- `/tf`
  - 来源：`robot_state_publisher`
  - 用途：发布装配模型 TF 树
- `/scaled_joint_trajectory_controller/follow_joint_trajectory`
  - 类型：`control_msgs/FollowJointTrajectory`
  - 用途：MoveIt2 Execute 到真实机械臂

## 关键节点

- `ur_ros2_control_node`
  - 官方 UR 控制核心
- `dashboard_client`
  - 示教器 Dashboard 通讯
- `joint_state_broadcaster`
  - 发布真实关节状态
- `scaled_joint_trajectory_controller`
  - 执行真实关节轨迹
- `move_group`
  - 规划与执行管理
- `robot_state_publisher`
  - 将关节状态转换为 TF
- `rviz2`
  - 可视化与交互规划入口
