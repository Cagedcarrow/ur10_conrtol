# 系统架构

```mermaid
graph TB
  subgraph Robot
    UR10["UR10 + External Control"]
  end

  subgraph Linux Host
    DRIVER["ur_robot_driver / ros2_control"]
    JSB["joint_state_broadcaster"]
    SCALED["scaled_joint_trajectory_controller"]
    RSP["robot_state_publisher"]
    MG["move_group"]
    RVIZ["rviz2"]
    GUI["real_control_gui.py"]
  end

  UR10 <-- RTDE / URScript / Dashboard --> DRIVER
  DRIVER --> JSB
  JSB -->|/joint_states| RSP
  JSB -->|/joint_states| MG
  RSP -->|/tf| RVIZ
  MG --> RVIZ
  MG -->|Execute| SCALED
  SCALED --> DRIVER
  GUI --> DRIVER
  GUI --> MG
  GUI --> RVIZ
```

## 运行原则

1. 真实姿态来源只能是官方 driver 发布的 `/joint_states`
2. 不能同时运行 `joint_state_publisher_gui` 或其他伪状态源
3. MoveIt2 只通过 `scaled_joint_trajectory_controller` 执行真实轨迹
4. 碰撞豁免按 `assembly_real.srdf` 的有限范围执行，不采用全局禁碰撞
