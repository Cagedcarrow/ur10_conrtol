# assembly_rviz_tcp_control

This package launches `RViz2 + MoveIt2 + ros2_control fake hardware` for the assembly robot and exposes the TCP interactive marker at `sensor_shovel_shovel_tcp`.

## Build

```bash
cd /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model
colcon build --symlink-install
```

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/liuxiaopeng/ws_moveit2/install/setup.bash
source /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/install/setup.bash
ros2 launch assembly_rviz_tcp_control assembly_rviz_tcp_control.launch.py
```

## Expected behavior

- `robot_state_publisher`, `ros2_control_node`, `joint_state_broadcaster`, and `joint_trajectory_controller` start on fake hardware.
- MoveIt launches with planning group `assembly_manipulator`.
- In RViz2, drag the end-effector interactive marker at `sensor_shovel_shovel_tcp`, then use `Plan` and `Execute`.
