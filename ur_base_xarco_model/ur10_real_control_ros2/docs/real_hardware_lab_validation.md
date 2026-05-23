# 实验室实机联调步骤

## 1. Linux 主机网卡

目标地址：

- 主机：`10.160.9.100`
- 机器人：`10.160.9.21`

示例手动配置：

```bash
ip -4 addr show
sudo ip addr flush dev <你的有线网卡名>
sudo ip addr add 10.160.9.100/24 dev <你的有线网卡名>
sudo ip link set <你的有线网卡名> up
ping 10.160.9.21
```

如果使用 NetworkManager，也可以用：

```bash
nmcli connection show
nmcli connection modify "<连接名>" ipv4.addresses 10.160.9.100/24 ipv4.method manual
nmcli connection up "<连接名>"
```

## 2. 机器人示教器

1. 确认机器人 IP 为 `10.160.9.21`
2. 安装并启用 `External Control` URCap
3. 在示教器程序树中运行 `External Control`
4. External Control 回连地址填写 Linux 主机 `10.160.9.100`

## 3. 预检

```bash
source /opt/ros/humble/setup.bash
source /home/liuxiaopeng/ws_moveit2/install/setup.bash
source /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/install/setup.bash
ros2 run ur10_real_control_ros2 check_real_ur10_ready.sh 10.160.9.21
```

至少应满足：

- `ROBOT_PING_OK`
- `/joint_states` 有数据
- `SCALED_CONTROLLER_ACTIVE`
- `EXEC_ACTION_ONLINE`
- `SPEED_SCALING_NONZERO`
- `RTDE_OK`

## 4. 启动完整链路

```bash
source /opt/ros/humble/setup.bash
source /home/liuxiaopeng/ws_moveit2/install/setup.bash
source /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/install/setup.bash
ros2 launch ur10_real_control_ros2 ur10_real_control.launch.py
```

## 5. 现场验收

1. RViz2 中机械臂初始姿态与真实机械臂一致
2. 真实机械臂小幅运动后，RViz2 中模型同步变化
3. `scaled_joint_trajectory_controller` 保持 `active`
4. 在 RViz2 中拖动 TCP 交互 marker
5. 先 `Plan`
6. 再小幅、低速 `Execute`
7. 实机成功执行且 RViz2 轨迹与实际一致

## 6. 故障排查

### 机器人不动

- 检查示教器是否仍在 `External Control`
- 检查速度缩放是否过低
- 检查 `scaled_joint_trajectory_controller` 是否 active

### 没有 `/joint_states`

- 检查 driver 是否启动
- 检查 `dashboard_client` 是否在线
- 检查网线和机器人 IP

### 出现控制冲突

关闭其他 RTDE 客户端，例如：

- 自定义 Python RTDE 脚本
- MATLAB / RoboDK / 其他上位机

必要时先执行：

```bash
ros2 run ur10_real_control_ros2 cleanup_real_control_processes.sh
```
