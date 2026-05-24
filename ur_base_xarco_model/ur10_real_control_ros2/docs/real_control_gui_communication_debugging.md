# UR10 实机通信问题复盘与 GUI 控制链说明

本文记录 `ur10_real_control_ros2` 从通信故障排查到最终通过 RViz2 控制真实 UR10 的完整链路。重点说明 `scripts/real_control_gui.py` 如何启动 `ur_robot_driver`、MoveIt2/RViz2、预检、详细通信检测和残余进程清理。

## 1. 最终可用网络配置

当前现场已验证可用的网络配置如下：

| 项目 | 地址/参数 | 说明 |
| --- | --- | --- |
| Linux 有线网卡 | `10.160.9.10/24` | GUI 中的“本机回连 IP”，也就是 `reverse_ip` |
| UR10 控制柜 | `10.160.9.21` | GUI 中的“机械臂 IP”，也就是 `robot_ip` |
| ROS 2 | Humble | 通过 `/opt/ros/humble/setup.bash` 加载 |
| MoveIt2 工作空间 | `/home/liuxiaopeng/ws_moveit2` | GUI 启动命令会 source |
| 本包工作空间 | `/home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model` | GUI 默认 `workspace` |

关键点：

- 示教器 External Control 的 Host IP 必须填写 Linux 主机实际可达地址：`10.160.9.10`。
- `robot_ip` 是机器人地址：`10.160.9.21`。
- `reverse_ip` 是机器人回连 Linux 主机时使用的本机地址：`10.160.9.10`。
- 如果示教器仍填写旧地址 `10.160.9.100`，机器人会回连不存在的主机地址，External Control 会出现 `no route` 或连接失败。

## 2. 通信端口

`ur_robot_driver` 和 UR 控制柜之间同时使用多类端口。基础端口通，不代表完整控制链已经可用。

| 端口 | 方向 | 协议/用途 | 在本次排障中的意义 |
| --- | --- | --- | --- |
| `29999` | Linux -> UR10 | Dashboard Server | 可读取/控制机器人状态；当前稳定链路默认不启动 `dashboard_client` |
| `30001` | Linux -> UR10 | Primary Interface | `ur_ros2_control_node` 初始化时需要从这里读配置包 |
| `30002` | Linux -> UR10 | Secondary Interface | 辅助状态接口 |
| `30003` | Linux -> UR10 | Realtime Interface | 可直接读实时关节角；参考 WSL2 工程中的 TCP 读包方式 |
| `30004` | Linux -> UR10 | RTDE | `ur_robot_driver` 的实时数据交换 |
| `50001` | UR10 -> Linux | reverse interface | External Control 回连 driver 的主控制通道 |
| `50002` | UR10 -> Linux | script sender | 机器人向 driver 请求 external control 脚本 |
| `50003` | UR10 -> Linux | trajectory interface | 轨迹相关通道 |
| `50004` | UR10 -> Linux | script command | URScript 命令转发通道 |

`30003` 能读到真实关节角，只能说明机器人实时状态流可用；真正能执行 MoveIt2 轨迹，还必须满足 `ur_ros2_control_node` 成功启动、控制器 active、External Control 成功回连。

## 3. GUI 如何组织实机控制

`real_control_gui.py` 是现场操作入口，主要做五类事情。

### 3.1 启动驱动

“启动驱动”按钮执行：

```bash
ros2 launch ur10_real_control_ros2 ur_driver_only.launch.py \
  robot_ip:=10.160.9.21 \
  reverse_ip:=10.160.9.10 \
  ur_type:=ur10 \
  headless_mode:=false \
  launch_dashboard_client:=false
```

这里最重要的稳定性设置是：

```bash
launch_dashboard_client:=false
```

现场反复出现过：

```text
Could not get configuration package within timeout, are you connected to the robot?(Configured timeout: 1 sec)
```

当 `dashboard_client` 与 `ur_ros2_control_node` 在启动阶段同时访问机器人 Primary/Dashboard 相关通信时，CB3 控制器上会出现初始化不稳定。关闭 Dashboard Client 后，driver 能稳定进入：

```text
System successfully started!
Successful 'activate' of hardware 'ur10'
```

Dashboard Client 关闭不影响 External Control 回连，也不影响 MoveIt2 通过 `scaled_joint_trajectory_controller` 执行轨迹。

### 3.2 启动 MoveIt + RViz

“启动 MoveIt + RViz”按钮执行：

```bash
ros2 launch ur10_real_control_ros2 ur10_real_control.launch.py \
  robot_ip:=10.160.9.21 \
  reverse_ip:=10.160.9.10 \
  ur_type:=ur10 \
  launch_driver:=false \
  launch_rviz:=true \
  launch_rsp:=false
```

该模式用于 driver 已经启动后的可视化和规划执行。GUI 允许 driver 已运行时再启动 MoveIt/RViz。

### 3.3 启动完整链路

“启动完整链路”按钮同时启动 driver、MoveIt 和 RViz：

```bash
ros2 launch ur10_real_control_ros2 ur10_real_control.launch.py \
  robot_ip:=10.160.9.21 \
  reverse_ip:=10.160.9.10 \
  ur_type:=ur10 \
  launch_driver:=true \
  launch_rviz:=true \
  launch_rsp:=false \
  launch_dashboard_client:=false
```

该模式也默认关闭 Dashboard Client，保持与“启动驱动”一致的稳定通信策略。

### 3.4 执行预检

“执行预检”按钮运行：

```bash
ros2 run ur10_real_control_ros2 check_real_ur10_ready.sh 10.160.9.21
```

按钮颜色含义：

| 颜色 | 含义 |
| --- | --- |
| 灰色 | 未开始或正在检查 |
| 绿色 | 所有关键 marker 通过 |
| 红色 | 任一关键 marker 缺失或失败 |

预检变绿色必须看到这些 marker：

- `ROBOT_PING_OK`
- `JOINT_STATES_OK`
- `SCALED_CONTROLLER_ACTIVE`
- `EXEC_ACTION_ONLINE`
- `SPEED_SCALING_NONZERO`
- `RTDE_OK`

这表示 ROS2 driver、`/joint_states`、控制器 action 和速度缩放状态都已经可用。

### 3.5 详细通信检测

GUI 中的“详细检测”按钮调用 `scripts/ur10_comm_diagnostics.py`。这些检测用于区分“网络通”和“控制链可用”。

| 按钮 | 检测内容 |
| --- | --- |
| 网络/路由 | 本机是否有 `10.160.9.10`，到 `10.160.9.21` 的路由源地址是否正确，ping 是否通 |
| Primary配置 | 连接 `30001`，读取 Primary Interface 包，确认包含 `URControl` 和足够长度的配置数据 |
| 机器人端口 | 检查 `29999`、`30001`、`30002`、`30003`、`30004` 是否可连接 |
| 30003关节流 | 参考 WSL2 工程的 TCP 方式，从 `30003` 读取实时包并解析 6 个关节角 |
| 残余/冲突 | 检查本机残余 ROS driver、RTDE、RoboDK、外部客户端进程，以及 `50001-50004` 监听情况 |
| 完整通信 | 顺序执行上述所有检测 |

详细检测以脚本输出为准：

- `DIAG_OK`：按钮变绿色
- `DIAG_FAIL`：按钮变红色

## 4. 本次问题复盘

### 4.1 External Control 报 `no route`

最初 GUI 和文档中曾使用 `reverse_ip=10.160.9.100`，但 Linux 有线网卡实际地址是 `10.160.9.10/24`。机器人按 `10.160.9.100` 回连时，网络上没有这个主机地址，因此示教器 External Control 报 `no route`。

修复方式：

- GUI 默认 `reverse_ip` 改为 `10.160.9.10`
- launch 文件默认 `reverse_ip` 也统一为 `10.160.9.10`
- 示教器 External Control Host IP 同步填写 `10.160.9.10`

### 4.2 清理和启动并发导致 driver 被杀

曾出现以下日志：

```text
[cleanup] [KILL] ros2 launch ur10_real_control_ros2 ur_driver_only.launch.py
[driver] BrokenPipeError: [Errno 32] Broken pipe
[driver] EXIT code=-9
```

原因是“清理残余进程”还没结束时又点了“启动驱动”。清理脚本第二轮 `KILL` 匹配到刚启动的新 driver，导致 driver 被杀掉。

修复方式：

- GUI 跟踪 `cleanup_process`
- 清理运行期间禁用启动、预检、MoveIt/RViz、完整链路和详细检测按钮
- 清理完成后再恢复按钮
- 清理脚本不再匹配并杀掉 GUI 自己

### 4.3 Driver 初始化失败

多次日志中出现：

```text
Could not get configuration package within timeout, are you connected to the robot?(Configured timeout: 1 sec)
```

同时日志显示：

```text
Successfully connected to Dashboard Server at 10.160.9.21
Negotiated RTDE protocol version to 2
Received URControl version 3.15.8.0
```

这说明不是网线、IP、ping 或 RTDE 端口不通，而是 `ur_ros2_control_node` 在初始化阶段没有及时从 Primary Interface 拿到配置包。现场验证发现，默认启动 Dashboard Client 会增加启动阶段通信并发，导致 CB3 控制器上该问题更容易出现。

修复方式：

- 默认 `launch_dashboard_client:=false`
- 只保留控制所需的 driver、RTDE、External Control 和 controller 链路
- 用详细检测确认 Dashboard 端口可通，但不默认启动 dashboard 节点

### 4.4 RTDE/外部客户端冲突

历史日志曾出现：

```text
Variable 'speed_slider_mask' is currently controlled by another RTDE client
```

这类错误表示有其他 RTDE 客户端或现场总线占用了机器人输入资源。可能来源包括：

- 自定义 Python RTDE 脚本
- RoboDK
- MATLAB 或其他上位机
- Ethernet/IP、Modbus 等现场总线配置
- 上一次失败后残留的 ROS driver 进程

处理方式：

- 先点 GUI 的“清理残余进程”
- 再点“残余/冲突”或“完整通信”
- 必要时关闭其他上位机程序或重启机器人控制器

## 5. 推荐现场操作顺序

稳定操作顺序如下：

1. 确认 Linux 有线网卡为 `10.160.9.10/24`
2. 确认机器人 IP 为 `10.160.9.21`
3. 示教器 External Control Host IP 填 `10.160.9.10`
4. 打开 GUI
5. 点“清理残余进程”，等待状态显示“清理完成”
6. 点“完整通信”，确认按钮变绿色
7. 点“启动驱动”
8. 等日志出现 `System successfully started`
9. 在示教器运行 External Control 程序
10. 点“执行预检”，确认按钮变绿色
11. 点“启动 MoveIt + RViz”，或直接使用“启动完整链路”
12. 在 RViz2 中先 `Plan`，再小幅低速 `Execute`

## 6. 判断控制链是否真正可用

以下条件同时满足，才认为真实控制链可用：

- `ur_ros2_control_node` 日志出现 `System successfully started`
- 硬件接口 `activate` 成功
- `/joint_states` 有真实数据
- `scaled_joint_trajectory_controller` 为 `active`
- `/scaled_joint_trajectory_controller/follow_joint_trajectory` action 在线
- `speed_scaling` 非零
- 示教器 External Control 正在运行并成功回连 driver
- RViz2 中模型姿态与实机一致，低速小幅 `Execute` 成功

不要只用 ping 或 `30003` 关节流判断控制链是否成功。它们只能证明基础通信可用，不能证明 ROS2 driver 已经具备轨迹执行能力。

## 7. 当前稳定策略总结

当前 GUI 的稳定策略是：

- `robot_ip=10.160.9.21`
- `reverse_ip=10.160.9.10`
- `launch_dashboard_client:=false`
- 清理期间禁止启动新 driver
- 预检按钮用关键 marker 判断绿色/红色
- 详细检测按钮用 `DIAG_OK` / `DIAG_FAIL` 判断绿色/红色
- 通过 `ur10_comm_diagnostics.py` 分层定位网络、Primary、端口、Realtime、残余进程问题

这套配置已经验证能够通过 RViz2 控制真实 UR10。
