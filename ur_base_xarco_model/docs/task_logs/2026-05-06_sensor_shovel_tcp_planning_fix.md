# 本次任务总结

## 1. 任务目标

修复 `sensor_shovel_tcp` 作为拖拽目标时“轻微移动也规划失败”的主链路问题，并消除运行态重复节点干扰。

## 2. 根因分析

1. 规划链路存在重复进程并存风险（`move_group/robot_state_publisher` 等），会污染场景与状态。
2. `assembly.urdf.xacro` 中 `sensor_shovel_tcp` 同时作为 joint 名和 link 名，存在语义歧义。
3. SRDF 未覆盖 shovel 与腕部邻接碰撞豁免，导致可视可达但碰撞判定失败概率升高。
4. 诊断时若用 `world` 作为基准会直接失败（当前固定帧实际为 `base_jizuo`）。

## 3. 修改文件清单

- `ur_base_xarco_model/assembly_xacro/assembly/assembly.urdf.xacro`
- `ur10_real_pose_sync/config/assembly_xacro.srdf`
- `ur10_real_pose_sync/launch/shovel_plan_execute.launch.py`
- `ur10_real_pose_sync/ur10_real_pose_sync/micro_plan_diagnostic_node.py`（新增）
- `ur10_real_pose_sync/setup.py`

## 4. 新增功能

1. 新增微位移规划诊断节点 `micro_plan_diagnostic_node`：
   - 读取当前关节状态
   - 以 `sensor_shovel_tcp` 为目标做 5mm 偏移
   - 调用 `MoveGroup` 进行 `plan_only` 规划
   - 失败时继续输出 IK 与状态有效性诊断
2. launch 新增参数：
   - `cleanup_on_start`（默认 `true`）
   - `run_micro_diag`（默认 `false`）

## 5. 核心实现逻辑

1. 将 fixed joint `sensor_shovel_tcp` 重命名为 `sensor_shovel_tcp_fixed`，保留末端 link 名 `sensor_shovel_tcp`。
2. 在 SRDF 补齐 shovel 邻接碰撞豁免：
   - `ur10_wrist_3` ↔ `sensor_shovel`
   - `sensor_shovel` ↔ `sensor_shovel_tcp`
   - `ur10_wrist_2` ↔ `sensor_shovel`
3. 在 `shovel_plan_execute.launch.py` 增加启动前清理与统一命名，减少重复节点干扰。
4. 诊断节点默认基准帧为 `base_jizuo`，与当前 RViz 固定帧一致。

## 6. 执行命令

```bash
cd /root/ur10_ws
colcon build --symlink-install --packages-select ur10_real_pose_sync
source install/setup.bash

xacro /root/ur10_ws/src/ur_base_xarco_model/assembly_xacro/assembly/assembly.urdf.xacro > /tmp/assembly_real.urdf
check_urdf /tmp/assembly_real.urdf

ros2 launch ur10_real_pose_sync shovel_plan_execute.launch.py
# 可选诊断
ros2 launch ur10_real_pose_sync shovel_plan_execute.launch.py run_micro_diag:=true
```

## 7. 测试结果

1. `colcon build`：通过。
2. `xacro + check_urdf`：通过，链路完整到 `sensor_shovel_tcp`。
3. 启动冒烟测试：`move_group`、`robot_state_publisher`、虚拟轨迹控制节点可启动。
4. 诊断节点联调：已修正基准帧由 `world` 改为 `base_jizuo`，避免 TF 误判。

## 8. 剩余问题

1. 需在你当前真实运行现场执行连续 10 次 5-20mm 拖拽验证成功率（目标 >=80%）。
2. 若仍失败，需要抓取 `move_group` 的具体 error code 统计，判断是否仍为碰撞主导或 IK 主导。

## 9. 下一步建议

1. 用单一命令启动并确保不再额外起其它 move_group 链路。
2. 在 RViz 固定姿态连续做小位移测试并记录成功率。
3. 如仍有失败，基于 `micro_plan_diagnostic_node` 输出进一步细化 `disable_collisions`（仅最小必要集合）。
