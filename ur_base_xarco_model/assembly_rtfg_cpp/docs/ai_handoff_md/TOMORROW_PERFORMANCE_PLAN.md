# 当前已跑通轨迹性能优化指南（Goal 模式专用）

> 适用对象：明早开启 goal 模式后继续提升 `assembly_rtfg_cpp` 性能的 AI / 开发者  
> 当前目标：只针对**当前已跑通的 safe 轨迹**做性能优化，在保持 `success=true` 与接口兼容的前提下，降低总耗时、IK 耗时与碰撞耗时

---

## 0. 开始前先做的三件事（必须）

1. 固定环境并隔离 ROS 图（避免被其他工程干扰）：

```bash
export ROS_DOMAIN_ID=66
cd /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model
source /opt/ros/humble/setup.bash
```

2. 构建并加载：

```bash
colcon build --symlink-install --packages-select assembly_rtfg_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

3. 启动最小链路（不启 RViz/MoveIt）：

```bash
ros2 launch assembly_rtfg_cpp rtfg_sim.launch.py launch_rviz:=false launch_moveit:=false
```

---

## 1. 总策略（只做 safe 轨迹）

1. **先拿基线**：safe 跑 5 次，算平均值（不是看单次）。  
2. **每次只改一个优化点**：改完立即复测，确认收益可归因。  
3. **先 IK 再碰撞**：当前瓶颈主要在 IK 总耗时。  
4. **不破坏行为约束**：
   - `/rtfg/*` 服务名不变
   - `safe` 仍需 `success=true`
   - 仅 `success=true` 更新缓存
5. **文档同步**：每轮优化后更新性能表（当前值/优化后值/变化原因）。

---

## 2. 优先级路线图（P0 → P1 → P2）

## P0（明早第一优先级，先做）

### P0-1：IK 候选搜索减冗余（`trajectory_solver.cpp`）

文件：

- `src/trajectory_solver.cpp`

动作：

1. 强化 early-exit 条件（`good_enough_safe()`）  
2. 限制 stage3/stage4 的 seed 展开量（分段预算）  
3. 对已明显劣于当前 best_safe 的候选提前剪枝（基于 `pos_err/cost` 门限）  
4. 对“连续性很差”的候选在进入碰撞前直接跳过

验收（safe）：

- `safe`：`success=true` 不变  
- `timing_ik_total_s` 显著下降（目标先降 10%+）

### P0-2：碰撞审计采样分级（`trajectory_solver.cpp`）

动作：

1. 现在 playback `collision_stride=10` 是固定值，改为**工况自适应**：  
   - 若 anchor 阶段全程 clearance 充裕且无冲突迹象，可用更大 stride（例如 16/20）  
   - 若接近阈值或曾出现可疑段，退回更小 stride（例如 6/8）
2. 保留首点、末点、段边界必检逻辑，避免漏检

验收（safe）：

- safe 工况碰撞耗时下降

---

## P1（P0 稳定后再做）

### P1-1：碰撞配对剪枝（`collision_checker.cpp`，仅围绕 safe）

文件：

- `src/collision_checker.cpp`

动作：

1. 增加可跳过 pair 的静态规则（在保持安全前提下）  
2. 优先处理 tool 相关对，普通 self pair 可加更严谨预过滤  
3. 仅在必要时执行完整 distance

风险：

- 剪枝过度会漏检；必须保守实现并做 safe 反复回归。

### P1-2：响应负载分级（保持兼容）

文件：

- `src/rtfg_solver_node.cpp`
- （如需）`srv/FitPreview.srv`（尽量避免破坏兼容）

动作：

1. 保持默认 full debug 行为  
2. 增加参数开关（节点参数）控制“轻量返回模式”：  
   - 少返回大数组，仅保留摘要指标（用于压测）

说明：

- 若改 srv 会破坏兼容，优先走“参数控制发布/填充策略”的方式。

---

## P2（有余力再做）

1. IK 工作空间缓存（减少 Eigen 临时分配）  
2. 局部失败段重试（而非整段放大全局搜索）  
3. 进一步收敛 seed 的统计式调度（按上一段成功模式动态选 seed）

---

## 3. 建议的“单轮优化模板”（照着跑）

1. 新建分支或记录本轮改动标签（可选）  
2. 修改一个优化点（例如 P0-1）  
3. `colcon build`  
4. 启动最小链路  
5. 跑 benchmark（safe 5 次）  
6. 汇总平均值写入 JSON 汇总表  
7. 更新文档性能章节  
8. 再进行下一个优化点

---

## 4. 建议命令清单（可直接复制）

### 4.1 safe 基准

```bash
python3 assembly_rtfg_cpp/benchmarks/rtfg_benchmark.py \
  --yaml /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model/assembly_rtfg_cpp/config/environment_runtime_config_safe.yaml \
  --output-tag safe
```

### 4.2 几何一致性检查

```bash
ros2 run assembly_rtfg_cpp rtfg_geometry_check
```

### 4.3 执行缓存验收

```bash
ros2 service call /rtfg/execute_cached assembly_rtfg_cpp/srv/ExecuteCached "{execute: true}"
```

---

## 5. 明早 goal 模式提示词建议（直接给 AI）

可直接使用：

1. “先执行 P0-1，仅优化 IK 候选减冗余，不改接口，改完跑 safe 5 次并汇总均值。”
2. “确认 safe 仍 success=true 且 execute_cached 成功，然后再执行 P0-2 的碰撞审计采样分级。”
3. “每轮优化后更新 `docs/rtfg_ros2_cpp_技术文档/05_Cpp性能基准与优化记录.html`，写明收益与原因。”

---

## 6. 判定“本轮可收工”的标准

至少满足：

1. safe 工况 `success=true` 且 `execute_cached success=true`  
2. safe 工况总耗时或 IK 耗时较本轮开始前平均值有可重复下降  
3. 文档和 benchmark 文件已同步更新

---

## 7. 关键文件索引（明早优先打开）

1. `src/trajectory_solver.cpp`
2. `src/collision_checker.cpp`
3. `src/rtfg_solver_node.cpp`
4. `benchmarks/rtfg_benchmark.py`
5. `docs/rtfg_ros2_cpp_技术文档/05_Cpp性能基准与优化记录.html`
6. `docs/ai_handoff_md/AI_QUICK_ONBOARDING.md`

---

## 8. 一句话交接

先在隔离 ROS 域下拿 safe 可重复基线，然后按 P0-1 → P0-2 做最小侵入优化；每次只改一个点、立即复测并回写文档。
