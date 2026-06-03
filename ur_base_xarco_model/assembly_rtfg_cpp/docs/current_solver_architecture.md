# assembly_rtfg_cpp 当前求解架构

更新时间: 2026-06-03 | 版本: v2.0

## 1. 当前求解链路 (v2.0)

```mermaid
flowchart LR
  A[轨迹点生成] --> B[目标位姿变换]
  B --> C[KDL LMA IK 求解]
  C --> D[候选排序]
  D --> E[碰撞流水线 Stage1 粗筛]
  E --> F[碰撞流水线 Stage2 FCL]
  F --> G[自适应回落检查]
  G --> H[播放轨迹连续化]
  H --> I[JointTrajectory 打包]
  I --> J[FollowJointTrajectory]
```

当前实现不依赖 MoveIt `plan()` 链路，使用 **自研 KDL LMA IK + FCL 碰撞检测** 组合。
节点入口: `/rtfg/fit_preview`，默认 `solver_backend="kdl"`。

## 2. 复杂度分析

设轨迹点数为 `N`，候选数为 `K`，碰撞对数为 `P`。

| 阶段 | 主要复杂度 | 说明 |
|---|---:|---|
| 轨迹点生成 | `O(N)` | 生成 YOZ 三段轨迹并升到 3D |
| 位姿转换 | `O(N)` | 每点做固定刚体变换 |
| IK 搜索 | `O(N * K * I)` | KDL LMA 迭代，`I` ≈ 80 次平均 |
| 碰撞检测 | `O(N * K * P)` | Stage2 FCL 是主要开销 |
| 播放轨迹连续化 | `O(N)` 到 `O(N * m)` | `m` 为每段插值子点数 |
| JointTrajectory 打包 | `O(M)` | `M` 为播放点总数 |

## 3. IK 后端架构

### 工厂模式设计

```cpp
// include/ik_backend.h
class IKSolverBase { /* 抽象接口 */ };
class KdlLMAIK       : public IKSolverBase { /* KDL LMA (默认) */ };
class CurrentNumericIK : public IKSolverBase { /* DLS 数值 (备用) */ };
class TracIKSolver    : public IKSolverBase { /* TracIK (未实现) */ };
class IKFastSolver    : public IKSolverBase { /* IKFast (未实现) */ };
class URKinematicsSolver : public IKSolverBase { /* UR 解析 (未实现) */ };

// src/ik_backend.cpp:60-77
std::unique_ptr<IKSolverBase> createIKSolverBackend(const SolverConfig& cfg);
```

### 默认后端: KDL LMA

- **算法**: Levenberg-Marquardt 非线性优化
- **精度**: pos_err ~7.7 μm, rot_err ~0.0004°
- **实现**: `src/ik_solver.cpp:187-256` solveSinglePoseKdl
- **选择条件**: `solver_backend == "kdl"` (默认)

### 备用后端: DLS 数值 IK

- **算法**: Damped Least Squares 雅可比伪逆
- **实现**: `src/ik_solver.cpp:22-111` solveSinglePose
- **选择条件**: `solver_backend == "numeric"`

## 4. 自适应回落 (Adaptive Fallback)

自适应回落机制允许在位置精度满足 `ik_position_tolerance` (默认 3 cm) 时，
接受 clearance 略低于阈值的候选解。

### 两阶段实现

**阶段 1** (`trajectory_solver.cpp:246-269` finalize_candidates):
- fallback 通过精确碰撞检查 → 直接提升为 safe
- fallback 未通过但 pos_err < 0.5×tolerance → 发出 `adaptive_clearance_warning` 后接受

**阶段 2** (`trajectory_solver.cpp:381-386` 主循环):
- 无 safe 候选但有 fallback 且 pos_err ≤ tolerance → 接受 fallback

```cpp
// 主循环自适应回落 (trajectory_solver.cpp:381-386)
if (search.safe.valid) {
  timing.n_custom_success++;
} else if (search.fallback.valid &&
           search.fallback.pos_err <= cfg.ik_position_tolerance) {
  search.safe = search.fallback;
  timing.n_custom_success++;
}
```

## 5. 当前主要耗时来源

| 阶段 | 耗时 | 占比 |
|------|------|------|
| IK 求解 (KDL LMA) | ~276 s | ~91% |
| FCL 碰撞检测 | ~27 s | ~9% |
| 轨迹生成 + 打包 | < 1 s | < 1% |

当前实测:
- `solver_mode=full, solver_backend=kdl`: 273 锚点, ~303 s 总耗时
- IK 每点约 1.01 s (含多 seed 搜索)
- FCL 每次调用约 20 ms

## 6. 是否适合实时系统

当前架构已具备实时化基础:
- 不依赖 MoveIt 规划链路 ✓
- 后端可替换 (KDL ↔ DLS) ✓
- 碰撞和 IK 都能独立插拔 ✓
- 自适应回落减少因 clearance 边界值导致的失败 ✓

**待改进**:
- full 模式 ~5 分钟/求解，不适合实时
- 每点多 seed 搜索开销大
- 可切换到 `realtime` 模式减少搜索预算
- 未来可加入连续预测和增量求解

## 7. 对应文件

| 文件 | 内容 |
|------|------|
| `src/rtfg_solver_node.cpp:140-141` | 节点默认 solver_backend="kdl" |
| `src/trajectory_solver.cpp:379-405` | 自适应回落核心逻辑 |
| `src/ik_solver.cpp:187-256` | solveSinglePoseKdl (KDL LMA) |
| `src/ik_solver.cpp:22-111` | solveSinglePose (DLS 数值) |
| `src/ik_backend.cpp:60-77` | 后端工厂函数 |
| `src/collision_checker.cpp` | FCL 碰撞检测 |
| `src/collision_pipeline.cpp` | 两阶段碰撞流水线 |
| `include/assembly_rtfg_cpp/types.h` | 核心数据结构 |

## 8. 代码示例

```cpp
// v2.0 默认配置 (使用 KDL LMA)
rtfg::SolverConfig cfg;
cfg.solver_mode = "full";
cfg.solver_backend = "kdl";        // KDL LMA (默认)
cfg.clearance_threshold = 0.002;   // 2 mm
cfg.ik_position_tolerance = 0.03;  // 3 cm
cfg.global_seed_budget = 48;       // 种子预算

rtfg::ContinuousTrajectorySolver solver(robot_, basin_boxes, cfg);
rtfg::RollingPlanner planner(cfg);
auto result = planner.solve(solver, target_plan.tforms, target_plan.segment_names,
                            current_q, runtime_.initial_q);
```
