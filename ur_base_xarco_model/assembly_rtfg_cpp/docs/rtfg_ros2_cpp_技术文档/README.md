# RTFG ROS2 C++ 技术文档

> 最后更新: 2026-06-03 | 版本: v2.0 (KDL LMA + Adaptive Fallback)

## 文档索引

| 文档 | 内容 |
|------|------|
| [01_系统架构概述](#01-系统架构概述) | 整体求解链路、模块关系、数据流 |
| [02_IK后端设计](#02-ik后端设计) | KDL LMA 主后端 + DLS 备用 + 工厂模式 |
| [03_自适应回落策略](#03-自适应回落策略) | adaptive fallback 两阶段逻辑 |
| [04_碰撞检测流水线](#04-碰撞检测流水线) | FCL 两阶段碰撞检查 |
| [05_轨迹求解流程](#05-轨迹求解流程) | 轨迹点生成 → IK → 碰撞 → 排序 → 连续化 |
| [06_GUI架构](#06-gui架构) | Flask + SocketIO + Three.js 简化版 |
| [07_ROS2服务接口](#07-ros2服务接口) | /rtfg/load_config, fit_preview, execute_cached |
| [08_关键参数表](#08-关键参数表) | MATLAB vs C++ 参数对照 |
| [09_构建与运行指南](#09-构建与运行指南) | 编译、启动、测试完整步骤 |
| [10_故障排查手册](#10-故障排查手册) | clearance 违规、IK 失败等常见问题 |

---

## 01 系统架构概述

### 求解链路 (v2.0)

```
轨迹点生成 → 目标位姿变换 → KDL LMA IK 求解
    → 候选排序 → FCL 碰撞检测(Stage1 粗筛 → Stage2 精确)
    → 自适应回落检查 → 播放轨迹连续化
    → JointTrajectory 打包 → FollowJointTrajectory 执行
```

### 核心变更 (v1.0 → v2.0)

| 项目 | v1.0 (旧) | v2.0 (当前) |
|------|----------|------------|
| **主 IK 后端** | CurrentNumericIK (DLS 数值) | **KdlLMAIK** (KDL LMA) |
| **默认 backend** | `"numeric"` | **`"kdl"`** |
| **回落策略** | 无 | **自适应回落** (adaptive fallback) |
| **种子预算** | 24 / 12 | **48 / 48** |
| **GUI 架构** | Flask + 参数滑块调整 | **Flask + 固定参数 + RViz2 集成** |
| **移动规划** | 无 | 无 (不依赖 MoveIt) |

### 模块依赖图

```
rtfg_solver_node.cpp  (ROS2 服务入口)
  ├── trajectory_solver.cpp  (主求解逻辑)
  │     ├── ik_solver.cpp  (solveSinglePose / solveSinglePoseKdl)
  │     │     └── ik_backend.cpp  (工厂: KdlLMAIK / CurrentNumericIK / ...)
  │     ├── collision_checker.cpp  (FCL 碰撞检测)
  │     │     └── collision_pipeline.cpp  (两阶段流水线)
  │     ├── trajectory_generator.cpp  (轨迹点生成)
  │     ├── continuous_trajectory_solver.cpp  (连续轨迹)
  │     └── rolling_planner.cpp  (滚动规划器)
  ├── robot_model.cpp  (URDF 模型加载)
  └── utils.cpp  (工具函数)
```

### 关键文件

| 文件 | 作用 |
|------|------|
| `include/types.h` | 核心数据结构 (SolverConfig, CandidateInfo, CollisionSummary 等) |
| `include/ik_backend.h` | IK 后端抽象接口 + 5 个后端类声明 |
| `include/collision_pipeline.h` | 两阶段碰撞流水线接口 |
| `include/trajectory_solver.h` | 轨迹求解器声明 |
| `src/rtfg_solver_node.cpp` | ROS2 节点主入口 |
| `src/trajectory_solver.cpp:379-405` | 自适应回落核心逻辑 |
| `src/ik_backend.cpp:60-77` | 后端工厂函数 |
| `src/ik_solver.cpp:187-256` | solveSinglePoseKdl (KDL LMA) |
| `src/ik_solver.cpp:22-111` | solveSinglePose (DLS 数值备用) |

---

## 02 IK后端设计

### 后端架构

采用**工厂模式 + 策略接口**，运行时根据 `SolverConfig::solver_backend` 选择 IK 后端。

```cpp
// include/ik_backend.h
class IKSolverBase {
public:
  virtual ~IKSolverBase() = default;
  virtual std::string name() const = 0;
  virtual CandidateInfo solve(...) const = 0;
};

// 可用后端:
class CurrentNumericIK : public IKSolverBase { /* DLS 数值 IK */ };
class KdlLMAIK       : public IKSolverBase { /* KDL LMA (默认) */ };
class TracIKSolver    : public IKSolverBase { /* TracIK (未实现) */ };
class IKFastSolver    : public IKSolverBase { /* IKFast (未实现) */ };
class URKinematicsSolver : public IKSolverBase { /* UR 解析 (未实现) */ };

// 工厂函数
std::unique_ptr<IKSolverBase> createIKSolverBackend(const SolverConfig& cfg);
```

### 后端切换规则

```cpp
// src/ik_backend.cpp:60-77
if (cfg.solver_backend == "kdl")           → KdlLMAIK       (默认)
if (cfg.solver_backend == "numeric")       → CurrentNumericIK
if (cfg.solver_backend == "tracik")        → TracIKSolver    (throw)
if (cfg.solver_backend == "ikfast")        → IKFastSolver    (throw)
if (cfg.solver_backend == "ur_kinematics") → URKinematicsSolver (throw)
```

### KDL LMA (默认后端)

- **算法**: Levenberg-Marquardt (LMA) 非线性优化
- **精度**: pos_err ~7.7 μm, rot_err ~0.0004°
- **实现**: `src/ik_solver.cpp:187-256` solveSinglePoseKdl
- **入口调用**: `trajectory_solver.cpp` 通过工厂创建后端后调用

### DLS 数值 IK (备用)

- **算法**: Damped Least Squares (DLS) 雅可比伪逆
- **实现**: `src/ik_solver.cpp:22-111` solveSinglePose
- **用途**: 作为 KDL LMA 失败时的 fallback

---

## 03 自适应回落策略

### 设计目标

KDL LMA 可以解出极高精度的 IK (μm 级)，但 FCL 碰撞检测可能因 clearance 略低于阈值 (如 1.757 mm < 2 mm) 而拒绝。自适应回落允许在位置精度满足要求时接受这些解。

### 两阶段回落逻辑

#### 阶段 1: 候选排序阶段 (finalize_candidates)

```cpp
// trajectory_solver.cpp:246-269
if (!result.safe.valid && result.fallback.valid &&
    result.fallback.pos_err <= search_cfg.ik_position_tolerance) {
  // 尝试将 fallback 提升为 safe (精确碰撞检查)
  CollisionSummary collision = collision_pipeline.preciseCheck(result.fallback.q);
  double clearance = collision.min_tool_basin_clearance;
  if (clearance >= clearance_threshold) {
    result.safe = result.fallback;  // 通过!
    result.safe.failure_reason = "accepted relaxed-orientation fallback";
  } else {
    // Adaptive fallback: 位置精度优秀时接受
    if (result.fallback.pos_err < 0.5 * search_cfg.ik_position_tolerance) {
      result.safe = result.fallback;
      result.safe.failure_reason =
        "adaptive_clearance_warning:" + result.fallback.failure_reason;
    }
  }
}
```

#### 阶段 2: 主循环回落 (trajectory_solver)

```cpp
// trajectory_solver.cpp:381-386
if (search.safe.valid) {
  timing.n_custom_success++;
} else if (search.fallback.valid &&
           search.fallback.pos_err <= cfg.ik_position_tolerance) {
  // Adaptive fallback: 接受最佳位置候选
  search.safe = search.fallback;
  timing.n_custom_success++;
}
```

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ik_position_tolerance` | 0.03 (3 cm) | 位置容忍度，回落触发条件 |
| `clearance_threshold` | 0.002 (2 mm) | FCL 碰撞检测阈值 |

---

## 04 碰撞检测流水线

### 两阶段架构

```
候选 q → Stage1: 粗筛 (关节限位 + 自碰撞快速检查)
       → Stage2: FCL 精确碰撞检测
         - 自碰撞 (self-collision)
         - 工具-本体 (tool-body)
         - 工具-盆体 (tool-basin)
```

### 数据结构

```cpp
// include/types.h
struct CollisionSummary {
  bool in_collision = false;
  bool self_collision = false;
  bool tool_body_collision = false;
  bool tool_basin_collision = false;
  double min_self_clearance = INF;
  double min_tool_body_clearance = INF;
  double min_tool_basin_clearance = INF;
  std::string colliding_object;  // 违规对象名称
};
```

### 性能特征 (实测)

| 指标 | 数值 |
|------|------|
| FCL 调用次数 (273 点求解) | ~14,000 |
| 碰撞检测总耗时 | ~27 s |
| 平均每次 FCL 耗时 | ~20 ms |
| 候选截断数 | ~766 |

---

## 05 轨迹求解流程

### 完整流程

1. **轨迹点生成** (`trajectory_generator.cpp`): 根据 YOZ 三段轨迹参数生成 3D 位姿
2. **位姿变换** (`buildTargetPlan`): 将环境位姿应用到轨迹点
3. **IK 求解** (`trajectory_solver.cpp` 主循环):
   - 每点进行多 seed 搜索 (global_seed_budget=48)
   - 每个 seed 调用 KDL LMA (默认) 或 DLS 备用
   - 生成候选集 (CandidateInfo)
4. **候选排序**: 按 pos_err + cost 排序
5. **碰撞检测**: 两阶段流水线 (Stage1 粗筛 → Stage2 FCL)
6. **自适应回落**: 检查 fallback 有效性
7. **播放轨迹连续化** (`continuous_trajectory_solver.cpp`): 锚点间插值
8. **JointTrajectory 打包**: 生成 ROS2 action 消息

### 性能实测 (solver_mode=full, solver_backend=kdl)

| 指标 | 数值 |
|------|------|
| 锚点数 | 273 |
| 播放点数 | 1,322 |
| 总耗时 | ~303 s (约 5 分钟) |
| IK 总耗时 | ~276 s |
| 碰撞检测总耗时 | ~27 s |
| 平均每点耗时 | ~1.1 s |
| 平均 IK 迭代次数 | 79.6 |
| 原始候选数 | 21,729 |
| 进入 FCL 候选数 | 13,036 |

---

## 06 GUI架构

### 简化版 GUI (v2.0)

采用 **Flask + SocketIO + Three.js** 前后端分离架构，**使用 MATLAB 固定默认参数**，无需参数调整。

### 5 步操作流程

```
① 环境检查 → ② 启动 RViz2 → ③ 移动到初始点 → ④ 开始规划 → ⑤ 开始执行
```

### 技术栈

| 层 | 技术 | 文件 |
|----|------|------|
| **后端** | Flask + Flask-SocketIO | `gui/rtfg_gui/app.py` |
| **ROS2 客户端** | rclpy Service/Action | `gui/rtfg_gui/ros_client.py` |
| **前端** | HTML5 + Bootstrap 5 | `gui/rtfg_gui/templates/index.html` |
| **3D 查看器** | Three.js (URDF 加载) | `gui/rtfg_gui/static/js/viewer.js` |
| **前端逻辑** | Vanilla JS | `gui/rtfg_gui/static/js/app.js` |
| **样式** | GitHub Dark Theme CSS | `gui/rtfg_gui/static/css/style.css` |
| **RViz2 配置** | rviz 插件配置 | `gui/config/rtfg_display.rviz` |

### REST API 端点

| 端点 | 方法 | 功能 |
|------|------|------|
| `/` | GET | 主页面 |
| `/api/status` | GET | ROS2 节点状态检查 |
| `/api/launch_rviz2` | POST | 启动 RViz2 |
| `/api/rviz2_status` | GET | RViz2 进程状态 |
| `/api/move_to_start` | POST | 移动到初始关节位姿 |
| `/api/fit_preview` | POST | 轨迹拟合预览 (使用固定 MATLAB 参数) |
| `/api/execute_cached` | POST | 执行缓存轨迹 |

### MATLAB 固定默认参数

```javascript
// 前端硬编码 (app.js)
const MATLAB_PARAMS = {
  left_wall_offset: 0.0,
  mud_height: 0.0,
  approach_len: 0.12,
  theta_deg: -30.0,
  depth: 0.12,
  x_plane: 0.0,
  pose_x: 0.45,
  pose_y: 0.0, pose_z: 0.0,
  roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
  clearance_threshold: 0.002,
};
```

---

## 07 ROS2服务接口

### 服务列表

| 服务 | 类型 | 功能 |
|------|------|------|
| `/rtfg/load_config` | `LoadConfig.srv` | 加载 YAML 配置 |
| `/rtfg/fit_preview` | `FitPreview.srv` | 执行轨迹拟合求解 |
| `/rtfg/execute_cached` | `ExecuteCached.srv` | 执行缓存的轨迹 |

### FitPreview 请求字段

```yaml
# 轨迹参数
left_wall_offset: 0.0      # 入泥点到左侧壁水平距离 (m)
mud_height: 0.0             # 泥面距盆底高度 (m)
approach_len: 0.12          # 起始到入泥点斜线长 (m)
theta_deg: -30.0            # 入泥角 (deg)
depth: 0.12                 # 入泥点下扎深度 (m)
x_plane: 0.0                # YOZ 轨迹 X 截面 (m)

# 环境位姿
pose_x: 0.45                # block_with_basin X 平移 (m)
pose_y: 0.0                 # Y 平移
pose_z: 0.0                 # Z 平移
roll_deg: 0.0               # 绕 X 旋转 (deg)
pitch_deg: 0.0              # 绕 Y 旋转
yaw_deg: 0.0                # 绕 Z 旋转

# 求解器配置
current_q: []               # 当前关节角 (空=使用初始值)
clearance_threshold: 0.002  # 碰撞阈值 (m)
```

### FitPreview 响应字段

```yaml
success: true               # 求解是否成功
message: "ok"
anchor_count: 273           # 锚点数量
playback_count: 1322        # 播放点数量
timing_total_wall_s: 303.3  # 总耗时
timing_ik_total_s: 276.0    # IK 耗时
timing_collision_total_s: 27.3  # 碰撞检测耗时
min_self_clearance: 0.018   # 最小自碰撞 clearance
min_tool_body_clearance: 0.028  # 最小工具-本体 clearance
min_tool_basin_clearance: 1.484  # 最小工具-盆体 clearance
min_tool_basin_object: "block_basin_left_wall"  # 最近对象
```

---

## 08 关键参数表

### MATLAB vs C++ 参数对照

| 参数 | MATLAB 默认 | C++ 求解器 | 说明 |
|------|------------|-----------|------|
| `left_wall_offset` | 0.0 m | 0.0 m | 入泥点左侧壁偏移 |
| `mud_height` | 0.0 m | 0.0 m | 泥面距盆底高度 |
| `approach_len` | 0.12 m | 0.12 m | 斜线接近长度 |
| `theta_deg` | -30.0° | -30.0° | 入泥角 (负=向下) |
| `depth` | 0.12 m | 0.12 m | 垂直下扎深度 |
| `x_plane` | 0.0 m | 0.0 m | YOZ 截面 X 位置 |
| `pose_x` | 0.45 m | 0.45 m | basin X 平移 |
| `pose_y / pose_z` | 0.0 m | 0.0 m | basin Y/Z 平移 |
| `roll / pitch / yaw` | 0.0° | 0.0° | basin 旋转 |

### 求解器内部参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `solver_backend` | `"kdl"` | IK 后端选择 |
| `solver_mode` | `"full"` | 求解模式 |
| `clearance_threshold` | 0.002 m | FCL 碰撞检测阈值 |
| `ik_position_tolerance` | 0.03 m | IK 位置容忍度 |
| `global_seed_budget` | 48 | 全局种子预算 |
| `max_collision_candidates_full` | 48 | full 模式碰撞候选上限 |

---

## 09 构建与运行指南

### 环境要求

- **OS**: Ubuntu 22.04+ (Linux)
- **ROS2**: Humble Hawksbill
- **编译器**: GCC 11+ (C++17)
- **依赖**: Eigen3, KDL, FCL, rclcpp, Flask, Three.js (CDN)

### 构建步骤

```bash
# 1. 源码 ROS2 环境
source /opt/ros/humble/setup.bash

# 2. 编译
cd /home/liuxiaopeng/ur10_conrtol/ur_base_xarco_model
colcon build --packages-select assembly_rtfg_cpp rtfg_gui --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. 安装环境
source install/setup.bash
```

### 运行基准测试

```bash
# 终端 1: 启动求解器节点
ros2 run assembly_rtfg_cpp rtfg_solver_node

# 终端 2: 运行基准测试
python3 assembly_rtfg_cpp/benchmarks/rtfg_benchmark.py
# 预期: success=true, anchor_count=273
```

### 启动 GUI

```bash
# 方法 1: 一键启动脚本
bash assembly_rtfg_cpp/gui/scripts/rtfg_one_click.sh

# 方法 2: 使用启动器
python3 assembly_rtfg_cpp/gui/scripts/rtfg_launcher.py

# 方法 3: 直接启动 Flask
python3 -c "from rtfg_gui.app import main; main()"
# 浏览器访问 http://localhost:5000
```

### GUI 操作流程

1. 点击 **环境检查** — 验证 ROS2 服务可用
2. 点击 **启动 RViz2** — 打开 RViz2 窗口查看场景
3. 点击 **移动到初始点** — 机器人移动到初始关节位姿
4. 点击 **开始规划** — 执行轨迹拟合 (~5 分钟)
5. 点击 **开始执行** — 执行拟合后的轨迹

---

## 10 故障排查手册

### 问题 1: /rtfg/load_config 不可用

**症状**: benchmark 报错 `/rtfg/load_config unavailable`

**原因**:
- 求解器节点未启动
- ROS_DOMAIN_ID 不匹配
- 编译未正确安装

**解决**:
```bash
# 确认节点运行
ros2 node list | grep rtfg_solver
# 确认服务
ros2 service list | grep rtfg
# 检查 domain ID
echo $ROS_DOMAIN_ID
```

### 问题 2: clearance 违规

**症状**: `clearance X.XXX m, clearance 违规对象: block_basin_left_wall`

**原因**:
- FCL 检测到 clearance < 阈值 (2 mm)
- 工具末端过于接近盆壁

**解决**:
- 自适应回落已自动处理 (pos_err ≤ 0.03 m 时接受)
- 如仍失败，检查 `ik_position_tolerance` 是否过大
- 可临时降低 `clearance_threshold` 到 0.001

### 问题 3: IK 求解超时

**症状**: FitPreview 超时 (超过 120s 无响应)

**原因**:
- KDL LMA 在当前参数下收敛慢
- 种子预算过大导致搜索空间爆炸

**解决**:
- 增加客户端超时时间 (≥ 600s)
- 切换到 `solver_mode=realtime` 减少搜索预算
- 检查是否所有 seed 都失败 (查看 solver log)

### 问题 4: RViz2 启动失败

**症状**: `/api/launch_rviz2` 返回 error

**原因**:
- RViz2 配置文件缺失
- ROS2 环境未 source

**解决**:
- 确认 `gui/config/rtfg_display.rviz` 存在
- 确认 `source /opt/ros/humble/setup.bash` 已执行
- 手动测试: `ros2 run rviz2 rviz2 -d assembly_rtfg_cpp/gui/config/rtfg_display.rviz`

### 问题 5: 编译失败

**症状**: `colcon build` 报错

**原因**:
- 依赖库缺失 (Eigen, KDL, FCL)
- CMake 缓存过期

**解决**:
```bash
# 清理重建
rm -rf build/assembly_rtfg_cpp install/assembly_rtfg_cpp
colcon build --packages-select assembly_rtfg_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release
```
