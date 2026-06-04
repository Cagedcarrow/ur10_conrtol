# 求解器参数与 CUDA 后端参数配置

## 参数总览

所有求解器参数通过 ROS2 参数系统声明，可在启动时覆写。

### CUDA 求解器参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `solver_backend` | `"cuda"` | string | 求解器后端: `cuda` / `kdl` / `numeric` |
| `solver_mode` | `"full"` | string | `full` (60 iter) / `realtime` (30 iter) |
| `max_iterations_full` | `60` | int | 全模式最大迭代次数 |
| `max_iterations_realtime` | `30` | int | 实时模式最大迭代次数 |
| `ik_position_tolerance` | `0.03` | double | 位置收敛容差 (m) |
| `stagnation_epsilon` | `1e-5` | double | 停滞检测门限 |
| `stagnation_patience` | `3` | int | 停滞容忍次数 |
| `dq_stop_threshold` | `1e-6` | double | 步长停止门限 |

### 碰撞检测参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `clearance_threshold` | `0.002` | double | 最小安全间隙 (m) |
| `max_collision_candidates_full` | `8` | int | 全模式最大碰撞候选数 |
| `max_collision_candidates_realtime` | `2` | int | 实时模式最大碰撞候选数 |
| `collision_check_stride_full` | `7` | int | 全模式碰撞检测步幅 |
| `collision_check_stride_realtime` | `5` | int | 实时模式碰撞检测步幅 |

### 可视化参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `publish_sparse_posearray_realtime` | `true` | bool | 实时模式是否稀疏发布 PoseArray |
| `posearray_stride_realtime` | `10` | int | 稀疏发布间隔 |
| `base_link` | `"base_jizuo"` | string | 基坐标系 |
| `tip_link` | `"sensor_shovel_tcp"` | string | 工具坐标系 |
| `controller_action` | `"/rtfg/joint_trajectory_controller/..."` | string | 动作控制器路径 |

### 窗口求解参数

| 参数名 | 默认值 | 类型 | 说明 |
|--------|--------|------|------|
| `enable_window_solve` | `false` | bool | 是否启用分窗口求解 |
| `window_size` | `100` | int | 窗口大小（目标点数） |
| `window_overlap` | `10` | int | 窗口重叠（目标点数） |

## CUDA 后端特有参数

以下参数在 `CudaBatchIK::initialize()` 中硬编码，通过常量内存 `c_lambda_params` 传递到 GPU：

### 自适应阻尼参数

| 参数 | 值 | 作用 |
|------|-----|------|
| `cfg.lambda` (base) | 2e-3 | 基础阻尼系数 |
| `lambda_max_far` | 0.1 | 远距离最大阻尼 |
| `lambda_floor_near` | 5e-4 | 近距离阻尼下限 |
| `lambda_ref_distance` | 0.05 | 阻尼缩放参考距离 |

内核中的自适应阻尼逻辑 (`cuda_kernels.cu:177-191`):

```cuda
if (pos_err > 0.1) {
    // 远距离：中等阻尼，随距离缩放
    s_lambda = fmax(2e-3, 8e-3 * (pos_err / 0.05));
    s_lambda = fmin(s_lambda, 0.15);
} else {
    // 近距离：更高下限防止振荡
    s_lambda = 2e-3 + 8e-3 * (pos_err / 0.05);
}
// 停滞提升
if (s_stagnation > 5) {
    s_lambda *= (1.0 + 0.3 * (s_stagnation - 5));
    s_lambda = fmin(s_lambda, 0.5);
}
```

### 权重调度表

4 级松弛调度，通过 `c_weight_schedule` 传递：

| 级别 | 位置权重 | 姿态权重 | 姿态限 (rad) | 用途 |
|------|---------|---------|-------------|------|
| 0 | [1,1,1] | [0.20,0.20,0.20] | π/6 | 一级求解 |
| 1 | [1,1,1] | [0.10,0.10,0.10] | π/4 | 二级松弛 |
| 2 | [1,1,1] | [0.03,0.03,0.03] | 70°·π/180 | 三级松弛 |
| 3 | [1,1,1] | [0.00,0.00,0.00] | ∞ | 仅位置 |

### 关节限位

通过 `c_joint_limits` 传递 6 关节的 `[lower, upper]` 对，在每步迭代后应用 (`cuda_kernels.cu:253-258`):

```cuda
double lo = c_joint_limits[i * 2 + 0];
double hi = c_joint_limits[i * 2 + 1];
s_q[i] = cuda_clamp(s_q[i] + s_dq[i], lo, hi);
```

## 参数传递流程

```mermaid
graph LR
    subgraph Host [CPU 端]
        LAUNCH[launch 参数] --> NODE[rtfg_solver_node]
        NODE --> CFG[SolverConfig 结构体]
        CFG --> INIT[CudaBatchIK::initialize()]
    end
    
    subgraph Transfer [常量内存上传]
        INIT --> CMA[cuMemcpyToSymbol]
    end
    
    subgraph Device [GPU 端]
        CMA --> CL[c_lambda_params]
        CMA --> CW[c_weight_schedule]
        CMA --> CJ[c_joint_limits]
        CL --> KERNEL[ik_batch_solve Kernel]
        CW --> KERNEL
        CJ --> KERNEL
    end
```

## 调参建议

- **增大 `max_iterations`**: 提高收敛率但增加延迟（设置见 `rtfg_solver_node.cpp:154-155`）
- **减小 `stagnation_epsilon`**: 更敏感地检测停滞（默认 1e-5）
- **增大 `clearance_threshold`**: 更保守的碰撞安全（默认 2 mm）
- **启用 `enable_window_solve`**: 对超长轨迹分段求解，减少内存占用
