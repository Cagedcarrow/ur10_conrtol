# Rolling Planner 设计

更新时间: 2026-06-02

## 1. 目标

把“整条 1300 点一次性求解”演进为:

```mermaid
flowchart LR
  A[未来窗口] --> B[窗口求解]
  B --> C[执行]
  C --> D[滚动更新]
  D --> A
```

本轮只建立框架，不做完整闭环。

## 2. 接口

文件:
- `include/assembly_rtfg_cpp/rolling_planner.h`
- `src/rolling_planner.cpp`

当前接口持有:
- `window_size`
- `window_overlap`
- `enable_window_solve`

## 3. 当前实现

现在的 `RollingPlanner` 只是框架:

- 如果 `enable_window_solve=false`，直接调用连续求解器
- 如果 `enable_window_solve=true`，只截取前 `window_size` 个点作为窗口求解

这意味着:
- 能跑
- 语义明确
- 但还不是完整的实时闭环规划器

## 4. 为什么需要它

一次性求解整条轨迹的问题是:
- 前段失败会影响整条轨迹
- 代价函数难以只针对局部时域优化
- 不能很好适配实时执行中的反馈更新

滚动窗口的优势:
- 每次只看局部未来
- 失败更容易局部重试
- 更接近工业机器人在线规划方式

## 5. 复杂度

设窗口大小为 `W`，总点数为 `N`:

- 全局一次求解: `O(N * K * P)`
- 滚动窗口: `O((N / step) * W * K * P)`

实际收益来自:
- 更短的重试链
- 更少的无效搜索
- 更好的局部性

## 6. 代码示例

```cpp
rtfg::RollingPlanner planner(cfg);
if (planner.enabled()) {
  auto result = planner.solve(solver, target_tforms, segment_names, current_q, home_q);
}
```

