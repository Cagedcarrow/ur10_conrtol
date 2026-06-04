# assembly_rtfg_cpp 功能包文档

**版本**: 3.0.0  
**最后更新**: 2026-06-04  
**ROS2 发行版**: Humble  
**硬件平台**: UR10 六轴机械臂  

---

本文档完整介绍了 `assembly_rtfg_cpp` 功能包的设计、实现、优化和性能表现。

## 目录

1. [整体介绍](assembly_rtfg_cpp_overview.md#1-整体介绍)
2. [节点信息图](assembly_rtfg_cpp_overview.md#2-节点信息图)
3. [运动学求解算法](assembly_rtfg_cpp_overview.md#3-运动学求解算法)
4. [相比 MATLAB 版本 MEX 的优化之处](assembly_rtfg_cpp_overview.md#4-相比-matlab-版本的优化之处)
5. [本功能包 GUI 程序的功能](assembly_rtfg_cpp_overview.md#5-本功能包-gui-程序的功能)
6. [耗时最多的地方，是如何用 C++ 优化的](assembly_rtfg_cpp_overview.md#6-耗时最多的地方是如何用-c-优化的)
7. [性能对比（与原版本 MATLAB MEX）](assembly_rtfg_cpp_overview.md#7-性能对比matlab-mex-vs-ros2-c)
8. [可以用 CUDA 13.3 进行重构改写的地方](assembly_rtfg_cpp_overview.md#8-可以用-cuda-133-进行重构改写的地方)
9. [附录 A：已知问题](assembly_rtfg_cpp_overview.md#附录-a已知问题)

## 快速导航

| 目的 | 文档章节 |
|------|---------|
| 了解功能包整体架构 | [章节 1](assembly_rtfg_cpp_overview.md#1-整体介绍) |
| 查看 ROS2 节点和 Service 定义（5 个 Service） | [章节 2](assembly_rtfg_cpp_overview.md#2-节点信息图) |
| 理解 IK 求解算法细节 | [章节 3](assembly_rtfg_cpp_overview.md#3-运动学求解算法) |
| 对比 MATLAB 和 C++ 实现差异 | [章节 4](assembly_rtfg_cpp_overview.md#4-相比-matlab-版本的优化之处) |
| 使用 GUI 程序（5 按钮 MATLAB 风格流程） | [章节 5](assembly_rtfg_cpp_overview.md#5-本功能包-gui-程序的功能) |
| 了解性能瓶颈和优化策略 | [章节 6](assembly_rtfg_cpp_overview.md#6-耗时最多的地方是如何用-c-优化的) |
| 查看性能对比数据 | [章节 7](assembly_rtfg_cpp_overview.md#7-性能对比matlab-mex-vs-ros2-c) |
| CUDA 重构规划 | [章节 8](assembly_rtfg_cpp_overview.md#8-可以用-cuda-133-进行重构改写的地方) |
| 已知问题（QTextCursor 警告） | [附录 A](assembly_rtfg_cpp_overview.md#附录-a已知问题) |

## 主要更新记录

### v3.0 (2026-06-04)
- 新增 `/rtfg/move_to_home` Service（第 5 个 Service），支持返回 URDF 初始位姿
- GUI 升级为 5 按钮 MATLAB 风格流程（①环境准备 ②RViz2 ③移到入泥点 ④拟合并播放 ⑤返回初始姿态）
- 新增 shell-launcher ROS2 环境初始化机制（替代 os.execve()）
- Playback 密度公式与 MEX 对齐（0.70°/seg + 3mm/seg + 0.30°/seg 三元复合准则）
- 仿真模式自动 joint-state playback 回退
- 新增 continuous_trajectory_solver、rolling_planner 模块
- 新增多个可视化 Topic（/rtfg/tcp_path, /rtfg/collision_markers, /rtfg/metrics）
- 文档更新反映最新代码结构

### v2.0 (2026-05-28)
- 初始 ROS2 C++ 实现
- 4 个 Service（load_config, fit_preview, execute_cached, move_to_start）
- 基本 GUI 界面

---

*详细文档请阅读 [assembly_rtfg_cpp_overview.md](assembly_rtfg_cpp_overview.md)*
