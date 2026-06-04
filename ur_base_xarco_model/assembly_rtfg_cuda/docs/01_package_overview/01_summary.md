# 功能包概述

## 基本信息

| 属性 | 值 |
|------|-----|
| 包名 | `assembly_rtfg_cuda` |
| 版本 | 0.1.0 |
| 描述 | CUDA 13.3 加速的 UR10 轨迹拟合 — GPU 批处理 IK 求解器 |
| 维护者 | liuxiaopeng |
| 编程语言 | C++17 + CUDA C++ |
| CUDA 版本 | 13.3 |
| GPU 架构 | sm_89 (Ada Lovelace, RTX 4060 Laptop) |
| ROS2 版本 | Humble Hawksbill |
| 构建系统 | CMake + colcon |

## 核心功能

`assembly_rtfg_cuda` 是 `assembly_rtfg_cpp` 的 GPU 加速版本，主要提供：

1. **GPU 批处理 IK 求解器**: 将 52,416 个独立 IK 任务映射到 GPU 线程层次结构，实现 843× 加速比
2. **URDF 约定的正运动学**: 使用 Rodrigues 公式支持任意轴旋转
3. **加权 DLS 迭代求解**: 自适应阻尼 + 停滞恢复
4. **ROS2 集成**: 通过标准 ROS2 服务/话题接口提供 IK 求解服务

## 关键性能指标

| 指标 | 值 |
|------|-----|
| IK 批处理 (273 目标) | 7.35 ms (GPU Kernel) |
| 单目标平均 | 27 μs |
| 平均迭代次数 | 7.9 |
| 加速比 (vs CPU C++) | **843×** (6.2 s → 7.35 ms) |
| 寄存器/线程 | 98 (零溢出) |
| 共享内存/Block | 1,676 bytes |
| Bank 冲突 | 0 |
| FP64 算术强度 | 157 FLOP/Byte (176× Ridge Point) |

## 设计目标

1. **实时性**: 批处理 IK 在 8 ms 内完成，端到端流水线 < 5 s
2. **精确性**: 位置误差 < 3 cm，姿态误差 < 30°
3. **可扩展性**: 增加目标数只需增加 Blocks
4. **可维护性**: 清晰的模块划分，完整的测试覆盖

## 与 CPU 版本的关系

```
assembly_rtfg_cpp (CPU 基版本)
    │
    ├── 共享: RobotModel, SolverConfig, IKSolverBase 接口
    ├── 共享: 服务定义 (srv), URDF, 配置文件
    ├── 共享: 碰撞检测 (FCL), 轨迹生成
    │
    └── assembly_rtfg_cuda (GPU 加速版本)
        ├── 替换: CudaBatchIK → IKSolverBase 实现
        ├── 新增: CUDA Kernels, DeviceBuffer, ConstantMemory
        └── 新增: 启动文件 (launch), 测试文件 (test)
```

## 文件数量统计

| 类别 | 数量 | 说明 |
|------|------|------|
| CUDA 源文件 | 5 | `.cu` + `.cuh` |
| C++ 源文件 | 4 | 从 cpp 包复制的 `.cpp` |
| 头文件 | 3 | `.h` |
| ROS2 相关 | 3 | launch, node |
| 配置文件 | 1 | CMakeLists.txt |
| 包描述 | 1 | package.xml |
| **总计** | **~16** | |

## 相关目录

- [文件清册](02_file_inventory.md)
- [构建系统](03_build_system.md)
- [节点架构](04_node_architecture.md)
