# 性能分析 — 文档索引

本目录分析 `assembly_rtfg_cuda` 性能数据和加速比，涵盖论文中的加速比数据、ncu 实测指标、Roofline 模型验证、三版本对比和 Amdahl 定律分析。

| 文件 | 内容 |
|------|------|
| [01_speedup_analysis.md](01_speedup_analysis.md) | 843× 加速比详细分析 |
| [02_ncu_metrics.md](02_ncu_metrics.md) | Nsight Compute 实测指标 |
| [03_roofline_model.md](03_roofline_model.md) | Roofline 模型验证与理论分析 |
| [04_three_versions_comparison.md](04_three_versions_comparison.md) | MATLAB → C++ → CUDA 三版本对比 |
| [05_amdal_law.md](05_amdal_law.md) | Amdahl 定律与端到端流水线分析 |
