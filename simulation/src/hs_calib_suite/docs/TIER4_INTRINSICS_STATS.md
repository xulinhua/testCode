# Tier4 内参标定统计图

> 对齐 [tier4/CalibrationTools](https://github.com/tier4/CalibrationTools) `intrinsic_camera_calibrator/calibrators/utils.py` 中  
> `plot_calibration_data_statistics` 与 `plot_calibration_results_statistics`。  
> 参数开关见 [`TIER4_INTRINSICS_UI_SPEC.md`](TIER4_INTRINSICS_UI_SPEC.md) §8.4。

---

## 1. 三张统计图

| 图 | Tier4 窗口标题 | 本工程弹窗 | 导出 PNG |
|----|----------------|------------|----------|
| 采集分布 | （采集统计） | `IntrinsicsStatsDialog` | `calibration_data_statistics.png` |
| 标定 vs single-shot | Calibration result statistics vs single shot calibration | `IntrinsicsCalibrationBarsDialog` | `calibration_result_vs_singleshot.png` |
| 标定 RMS 热力图 | Calibration result statistics | `IntrinsicsCalibrationRmsDialog` | `calibration_result_rms.png` |

**打开方式**

- 工作台：标定完成后，若 `plot_calibration_*` 为 true，可自动弹出（取决于会话配置）。
- 复核页：「标定统计…」→ 依次打开上述三窗（需已标定且训练集 ≥ 3 帧；柱状图/RMS 图需 `has_calibrated`）。

**导出**：复核页「导出 YAML」时，若 `gui.stats_backend=matplotlib` 且标定已完成，三张 PNG 写入项目 `results/` 目录。

---

## 2. 后端切换 `gui.stats_backend`

| 值 | 行为 |
|----|------|
| `qt`（默认） | 采集统计为轻量 Qt 摘要图；**完整 Tier4 三图需 matplotlib** |
| `matplotlib` | C++ 导出 JSON → `share/hs_calib_suite/scripts/*.py` 渲染 PNG → GUI 显示 |

配置位置：标定设置页「统计图后端」，或 `config/cam_intrinsics.yaml`：

```yaml
gui.stats_backend: matplotlib
```

依赖：`python3`、`python3-matplotlib`（Jetson 可 `sudo apt install python3-matplotlib`）。  
修改 Python 脚本后**无需重编译**（`--symlink-install` 下 share 为符号链接），重新打开统计弹窗即可。

---

## 3. 架构（C++ 与 Python 解耦）

```text
SessionController / IntrinsicsSessionState
        │
        ▼
build_intrinsics_plot_input()          ← gui/plotting/intrinsics_plot_session.cpp
        │
        ▼
build_plot_pipeline_stages()           ← core/.../intrinsics_plot_statistics.cpp
  compute_intrinsics_pipeline_stage_views()
        │
        ├── export_collection_statistics_json()
        ├── export_calibration_bars_json()
        └── export_calibration_rms_json()
        │
        ▼
IntrinsicsPlotRenderer::render()       ← gui/plotting/intrinsics_plot_renderer.cpp
  调用 scripts/intrinsics_*_plot.py
        │
        ▼
IntrinsicsAsyncPlotController          ← 后台 std::thread，主线程回调更新 QPixmap
```

| 层 | 路径 | 职责 |
|----|------|------|
| 流水线阶段 | `core/calibrators/intrinsics/intrinsics_pipeline.*` | Training / Pre-RANSAC / Subsampled / Post / Evaluation 视图划分 |
| 统计 JSON | `core/calibrators/intrinsics/intrinsics_plot_statistics.*` | 纯数据导出，无 GUI |
| 会话输入 | `gui/plotting/intrinsics_plot_session.*` | 从 `SessionController` 组装 `IntrinsicsPlotInput` |
| 渲染 | `gui/plotting/intrinsics_plot_renderer.*` | 统一 matplotlib 子进程入口 |
| 异步 | `gui/plotting/intrinsics_async_plot_controller.*` | 避免统计计算阻塞 UI |
| 导出 | `gui/plotting/intrinsics_plot_export.*` | 复核页写 PNG |
| Python | `scripts/plot_common.py`、`intrinsics_collection_stats_plot.py`、`intrinsics_calibration_bars_plot.py`、`intrinsics_calibration_rms_plot.py` | 读 JSON 绘图 |

---

## 4. 柱状图：蓝柱 vs 橙柱（重要）

Tier4 `plot_calibration_vs_single_shot_calibration` 语义：

| 系列 | 计算方式 |
|------|----------|
| **蓝 — Calibrated intrinsics** | 全局标定 `K,D` + 该帧 `solvePnP` 位姿 → 重投影 RMS |
| **橙 — Single-shot intrinsics (lower bound)** | **仅该帧**调用 `calibrateCamera` 得到单帧内参 → 重投影 RMS |

橙柱不是「采集期 partial 模型」或「标定前 provisional 模型」套到各帧；而是每帧独立的单帧标定下界。  
实现：`compute_single_shot_view_rms()`（`intrinsics_pipeline.cpp`），柱状 JSON 由 `export_calibration_bars_json()` 写出。

采集统计五阶段子图（像面热力图、倾角热力图、深度直方图）见 `intrinsics_collection_stats_plot.py`，列标题与 Tier4 一致（Training / Pre rejection inliers / Subsampled / Post rejection inliers / Evaluation）。

---

## 5. 常见问题

| 现象 | 处理 |
|------|------|
| 统计弹窗空白或提示 matplotlib 失败 | 安装 `python3-matplotlib`，并将 `gui.stats_backend` 设为 `matplotlib` |
| 橙柱与蓝柱完全相同 | 已修复：确认使用含 `compute_single_shot_view_rms` 的版本并重新标定后打开统计 |
| 采集统计列标题与子图略重叠 | 调 `intrinsics_collection_stats_plot.py` 中 `figsize` / `wspace` / `title_pad` |
| 修改脚本不生效 | 确认 `colcon build --symlink-install`，并**关闭后重新打开**统计弹窗 |

---

## 6. 相关文档

- [`TIER4_INTRINSICS_UI_SPEC.md`](TIER4_INTRINSICS_UI_SPEC.md) — 参数与 UI 模块
- [`TIER4_FUSION_PLAN.md`](TIER4_FUSION_PLAN.md) — 融合方案与实现状态
- [`TIER4_GAP.md`](TIER4_GAP.md) — 与 Tier4 能力对比
