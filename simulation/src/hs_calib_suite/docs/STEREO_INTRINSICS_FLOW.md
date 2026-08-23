# 双目内参在线标定 — 流程与实现说明

> 版本：2026-08-23  
> 范围：`stereo_intrinsics` 任务 · Qt GUI + `session_controller_stereo.cpp`

---

## 1. 用户流程（6 步）

`stereo_intrinsics` 在通用五步之外增加 **校正验证** 页（仅该任务显示 6 步步骤条）：

| 步 | 页面 | 作用 |
|----|------|------|
| 1 | 选择任务 | 选 `stereo_intrinsics` |
| 2 | 数据源设置 | 选 ROS 左右话题 / 离线目录 / Bag；**此页不订阅图像** |
| 3 | 标定设置 | 靶标、求解器、`stereo_joint_refine` 等 |
| 4 | 采集求解 | **仅此页**订阅 ROS、实时预览、成对采集、求解 |
| 5 | 校正验证 | 立体校正预览、极线叠加、基线 / stereo RMS |
| 6 | 复核导出 | 左右内参摘要、残差图、导出 YAML |

单目 `cam_intrinsics` 仍为 5 步（无第 5 步校正页）。

---

## 2. ROS 图像订阅策略

为降低非采集页卡顿：

- **进入「采集求解」**：`start_ros_image_pipeline()` 按当前话题订阅
- **离开采集求解**（去设置 / 校正 / 复核等）：`stop_ros_image_pipeline()` 立即退订、取消检测、清空 live 帧与预览
- **数据源页选话题**：只写入 `solve_options`（`sync_pending_ros_topics`），不触发订阅
- 话题不稳定时仅影响采集页刷新，不阻塞其它 Tab 操作

调试台 `DetectLab` 单独订阅逻辑，与正式五步独立。

---

## 3. 成对采集与立体校正

### 3.1 成对采集

- 模式：`stereo_capture_mode = paired`（默认）
- 在线：检测完成后「成对采集」，左右各一条观测 + `StereoPairRecord`
- 原图缓存：检测线程保留 BGR，采集时写入临时目录 `hs_calib_capture_*`（`0000.png` 左、`0001.png` 右…）
- 离线 / Bag：路径写入 `StereoPairRecord.left_image_path` / `right_image_path`

### 3.2 求解

- **Tier4 路径**：左右 `IntrinsicsSessionState` 各自 `calibrate()`，再 `merge_stereo_calib_results`
- **经典路径**：`StereoIntrinsicsCalibrator` 按 `left` / `right` 分侧标定（`batch_` 中带 `frame_id`）

两条路径在成功后均调用：

1. `append_stereo_rectified_meta()` — `stereoCalibrate` + `stereoRectify`，写入 `stereo_rectified`、`R1/P1/Q`、`baseline_m`、`stereo_rms`
2. `rebuild_stereo_rectify_maps()` — `initUndistortRectifyMap` 供预览 `remap`

进入校正页时 `ensure_stereo_rectification()` 可为**已有标定结果**补建上述元数据与映射（无需重新求解）。

### 3.3 校正验证页

- 指标：基线 (m)、stereo RMS (px)、左右亮度一致性
- 预览：`stereo_rectified_preview()` → 校正图 + 绿色极线
- 图像对滑块：按 `stereo_pairs_` 数量；`load_stereo_pair_bgr()` 从缓存 / 离线路径 / Bag 加载

配置项（标定设置）：

| 键 | 说明 |
|----|------|
| `generate_stereo_rectified` | 默认 `true`；求解后是否做立体校正 |
| `stereo_joint_refine` | 默认 `false`；`stereoCalibrate` 是否联合优化内参 |
| `stereo_max_sync_ms` | 成对采集允许的最大 Δt（默认 30ms） |

---

## 4. 导出

| 文件 | 内容 |
|------|------|
| `camera_left.yaml` / `camera_right.yaml` | 分侧内参 |
| `stereo_rectified.yaml` | 立体校正参数（`has_stereo_rectified` 时） |
| `images/original/`、`images/overlay/` | 分目录原图与检测叠加 |

复核页 `format_intrinsics_text` 在 `stereo_rectified=true` 时追加 `[stereo geometry / rectification]` 段。

---

## 5. 关键代码位置

| 模块 | 路径 |
|------|------|
| 双目会话逻辑 | `src/gui/session/session_controller_stereo.cpp` |
| 页面与校正 UI | `src/gui/window/main_window_pages.cpp`（`build_stereo_rectify_page`） |
| 订阅开关 | `main_window_workbench.cpp`：`start/stop_ros_image_pipeline` |
| 步骤条 6 步 | `main_window.cpp`：`uses_stereo_rectify_flow()` |
| ROS 双目桥 | `src/gui/bridges/ros_stereo_image_bridge.cpp` |
| 导出 | `src/core/io/export_camera_yaml.cpp` |

---

## 6. 常见问题

| 现象 | 原因 / 处理 |
|------|-------------|
| 校正页有 RMS 无图 | 经典路径曾未建 remap（已修）；或在线采集未缓存原图 → 补采几帧 |
| 复核有标定、校正页提示未生成 | 旧会话未写 `stereo_rectified` → 进入校正页自动 `ensure_stereo_rectification()` |
| 数据源页卡顿 | 确认未在非采集页订阅（日志不应出现 `双目订阅`） |
| 滑块 10/15 | 已改为以 `stereo_pairs_` 为准 |
