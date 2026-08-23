# HS Calib Suite

统一标定软件（ROS 2 Humble / C++17 单功能包）。分层思路参考 [TIER IV CalibrationTools](https://github.com/tier4/CalibrationTools)，代码独立实现。

> 当前可用：`cam_intrinsics`、`stereo_intrinsics`、`stereo_extrinsics`、`trihedral_oneshot`、`eye_in_hand`、`eye_to_hand`、检测调试 `detect_lab_identify` / `detect_lab` / `detect_lab_full`，以及 Qt 标定管理界面。
> **项目工作区**：一项目一文件夹（`project.yaml` + `config/` + `images/` + `results/`），由 `gui/projects/ProjectWorkspace` 管理；清单扫描见 `ProjectCatalog`。
> 仿真采图可配合独立扩展 [`isaac_calib_sim`](../isaac_calib_sim/)（无共享代码耦合）。

---

## 1. 设计约束

| 规则              | 说明                                                               |
| ----------------- | ------------------------------------------------------------------ |
| 算法在`core/`   | 数值求解（检测、PnP、`calibrateCamera`、手眼等）只允许放在算法库 |
| `core/` 无 ROS  | 不得包含`rclcpp` / `sensor_msgs` 等                            |
| `ros/` 只做 I/O | 话题、TF、服务、参数；把观测交给`core/`                          |
| `gui/` 不做求解 | 配置、采集编排、预览与导出；可直接调`core/` 或经服务调节点       |
| 启动方式          | 参数写在`config/*.yaml`，优先 `ros2 run`（默认不用 launch）    |
| 与旧工程边界      | 不依赖、不移植`calib_sim`、`project/src/calib/*`               |

注释规范：公开头文件使用 Doxygen（`/// \brief` / `\\param` / `\\return`），**正文用中文**；标识符与 API 名为英文。
`.cpp` 中每个对外实现与关键内部辅助函数也应有一行 `/// \brief`；算法步骤用短块注释标明阶段。
适用范围：`core/`、`gui/`、`ros/` 全层；与解码无关的通用图像操作放在 `core/util/cv_image_ops.*`（如 `to_gray`、`guess_K`），勿在各检测器内复制。
Cursor 规则：仓库 `.cursor/rules/hs-calib-suite-comments.mdc`（改本包代码时自动提醒）。

---

## 2. 目录结构

```text
hs_calib_suite/
├── include/hs_calib_suite/
│   ├── core/
│   │   ├── base/          # *_base、interfaces
│   │   ├── types/         # 公共数据结构
│   │   ├── registry/      # 标定器注册表
│   │   ├── targets/       # 靶标几何模型
│   │   ├── detectors/     # 特征检测
│   │   ├── calibrators/   # 标定求解器
│   │   │   └── intrinsics/  # Tier4 内参：采集双库、流水线、统计 JSON
│   │   ├── io/            # YAML 读写、板位姿
│   │   └── util/          # ImageFrame 桥接、通用图像工具（to_gray 等）
│   ├── ros/
│   └── gui/
│       ├── window/        # 主窗口
│       ├── panels/        # 配置面板等
│       ├── session/       # 会话编排
│       ├── intrinsics/    # 内参工作台右栏、参数弹窗、预览叠加
│       ├── plotting/      # Tier4 统计图（异步 matplotlib / 导出）
│       ├── bridges/       # ROS 图像 / TF / Rosbag
│       ├── widgets/       # 预览图像控件（缩放/平移，无新依赖）
│       ├── log/           # 日志等级 + rclcpp 终端输出
│       ├── data/          # CSV 等位姿存储
│       └── theme/         # 样式主题
├── scripts/               # matplotlib 统计图（install/share 下供运行时调用）
├── third_party/           # apriltag、ceres_intrinsic_camera_calibrator（vendored）
├── src/
│   ├── core/{base,types,registry,targets,detectors,calibrators,io,tools,util}/
│   ├── ros/
│   └── gui/{app,window,panels,session,intrinsics,plotting,bridges,widgets,log,data,theme}/
├── msg/  srv/
├── config/
└── docs/
```

---

## 3. 已实现能力

| 标定器 ID             | 靶标                                                            | 说明                                                                            |
| --------------------- | --------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| `cam_intrinsics`    | 平面棋盘 / ChArUco / ArUco 阵列 / AprilGrid / 圆点              | Tier4 流水线；训练/评估双库；工作台右栏；**三张 matplotlib 统计图**（采集分布 / single-shot 柱状 / RMS 热力图）；复核页「标定统计…」与导出 PNG |
| `stereo_intrinsics` | 同左（左右分侧采集）                                            | 复用内参 Tier4 工作台布局；左右目各自多姿态 → `camera_left.yaml` + `camera_right.yaml` |
| `stereo_extrinsics` | 同左（成对左右观测）                                            | 固定内参 → `stereoCalibrate` + 校正 → `stereo_extrinsics.yaml`              |
| `trihedral_oneshot` | 直角三面**ChArUco**（推荐）/ 棋盘；正方形面 + 约 1 格白边 | 单帧：≥2 面，搜焦距 + PnP；多帧：`calibrateCamera`；三面可同码，几何聚类分面 |
| `eye_in_hand`       | 棋盘 + 位姿                                                     | 末端相机：求解 gripper→camera                                                  |
| `eye_to_hand`       | 棋盘 + 位姿                                                     | 固定相机：求解 base→camera                                                     |
| `detect_lab_identify` | 自动试探 | 标定板类型识别：只判类型，不管尺寸；输出 Top-K |
| `detect_lab`        | 同上 + 三面                                                     | 局部特征检测：非完整板尽量检出（Thorough/Fast）                               |
| `detect_lab_full`   | 同上                                                            | 完整标定板检测：残缺帧直接失败                                                   |

手眼位姿来源：离线 CSV（`image,tx,ty,tz,qx,qy,qz,qw`）或在线 TF（`base_frame`→`gripper_frame`）；需求先提供内参 YAML。

默认配置：

| 文件                                                              | 用途                 |
| ----------------------------------------------------------------- | -------------------- |
| [`config/cam_intrinsics.yaml`](config/cam_intrinsics.yaml)       | 单目棋盘参数         |
| [`config/stereo_intrinsics.yaml`](config/stereo_intrinsics.yaml) | 双目各自内参         |
| [`config/stereo_extrinsics.yaml`](config/stereo_extrinsics.yaml) | 双目相对外参         |
| [`config/trihedral_oneshot.yaml`](config/trihedral_oneshot.yaml) | 三面 ChArUco（推荐） |
| [`config/trihedral_chess.yaml`](config/trihedral_chess.yaml)     | 三面纯棋盘（仅角点） |
| [`config/eye_in_hand.yaml`](config/eye_in_hand.yaml)             | 眼在手上             |
| [`config/eye_to_hand.yaml`](config/eye_to_hand.yaml)             | 眼在手外             |

---

## 4. 编译

在 **ROS 2 工作空间根目录**（含 `src/hs_calib_suite` 的那一层，例如本机可能是 `~/project/testCode/simulation`）执行：

```bash
cd <你的 simulation 工作空间根目录>   # 必须成功；pwd 应能看到 src/hs_calib_suite
source /opt/ros/humble/setup.bash
colcon build --packages-select hs_calib_suite --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

> **常见误报**：若 `cd` 路径写错，会在 `~` 下执行 `colcon`，扫描到家目录里其他工程（如 `dify-*`）并出现 `ModuleNotFoundError: flask` 等无关错误——与 `hs_calib_suite` 无关，先修正 `cd` 路径即可。

依赖：ROS 2 Humble、OpenCV、Eigen3、Qt5 Widgets、`cv_bridge`、`tf2_ros`、**Ceres Solver**（`libceres-dev`，用于 Ceres/C2 内参求解）。

```bash
# Ubuntu / Jetson（示例）
sudo apt install libceres-dev
```

内参 **Brown 模型** 在 GUI「求解预设」可选：

| 预设 | 求解器 | 说明 |
|------|--------|------|
| General | OpenCV | k1–k2，采集 RMS≤0.5 px |
| C1 | OpenCV | k1–k3，采集更严（RMS≤0.3 px） |
| Ceres | Ceres | k1–k3 + 有理畸变，系数正则 0.2 |
| C2 | Ceres | 同 Ceres，并启用 FOV 正则 |

Tier4 `ceres_intrinsic_camera_calibrator` 以 Apache-2.0  vendored 于 `third_party/ceres_intrinsic_camera_calibrator/`（仅标定器，无 pybind/ROS 依赖）。
Kannala–Brandt / CMei 仍走 OpenCV 原生路径。

内参 Tier4 UI 与统计图：

| 文档 | 说明 |
|------|------|
| [`docs/TIER4_FUSION_PLAN.md`](docs/TIER4_FUSION_PLAN.md) | 融合方案与实现状态 |
| [`docs/TIER4_INTRINSICS_UI_SPEC.md`](docs/TIER4_INTRINSICS_UI_SPEC.md) | 参数与界面模块规格 |
| [`docs/TIER4_INTRINSICS_STATS.md`](docs/TIER4_INTRINSICS_STATS.md) | **三张统计图**、异步绘制、`gui.stats_backend`、蓝/橙柱语义 |

配置 `gui.stats_backend`：`qt`（默认，采集统计为轻量 Qt 图）或 `matplotlib`（完整 Tier4 三图 + 导出 PNG）。复核页导出 YAML 时同步写入 `calibration_data_statistics.png`、`calibration_result_vs_singleshot.png`、`calibration_result_rms.png`（需 matplotlib 后端且已标定）。

## 5. 运行

```bash
# 标定管理界面（首页选类型 → 配置 → 工作台采集 → 复核导出）
ros2 run hs_calib_suite hs_calib_gui

# 冒烟
ros2 run hs_calib_suite smoke_cam_intrinsics [/tmp/out_dir]
ros2 run hs_calib_suite smoke_handeye

# 占位节点（服务骨架）
ros2 run hs_calib_suite placeholder_calibrator
```

GUI 可「文件 → 重新加载默认棋盘配置」从 `config/*.yaml` 刷新靶标参数。
在线图像：订阅 `sensor_msgs/Image`（例如 Isaac 扩展发布的 `/calib_sim/camera/image_raw`）。

---

## 6. 与 Isaac 仿真

独立包 [`isaac_calib_sim`](../isaac_calib_sim/)：在 Isaac Sim 中生成棋盘 / 三面靶并发布相机话题，供本 GUI 采集。
双方**不共享源码、不互相改工程**；仅通过 ROS 话题衔接。

---

## 7. 文档

| 文档                                            | 说明                     |
| ----------------------------------------------- | ------------------------ |
| [docs/PLAN.md](docs/PLAN.md)                     | 总体计划与阶段           |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)     | 分层、接口与数据流       |
| [docs/CLASS_DIAGRAMS.md](docs/CLASS_DIAGRAMS.md) | UI / 算法类图            |
| [docs/TAXONOMY.md](docs/TAXONOMY.md)             | 靶标与标定类型一览       |
| [docs/UI_CONCEPT.md](docs/UI_CONCEPT.md)         | 界面分区与操作流程       |
| [docs/HANDEYE_MONO.md](docs/HANDEYE_MONO.md)     | 单目内参与手眼用法       |
| [docs/TIER4_FUSION_PLAN.md](docs/TIER4_FUSION_PLAN.md)           | Tier4 内参融合方案与交付状态 |
| [docs/TIER4_INTRINSICS_UI_SPEC.md](docs/TIER4_INTRINSICS_UI_SPEC.md) | Tier4 内参 UI / 参数规格 |
| [docs/TIER4_INTRINSICS_STATS.md](docs/TIER4_INTRINSICS_STATS.md) | Tier4 统计图架构与使用 |
| [docs/TIER4_GAP.md](docs/TIER4_GAP.md)           | 与 TIER IV 对比          |
| [docs/MIGRATION.md](docs/MIGRATION.md)           | 与仓库内旧标定代码的边界 |

新增标定器须在 `CalibratorRegistry` 注册，并在 `TAXONOMY.md` 补充说明。
