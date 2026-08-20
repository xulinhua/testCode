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
│   │   ├── io/            # YAML 读写、板位姿
│   │   └── util/          # ImageFrame 桥接、通用图像工具（to_gray 等）
│   ├── ros/
│   └── gui/
│       ├── window/        # 主窗口
│       ├── panels/        # 配置面板等
│       ├── session/       # 会话编排
│       ├── bridges/       # ROS 图像 / TF
│       ├── widgets/       # 预览图像控件（缩放/平移，无新依赖）
│       ├── log/           # 日志等级 + rclcpp 终端输出
│       ├── data/          # CSV 等位姿存储
│       └── theme/         # 样式主题
├── src/
│   ├── core/{base,types,registry,targets,detectors,calibrators,io,tools,util}/
│   ├── ros/
│   └── gui/{app,window,panels,session,bridges,widgets,log,data,theme}/
├── msg/  srv/
├── config/
└── docs/
```

---

## 3. 已实现能力

| 标定器 ID             | 靶标                                                            | 说明                                                                            |
| --------------------- | --------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| `cam_intrinsics`    | 平面棋盘 / ChArUco / ArUco 阵列 / 圆点                          | 多姿态 → Brown / Kannala–Brandt / CMei → 内参 YAML                          |
| `stereo_intrinsics` | 同左（左右分侧采集）                                            | 左右目各自多姿态内参 → `camera_left.yaml` + `camera_right.yaml`            |
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

```bash
cd /home/hs/testCode/simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select hs_calib_suite --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

依赖：ROS 2 Humble、OpenCV、Eigen3、Qt5 Widgets、`cv_bridge`、`tf2_ros`。

---

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
| [docs/TIER4_GAP.md](docs/TIER4_GAP.md)           | 与 TIER IV 对比          |
| [docs/MIGRATION.md](docs/MIGRATION.md)           | 与仓库内旧标定代码的边界 |

新增标定器须在 `CalibratorRegistry` 注册，并在 `TAXONOMY.md` 补充说明。
