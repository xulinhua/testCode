# HS Calib Suite — 总体计划

> 文档目录：`simulation/src/hs_calib_suite/docs/`  
> 架构参考：[TIER IV CalibrationTools](https://github.com/tier4/CalibrationTools)（只学其分层与流程，不复制代码）  
> 当前状态：**单目内参 + 手眼（棋盘）已可用**；其余类型按阶段推进

---

## 1. 目标

在 `simulation` 工作空间中建设统一的标定软件 `hs_calib_suite`，具备：

- 常见靶标：棋盘格、ArUco / AprilTag、ChArUco、直角三面靶等
- 常见标定：相机内参、传感器外参、手眼标定、多传感器联合标定等
- 清晰分层：**算法库**与 **ROS 节点**分离；Qt 界面只负责配置、采集编排与结果展示
- 兼容 ROS 2 Humble；支持不启动 ROS、仅调用算法库的离线用法

本工程**独立实现**，不依赖、不移植仓库内既有标定包（如 `calib_sim`、`project/src/calib`）。详见 [MIGRATION.md](MIGRATION.md)。

---

## 2. 技术选型（P0 已定）

使用**一个** ament 功能包 `hs_calib_suite`，用目录区分职责：

| 目录 | 职责 | 技术 |
|------|------|------|
| `src/core`、`include/.../core` | 标定算法（不依赖 ROS） | C++17、Eigen、OpenCV；子目录 `targets/ detectors/ calibrators/ io/ util/` |
| `src/ros`、`include/.../ros` | ROS 节点（订阅、TF、服务） | rclcpp |
| `src/gui`、`include/.../gui` | 标定管理界面 | Qt5 Widgets |
| `msg/`、`srv/` | 消息与服务定义 | rosidl |
| `config/` | 节点参数 | YAML；`ros2 run` 时自动加载 |
| `docs/` | 设计文档 | Markdown |

业务与算法使用 C++；不引入 Python 业务逻辑（rosidl 代码生成除外）。

---

## 3. 设计约束

1. `core/` 不得包含 `rclcpp`、`sensor_msgs` 等 ROS 头文件。  
2. `ros/` 只做通信与参数：订阅话题、查询 TF、提供服务、读取配置，并把观测数据交给 `core/`。  
3. `gui/` 不做数值优化求解；可离线直接调用 `core/`，或在线通过服务调用 `ros/` 节点。  
4. 新增标定能力须在 `CalibratorRegistry` 注册，并在 [TAXONOMY.md](TAXONOMY.md) 中补充类型说明。

---

## 4. 目录结构

```text
simulation/src/hs_calib_suite/
  include/hs_calib_suite/{core,ros,gui}/
  src/{core,ros,gui}/
  msg/  srv/
  config/          # 节点参数；ros2 run 自动加载
  docs/
  CMakeLists.txt  package.xml  README.md
```

启动方式：参数写在 `config/*.yaml`，使用 `ros2 run`；默认不使用 launch 文件。

```bash
cd /home/hs/testCode/simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select hs_calib_suite --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

ros2 run hs_calib_suite placeholder_calibrator
ros2 run hs_calib_suite hs_calib_gui
```

---

## 5. 与 TIER IV 的对应关系

| TIER IV 概念 | 本工程对应 |
|--------------|------------|
| `sensor_calibration_manager` | `gui/` 标定管理界面 |
| 外参标定节点 + 服务 | `ros/` + `srv/Calibrate` |
| Project / Calibrator 注册 | `core/` 注册表 + 界面侧项目配置 |
| 结果坐标系后处理（如 optical ↔ link） | 计划中的 `ResultPostProcessor`（P2） |
| 内参工具较弱、手眼能力不足 | 本工程将内参、双目、手眼作为重点能力 |

详见 [TIER4_GAP.md](TIER4_GAP.md)。

---

## 6. 开发阶段

| 阶段 | 交付内容 |
|------|----------|
| **P0** | 单包骨架、接口占位、文档、Qt **四页线框**（首页/配置/工作台/复核） |
| **P1** | 棋盘检测；**相机内参已可用**（离线/ROS）；双目/三面仍为后续 |
| **P2** | **手眼 eye_in_hand / eye_to_hand 已可用**（棋盘 PnP + CSV/TF）；激光外参等仍为后续 |
| **P3** | 激光–激光 / 地面约束；多相机套件；时间同步标定；传感器套件参数导出 |

当前可运行：`smoke_cam_intrinsics`、`smoke_handeye`、`hs_calib_gui`。详见 [HANDEYE_MONO.md](HANDEYE_MONO.md)。

- 类型与靶标表：[TAXONOMY.md](TAXONOMY.md)  
- 架构与接口：[ARCHITECTURE.md](ARCHITECTURE.md)  
- UI / 算法类图：[CLASS_DIAGRAMS.md](CLASS_DIAGRAMS.md)  
- 界面概念：[UI_CONCEPT.md](UI_CONCEPT.md)

---

## 7. P0 验收清单

- [x] 功能包 `hs_calib_suite` 可被 ament / colcon 编译安装  
- [x] 已定义 `Calibrate`、`GetCalibratorInfo` 服务  
- [x] `core/` 已有 `CalibratorBase` 与注册表占位  
- [x] 可执行文件 `hs_calib_gui` 四页线框可启动（首页 / 配置 / 工作台 / 复核）  
- [x] docs 含 UI 概念、架构与 CLASS_DIAGRAMS  
- [x] 本目录六份文档齐全  
