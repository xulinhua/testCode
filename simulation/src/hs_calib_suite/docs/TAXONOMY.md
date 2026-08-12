# 靶标类型与标定类型一览

## 1. 靶标（标定板 / Target）

| ID | 名称 | 几何 | 优先级 | 说明 |
|----|------|------|--------|------|
| `chessboard` | 棋盘格 | 平面 | P1 | OpenCV 棋盘角点检测 |
| `charuco` | ChArUco | 平面 | P1 | 棋盘角点 + ArUco 编码，遮挡时更稳 |
| `aruco_grid` | ArUco / AprilTag 阵列 | 平面 | P1 | 多个编码标记组成的平面板 |
| `aruco_single` | 单个编码标记 | 平面 | P1 | 手眼标定中常用 |
| `trihedral_chess` | 直角三面棋盘 | 三正交平面 | P1 | 适合少帧 / 单帧；夹角可配置 |
| `trihedral_charuco` | 直角三面 ChArUco | 三正交平面 | P1 | **推荐**的三面靶方案 |
| `trihedral_tag` | 直角三面纯 Tag | 三正交平面 | P2 | 检测稳定，但角点数量较少 |
| `circle_grid` | 圆点阵列 | 平面 | P2 | 工业相机标定中常见 |
| `custom_mesh` | 已知 CAD / 点云靶 | 任意 | P3 | 多传感器共用的已知三维靶 |

统一抽象链：`TargetModel`（三维模型）→ `Detector`（图像检测）→ `Observation`（一次观测）。详见 [ARCHITECTURE.md](ARCHITECTURE.md)。

直角三面靶要求：**面间夹角必须已知**（默认按 90°，或以实测角度写入模型）。夹角不准会给内外参带来系统性偏差。

---

## 2. 标定类型（Calibrator）

### A. 内参

| ID | 名称 | 对象 | 优先级 |
|----|------|------|--------|
| `cam_intrinsics` | 相机内参与畸变 | 单相机 | P1 |
| `stereo_intrinsics` | 左右目各自内参 | 双目相机 | P1 |
| `imu_intrinsics` | IMU 内参（偏置、尺度等） | IMU | P3 |

相机投影模型可选：`pinhole`（针孔）、`fisheye`（鱼眼）。

### B. 外参

| ID | 名称 | 传感器组合 | 优先级 | 与 TIER IV 的关系 |
|----|------|------------|--------|-------------------|
| `stereo_extrinsics` | 双目相对外参与立体校正 | 相机–相机 | P1 | TIER IV 非重点，本工程完整支持 |
| `trihedral_oneshot` | 直角三面单帧内外参 | 单目或双目 | P1 | **本工程特色** |
| `cam_lidar` | 相机–激光雷达外参 | 相机–激光 | P2 | 对应其 tag / 交互 / 建图类工具 |
| `lidar_lidar` | 激光雷达–激光雷达外参 | 激光–激光 | P3 | 对应其 mapping / 2D 类工具 |
| `base_lidar_ground` | 车体–激光（地面约束） | base–激光 | P3 | 对应其 ground plane 类工具 |
| `cam_cam_rig` | 多相机相对外参 | 多路相机 | P3 | — |
| `cam_radar` / `lidar_radar` | 与毫米波雷达相关的外参 | 雷达相关 | P3 | 对应其 marker radar 类工具 |

### C. 手眼标定

| ID | 名称 | 求解结果 | 优先级 |
|----|------|----------|--------|
| `eye_in_hand` | 眼在手上 | 末端执行器到相机的变换 | P2 |
| `eye_to_hand` | 眼在手外 | 机器人基座到固定相机的变换 | P2 |
| `known_target_mount` | 标定板安装位姿已知 | 在已知安装约束下估计固定相机外参 | P2 |

### D. 多传感器与系统级

| ID | 名称 | 说明 | 优先级 |
|----|------|------|--------|
| `sensor_kit_bundle` | 传感器套件联合标定 | 管理界面按顺序调用多个标定器 | P3 |
| `time_offset` | 时间偏移标定 | 例如相机与激光雷达之间的时间差 | P2 |
| `scale_align` | 尺度对齐 | 例如单目深度与双目 / 激光尺度对齐 | P2 |

---

## 3. 靶标与标定类型的适用关系（P1 重点）

|  | chessboard | charuco | aruco_* | circles_* | trihedral_* |
|--|:---:|:---:|:---:|:---:|:---:|
| `cam_intrinsics` | **已实现** | **已实现** | **已实现** | **已实现** | 用 `trihedral_oneshot` |
| `stereo_intrinsics` | 支持 | 支持 | 支持 | 支持 | 支持 |
| `stereo_extrinsics` | 支持 | 支持 | 支持 | 支持 | 支持（可单组左右图） |
| `trihedral_oneshot` | — | — | — | — | **已实现（ChArUco 推荐 / 棋盘可用）** |
| 手眼标定 | **已实现** | 可用 | **优先** | 可选 | 可选 |
| `cam_lidar` | 较少使用 | 可用 | **优先** | 较少使用 | 可选 |

「已实现」表示代码已接通；「支持」表示计划实现；「—」表示该组合不是目标方案。

---

## 4. 标定器 ID 命名

建议完整 ID：

```text
<类别>.<名称>
例如：
  intrinsic.cam_intrinsics
  extrinsic.stereo_extrinsics
  extrinsic.trihedral_oneshot
  handeye.eye_in_hand
```

上表中的短 ID 用于代码与服务字段；界面展示名称使用独立的 `display_name`（中英文均可）。
