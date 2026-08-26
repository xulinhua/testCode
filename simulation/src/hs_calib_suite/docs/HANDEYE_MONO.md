# 单目内参、三面与手眼

## 已接通

| ID | 说明 |
|----|------|
| `cam_intrinsics` | 多姿态平面靶 → K/D YAML（chessboard / charuco / aruco_grid / aprilgrid / circles_*） |
| `trihedral_oneshot` | 直角三面 ChArUco（推荐）或棋盘：单帧（≥2 面，搜焦距+PnP）或少帧 `calibrateCamera` |
| `eye_in_hand` | 末端相机：求解 `T_gripper_camera`（结果帧默认 `tool0` → `camera_optical_frame`） |
| `eye_to_hand` | 固定相机：求解 `T_base_camera`（结果帧默认 `base` → `camera_optical_frame`） |

手眼位姿：离线 CSV（`image,tx,ty,tz,qx,qy,qz,qw`）或在线 TF（`base_frame`→`gripper_frame`）。需先提供内参 YAML（或 CameraInfo）。

靶标：棋盘 / ChArUco / 单码 ArUco / ArUco 网格 / AprilGrid。算法：Tsai / Park / Horaud / Andreff / Daniilidis（OpenCV `calibrateHandEye`）。有效样本 ≥3，建议 ≥8。

```bash
ros2 run hs_calib_suite smoke_cam_intrinsics [/tmp/out_dir]
ros2 run hs_calib_suite smoke_handeye
ros2 run hs_calib_suite hs_calib_gui
```

## 话题与坐标系：写在 YAML 里

选择「眼在手上 / 眼在手外」或菜单 **文件 → 重新加载默认 YAML 配置** 时，会读入对应文件：

| 文件 | 用途 |
|------|------|
| [`config/eye_in_hand.yaml`](../config/eye_in_hand.yaml) | 眼在手上：图像话题、CameraInfo、TF 帧、算法、靶标 |
| [`config/eye_to_hand.yaml`](../config/eye_to_hand.yaml) | 眼在手外 |
| [`config/projects/*.yaml`](../config/projects/) | 项目级覆盖：`image_topic` / `camera_info_topic` / `pose_source` / 坐标系 |

现场改 YAML 即可，不必每次在 GUI 手填话题。刷新 ROS 话题列表时，**已填写的话题名会被保留**（即使当时未发布）。

常用键：

```yaml
source_mode: ros_topic          # offline | ros_topic | rosbag
image_topic: /camera/color/image_raw
camera_info_topic: /camera/color/camera_info
pose_source: tf                 # csv | tf
base_frame: base
gripper_frame: tool0
parent_frame: tool0             # 结果 T_parent_child
child_frame: camera_optical_frame
method: tsai
min_views: 8
target: chessboard
```

项目 YAML 若填写 `image_topic`，进入数据源页时会覆盖标定器默认话题（便于一机一配置）。

改包内 `config/*.yaml` 后需重新 `colcon build --packages-select hs_calib_suite`（GUI 从 install/share 读取）。项目工作区里的 `project.yaml` 可直接改，下次进入数据源即生效。

总览与约束见根目录 [README.md](../README.md)。
