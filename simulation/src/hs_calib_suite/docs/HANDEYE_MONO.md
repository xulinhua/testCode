# 单目内参、三面与手眼

## 已接通

| ID | 说明 |
|----|------|
| `cam_intrinsics` | 多姿态平面靶 → K/D YAML（chessboard / charuco / aruco_grid / circles_*） |
| `trihedral_oneshot` | 直角三面 ChArUco（推荐）或棋盘：单帧（≥2 面，搜焦距+PnP）或少帧 `calibrateCamera` |
| `eye_in_hand` | 末端相机：求解 gripper→camera |
| `eye_to_hand` | 固定相机：求解 base→camera |

手眼位姿：离线 CSV（`image,tx,ty,tz,qx,qy,qz,qw`）或在线 TF（`base_frame`→`gripper_frame`）。需先提供内参 YAML。

```bash
ros2 run hs_calib_suite smoke_cam_intrinsics [/tmp/out_dir]
ros2 run hs_calib_suite smoke_handeye
ros2 run hs_calib_suite hs_calib_gui
```

配置见包内 `config/cam_intrinsics.yaml`、`config/trihedral_oneshot.yaml`、`config/eye_*.yaml`。  
总览与约束见根目录 [README.md](../README.md)。
