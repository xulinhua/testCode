# Hs Robot Spot Room Sim

Isaac Sim Kit 扩展：在 Simple Room 里跑 Boston Dynamics Spot，前方 RGB-D / 点云，用于障碍建图测试。

完整说明见仓库根目录：

`/home/hs/testCode/simulation/src/spot_room_sim/README.md`

## 启动

```bash
cd /home/hs/testCode/simulation/src/spot_room_sim/hs.robot.spot_room_sim
bash scripts/start_isaac.sh
```

## 流程

1. 面板 **Hs Robot Spot Room Sim** -> **Load**
2. Timeline **Play**
3. `ROS_DOMAIN_ID=31` 下发 `/cmd_vel`，订阅相机与 TF

## 话题速查

默认同时发布 **color / depth / points / camera_info**。

- `/cmd_vel` `/odom` `/tf` `/tf_static` `/joint_states`
- `/cam_0/color/image_raw`（`cam_0_color_optical_frame`）
- `/cam_0/depth/image_raw`（`cam_0_depth_optical_frame`）
- `/cam_0/depth/points`
- `/cam_0/depth/camera_info`

TF：`base_link → cam_0_link`（Z 上）→ `*_frame` → `*_optical_frame`（Z 前，`rpy(-π/2,0,-π/2)`，与 cam_mgr/Gazebo 一致）。

## 资产

```bash
python3 scripts/download_assets.py
python3 scripts/make_lite_textures.py
```
