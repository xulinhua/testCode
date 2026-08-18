# Hs Robot Spot Factory Sim

Isaac Sim Kit 扩展：在工厂仓库（`full_warehouse`，多货架）里跑 **Spot** 或 **Carter 小车**，前方 RGB-D / IMU / RTX 雷达，用于障碍建图测试。

完整说明见仓库根目录：

`/home/hs/testCode/simulation/src/spot_factory_sim/README.md`

## 启动

```bash
cd /home/hs/testCode/simulation/src/spot_factory_sim/hs.robot.spot_factory_sim
bash scripts/start_isaac.sh
```

## 流程

1. 面板 **Robot → Type** 选 Spot 或 Cart (Carter)
2. **Load** → Timeline **Play**
3. `ROS_DOMAIN_ID=31` 下发 `/cmd_vel`，订阅相机与 TF（话题两种机器人相同）

## 话题速查

默认同时发布 **color / depth / points / camera_info / imu / lidar points**。

- `/cmd_vel` `/odom` `/tf` `/tf_static` `/joint_states`
- `/cam_0/color/image_raw`（`cam_0_color_optical_frame`）
- `/cam_0/depth/image_raw`（`cam_0_depth_optical_frame`）
- `/cam_0/depth/points`
- `/cam_0/depth/camera_info`
- `/imu`（`imu_link`）
- `/lidar/points`（`lidar_link`）

TF：`base_link → cam_0_link` / `imu_link` / `lidar_link`；相机光学系 `rpy(-π/2,0,-π/2)`。

## 资产

```bash
python3 scripts/download_assets.py
python3 scripts/make_lite_textures.py
```
