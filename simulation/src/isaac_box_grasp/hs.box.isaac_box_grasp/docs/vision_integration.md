# 外部识别项目对接

扩展 `hs.box.isaac_box_grasp` 不内置视觉推理，只提供仿真出流与抓取执行。

## 订阅（来自本扩展）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/box_grasp/camera/color/image_raw` | `sensor_msgs/Image` | 默认可改 |
| `/box_grasp/camera/depth/image_raw` | `sensor_msgs/Image` | 深度 |
| `/box_grasp/camera/depth/points` | `sensor_msgs/PointCloud2` | 点云 |
| TF `world` → `camera_optical_frame` | | 与相机光学系一致 |

需在 Isaac 中 **Timeline Play** 后才有数据。

## 发布（识别项目输出）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/box_grasp/pose_world` | `geometry_msgs/PoseStamped` | `frame_id=world`，左臂抓取目标 |

扩展内 **Start** 后订阅该话题；收到位姿后驱动左臂（`left_wrist_roll_link` 链）。

## 移动底盘（可选）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/box_grasp/cmd_vel` | `geometry_msgs/Twist` | `linear.x` / `angular.z`，差速模型 |

Timeline Play 后生效，与抓取 Start 无关。

## 推荐联调顺序

1. 扩展 **Load**（机器人 + 相机；场景 USD 就绪后再填 Scene 字段）
2. **Play** → 检查相机话题与 TF
3. 识别节点发布 `pose_world` → **Start** → 观察左臂与 `/World/grasp_target_debug`
4. 需要动车时发布 `cmd_vel`

## 标定

- 面板 **Calibration** 可采样外参并 Export JSON 到 `calib_output/`
- 内参由 UI 分辨率与 FOV 计算，与 Stage 相机一致
