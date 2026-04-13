# Object Detection Node

通用目标检测节点，支持多种检测算法（YOLO、DETR、SSD等）的动态切换。

## 架构设计

- **抽象接口**: 使用 `IDetectionTask` 接口，与具体算法解耦
- **工厂模式**: 通过 `TaskFactory` 创建检测任务实例
- **可扩展**: 支持添加新的检测算法实现

## 构建与运行

```bash
# 构建
colcon build --packages-select object_det

# 运行
source install/setup.bash
ros2 launch object_det object_det.py
```

## 多相机配置运行

```bash
# 启动系统配置节点
ros2 run bas_sys_config_ros sys_config_ros_node
ros2 run cam_mgr_ros cam_mgr_node

# 启动检测节点
ros2 run object_det object_det

# 控制相机
ros2 service call /camera_control cam_mgr_ros/srv/CameraControl \
  "{cam_id: 0, sence_id: 1, operate_type: 3}"
```

## 动态算法切换

```bash
# 切换到 DETR 算法
ros2 service call /object_det_node_0/switch_algorithm custom_msgs_comm/srv/SwitchAlgorithm \
  "{algorithm: 'detr', model_path: 'install/object_det/models/detr.engine', engine_type: 'tensorrt'}"

# 切换回 YOLO 算法
ros2 service call /object_det_node_1/switch_algorithm custom_msgs_comm/srv/SwitchAlgorithm \
  "{algorithm: 'yolo', model_path: 'install/object_det/models/coco_stairs.engine', engine_type: 'tensorrt'}"
```

## 运行时参数修改

```bash
# 修改置信度阈值
ros2 param set /object_det_node_0 conf_threshold 0.7

# 修改 NMS 阈值
ros2 param set /object_det_node_0 nms_threshold 0.5

# 修改最大检测数
ros2 param set /object_det_node_0 max_detections 50

# 启用/禁用GPU加速
ros2 param set /object_det_node_0 use_gpu_preprocess true
ros2 param set /object_det_node_0 use_gpu_postprocess true
```

## Launch 参数

```bash
# 指定相机参数启动
ros2 launch object_det object_det.py camera_id:=0 camera_type:=orbbec arm_id:=0
ros2 launch object_det object_det.py camera_id:=1 camera_type:=realsense arm_id:=0
```

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cam_{id}/object_det_image` | `sensor_msgs/Image` | 检测结果图像 |
| `/cam_{id}/object_det_res` | `vision_msgs/Detection2DArray` | 检测结果（包含3D坐标） |
