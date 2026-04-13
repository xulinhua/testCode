# Instance Segmentation Node

通用实例分割节点，支持多种分割算法（YOLO-Seg、Mask R-CNN等）的动态切换。

## 架构设计

- **抽象接口**: 使用 `ISegmentationTask` 接口，与具体算法解耦
- **工厂模式**: 通过 `TaskFactory` 创建分割任务实例
- **可扩展**: 支持添加新的分割算法实现

## 构建与运行

```bash
# 构建
colcon build --packages-select object_seg

# 运行
source install/setup.bash
ros2 launch object_seg object_seg.py
```

## 多相机配置运行

```bash
# 启动系统配置节点
ros2 run bas_sys_config_ros sys_config_ros_node
ros2 run cam_mgr_ros cam_mgr_node

# 启动分割节点
ros2 run object_seg object_seg

# 控制相机
ros2 service call /camera_control cam_mgr_ros/srv/CameraControl \
  "{cam_id: 0, sence_id: 1, operate_type: 3}"
```

## 动态算法切换

```bash
# 切换分割算法
ros2 service call /object_seg_node_0/switch_algorithm custom_msgs_comm/srv/SwitchAlgorithm \
  "{algorithm: 'yolo_seg', model_path: 'install/object_seg/models/yolo11n-seg.engine', engine_type: 'tensorrt'}"

# 切换到其他模型
ros2 service call /object_seg_node_1/switch_algorithm custom_msgs_comm/srv/SwitchAlgorithm \
  "{algorithm: 'yolo_seg', model_path: 'install/object_seg/models/jetson-box.engine', engine_type: 'tensorrt'}"
```

## 运行时参数修改

```bash
# 修改置信度阈值
ros2 param set /object_seg_node_0 conf_threshold 0.6

# 修改 NMS 阈值
ros2 param set /object_seg_node_0 nms_threshold 0.5

# 修改掩码阈值
ros2 param set /object_seg_node_0 mask_threshold 0.5

# 修改最大检测数
ros2 param set /object_seg_node_0 max_detections 100
```

## Launch 参数

```bash
# 指定相机参数启动
ros2 launch object_seg object_seg.py camera_id:=0 camera_type:=orbbec arm_id:=0
ros2 launch object_seg object_seg.py camera_id:=1 camera_type:=realsense arm_id:=0
```

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cam_{id}/object_seg_image` | `sensor_msgs/Image` | 分割结果图像 |
| `/cam_{id}/object_seg_res` | `vision_msgs/Detection2DArray` | 分割结果（包含3D坐标） |
