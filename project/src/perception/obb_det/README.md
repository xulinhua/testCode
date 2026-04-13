# OBB Detection Node

旋转目标检测（Oriented Bounding Box）节点，支持多种旋转检测算法（YOLO-OBB等）的动态切换。

## 架构设计

- **抽象接口**: 使用 `IOBBDetectionTask` 接口，与具体算法解耦
- **工厂模式**: 通过 `TaskFactory` 创建OBB检测任务实例
- **可扩展**: 支持添加新的旋转检测算法实现

## 构建与运行

```bash
# 构建
colcon build --packages-select obb_det

# 运行
source install/setup.bash
ros2 launch obb_det obb_det.py
```

## 多相机配置运行

```bash
# 启动系统配置节点
ros2 run bas_sys_config_ros sys_config_ros_node
ros2 run cam_mgr_ros cam_mgr_node

# 启动OBB检测节点
ros2 run obb_det obb_det

# 控制相机
ros2 service call /camera_control cam_mgr_ros/srv/CameraControl \
  "{cam_id: 0, sence_id: 1, operate_type: 3}"
```

## 动态算法切换

```bash
# 切换OBB检测算法
ros2 service call /obb_det_node_0/switch_algorithm custom_msgs_comm/srv/SwitchAlgorithm \
  "{algorithm: 'yolo_obb', model_path: 'install/obb_det/models/cassette-obb.engine', engine_type: 'tensorrt'}"

# 切换到其他模型
ros2 service call /obb_det_node_1/switch_algorithm custom_msgs_comm/srv/SwitchAlgorithm \
  "{algorithm: 'yolo_obb', model_path: 'install/obb_det/models/airplane-obb.engine', engine_type: 'tensorrt'}"
```

## 运行时参数修改

```bash
# 修改置信度阈值
ros2 param set /obb_det_node_0 conf_threshold 0.5

# 修改 NMS 阈值
ros2 param set /obb_det_node_0 nms_threshold 0.45

# 修改最大检测数
ros2 param set /obb_det_node_0 max_detections 100

# 启用/禁用GPU加速
ros2 param set /obb_det_node_0 use_gpu_preprocess true
ros2 param set /obb_det_node_0 use_gpu_postprocess true
```

## Launch 参数

```bash
# 指定相机参数启动
ros2 launch obb_det obb_det.py camera_id:=0 camera_type:=orbbec arm_id:=0
ros2 launch obb_det obb_det.py camera_id:=1 camera_type:=realsense arm_id:=0
```

## 服务接口

| 服务 | 类型 | 说明 |
|------|------|------|
| `/cam_{id}/get_obb_detection` | `custom_msgs_comm/GetOBBDetection` | 主动触发检测服务 |

### 服务调用示例

```bash
ros2 service call /cam_0/get_obb_detection custom_msgs_comm/srv/GetOBBDetection \
  "{request_id: 'test_001'}"
```

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cam_{id}/obb_det_image` | `sensor_msgs/Image` | 检测结果图像（带旋转框） |
| `/cam_{id}/obb_det_res` | `vision_msgs/Detection2DArray` | 检测结果（包含旋转角度和3D坐标） |

## 检测结果说明

OBB检测结果包含旋转角度信息：
- `center`: 旋转框中心点坐标
- `width`, `height`: 旋转框宽高
- `angle`: 旋转角度（弧度）
- `confidence`: 置信度
- `class_id`, `class_name`: 类别信息
- `position_3d`: 3D坐标（结合深度信息）
