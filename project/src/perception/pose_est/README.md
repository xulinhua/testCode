## Pose Est 6D位姿估计节点

### 功能
- 基于FoundationPose算法实现6D位姿估计（Register + Track）
- 集成YOLO实例分割(yolo_seg_alg)自动生成物体mask
- 支持在线深度相机数据输入
- 支持多相机配置
- 支持标定矩阵转换（camera_link -> base_link）

### 构建与运行

```bash
# 构建
colcon build --packages-select foundationpose_alg yolo_seg_alg pose_est

# 运行（自动从配置文件读取相机列表）
source install/setup.bash
ros2 launch pose_est pose_est.py

# 多相机配置运行
ros2 run bas_sys_config_ros sys_config_ros_node
ros2 run cam_mgr_ros cam_mgr_node
ros2 run pose_est pose_est

ros2 service call /camera_control cam_mgr_ros/srv/CameraControl \
  "{cam_id: 0, sence_id: 1, operate_type: 3}"

ros2 launch pose_est pose_est.py camera_id:=0 camera_type:=orbbec arm_id:=0
ros2 launch pose_est pose_est.py camera_id:=1 camera_type:=realsense arm_id:=0

### 话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/cam_{id}/pose_est_pose` | PoseStamped | 6D位姿（camera_link坐标系） |
| `/cam_{id}/pose_est_pose_base` | PoseStamped | 6D位姿（base_link坐标系） |
| `/cam_{id}/pose_est_image` | Image | 可视化结果图像 |
