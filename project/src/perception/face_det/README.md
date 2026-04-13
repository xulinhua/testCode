# 重新构建
colcon build --packages-select face_det_alg face_det

# 重新运行
source install/setup.bash
ros2 launch face_det face_det.py camera_type:=Gemini


# 多相机配置运行
ros2 run bas_sys_config_ros sys_config_ros_node
ros2 run cam_mgr_ros cam_mgr_node
ros2 run face_det face_det

ros2 service call /camera_control cam_mgr_ros/srv/CameraControl \
  "{cam_id: 0, sence_id: 1, operate_type: 3}"

ros2 launch face_det face_det.py camera_id:=0 camera_type:=orbbec arm_id:=0
ros2 launch face_det face_det.py camera_id:=1 camera_type:=realsense arm_id:=0

---

## 交互指令说明

### 话题信息
- **话题名称**: `/face_interaction`
- **消息类型**: `std_msgs/String`
- **发布者**: `face_detection_node`

### 指令类型

#### 1. `greeting` - 打招呼指令
- **触发条件**: 检测到人脸从0变为1，且人脸在1.5米范围内
- **场景**: 有人走近并进入触发区域时
- **建议响应**: 机器人可执行打招呼动作（如挥手、点头、语音问候等）

#### 2. `goodbye` - 挥手离别指令
- **触发条件**: 检测到人脸数量从1+变为0（所有人脸消失）
- **场景**: 有人离开视野区域时
- **建议响应**: 机器人可执行挥手告别动作或语音道别

#### 3. `multi_interaction` - 多人互动指令
- **触发条件**: 指定距离范围内检测到的人脸数量达到或超过阈值（默认3人）
- **触发距离**: 默认3米范围内（可通过参数配置）
- **触发阈值**: 默认3人（可通过参数配置）
- **场景**: 多人同时出现在互动区域时
- **建议响应**: 机器人可执行多人互动动作（如群体问候、舞蹈表演等）
- **注意**: 该指令仅在首次达到阈值时触发一次，直到人数降到阈值以下才会重置

### 参数配置

多人互动参数可在launch文件中动态配置：
