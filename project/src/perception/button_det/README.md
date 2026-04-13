执行流程

# 重新构建
colcon build --packages-select button_det

# 重新运行
source install/setup.bash
ros2 launch button_det button_det.py


# 多相机配置运行
ros2 run bas_sys_config_ros sys_config_ros_node
ros2 run cam_mgr_ros cam_mgr_node
ros2 run button_det button_det

ros2 service call /camera_control cam_mgr_ros/srv/CameraControl \
  "{cam_id: 0, sence_id: 1, operate_type: 3}"

ros2 launch button_det button_det.py camera_id:=0 camera_type:=orbbec arm_id:=0
ros2 launch button_det button_det.py camera_id:=1 camera_type:=realsense arm_id:=0
