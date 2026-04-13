cmake_minimum_required(VERSION 3.8)
project(custom_msgs_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)

set(msg_files
  "msg/AICoordinateData.msg"
  "msg/CalibrationCommand.msg"
  "msg/CalibrationResponse.msg"
  "msg/LaserScanData.msg"
  "msg/PointCloudData.msg"
  "msg/ProjectCommand.msg"
  "msg/ProjectStatus.msg"
  "msg/Command.msg"
  "msg/RobotPose.msg"
  "msg/CassetteWithHoles.msg"
  "msg/CassetteWithHolesArray.msg"
  "msg/InteractiveCommand.msg"
  "msg/VoiceCommand.msg"
  "msg/VoiceReply.msg"
  "msg/SpeechStatus.msg"  # 添加SpeechStatus消息文件
  "msg/AudioDeviceStatus.msg"  # 添加AudioDeviceStatus消息文件
  "msg/ElevatorCommand.msg"
  "msg/SingleMarkerInfo.msg"  # 添加SingleMarkerInfo消息文件
  "msg/RosCommInfo.msg"  # 添加RosCommInfo消息文件
  "msg/RobotStdPose.msg"  # 添加RobotStdPose消息文件
  "msg/BoolStamped.msg"  # 添加BoolStamped消息文件
)

set(srv_files
  "srv/ProjectControl.srv"
  "srv/GetMarkerDetection.srv"
  "srv/GetMarkerSrcImage.srv"
  "srv/GetCalibrationPoints.srv"
  "srv/GetMarkersInfo.srv"  # 添加GetMarkersInfo服务文件
  "srv/GetRobotPose.srv"  # 添加GetRobotPose服务文件
  "srv/GetCommInfo.srv"  # 添加GetCommInfo服务文件
  "srv/GetCamIntr.srv"  # 添加GetCamIntr服务文件
  "srv/KeyboardControl.srv"  # 添加KeyboardControl服务文件
  "srv/MoveToPose.srv"  # 添加MoveToPose服务文件
  "srv/GetOBBDetection.srv"  # 添加GetOBBDetection服务文件
  "srv/SwitchScene.srv"  # 添加SwitchScene服务文件
  "srv/SetImageSave.srv"  # 添加SetImageSave服务文件
  "srv/CameraControl.srv"  # 添加CameraControl服务文件
  "srv/SwitchAlgorithm.srv"  # 添加算法切换服务文件
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs vision_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# install msg files
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

# install srv files
install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

# 添加测试部分以避免catkin_pkg错误
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 避免使用需要catkin_pkg的lint工具
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()