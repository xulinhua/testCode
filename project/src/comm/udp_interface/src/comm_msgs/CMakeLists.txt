cmake_minimum_required(VERSION 3.8)
project(comm_msgs)

# 忽略未使用的CATKIN_INSTALL_INTO_PREFIX_ROOT变量，消除警告
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(CATKIN_INSTALL_INTO_PREFIX_ROOT "${CATKIN_INSTALL_INTO_PREFIX_ROOT}" CACHE INTERNAL "")
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 定义消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointCloudData.msg"
  "msg/AICoordinateData.msg"
  "msg/LaserScanData.msg"
  "msg/CalibrationCommand.msg" 
  "msg/CalibrationResponse.msg" 
  "msg/ProjectCommand.msg"
  "msg/ProjectStatus.msg"
  "srv/ProjectControl.srv"
  "srv/ProjectStatus.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# 安装消息文件
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()
