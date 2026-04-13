cmake_minimum_required(VERSION 3.8)
project(object_det)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_msgs_comm REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(realsense2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(bas_sys_config_ros REQUIRED)
find_package(hand_eye_calib REQUIRED)
find_package(log_system REQUIRED)
find_package(yaml-cpp REQUIRED)

# 查找OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")

# 【重构】查找抽象接口包
find_package(vision_base REQUIRED)

# 包含目录
include_directories(
  include
  ${vision_base_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 创建ROS节点可执行文件
add_executable(object_det
  src/object_det_node.cpp
)

# 链接ROS2依赖
ament_target_dependencies(object_det
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  custom_msgs_comm
  sensor_msgs
  geometry_msgs
  nav_msgs
  vision_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
  rosbag2_cpp
  message_filters
  realsense2
  bas_operate_ros
  bas_sys_config_ros
  hand_eye_calib
  log_system
  vision_base # 【重构】链接抽象接口包
  yaml-cpp
)

# 链接系统库
target_link_libraries(object_det
  ${OpenCV_LIBS}
)

# 安装目标
install(TARGETS object_det
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# 安装配置文件
install(DIRECTORY models/ DESTINATION models)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 完成包配置
ament_package()
