cmake_minimum_required(VERSION 3.8)
project(object_seg)

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 查找OpenCV路径
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# 查找 vision_base 和 yolo_seg_tensorrt（新架构）
find_package(vision_base REQUIRED)
find_package(yolo_seg_tensorrt REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(hand_eye_calib REQUIRED)
find_package(bas_sys_config_ros REQUIRED)
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)
find_package(OpenMP REQUIRED)  # OpenMP 支持

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${vision_base_INCLUDE_DIRS}
  ${yolo_seg_tensorrt_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
  ${bas_operate_INCLUDE_DIRS}
)

# 创建ROS节点可执行文件
add_executable(object_seg
  src/object_seg_node.cpp
)

# 链接ROS2依赖
ament_target_dependencies(object_seg
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  cv_bridge
  image_transport
  vision_base
  yolo_seg_tensorrt
  ament_index_cpp
  yaml-cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  bas_operate_ros
  hand_eye_calib
  bas_sys_config_ros
  log_system
  bas_operate
)

# 链接系统库
target_link_libraries(object_seg
  ${OpenCV_LIBS}
  yaml-cpp
  bas_sys_config_ros::bas_sys_config_ros_core
  OpenMP::OpenMP_CXX
)

# 安装目标
install(TARGETS object_seg
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
