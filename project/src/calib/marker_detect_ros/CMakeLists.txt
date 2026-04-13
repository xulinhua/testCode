cmake_minimum_required(VERSION 3.8)
project(marker_detect_ros)

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

# 查找OpenCV 4.10
find_package(OpenCV 4.10 REQUIRED)
message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")

# 查找aruco_alg包
find_package(aruco_alg REQUIRED)
find_package(chessboard_alg REQUIRED)
find_package(comm_alg REQUIRED)

# 查找log_system包
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)

# 查找visualization包
find_package(visualization REQUIRED)

# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)

# 查找custom_msgs_comm包
find_package(custom_msgs_comm REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 查找hand_eye_calib包
find_package(hand_eye_calib REQUIRED)    # 添加hand_eye_calib库依赖

# 查找bas_sys_config_ros包
find_package(bas_sys_config_ros REQUIRED)

# 查找bas_operate_ros包
find_package(bas_operate_ros REQUIRED)

# 包含目录
include_directories(
  src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${log_system_INCLUDE_DIRS}
  ${bas_operate_INCLUDE_DIRS}
  ${hand_eye_calib_INCLUDE_DIRS}
  ${bas_sys_config_ros_INCLUDE_DIRS}
  ${bas_operate_ros_INCLUDE_DIRS}
)

# 创建ROS节点可执行文件
add_executable(marker_detect_node
  src/src/marker_detect_ros/marker_detect_node.cpp
  src/src/marker_detect_ros/keyboard_handler.cpp
  src/src/marker_detect_main.cpp
)
# 链接依赖
ament_target_dependencies(marker_detect_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  cv_bridge
  image_transport
  ament_index_cpp
  aruco_alg
  chessboard_alg
  comm_alg
  visualization
  log_system
  bas_operate
  custom_msgs_comm
  hand_eye_calib
  bas_sys_config_ros
  bas_operate_ros
  std_srvs
)

# 链接OpenCV库和yaml-cpp
target_link_libraries(marker_detect_node
  ${OpenCV_LIBS}
  yaml-cpp
)# 安装目标
install(TARGETS marker_detect_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY src/launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# 安装配置文件到share目录
install(DIRECTORY src/config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 完成包配置
ament_package()