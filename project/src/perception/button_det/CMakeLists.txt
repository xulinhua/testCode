cmake_minimum_required(VERSION 3.8)
project(button_det)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 查找算法依赖
find_package(yolo_alg REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(hand_eye_calib REQUIRED)
find_package(log_system REQUIRED)
find_package(bas_sys_config_ros REQUIRED)

# 查找并包含TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS /usr/include/aarch64-linux-gnu
    /usr/include
    /usr/local/TensorRT-10.8.0.43/include
    DOC "TensorRT include path")

find_library(TENSORRT_LIBRARY nvinfer
    PATHS /usr/lib/aarch64-linux-gnu
    /usr/lib
    /usr/local/TensorRT-10.8.0.43/lib
    DOC "TensorRT library")

if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY)
    set(TENSORRT_FOUND TRUE)
    message(STATUS "Found TensorRT includes: ${TENSORRT_INCLUDE_DIR}")
    message(STATUS "Found TensorRT library: ${TENSORRT_LIBRARY}")
else()
    message(FATAL_ERROR "TensorRT not found. Please install TensorRT.")
endif()

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
)

# 可执行文件
add_executable(button_det
    src/button_det_node.cpp
)

# 链接ROS2依赖
ament_target_dependencies(button_det
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
  visualization_msgs
  message_filters
  vision_msgs
  tf2_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
  yolo_alg
  bas_operate_ros
  hand_eye_calib
  log_system
  bas_sys_config_ros
)

# 链接系统库
target_link_libraries(button_det
  ${OpenCV_LIBS}
  ${TENSORRT_LIBRARY}
  ${CUDA_LIBRARIES}
)
# 安装目标
install(TARGETS button_det
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件和配置
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# 安装模型文件
install(DIRECTORY models/ DESTINATION models)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 完成包配置
ament_package()