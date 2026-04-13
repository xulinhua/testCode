cmake_minimum_required(VERSION 3.8)
project(ocr)

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
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(bas_sys_config_ros REQUIRED)
find_package(hand_eye_calib REQUIRED)
find_package(log_system REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 查找OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")

# 查找FreeType（使用pkg-config）
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREETYPE REQUIRED freetype2)
message(STATUS "Found FreeType include: ${FREETYPE_INCLUDE_DIRS}")
message(STATUS "Found FreeType library: ${FREETYPE_LIBRARIES}")

# 查找ppocr_alg包
find_package(ppocr_alg REQUIRED)

# 查找CUDA和TensorRT（ppocr_alg头文件依赖）
find_package(CUDA REQUIRED)

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
  ${FREETYPE_INCLUDE_DIRS}
)

# 创建ROS节点可执行文件
add_executable(ocr_node
  src/ocr_node.cpp
)

# 链接ROS2依赖
ament_target_dependencies(ocr_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  vision_msgs
  geometry_msgs
  cv_bridge
  bas_operate_ros
  bas_sys_config_ros
  hand_eye_calib
  log_system
  ppocr_alg
  ament_index_cpp
  yaml-cpp
)

# 链接系统库
target_link_libraries(ocr_node
  ${OpenCV_LIBS}
  ${TENSORRT_LIBRARY}
  ${CUDA_LIBRARIES}
  ${CUDA_cudart_LIBRARY}
  ${CUDA_cudadevrt_LIBRARY}
  ${FREETYPE_LIBRARIES}
  yaml-cpp
)

# 安装目标
install(TARGETS ocr_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# 安装配置文件
install(DIRECTORY models/ DESTINATION models)
install(DIRECTORY dict/ DESTINATION dict)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 完成包配置
ament_package()
