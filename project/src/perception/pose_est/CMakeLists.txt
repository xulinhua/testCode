cmake_minimum_required(VERSION 3.8)
project(pose_est)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Find algorithm and infrastructure packages
find_package(yolo_seg_alg REQUIRED)
find_package(foundationpose_alg REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(hand_eye_calib REQUIRED)
find_package(bas_sys_config_ros REQUIRED)
find_package(log_system REQUIRED)

# Find CUDA and TensorRT
find_package(CUDA REQUIRED)

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

# Find Eigen3 and glog
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

# Create executable
add_executable(${PROJECT_NAME}
  src/pose_est_node.cpp
)

# Link ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  ament_index_cpp
  yaml-cpp
  tf2
  tf2_ros
  vision_msgs
  yolo_seg_alg
  bas_operate_ros
  hand_eye_calib
  bas_sys_config_ros
  log_system
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${CUDA_cudart_LIBRARY}
  ${TENSORRT_LIBRARY}
  foundationpose_alg::foundationpose_alg
  Eigen3::Eigen
  glog::glog
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install models
install(DIRECTORY models/
  DESTINATION models
)

# Install test data
install(DIRECTORY test_data/
  DESTINATION test_data
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
