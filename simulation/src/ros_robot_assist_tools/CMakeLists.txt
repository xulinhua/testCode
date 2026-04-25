cmake_minimum_required(VERSION 3.8)
project(ros_robot_assist_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Keep compatibility with locally compiled OpenCV + CUDA 12.4.
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(ros_robot_assist_tools_core STATIC
  src/ros_robot_assist_tools_ui.cpp
  src/ui/main_window.cpp
  src/ui/shared_refresh_pool.cpp
  src/ui/system_status_widget.cpp
  src/module/system_status_module.cpp
  src/ui/zoomable_image_widget.cpp
  src/module/intrinsic_calibration_module.cpp
  src/module/stereo_calibration_module.cpp
  src/module/multi_sensor_calibration_module.cpp
  src/module/handeye_calibration_module.cpp
  src/ui/intrinsic_calibration_widget.cpp
  src/ui/stereo_calibration_widget.cpp
  src/ui/multi_sensor_calibration_widget.cpp
  src/ui/handeye_calibration_widget.cpp
  src/module/calibration_module.cpp
  src/ui/board_generator_widget.cpp
  src/module/board_generator_module.cpp
  src/ui/pose_transform_widget.cpp
  src/module/pose_transform_module.cpp
  src/ui/kinematics_solver_widget.cpp
  src/module/kinematics_solver_module.cpp
  src/kinematics/arm_kinematics_kdl.cpp
  src/kinematics/mdh_craig1989.cpp
  src/kinematics/arm_kinematics_moveit.cpp
  src/kinematics/mobile_kinematics.cpp
  src/ui/tf_viewer_widget.cpp
  src/module/tf_viewer_module.cpp
  src/manage/feature_data_manager_base.cpp
  src/manage/system_status_data_manager.cpp
  src/manage/board_generator_data_manager.cpp
  src/manage/pose_transform_data_manager.cpp
  src/manage/kinematics_solver_data_manager.cpp
  src/manage/tf_viewer_data_manager.cpp
  src/manage/intrinsic_calibration_data_manager.cpp
  src/manage/stereo_calibration_data_manager.cpp
  src/manage/multi_sensor_calibration_data_manager.cpp
  src/manage/handeye_calibration_data_manager.cpp
  src/manage/feature_data_manager_hub.cpp
)

ament_target_dependencies(ros_robot_assist_tools_core
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  example_interfaces
  tf2
  tf2_ros
  geometry_msgs
  moveit_msgs
  urdf
  kdl_parser
  orocos_kdl
)

target_link_libraries(ros_robot_assist_tools_core
  ${OpenCV_LIBS}
  Qt5::Widgets
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)

# 主 UI 节点
add_executable(ros_robot_assist_tools_ui
  src/ros_robot_assist_tools_ui_node.cpp
)

ament_target_dependencies(ros_robot_assist_tools_ui
  rclcpp
)

target_link_libraries(ros_robot_assist_tools_ui
  ros_robot_assist_tools_core
)

install(TARGETS
  ros_robot_assist_tools_core
  ros_robot_assist_tools_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
