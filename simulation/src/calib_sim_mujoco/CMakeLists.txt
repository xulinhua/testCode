cmake_minimum_required(VERSION 3.8)
project(calib_sim_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Match local OpenCV build environment.
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmPose.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(calib_sim_core STATIC
  src/config_data_manager.cpp
  src/calib_node.cpp
  src/calib_ui_node.cpp
  src/calib_qt_ui.cpp
)

ament_target_dependencies(calib_sim_core
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(calib_sim_core
  ${OpenCV_LIBS}
  Qt5::Widgets
)

rosidl_target_interfaces(calib_sim_core
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(calib_sim_node
  src/calib_sim_node.cpp
)
target_link_libraries(calib_sim_node calib_sim_core)
ament_target_dependencies(calib_sim_node rclcpp ament_index_cpp)

add_executable(calib_sim_ui_node
  src/calib_sim_ui_node.cpp
)
ament_target_dependencies(calib_sim_ui_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
)
target_link_libraries(calib_sim_ui_node
  calib_sim_core
  ${OpenCV_LIBS}
)

add_executable(calib_qt_ui_node
  src/calib_qt_ui_node.cpp
)
ament_target_dependencies(calib_qt_ui_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(calib_qt_ui_node
  calib_sim_core
)

install(TARGETS
  calib_sim_node
  calib_sim_ui_node
  calib_qt_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
ament_export_include_directories(include)

ament_package()
