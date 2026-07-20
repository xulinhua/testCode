cmake_minimum_required(VERSION 3.8)
project(nova_grasp_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Concurrent)

set(CMAKE_AUTOMOC ON)
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmPose.msg"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(
  arm_pose_cpp_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(
  arm_pose_fastrtps_typesupport ${PROJECT_NAME} "rosidl_typesupport_fastrtps_cpp")

include_directories(include)

# Qt/ROS 后端共用静态库；grasp_qt_ui_node 只保留 QApplication 与 ROS spin 入口。
add_library(nova_grasp_moveit_core STATIC
  src/grasp_planner.cpp
  src/grasp_executor.cpp
  src/grasp_ros_node.cpp
  src/grasp_qt_ui.cpp
)

ament_target_dependencies(nova_grasp_moveit_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
)

target_link_libraries(nova_grasp_moveit_core
  Qt5::Widgets
  Qt5::Concurrent
  "${arm_pose_cpp_typesupport}"
  "${arm_pose_fastrtps_typesupport}"
)

add_executable(grasp_qt_ui_node src/grasp_qt_ui_node.cpp)
target_link_libraries(grasp_qt_ui_node
  nova_grasp_moveit_core
  "${arm_pose_cpp_typesupport}"
  "${arm_pose_fastrtps_typesupport}"
)
ament_target_dependencies(grasp_qt_ui_node rclcpp)

add_executable(grasp_arm_executor_node
  src/grasp_arm_executor_main.cpp
  src/grasp_arm_executor.cpp
)
ament_target_dependencies(grasp_arm_executor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
)
target_link_libraries(grasp_arm_executor_node
  "${arm_pose_cpp_typesupport}"
  "${arm_pose_fastrtps_typesupport}"
)

install(TARGETS grasp_qt_ui_node grasp_arm_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
