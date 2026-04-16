cmake_minimum_required(VERSION 3.8)
project(nova_sim)

# 默认使用C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Concurrent)

add_executable(moveit2_arm_executor_cpp
  src/moveit2_arm_executor.cpp
  src/moveit2_arm_executor_main.cpp
)
ament_target_dependencies(moveit2_arm_executor_cpp
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
)

add_executable(nova_control_ui_cpp
  src/nova_control_ui.cpp
)
ament_target_dependencies(nova_control_ui_cpp
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(nova_control_ui_qt
  src/nova_control_ui_qt.cpp
)
ament_target_dependencies(nova_control_ui_qt
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
  tf2_ros
  tf2
)
target_link_libraries(nova_control_ui_qt Qt5::Widgets Qt5::Concurrent)

# 安装资源文件
install(DIRECTORY
  include
  urdf
  meshes
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装脚本文件
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/start_gazebo.sh
  scripts/moveit2_arm_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  moveit2_arm_executor_cpp
  nova_control_ui_cpp
  nova_control_ui_qt
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
