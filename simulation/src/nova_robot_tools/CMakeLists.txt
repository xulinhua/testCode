cmake_minimum_required(VERSION 3.8)
project(nova_robot_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Match local OpenCV build environment (CUDA 12.4)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 核心库 - 所有 Qt 代码在 .cpp 中实现，不使用 AUTOMOC
add_library(nova_robot_tools_core STATIC
  src/nova_robot_tools_ui.cpp
)

ament_target_dependencies(nova_robot_tools_core
  rclcpp
  kdl_parser
  tf2
  tf2_geometry_msgs
  urdf
)

target_link_libraries(nova_robot_tools_core
  ${OpenCV_LIBS}
  Qt5::Widgets
  ${YAML_CPP_LIBRARIES}
  orocos-kdl
)

# 主 UI 节点
add_executable(nova_robot_tools_ui
  src/nova_robot_tools_ui_node.cpp
)

ament_target_dependencies(nova_robot_tools_ui
  rclcpp
)

target_link_libraries(nova_robot_tools_ui
  nova_robot_tools_core
)

install(TARGETS
  nova_robot_tools_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
