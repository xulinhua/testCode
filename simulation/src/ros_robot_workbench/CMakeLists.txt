cmake_minimum_required(VERSION 3.8)
project(ros_robot_workbench)

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
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

include(cmake/WorkbenchOptions.cmake)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_robot_workbench/workbench_build_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/ros_robot_workbench/workbench_build_config.hpp
  @ONLY)

include(cmake/WorkbenchSources.cmake)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

if(WORKBENCH_WITH_OPENCV)
  find_package(OpenCV REQUIRED)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

if(WORKBENCH_WITH_KDL)
  find_package(urdf REQUIRED)
  find_package(kdl_parser REQUIRED)
  find_package(orocos_kdl REQUIRED)
endif()

if(WORKBENCH_WITH_MOVEIT)
  find_package(moveit_msgs REQUIRED)
endif()

add_library(ros_robot_workbench_core STATIC ${WORKBENCH_SOURCES})

ament_target_dependencies(ros_robot_workbench_core
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  example_interfaces
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  rosgraph_msgs
)

if(WORKBENCH_WITH_KDL)
  ament_target_dependencies(ros_robot_workbench_core urdf kdl_parser orocos_kdl)
endif()

if(WORKBENCH_WITH_MOVEIT)
  ament_target_dependencies(ros_robot_workbench_core moveit_msgs)
endif()

target_link_libraries(ros_robot_workbench_core
  Qt5::Widgets
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)

if(WORKBENCH_WITH_OPENCV)
  target_link_libraries(ros_robot_workbench_core ${OpenCV_LIBS})
endif()

add_executable(ros_robot_workbench_ui src/ros_robot_workbench_ui_node.cpp)

ament_target_dependencies(ros_robot_workbench_ui rclcpp)
target_link_libraries(ros_robot_workbench_ui ros_robot_workbench_core)

install(TARGETS ros_robot_workbench_core ros_robot_workbench_ui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

ament_package()
