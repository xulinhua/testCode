cmake_minimum_required(VERSION 3.8)
project(cmd_dispatcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(custom_msgs_comm REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  vision_msgs
  custom_msgs_comm
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# 可执行文件
add_executable(cmd_dispatcher
  src/cmd_dispatcher_node.cpp
  src/speech_commands.cpp
)
ament_target_dependencies(
  cmd_dispatcher
  ${dependencies}
)

install(TARGETS cmd_dispatcher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()