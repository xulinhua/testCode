cmake_minimum_required(VERSION 3.8)
project(elevator_rec_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(vision_msgs REQUIRED)

set(msg_files
  "msg/ElevatorRecognition.msg"
  "msg/ElevatorRecognitionArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs vision_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# 安装消息文件
install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

ament_package()