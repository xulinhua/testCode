cmake_minimum_required(VERSION 3.8)
project(udp_comm_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(custom_msgs_comm REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(log_system REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
)

# Add C++ executables
add_executable(udp_client_node 
  src/main.cpp
  src/udp_client_node.cpp
  src/project_manager.cpp
)

ament_target_dependencies(udp_client_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
  custom_msgs_comm
  sensor_msgs
  log_system
)

target_link_libraries(udp_client_node
  ${JSONCPP_LIBRARIES}
)

# Install C++ executables
install(TARGETS
  udp_client_node
  DESTINATION lib/${PROJECT_NAME}
)



# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()