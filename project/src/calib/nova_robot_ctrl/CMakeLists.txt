cmake_minimum_required(VERSION 3.8)
project(nova_robot_ctrl)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)


find_package(Boost REQUIRED COMPONENTS system)

# Include nlohmann_json
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
  include
  FTServo/include
  ${Boost_INCLUDE_DIRS}

)

# NovaRobotCtrl library
add_library(nova_robot_ctrl
  src/nova_robot_ctrl.cpp
  src/nova_gripper_ctrl.cpp
  src/calib_robot_pos_mgr.cpp
  src/robot_mgr.cpp
  FTServo/src/SCSCL.cpp
  FTServo/src/SCSerial.cpp
  FTServo/src/SCS.cpp
  FTServo/src/SMSBL.cpp
  FTServo/src/SMSCL.cpp
  FTServo/src/SMS_STS.cpp
)

# Set PIC flag for the library
set_property(TARGET nova_robot_ctrl PROPERTY POSITION_INDEPENDENT_CODE ON)

# Link libraries
ament_target_dependencies(nova_robot_ctrl
  rclcpp
  std_msgs
  geometry_msgs
  yaml_cpp_vendor
)

target_link_libraries(nova_robot_ctrl nlohmann_json::nlohmann_json)

# Link to standard libraries
target_link_libraries(nova_robot_ctrl
  ${Boost_LIBRARIES}
  fmt
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install FTServo include directory
install(
  DIRECTORY FTServo/include/
  DESTINATION include/FTServo
)

# Install library
install(
  TARGETS nova_robot_ctrl
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Robot move test executable
add_executable(robot_move_test test/robot_move_test.cpp)
ament_target_dependencies(robot_move_test
  rclcpp
  std_msgs
  geometry_msgs
  yaml_cpp_vendor
)

target_link_libraries(robot_move_test
  nova_robot_ctrl
  ${Boost_LIBRARIES}
  fmt

  nlohmann_json::nlohmann_json
)

# Install executable
install(TARGETS
  robot_move_test
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  yaml_cpp_vendor

  Boost
  nlohmann_json
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(nova_robot_ctrl)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()