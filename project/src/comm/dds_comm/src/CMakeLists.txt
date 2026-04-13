cmake_minimum_required(VERSION 3.8)
project(dds_comm)

# ROS ament_cmake配置
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 查找消息依赖
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs_comm REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找第三方库
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# 查找log_system包
find_package(log_system REQUIRED)

# 包含目录
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${log_system_INCLUDE_DIRS})

# 创建核心库
add_library(${PROJECT_NAME}_core SHARED
  src/dds_service.cpp
  src/command_manager.cpp
  src/project_manager.cpp
  src/data_publisher.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${log_system_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  custom_msgs_comm
  log_system
)

target_link_libraries(${PROJECT_NAME}_core
  yaml-cpp
  nlohmann_json::nlohmann_json
  stdc++fs
)

# 创建独立可执行文件
add_executable(dds_service src/main.cpp)
target_link_libraries(dds_service ${PROJECT_NAME}_core)

# 创建项目控制节点可执行文件
add_executable(project_control_node src/project_control_node.cpp)
target_link_libraries(project_control_node ${PROJECT_NAME}_core)

# 安装目标
install(TARGETS
  ${PROJECT_NAME}_core
  dds_service
  project_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装测试脚本
install(DIRECTORY test/
  DESTINATION share/${PROJECT_NAME}/test
)

# 导出包
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(custom_msgs_comm)

# 生成环境脚本
ament_package()