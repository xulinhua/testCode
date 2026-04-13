cmake_minimum_required(VERSION 3.8)
project(bas_control)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bas_operate REQUIRED)
find_package(data_handler REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(log_system REQUIRED)
find_package(custom_msgs_comm REQUIRED)

#包含目录
include_directories(
  include
  ${bas_operate_INCLUDE_DIRS}
  ${data_handler_INCLUDE_DIRS}
  ${bas_operate_ros_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
)

# 创建库
add_library(${PROJECT_NAME}_core
  src/launch_mgr.cpp
  src/status_monitor.cpp
  src/task_scheduler.cpp
  src/config_hot_updater.cpp
  src/system_mgr.cpp
)

# 启用 YAML 支持
target_compile_definitions(${PROJECT_NAME}_core PRIVATE USE_YAML_CPP)

#链接库
target_link_libraries(${PROJECT_NAME}_core
  ${YAML_CPP_LIBRARIES}
  ${bas_operate_LIBRARIES}
  ${data_handler_LIBRARIES}
  ${bas_operate_ros_LIBRARIES}
  ${log_system_LIBRARIES}
)

# 创建可执行文件
add_executable(bas_control_node
  src/bas_control_node.cpp
)

# 启用 YAML 支持 (必须在 add_executable 之后)
target_compile_definitions(bas_control_node PRIVATE USE_YAML_CPP)

#链接可执行文件
target_link_libraries(bas_control_node
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  ${YAML_CPP_LIBRARIES}
  ${log_system_LIBRARIES}
)

# 添加ament依赖
ament_target_dependencies(bas_control_node
  rclcpp
  std_msgs
  std_srvs
  builtin_interfaces
  yaml_cpp_vendor
  ament_index_cpp
  log_system
  custom_msgs_comm
)

#安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

#安装可执行文件
install(TARGETS
  bas_control_node
  DESTINATION lib/${PROJECT_NAME}
)

#安配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

#导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  std_srvs
  builtin_interfaces
  yaml_cpp_vendor
  ament_index_cpp
  bas_operate
  data_handler
  bas_operate_ros
  log_system
  custom_msgs_comm
)

#导出包含目录
ament_export_include_directories(include)

#导出库
ament_export_libraries(${PROJECT_NAME}_core)

#测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # 添加单元测试
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_launch_mgr test/test_launch_mgr.cpp)
  target_link_libraries(test_launch_mgr ${PROJECT_NAME}_core)
  
  ament_add_gtest(test_status_monitor test/test_status_monitor.cpp)
  target_link_libraries(test_status_monitor ${PROJECT_NAME}_core)
  
  ament_add_gtest(test_task_scheduler test/test_task_scheduler.cpp)
  target_link_libraries(test_task_scheduler ${PROJECT_NAME}_core)
endif()

ament_package()