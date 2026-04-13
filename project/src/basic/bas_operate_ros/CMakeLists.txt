cmake_minimum_required(VERSION 3.8)
project(bas_operate_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
# 由于依赖 data_handler（它依赖 OpenCV），需要显式指定 CUDA 12.4 路径
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

find_package(data_handler REQUIRED)
# 尝试查找log_system包，如果失败则尝试另一种方式
# 由于log_system包在工作区内，使用find_package应该能找到它
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)

# 包含目录
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 源文件
set(SOURCES
    src/param_utils.cpp
    src/param_from_server.cpp
    src/param_to_server.cpp
    src/status_node.cpp
)

# 创建库
add_library(${PROJECT_NAME} ${SOURCES})

# 设置位置无关代码，使库可以被链接到共享库中
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${bas_operate_INCLUDE_DIRS})

# 设置库的依赖关系
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rcl_interfaces
  log_system
  data_handler
  bas_operate
)

# 添加log_system的头文件路径
if(log_system_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${log_system_INCLUDE_DIRS})
endif()

# 链接库和依赖
target_link_libraries(${PROJECT_NAME}
  ${data_handler_LIBRARIES}
)

# 确保bas_operate的头文件路径也被导出
if(data_handler_INCLUDE_DIRS)
  target_include_directories(${PROJECT_NAME} PUBLIC ${data_handler_INCLUDE_DIRS})
endif()

# 导出库
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rcl_interfaces log_system data_handler bas_operate)

# 安装库和头文件
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(FILES config/log_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# 添加示例节点
add_executable(example_node examples/example_node.cpp)
target_link_libraries(example_node ${PROJECT_NAME})
ament_target_dependencies(example_node
  rclcpp
  log_system
  data_handler
)

install(TARGETS example_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # 添加测试
  ament_add_gtest(test_param_utils test/test_param_utils.cpp)
  target_link_libraries(test_param_utils ${PROJECT_NAME})
  ament_target_dependencies(test_param_utils
    rclcpp
    log_system
  )
  target_include_directories(test_param_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_param_utils
    ${PROJECT_NAME}
    ${data_handler_LIBRARIES}
  )
endif()

ament_package()