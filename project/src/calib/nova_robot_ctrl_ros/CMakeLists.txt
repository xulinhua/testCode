cmake_minimum_required(VERSION 3.8)
project(nova_robot_ctrl_ros)

# 设置默认构建类型为 Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_msgs_comm REQUIRED)
find_package(nova_robot_ctrl REQUIRED)  # 添加nova_robot_ctrl库依赖
find_package(Boost REQUIRED COMPONENTS system)  # 添加Boost依赖
find_package(nlohmann_json REQUIRED)    # 添加nlohmann_json依赖
find_package(tf2_geometry_msgs REQUIRED)  # 添加tf2_geometry_msgs依赖
find_package(bas_sys_config REQUIRED)  # 添加bas_sys_config依赖
find_package(bas_sys_config_ros REQUIRED)  # 添加bas_sys_config_ros依赖

# 包含目录
include_directories(
  include
  ${Boost_INCLUDE_DIRS}  # 添加Boost包含目录
  ../nova_robot_ctrl/FTServo/include  # 添加SCSCL.h所在的目录
)

# 添加库
add_library(${PROJECT_NAME} SHARED
  src/nova_robot_ctrl_node.cpp
)

# 链接库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  custom_msgs_comm
  nova_robot_ctrl  # 添加nova_robot_ctrl依赖
  tf2_geometry_msgs  # 添加tf2_geometry_msgs依赖
  bas_sys_config  # 添加bas_sys_config依赖
  bas_sys_config_ros  # 添加bas_sys_config_ros依赖
)

# 链接额外的库
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}  # 链接Boost库
  nlohmann_json::nlohmann_json  # 链接nlohmann_json库
)

# 添加可执行文件
add_executable(nova_robot_ctrl_node
  src/main.cpp
)

# 链接可执行文件
target_link_libraries(nova_robot_ctrl_node
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}  # 链接Boost库
  nlohmann_json::nlohmann_json  # 链接nlohmann_json库
)

# 链接可执行文件的依赖
ament_target_dependencies(nova_robot_ctrl_node
  rclcpp
  nova_robot_ctrl  # 添加nova_robot_ctrl依赖
  tf2_geometry_msgs  # 添加tf2_geometry_msgs依赖
  bas_sys_config  # 添加bas_sys_config依赖
  bas_sys_config_ros  # 添加bas_sys_config_ros依赖
)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装可执行文件
install(
  TARGETS nova_robot_ctrl_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  custom_msgs_comm
  nova_robot_ctrl  # 添加nova_robot_ctrl导出依赖
  Boost  # 导出Boost依赖
  nlohmann_json  # 导出nlohmann_json依赖
  tf2_geometry_msgs  # 导出tf2_geometry_msgs依赖
  bas_sys_config  # 导出bas_sys_config依赖
  bas_sys_config_ros  # 导出bas_sys_config_ros依赖
)

# 导出包含目录
ament_export_include_directories(include)

# 导出库
ament_export_libraries(${PROJECT_NAME})

# 导出目标
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 包结束
ament_package()