cmake_minimum_required(VERSION 3.8)
project(hand_eye_calib_ros)

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
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_msgs_comm REQUIRED)  # 添加自定义消息包依赖
find_package(hand_eye_calib REQUIRED)    # 添加hand_eye_calib库依赖
find_package(log_system REQUIRED)        # 添加日志系统依赖
find_package(bas_operate_ros REQUIRED)    # 添加bas_operate_ros依赖
find_package(OpenCV REQUIRED)            # 添加OpenCV依赖
find_package(cv_bridge REQUIRED)         # 添加cv_bridge依赖
find_package(yaml-cpp REQUIRED)          # 添加 yaml-cpp 依赖
find_package(Eigen3 REQUIRED)            # 添加 Eigen3 依赖
find_package(bas_sys_config_ros REQUIRED) # 添加 bas_sys_config_ros 依赖

# 包含目录
include_directories(
  include
  ${log_system_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${bas_sys_config_ros_INCLUDE_DIRS}
)

# 添加库
add_library(${PROJECT_NAME} SHARED
  src/hand_eye_calib_node.cpp
  src/calib_test_node.cpp
  src/keyboard_handler.cpp
)
# 链接库
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  std_srvs
  custom_msgs_comm  # 添加自定义消息包依赖
  hand_eye_calib    # 添加hand_eye_calib库依赖
  log_system        # 添加日志系统依赖
  bas_operate_ros   # 添加bas_operate_ros依赖
  bas_sys_config_ros # 添加bas_sys_config_ros依赖
  OpenCV            # 添加OpenCV依赖
  yaml-cpp          # 添加yaml-cpp依赖
  cv_bridge         # 添加cv_bridge依赖
)

# 链接 yaml-cpp 库和 Eigen3
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)

# 添加可执行文件
add_executable(hand_eye_calib_node src/main.cpp)
target_link_libraries(hand_eye_calib_node ${PROJECT_NAME})

# 添加测试节点可执行文件
add_executable(calib_test_node src/calib_test_main.cpp)
target_link_libraries(calib_test_node ${PROJECT_NAME})

# 添加简单的键盘测试程序
add_executable(simple_keyboard_test src/simple_keyboard_test.cpp)
target_link_libraries(simple_keyboard_test ${PROJECT_NAME})

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
install(TARGETS
  hand_eye_calib_node
  calib_test_node
  simple_keyboard_test
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
  std_srvs
  custom_msgs_comm  # 添加自定义消息包依赖
  hand_eye_calib    # 添加hand_eye_calib库依赖
  log_system        # 添加日志系统依赖
  bas_operate_ros   # 添加bas_operate_ros依赖
  bas_sys_config_ros # 添加bas_sys_config_ros依赖
  OpenCV            # 添加OpenCV依赖
  yaml-cpp          # 添加yaml-cpp依赖
  cv_bridge         # 添加cv_bridge依赖
)

# 导出包含目录
ament_export_include_directories(include)

# 导出库
ament_export_libraries(${PROJECT_NAME})

# 注册包
ament_package()