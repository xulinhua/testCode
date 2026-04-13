cmake_minimum_required(VERSION 3.22)
project(xtrainer_moveit_config)

find_package(ament_cmake REQUIRED)

# 添加硬件接口相关依赖
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fmt REQUIRED)

# 编译硬件接口
add_library(xtrainer_hardware_interface SHARED
  xtrainer_hardware_interface/xtrainer_hardware_interface.cpp
)

target_link_libraries(xtrainer_hardware_interface
  /home/hoson/dobot_xtrainer/xtrainer_driver/build/libxTrainer_driver.a
  /home/hoson/third_party/FTServo_Linux/build/libSCServo.a
  fmt::fmt
  pthread
)

target_include_directories(xtrainer_hardware_interface PRIVATE
  xtrainer_hardware_interface
  /home/hoson/dobot_xtrainer/xtrainer_driver
  /home/hoson//third_party/asio/asio/include
)

ament_target_dependencies(xtrainer_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 导出插件
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# 安装硬件接口
install(
  TARGETS xtrainer_hardware_interface
  DESTINATION lib
)

# 安装插件描述文件
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# 编译C++抓取节点
add_executable(fixed_position_grasper_cpp fixed_position_grasper.cpp)
add_executable(working_grasper working_grasper.cpp)

find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

ament_target_dependencies(fixed_position_grasper_cpp
  "rclcpp"
  "geometry_msgs"
  "trajectory_msgs"
  "moveit_ros_planning_interface"
)

ament_target_dependencies(working_grasper
  "rclcpp"
  "trajectory_msgs"
  "std_msgs"
)

# 安装Python脚本作为可执行文件
install(PROGRAMS
  yolo_object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装C++可执行文件
install(
  TARGETS fixed_position_grasper_cpp working_grasper
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()