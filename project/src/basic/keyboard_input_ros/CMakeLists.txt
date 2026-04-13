cmake_minimum_required(VERSION 3.8)
project(keyboard_input_node)

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
find_package(custom_msgs_comm REQUIRED)

# 包含目录
include_directories(include)

# 添加可执行文件
add_executable(keyboard_input_node
  src/main.cpp
  src/keyboard_input_node.cpp
)

# 设置依赖
ament_target_dependencies(keyboard_input_node
  rclcpp
  std_msgs
  custom_msgs_comm
)

# 安装可执行文件
install(TARGETS keyboard_input_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装launch文件
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# 包结束
ament_package()
