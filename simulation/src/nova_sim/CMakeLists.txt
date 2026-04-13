cmake_minimum_required(VERSION 3.8)
project(nova_sim)

# 默认使用C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# 安装资源文件
install(DIRECTORY
  urdf
  meshes
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 安装脚本文件
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本
install(PROGRAMS
  scripts/start_gazebo.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
