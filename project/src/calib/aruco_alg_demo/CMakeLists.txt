cmake_minimum_required(VERSION 3.8)
project(aruco_alg_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(aruco_alg REQUIRED)
find_package(comm_alg REQUIRED)
find_package(OpenCV REQUIRED)


# Add executable
add_executable(${PROJECT_NAME}_node
  src/src/aruco_demo_node.cpp
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  "aruco_alg"
  "comm_alg"
  "OpenCV"
)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY src/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 修复可能导致catkin_pkg问题的测试依赖
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 避免使用需要catkin_pkg的lint工具
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()