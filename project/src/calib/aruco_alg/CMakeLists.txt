cmake_minimum_required(VERSION 3.8)
project(aruco_alg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(comm_alg REQUIRED)
find_package(log_system REQUIRED)

# Include directories
include_directories(src/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/include)
include_directories(${log_system_INCLUDE_DIRS})

# Add library
add_library(${PROJECT_NAME} SHARED
  src/src/aruco_detector.cpp
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME} 
  "OpenCV"
  "Eigen3"
  "log_system"
  "comm_alg"
)

# if(realsense2_FOUND)
#   target_link_libraries(${PROJECT_NAME} realsense2::realsense2)
# endif()

# 安装配置文件
install(DIRECTORY src/config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装头文件
install(DIRECTORY src/include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Chessboard pose example moved to separate chessboard_alg module

# Install export targets
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV Eigen3 log_system)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 避免使用需要catkin_pkg的lint工具
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()