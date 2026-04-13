cmake_minimum_required(VERSION 3.8)
project(pcl_proc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(log_system REQUIRED)

message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "PCL include directories: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
message(STATUS "PCL definitions: ${PCL_DEFINITIONS}")

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable C++17 filesystem support
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# Include directories
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${log_system_INCLUDE_DIRS}
)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/src/pcl_proc_filter.cpp
  src/src/pcl_proc_segment.cpp
  src/src/pcl_proc_feature.cpp
  src/src/pcl_proc_io.cpp
  src/src/pcl_proc_sample.cpp
  src/src/pcl_proc_visualization.cpp
  src/src/pcl_proc_surface.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES}
  ${log_system_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  sensor_msgs
  pcl_conversions
  log_system
)

# Example executable - replace with your actual nodes
add_executable(pcl_test src/test/pcl_test.cpp)
target_link_libraries(pcl_test ${PCL_LIBRARIES} ${PROJECT_NAME})
ament_target_dependencies(pcl_test 
  rclcpp
  sensor_msgs
  pcl_conversions
  log_system
)

# Install executables
install(TARGETS
  pcl_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install headers
install(DIRECTORY src/include/
  DESTINATION include/
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export include directories
ament_export_include_directories(include)

# Export libraries
ament_export_libraries(${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp sensor_msgs pcl_conversions PCL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()