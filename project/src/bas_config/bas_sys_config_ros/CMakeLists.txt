cmake_minimum_required(VERSION 3.8)
project(bas_sys_config_ros)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
# 必须在所有 find_package 之前设置
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs_comm REQUIRED)
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(bas_sys_config REQUIRED)
find_package(hand_eye_calib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# include directories
include_directories(
  include
  src
  ${EIGEN3_INCLUDE_DIRS}
)
# Create library for header-only utilities
add_library(${PROJECT_NAME}_utils INTERFACE)
target_include_directories(${PROJECT_NAME}_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)

# Create a library for our implementation
add_library(${PROJECT_NAME}_core
  calib_info_src/calib_info_server.cpp
  sys_info_src/sys_info_server.cpp)  
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBS})
target_compile_features(${PROJECT_NAME}_core PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  custom_msgs_comm
  log_system
  bas_operate
  bas_operate_ros
  bas_sys_config
  hand_eye_calib
  OpenCV)

# Create executable
add_executable(sys_config_ros_node src/sys_config_ros_node.cpp)
target_include_directories(sys_config_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sys_config_ros_node ${PROJECT_NAME}_core stdc++fs ${OpenCV_LIBS})
target_compile_features(sys_config_ros_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(sys_config_ros_node
  rclcpp
  std_msgs
  custom_msgs_comm
  log_system
  bas_operate
  bas_operate_ros
  bas_sys_config
  hand_eye_calib
  OpenCV)


# Install header-only utilities
install(TARGETS ${PROJECT_NAME}_utils  EXPORT ${PROJECT_NAME}_utils
  DESTINATION lib/${PROJECT_NAME})

# Install exported targets for utils
install(EXPORT ${PROJECT_NAME}_utils
  FILE ${PROJECT_NAME}_utils.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

# Install core library with export
install(TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Install exported targets for core
install(EXPORT ${PROJECT_NAME}_core
  FILE ${PROJECT_NAME}_core.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
  PATTERN "*.tpp" EXCLUDE)

# Install headers from subdirectories to the project include directory
install(DIRECTORY include/calib_info_src/
  DESTINATION include/bas_sys_config_ros
  FILES_MATCHING PATTERN "*.h" PATTERN "*.tpp")

install(DIRECTORY include/sys_info_src/
  DESTINATION include/bas_sys_config_ros
  FILES_MATCHING PATTERN "*.h" PATTERN "*.tpp")

# Install template implementation files
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.tpp")

# Install executables
install(TARGETS sys_config_ros_node
  DESTINATION lib/${PROJECT_NAME})

# Install config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Export library
ament_export_include_directories(include)
# Also export the subdirectories that contain headers
ament_export_include_directories(include/calib_info_src include/sys_info_src)
ament_export_dependencies(
  rclcpp
  std_msgs
  custom_msgs_comm
  log_system
  bas_operate
  bas_operate_ros
  bas_sys_config
  hand_eye_calib
  OpenCV)

# Export include directories for other packages
ament_export_targets(${PROJECT_NAME}_utils HAS_LIBRARY_TARGET)
ament_export_targets(${PROJECT_NAME}_core HAS_LIBRARY_TARGET)

# Register package
ament_package()