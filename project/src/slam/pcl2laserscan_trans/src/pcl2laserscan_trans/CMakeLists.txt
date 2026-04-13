cmake_minimum_required(VERSION 3.5)
project(pcl2laserscan_trans)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
# 系统中有 CUDA 12.4，需要显式指定路径
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -O3")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-std=c++14 -fexceptions -pthread)

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# 自动检测架构并设置PCL路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("Detected ARM64 architecture")
    set(PCL_DIR "/usr/lib/aarch64-linux-gnu/cmake/pcl")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message("Detected x86_64 architecture")
    set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
else()
    message(WARNING "Unknown architecture, using default PCL search paths")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")  # Jetson ARM64
    add_compile_options(-O3 -mcpu=native -mtune=native -ffast-math)
    message("Added ARM64-specific optimization flags")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")  # x86
    add_compile_options(-O3 -march=native -mtune=native -ffast-math)
    message("Added x86_64-specific optimization flags")
endif()

# 添加架构特定的编译选项
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

# CUDA 版本兼容性处理：解决 OpenCV 要求的 CUDA 版本与系统版本不匹配问题
# 当 find_package(OpenCV) 调用 find_host_package(CUDA) 时，如果版本不匹配会导致失败
# 通过设置 CUDA_VERSION_OK 变量，允许使用更高版本的 CUDA
set(CUDA_VERSION_OK TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)

# 添加新依赖项
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)
find_package(bas_operate_ros REQUIRED)
find_package(pcl_proc REQUIRED)

# 添加架构特定的PCL路径
if(DEFINED PCL_DIR)
    list(APPEND CMAKE_PREFIX_PATH ${PCL_DIR})
    message("Added PCL_DIR to CMAKE_PREFIX_PATH: ${PCL_DIR}")
endif()

find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
  ${bas_operate_INCLUDE_DIRS}
  ${bas_operate_ros_INCLUDE_DIRS}
  ${pcl_proc_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# 添加日志模块

#相机点云转激光scan节点（独立可执行文件）
add_executable(camera_pcl2laserscan_node
  src/camera_pcl2laserscan.cpp
)

target_include_directories(camera_pcl2laserscan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(camera_pcl2laserscan_node
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}

)
ament_target_dependencies(camera_pcl2laserscan_node
  "rclcpp"
  "rclcpp_components"
  "nav_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "std_msgs"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_sensor_msgs"
  "PCL"
  "pcl_conversions"
  "log_system"
  "bas_operate"
  "pcl_proc"
)

add_library(lidar_pcl2laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp
)
target_include_directories(lidar_pcl2laserscan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_pcl2laserscan
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(lidar_pcl2laserscan
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "message_filters"
  "tf2_ros"
  "tf2_sensor_msgs"
  "PCL"
  "pcl_conversions"
  "laser_geometry"
  "log_system"
  "bas_operate"
  "bas_operate_ros"
  "pcl_proc"
)
rclcpp_components_register_node(lidar_pcl2laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE lidar_pcl2laserscan_node
)

# 配置管理库
add_library(pcl_config_mgr SHARED
  src/pcl_config_manager.cpp
)
target_include_directories(pcl_config_mgr PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pcl_config_mgr
  ${YAML_CPP_LIBRARIES}
)

# 激光扫描转点云库
# 已移除Virtual模式，注释掉相关编译配置
# add_library(laserscan_to_pointcloud SHARED
#   src/laserscan_to_pointcloud_node.cpp
# )
# target_include_directories(laserscan_to_pointcloud PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(laserscan_to_pointcloud
#   ${PCL_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
# )
# ament_target_dependencies(laserscan_to_pointcloud
#   "rclcpp"
#   "rclcpp_components"
#   "sensor_msgs"
#   "message_filters"
#   "tf2_ros"
#   "tf2_sensor_msgs"
#   "PCL"
#   "pcl_conversions"
#   "laser_geometry"
#   "log_system"
#   "bas_operate"
#   "pcl_proc"
# )
# rclcpp_components_register_node(laserscan_to_pointcloud
#   PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
#   EXECUTABLE laserscan_to_pointcloud_node
# )

# 添加 C++17 支持
target_compile_features(pcl_config_mgr PUBLIC cxx_std_17)
target_compile_features(camera_pcl2laserscan_node PUBLIC cxx_std_17)
target_compile_features(lidar_pcl2laserscan PUBLIC cxx_std_17)
# target_compile_features(laserscan_to_pointcloud PUBLIC cxx_std_17)

# 安装规则
install(TARGETS
  camera_pcl2laserscan_node  
  lidar_pcl2laserscan       
  # laserscan_to_pointcloud   # 已移除Virtual模式
  pcl_config_mgr
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY include/
  DESTINATION include
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
