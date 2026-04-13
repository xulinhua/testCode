cmake_minimum_required(VERSION 3.8)
project(cam_mgr_ros)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
# 必须在所有 find_package 之前设置
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
# 添加对服务消息的支持
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)  # 添加json库依赖
# 查找RealSense SDK
find_package(realsense2 REQUIRED)

# 查找cam_config_mgr包依赖
find_package(cam_config_mgr REQUIRED)

# 查找bas_operate包依赖
find_package(bas_operate REQUIRED)
# 查找log_system包依赖
find_package(log_system REQUIRED)
# 查找visualization包依赖
find_package(visualization REQUIRED)
# 查找 cv_bridge 包依赖
find_package(cv_bridge REQUIRED)
# 查找 bas_sys_config_ros 包依赖
find_package(bas_sys_config_ros REQUIRED)
# 查找 custom_msgs_comm 包依赖
find_package(custom_msgs_comm REQUIRED)
# 查找 Eigen3 包依赖
find_package(Eigen3 REQUIRED)
# 查找OpenCV依赖
find_package(OpenCV REQUIRED)
# 查找hand_eye_calib包依赖
find_package(hand_eye_calib REQUIRED)
# 查找rosbag2包依赖
find_package(rosbag2_cpp REQUIRED)

# 定义消息文件


# 包含目录
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${cam_config_mgr_INCLUDE_DIRS}
  ${bas_operate_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
  ${log_system_INCLUDE_DIRS}
  ${visualization_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${bas_sys_config_ros_INCLUDE_DIRS}
  ${custom_msgs_comm_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${hand_eye_calib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /opt/OrbbecSDK/include
)

# 链接目录
link_directories(
  ${bas_operate_LIBRARY_DIRS}
  ${visualization_LIBRARY_DIRS}
  /opt/OrbbecSDK/lib
)

# 添加可执行文件
add_executable(cam_mgr_node
  src/cam_mgr.cpp
  src/cam_base.cpp
  src/cam_rs.cpp
  src/cam_ob.cpp
  src/cam_csi.cpp
  src/cam_mgr_node.cpp
  src/utils.cpp
)

# 链接依赖
ament_target_dependencies(cam_mgr_node
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
  cam_config_mgr
  bas_operate
  log_system
  visualization
  cv_bridge
  bas_sys_config_ros
  custom_msgs_comm
  OpenCV
  hand_eye_calib
  rosbag2_cpp
)

# 查找并链接Orbbec库
find_library(ORBBEC_LIB NAMES orbbec libOrbbecSDK.so libOrbbecSDK.so.2 libOrbbecSDK.so.2.4.11 libOrbbecSDK)
if(ORBBEC_LIB)
    target_link_libraries(cam_mgr_node ${ORBBEC_LIB})
    target_include_directories(cam_mgr_node PRIVATE /opt/OrbbecSDK/include)
    message(STATUS "Found Orbbec library at: ${ORBBEC_LIB}")
else()
    # 如果没找到，尝试使用pkg-config查找
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(ORBBEC QUIET orbbec)
        if(ORBBEC_FOUND)
            target_link_libraries(cam_mgr_node ${ORBBEC_LIBRARIES})
            target_include_directories(cam_mgr_node PRIVATE ${ORBBEC_INCLUDE_DIRS})
            message(STATUS "Found Orbbec library via pkg-config: ${ORBBEC_LIBRARIES}")
        else()
            # 最后尝试在/opt/OrbbecSDK/lib目录中查找
            find_library(ORBBEC_LIB NAMES OrbbecSDK libOrbbecSDK.so.2 libOrbbecSDK.so.2.4.11 libOrbbecSDK PATHS /opt/OrbbecSDK/lib)
            if(ORBBEC_LIB)
                target_link_libraries(cam_mgr_node ${ORBBEC_LIB})
                target_include_directories(cam_mgr_node PRIVATE /opt/OrbbecSDK/include)
                message(STATUS "Found Orbbec library at: ${ORBBEC_LIB}")
            else()
                message(WARNING "Orbbec library not found")
            endif()
        endif()
    else()
        # 如果pkg-config不可用，直接在/opt/OrbbecSDK/lib目录中查找
        find_library(ORBBEC_LIB NAMES OrbbecSDK libOrbbecSDK.so.2 libOrbbecSDK.so.2.4.11 libOrbbecSDK PATHS /opt/OrbbecSDK/lib)
        if(ORBBEC_LIB)
            target_link_libraries(cam_mgr_node ${ORBBEC_LIB})
            target_include_directories(cam_mgr_node PRIVATE /opt/OrbbecSDK/include)
            message(STATUS "Found Orbbec library at: ${ORBBEC_LIB}")
        else()
            message(WARNING "Orbbec library not found")
        endif()
    endif()
endif()

target_link_libraries(cam_mgr_node
  yaml-cpp
  stdc++fs
  nlohmann_json::nlohmann_json  # 链接json库
  ${bas_operate_LIBRARIES}
  ${realsense2_LIBRARY}
  ${visualization_LIBRARIES}
  ${OpenCV_LIBS}
  ${hand_eye_calib_LIBRARIES}
)



# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

# 安装
install(TARGETS
  cam_mgr_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# launch directory has been removed
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# 删除对本地srv目录的安装，因为所有服务都在其他包中定义
# install(DIRECTORY
#   srv/
#   DESTINATION share/${PROJECT_NAME}/srv
# )

# 导出include目录
ament_export_include_directories(include)

# 导出依赖
ament_package()