cmake_minimum_required(VERSION 3.8)
project(bas_sys_config)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
# 必须在所有 find_package 之前设置
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

# 检查编译器
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(log_system REQUIRED)
find_package(hand_eye_calib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(data_handler REQUIRED)
find_package(bas_operate REQUIRED)

# 获取log_system包含目录
get_filename_component(log_system_INCLUDE_DIRS "${log_system_DIR}/../../../include" ABSOLUTE)

# 包含目录
include_directories(
  include
  ${log_system_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 获取data_handler包含目录
get_filename_component(data_handler_INCLUDE_DIRS "${data_handler_DIR}/../../../include" ABSOLUTE)
include_directories(${data_handler_INCLUDE_DIRS})

# 获取bas_operate包含目录
get_filename_component(bas_operate_INCLUDE_DIRS "${bas_operate_DIR}/../../../include" ABSOLUTE)
include_directories(${bas_operate_INCLUDE_DIRS})

# 创建核心库
add_library(${PROJECT_NAME}_core
  src/sys_config_mgr.cpp
  src/sys_config_struct.cpp
  src/config_reflector.cpp
)

# 设置位置无关代码，使库可以被链接到共享库中
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}_core yaml-cpp ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_core ${data_handler_LIBRARIES})
target_compile_features(${PROJECT_NAME}_core PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME}_core
  yaml-cpp
  log_system
  hand_eye_calib
  OpenCV
  data_handler
  bas_operate
)

# 安装目标
install(TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}_core
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件目录
install(DIRECTORY include/
  DESTINATION include/)

# 安装源文件目录
install(DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "*.cpp" EXCLUDE)

# 复制系统配置文件到安装目录
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../bas_config_data
  DESTINATION ../
  FILES_MATCHING PATTERN "*")

# Install config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
# 导出库
ament_export_include_directories(include)
ament_export_dependencies(
  yaml-cpp
  log_system
  hand_eye_calib
  OpenCV
  data_handler
  bas_operate)

# 导出目标
ament_export_targets(${PROJECT_NAME}_core HAS_LIBRARY_TARGET)

# 注册包
ament_package()