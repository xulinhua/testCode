cmake_minimum_required(VERSION 3.8)
project(data_handler)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
# 系统中有 CUDA 12.4，需要显式指定路径
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(log_system REQUIRED)  # 依赖 log_system 包
find_package(bas_operate REQUIRED) # 依赖 bas_operate 包

# CUDA 版本兼容性处理：解决 OpenCV 要求的 CUDA 版本与系统版本不匹配问题
# 当 find_package(OpenCV) 调用 find_host_package(CUDA) 时，如果版本不匹配会导致失败
# 通过设置 CUDA_VERSION_OK 变量，允许使用更高版本的 CUDA
set(CUDA_VERSION_OK TRUE)

find_package(OpenCV REQUIRED)      # 依赖 OpenCV 包

# nlohmann_json
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSON REQUIRED nlohmann_json)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用C++17 filesystem支持（Ubuntu 22.04需要显式启用）
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# 包含目录
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${log_system_INCLUDE_DIRS}
    ${bas_operate_INCLUDE_DIRS}
    ${JSON_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 源文件
set(SOURCES
    src/yaml_operate.cpp
    src/param_reflector.cpp
    src/json_operate.cpp
)

# 创建数据处理库
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

# 链接库和依赖
ament_target_dependencies(${PROJECT_NAME}
    rclcpp  # ROS2 C++客户端库
    log_system  # 日志系统依赖
    bas_operate  # 基础操作库依赖
)

# 链接JSON库
if(JSON_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} ${JSON_LIBRARIES})
endif()

# 链接OpenCV库
if(OpenCV_LIBS)
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()

# 编译选项
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall      # 启用所有警告
    -Wextra     # 启用额外警告
    -Wpedantic  # 启用严格标准检查
)

# ROS包安装配置 - 安装头文件
install(DIRECTORY include/data_handler/
  DESTINATION include/data_handler  # 安装头文件到include/data_handler
)

# 安装JSON操作头文件
install(FILES include/data_handler/json_operate.h
  DESTINATION include/data_handler
)

# 安装YAML操作头文件
install(FILES include/data_handler/yaml_operate.h
  DESTINATION include/data_handler
)

# 安装参数反射器头文件
install(FILES include/data_handler/param_reflector.hpp
  DESTINATION include/data_handler
)

# 安装配置文件
install(FILES config/log_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib   # 静态库安装目录
  LIBRARY DESTINATION lib   # 动态库安装目录
  RUNTIME DESTINATION bin   # 可执行文件安装目录
)

# 导出配置 - 确保其他包能找到data_handler库
ament_export_include_directories(include)  # 导出安装后的头文件目录
ament_export_libraries(${PROJECT_NAME})           # 导出库文件
ament_export_targets(export_${PROJECT_NAME})  # 导出目标
ament_export_dependencies(rclcpp log_system bas_operate nlohmann_json OpenCV) # 导出依赖关系

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 避免使用需要catkin_pkg的lint工具
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()