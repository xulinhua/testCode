cmake_minimum_required(VERSION 3.8)
project(visualization)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用C++17 filesystem支持
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# 查找必要的包
find_package(ament_cmake REQUIRED)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

find_package(OpenCV REQUIRED)
find_package(log_system REQUIRED)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
)

# 源文件
set(SOURCES
  src/visualization_mgr.cpp
)

# 创建库
add_library(${PROJECT_NAME} STATIC
  src/visualization_mgr.cpp
)

# 设置位置无关代码，使库可以被链接到共享库中
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 目标链接库
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS}
)

# 目标依赖
ament_target_dependencies(${PROJECT_NAME} 
  "OpenCV"
  "log_system"
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(FILES config/log_config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装库
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 导出配置
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV log_system)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()