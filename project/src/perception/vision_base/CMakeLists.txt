cmake_minimum_required(VERSION 3.8)
project(vision_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包含目录
include_directories(
  include
  src
  ${OpenCV_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
  ${bas_operate_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# 创建task_factory库（包含工厂实现）
add_library(task_factory SHARED
  src/task_factory.cpp
  src/algorithm_loader.cpp
)

# 链接OpenCV和yaml-cpp
target_link_libraries(task_factory
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

# 设置target属性
target_include_directories(task_factory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(task_factory
  OpenCV
  log_system
  bas_operate
)

# ament_export_dependencies 添加 yaml-cpp
ament_export_dependencies(yaml-cpp)

# 安装头文件（直接安装到include目录，不带项目名前缀）
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# 安装库
install(TARGETS task_factory
  EXPORT task_factory_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
# 导出
ament_export_targets(task_factory_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(
  include
)
ament_export_libraries(task_factory)
ament_export_dependencies(OpenCV log_system)

ament_package()
