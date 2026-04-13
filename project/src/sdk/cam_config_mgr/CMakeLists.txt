cmake_minimum_required(VERSION 3.8)
project(cam_config_mgr)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)

# 包含目录
include_directories(
  src/include
  ${YAML_CPP_INCLUDE_DIRS}
  ${NLOHMANN_JSON_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
  ${bas_operate_INCLUDE_DIRS}
)

# 添加库
add_library(cam_config_mgr STATIC
  src/src/camera_config_manager.cpp
)

# 设置位置无关代码，使库可以被链接到共享库中
set_target_properties(cam_config_mgr PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 链接依赖
target_link_libraries(cam_config_mgr
  yaml-cpp
  nlohmann_json::nlohmann_json
  stdc++fs
  ${log_system_LIBRARIES}
  ${bas_operate_LIBRARIES}
)

# 安装头文件
install(DIRECTORY
  src/include/
  DESTINATION include
)

# 安装库文件
install(TARGETS
  cam_config_mgr
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 安装配置文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 移除对已删除示例程序的引用
# add_executable(cam_config_example
#   src/examples/test.cpp
# )

# target_link_libraries(cam_config_example
#   cam_config_mgr
#   ${log_system_LIBRARIES}
#   ${bas_operate_LIBRARIES}
# )

# 添加综合测试程序
# add_executable(test_config_loading
#   test/test_config_loading.cpp
# )
#
# target_link_libraries(test_config_loading
#   cam_config_mgr
# )

# 添加内参外参测试程序
# add_executable(test_intrinsics
#   src/examples/test_intrinsics.cpp
# )

# target_link_libraries(test_intrinsics
#   cam_config_mgr
#   bas_operate
# )

# install(TARGETS
#   cam_config_example
#   # cam_config_comprehensive_test
#   # test_intrinsics
#   DESTINATION lib/${PROJECT_NAME}
# )

# 导出依赖
ament_export_dependencies(yaml-cpp nlohmann_json log_system bas_operate)
ament_export_include_directories(include)
ament_export_libraries(cam_config_mgr)

# 导出包
ament_package()