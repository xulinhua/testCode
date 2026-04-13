cmake_minimum_required(VERSION 3.8)
project(bas_operate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用C++17 filesystem支持（Ubuntu 22.04需要显式启用）
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# 包含目录
include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 源文件
set(SOURCES
    src/file_operate.cpp #文件基础操作
    src/bas_utils.cpp    #基础工具类
)

# 创建系统库
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

# 链接库和依赖
ament_target_dependencies(${PROJECT_NAME}
    rclcpp  # ROS2 C++客户端库
    yaml_cpp_vendor  # yaml-cpp支持
)

# 启用YAML支持
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_YAML_CPP)

# 编译选项
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall      # 启用所有警告
    -Wextra     # 启用额外警告
    -Wpedantic  # 启用严格标准检查
)

# ROS包安装配置 - 安装头文件
install(DIRECTORY include/bas_operate/
  DESTINATION include/bas_operate  # 安装头文件到include/bas_operate
)



install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib   # 静态库安装目录
  LIBRARY DESTINATION lib   # 动态库安装目录
  RUNTIME DESTINATION bin   # 可执行文件安装目录
)

# 导出配置 - 确保其他包能找到bas_operate库
ament_export_include_directories(include)  # 导出安装后的头文件目录
ament_export_libraries(${PROJECT_NAME})           # 导出库文件
ament_export_targets(export_${PROJECT_NAME})  # 导出目标
ament_export_dependencies(rclcpp)                # 导出依赖关系
ament_export_dependencies(yaml_cpp_vendor)       # 导出yaml-cpp依赖

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 避免使用需要catkin_pkg的lint工具
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 添加测试程序
add_executable(bas_utils_test test/bas_utils_test.cpp)
ament_target_dependencies(bas_utils_test rclcpp)
target_link_libraries(bas_utils_test ${PROJECT_NAME})

# 添加格式检查测试程序
add_executable(test_format_checker src/test_format_checker.cpp)
target_link_libraries(test_format_checker ${PROJECT_NAME})

# 安装测试程序
install(TARGETS bas_utils_test test_format_checker
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()