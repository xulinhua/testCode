# debug_viewer CMake配置 - 专为Ubuntu22.04 + ROS Humble优化
#
# 构建配置说明：
# - 使用C++17标准，支持现代C++特性
# - 集成ROS2 ament构建系统
# - 提供完整的库构建和安装配置
# - 支持测试框架和代码检查
# - 优化编译性能和链接配置
# - 确保依赖库正确链接
#
# 关键配置项：
# - C++17标准：启用filesystem等现代特性
# - 编译器警告：启用严格编译检查
# - 依赖管理：自动处理ROS2依赖关系
# - 安装配置：支持系统级安装和部署
# - 编译优化：启用性能优化标志

cmake_minimum_required(VERSION 3.8)
project(debug_viewer)

# 默认使用ROS ament构建系统
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 查找hsv_custrct包
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../install")
find_package(hsv_custrct REQUIRED)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用C++17 filesystem支持（Ubuntu 22.04需要显式启用）
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# 编译优化选项
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-deprecated-declarations")

# 确保库路径正确
message(STATUS "OpenCV library dir: ${OpenCV_LIBRARY_DIRS}")
message(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "hsv_custrct include dir: ${hsv_custrct_INCLUDE_DIRS}")
message(STATUS "hsv_custrct libraries: ${hsv_custrct_LIBRARIES}")
message(STATUS "Freetype library: ${FREETYPE_LIBRARIES}")

# 使用pkg-config查找freetype（更可靠的方法）
find_package(PkgConfig REQUIRED)
pkg_check_modules(FREETYPE REQUIRED freetype2)

# 更新freetype库路径
set(FREETYPE_LIBRARIES ${FREETYPE_LIBRARIES})
set(FREETYPE_INCLUDE_DIRS ${FREETYPE_INCLUDE_DIRS})

# 包含目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${hsv_custrct_INCLUDE_DIRS}
)

# 添加freetype到包含目录
include_directories(
  ${FREETYPE_INCLUDE_DIRS}
)

# 源文件
set(SOURCES
  src/ChartDraw.cpp      # 图表绘制实现
  src/ChartDraw_Bar.cpp   # 柱状图绘制实现
  src/ChartDraw_Line.cpp  # 折线图绘制实现
  src/ChartDraw_Pie.cpp   # 饼图绘制实现
  src/ChineseTextSupport.cpp  # 中文文本支持实现
  src/DbgInfoDefine.cpp   # 调试信息定义实现
  src/DebugViewer.cpp     # 调试查看器主实现
  src/DispComm.cpp        # 显示通用功能实现
  src/DrawFunc.cpp        # 绘制功能实现
  src/GCImage.cpp         # 图像处理实现
  src/Page.cpp            # 页面管理实现
  src/TableDraw.cpp       # 表格绘制实现
  src/TextDisp.cpp        # 文本显示实现

)

# 创建调试查看器库
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

# 链接库和依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp  # ROS2 C++客户端库
  std_msgs  # 标准消息库
)

# 显式链接外部库
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES}
  ${FREETYPE_LIBRARIES}
  ${hsv_custrct_LIBRARIES}
  stdc++fs
)

# 创建可执行文件
add_executable(debug_viewer_node src/debug_viewer_node.cpp)

# 创建测试程序
add_executable(test_debug_viewer test/test_debug_viewer.cpp)
# 链接可执行文件
target_link_libraries(debug_viewer_node
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${FREETYPE_LIBRARIES}
  ${hsv_custrct_LIBRARIES}
  stdc++fs
)
ament_target_dependencies(debug_viewer_node
  rclcpp
  std_msgs
)

# 链接测试程序
target_link_libraries(test_debug_viewer
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${FREETYPE_LIBRARIES}
  ${hsv_custrct_LIBRARIES}
  stdc++fs
)

ament_target_dependencies(test_debug_viewer
  rclcpp
  std_msgs
)

# 设置库属性
set_target_properties(${PROJECT_NAME} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# ROS包安装配置 - 安装头文件
install(DIRECTORY include/debug_viewer/
  DESTINATION include/debug_viewer  # 安装头文件到include/debug_viewer目录
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib   # 静态库安装目录
  LIBRARY DESTINATION lib   # 动态库安装目录
  RUNTIME DESTINATION bin   # 可执行文件安装目录
)

install(TARGETS debug_viewer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 可执行文件安装目录
)

install(TARGETS test_debug_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 测试程序安装目录
)

# 安装测试图片
install(FILES image/test.bmp
  DESTINATION share/${PROJECT_NAME}/image  # 安装测试图片到share/debug_viewer/image目录
)

# 导出配置 - 确保其他包能找到debug_viewer库
ament_export_include_directories(include)  # 导出安装后的头文件目录
ament_export_libraries(${PROJECT_NAME})           # 导出库文件
ament_export_targets(export_${PROJECT_NAME})  # 导出目标
ament_export_dependencies(rclcpp std_msgs OpenCV Freetype hsv_custrct)                # 导出依赖关系

# 添加ament包配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()  # 自动发现测试依赖
endif()

# 完成包配置
ament_package()