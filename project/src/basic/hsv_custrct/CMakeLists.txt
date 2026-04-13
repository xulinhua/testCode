# hsv_custrct CMake配置 - 专为Ubuntu22.04 + ROS Humble优化
#
# 构建配置说明：
# - 使用C++17标准，支持现代C++特性
# - 集成ROS2 ament构建系统
# - 提供完整的库构建和安装配置
# - 支持测试框架和代码检查
#
# 关键配置项：
# - C++17标准：启用filesystem等现代特性
# - 编译器警告：启用严格编译检查
# - 依赖管理：自动处理ROS2依赖关系
# - 安装配置：支持系统级安装和部署

cmake_minimum_required(VERSION 3.8)
project(hsv_custrct)

# 默认使用ROS ament构建系统
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用C++17 filesystem支持（Ubuntu 22.04需要显式启用）
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# 定义宏以使用新的IAPoint结构
add_definitions(-DUSING_OLD_IAPOINT_STRUCT=0)

# 包含目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# 添加hsv_custrct子目录到包含路径
include_directories(
  include/hsv_custrct
)

# 源文件
set(SOURCES
  src/GCImage.cpp          # 图像处理实现
  src/HSDrawObj.cpp         # 绘制对象基类实现
  src/HSDrawObj_Arrow.cpp   # 箭头绘制对象实现
  src/HSDrawObj_Circle.cpp  # 圆形绘制对象实现
  src/HSDrawObj_Contours.cpp # 轮廓绘制对象实现
  src/HSDrawObj_Cross.cpp   # 十字绘制对象实现
  src/HSDrawObj_Ellipse.cpp # 椭圆绘制对象实现
  src/HSDrawObj_Line.cpp    # 直线绘制对象实现
  src/HSDrawObj_Point.cpp   # 点绘制对象实现
  src/HSDrawObj_Polygon.cpp # 多边形绘制对象实现
  src/HSDrawObj_Rect.cpp    # 矩形绘制对象实现
  src/HSDrawObj_Rect2.cpp   # 矩形2绘制对象实现
  src/HSDrawObj_Text.cpp    # 文本绘制对象实现
  src/HSDrawRegInfo.cpp     # 绘制区域信息实现
  src/HSGeo.cpp             # 几何计算实现
  src/HSPoint.cpp           # 点结构实现
  src/HSRoi.cpp             # 感兴趣区域实现
  src/IAPtStruct.cpp        # 智能点结构实现

)

# 创建HSV结构库
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

# 链接库和依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp  # ROS2 C++客户端库
  std_msgs  # 标准消息库
)

# 显式链接OpenCV库
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBRARIES}
)

# 编译选项
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall      # 启用所有警告
  -Wextra     # 启用额外警告
  -Wpedantic  # 启用严格标准检查
)

# ROS包安装配置 - 安装头文件
install(DIRECTORY include/hsv_custrct/
  DESTINATION include/hsv_custrct  # 安装头文件到include/hsv_custrct目录
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib   # 静态库安装目录
  LIBRARY DESTINATION lib   # 动态库安装目录
  RUNTIME DESTINATION bin   # 可执行文件安装目录
)

# 导出配置 - 确保其他包能找到hsv_custrct库
ament_export_include_directories(include)  # 导出安装后的头文件目录
ament_export_libraries(${PROJECT_NAME})           # 导出库文件
ament_export_targets(export_${PROJECT_NAME})  # 导出目标
ament_export_dependencies(rclcpp std_msgs OpenCV)                # 导出依赖关系

# 添加ament包配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()  # 自动发现测试依赖
endif()

# 完成包配置
ament_package()