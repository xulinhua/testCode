cmake_minimum_required(VERSION 3.8)
project(cam_manage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(log_system REQUIRED)
find_package(nlohmann_json REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "PCL include directories: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
message(STATUS "PCL definitions: ${PCL_DEFINITIONS}")

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用C++17 filesystem支持（Ubuntu 22.04需要显式启用）
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 保持其他警告，只禁用特定的匿名结构体警告
    add_compile_options(-Wno-unknown-pragmas -Wno-pedantic)
    add_compile_options(-Wno-pedantic -Wno-extra-semi)
    add_compile_options(-Wno-sign-compare)
endif()
# 包含目录
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${log_system_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 源文件
set(SOURCES
    src/cam_base.cpp       # 相机基类
    src/cam_orbbec.cpp     # orbbec相机
    src/cam_realsense.cpp  # realsense相机
    src/cam_manage.cpp     # 相机管理类
)

# 创建相机库
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

# 链接库和依赖
ament_target_dependencies(${PROJECT_NAME}
    rclcpp  # ROS2 C++客户端库
    log_system
)

# 编译选项
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall      # 启用所有警告
    -Wextra     # 启用额外警告
    -Wpedantic  # 启用严格标准检查
)

# 测试程序
add_executable(cam_test test/cam_test.cpp)
target_link_libraries(cam_test ${PROJECT_NAME})
ament_target_dependencies(cam_test rclcpp)

# 保存内参测试程序
# add_executable(save_intrinsics_test test/save_intrinsics_test.cpp)
# target_link_libraries(save_intrinsics_test ${PROJECT_NAME})
# ament_target_dependencies(save_intrinsics_test rclcpp)

# 查找并链接RealSense库
find_library(REALSENSE_LIB NAMES librealsense2.so realsense2)
if(REALSENSE_LIB)
    target_link_libraries(${PROJECT_NAME} ${REALSENSE_LIB})
    target_link_libraries(cam_test ${REALSENSE_LIB})
    # target_link_libraries(save_intrinsics_test ${REALSENSE_LIB})
else()
    pkg_check_modules(REALSENSE REQUIRED realsense2)
    if(REALSENSE_FOUND)
        target_link_libraries(${PROJECT_NAME} ${REALSENSE_LIBRARIES})
        target_link_libraries(cam_test ${REALSENSE_LIBRARIES})
        # target_link_libraries(save_intrinsics_test ${REALSENSE_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PRIVATE ${REALSENSE_INCLUDE_DIRS})
        target_include_directories(cam_test PRIVATE ${REALSENSE_INCLUDE_DIRS})
        # target_include_directories(save_intrinsics_test PRIVATE ${REALSENSE_INCLUDE_DIRS})
    endif()
endif()

# 查找并链接Orbbec库
find_library(ORBBEC_LIB NAMES orbbec libOrbbecSDK.so)
if(ORBBEC_LIB)
    target_link_libraries(${PROJECT_NAME} ${ORBBEC_LIB})
    target_link_libraries(cam_test ${ORBBEC_LIB})
    # target_link_libraries(save_intrinsics_test ${ORBBEC_LIB})
    target_include_directories(${PROJECT_NAME} PRIVATE /opt/OrbbecSDK/include)
else()
    pkg_check_modules(ORBBEC REQUIRED orbbec)
    if(ORBBEC_FOUND)
        target_link_libraries(${PROJECT_NAME} ${ORBBEC_LIBRARIES})
        target_link_libraries(cam_test ${ORBBEC_LIBRARIES})
        # target_link_libraries(save_intrinsics_test ${ORBBEC_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PRIVATE ${ORBBEC_INCLUDE_DIRS})
        target_include_directories(cam_test PRIVATE ${ORBBEC_INCLUDE_DIRS})
        # target_include_directories(save_intrinsics_test PRIVATE ${ORBBEC_INCLUDE_DIRS})
    endif()
endif()

target_link_libraries(cam_test ${OpenCV_LIBS})
target_link_libraries(cam_test ${PCL_LIBRARIES})
# target_link_libraries(save_intrinsics_test ${OpenCV_LIBS})
# target_link_libraries(save_intrinsics_test ${PCL_LIBRARIES})

# ROS包安装配置 - 安装头文件
install(DIRECTORY include/cam_manage/
  DESTINATION include/cam_manage  # 安装头文件到include/cam_manage目录
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib   # 静态库安装目录
  LIBRARY DESTINATION lib   # 动态库安装目录
  RUNTIME DESTINATION bin   # 可执行文件安装目录
)

# 导出配置 - 注意相机sdk要同时导出
ament_export_include_directories(include)  # 导出安装后的头文件目录
ament_export_libraries(${PROJECT_NAME})           # 导出库文件
ament_export_dependencies(rclcpp OpenCV PCL log_system nlohmann_json)   # 导出依赖关系

# 导出RealSense依赖
if(REALSENSE_LIB)
    ament_export_libraries(${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
    ament_export_dependencies(realsense2)
endif()

# 导出Orbbec依赖
if(ORBBEC_LIB)
    ament_export_libraries(${ORBBEC_LIB})
elseif(ORBBEC_FOUND)
    ament_export_dependencies(orbbec)
endif()

install(TARGETS cam_test # save_intrinsics_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 测试程序安装目录
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()