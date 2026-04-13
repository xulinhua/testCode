cmake_minimum_required(VERSION 3.10)
project(hand_eye_calib)

# 设置构建目录到外部
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CMAKE_BINARY_DIR "${CMAKE_SOURCE_DIR}/../build/${PROJECT_NAME}" CACHE PATH "Output directory for hand_eye_calib" FORCE)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用C++17 filesystem支持
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(aruco_alg REQUIRED)
find_package(visualization REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(log_system REQUIRED)
find_package(data_handler REQUIRED)
find_package(bas_operate REQUIRED)

# 查找第三方库
# nlohmann_json
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSON REQUIRED nlohmann_json)

# 查找realsense库
find_library(REALSENSE_LIB NAMES librealsense2.so realsense2)
if(NOT REALSENSE_LIB)
    pkg_check_modules(REALSENSE REQUIRED realsense2)
endif()

# 包含目录
include_directories(
  include
  src
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${JSON_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${data_handler_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
  ${bas_operate_INCLUDE_DIRS}
)

# 添加aruco_alg的包含路径
include_directories(
  ${PROJECT_SOURCE_DIR}/../calib/aruco_alg/src/include
)

# 链接目录
link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${JSON_LIBRARY_DIRS}
)

# 源文件
set(SOURCES
  src/main.cpp
  src/calib_data_collector.cpp
  src/calib_utils.cpp
  src/calib_struct.cpp
  src/calib_robot_pos_mgr.cpp
  src/calib_config.cpp
  src/sys_calib_data_mgr.cpp
  src/calib_reflector.cpp
  src/calib_data_operate.cpp
  src/calib_backend.cpp
  src/head_utils.cpp
)

# 创建库
add_library(hand_eye_calib_core STATIC
  src/calib_data_collector.cpp
  src/calib_utils.cpp
  src/calib_struct.cpp
  src/calib_robot_pos_mgr.cpp
  src/calib_config.cpp
  src/sys_calib_data_mgr.cpp
  src/calib_reflector.cpp
  src/calib_data_operate.cpp
  src/calib_backend.cpp
  src/head_utils.cpp
)

# 设置位置无关代码，使库可以被链接到共享库中
set_target_properties(hand_eye_calib_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 创建可执行文件
add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(hand_eye_calib_core
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
)

target_link_libraries(hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 如果找到了realsense库，则链接它
if(REALSENSE_LIB)
  target_link_libraries(hand_eye_calib_core ${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
  target_link_libraries(hand_eye_calib_core ${REALSENSE_LIBRARIES})
endif()

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
)

target_link_libraries(${PROJECT_NAME}
  hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装库
install(TARGETS hand_eye_calib_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# 导出配置
ament_export_include_directories(include)
ament_export_libraries(hand_eye_calib_core)
ament_export_dependencies(OpenCV Eigen3 aruco_alg visualization yaml-cpp log_system data_handler bas_operate)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 添加测试程序
add_executable(test_calibration test_calibration.cpp)

ament_target_dependencies(test_calibration
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
)

target_link_libraries(test_calibration
  hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 如果找到了realsense库，则链接它
if(REALSENSE_LIB)
  target_link_libraries(test_calibration ${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
  target_link_libraries(test_calibration ${REALSENSE_LIBRARIES})
endif()

# 安装核心算法测试程序
install(TARGETS test_calibration
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 添加数据采集模块测试程序
add_executable(test_data_collector test/test_data_collector.cpp)

ament_target_dependencies(test_data_collector
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
  ament_index_cpp
)

target_link_libraries(test_data_collector
  hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 如果找到了realsense库，则链接它
if(REALSENSE_LIB)
  target_link_libraries(test_data_collector ${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
  target_link_libraries(test_data_collector ${REALSENSE_LIBRARIES})
endif()

# 安装数据采集测试程序
install(TARGETS test_data_collector
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 添加标定矩阵生成模块测试程序
add_executable(test_calib_matrix test/test_calib_matrix.cpp)

ament_target_dependencies(test_calib_matrix
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
)

target_link_libraries(test_calib_matrix
  hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 如果找到了realsense库，则链接它
if(REALSENSE_LIB)
  target_link_libraries(test_calib_matrix ${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
  target_link_libraries(test_calib_matrix ${REALSENSE_LIBRARIES})
endif()

# 安装标定矩阵生成测试程序
install(TARGETS test_calib_matrix
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 添加标定分析模块测试程序
add_executable(test_calib_analysis test/test_calib_analysis.cpp)

ament_target_dependencies(test_calib_analysis
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
)

target_link_libraries(test_calib_analysis
  hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 如果找到了realsense库，则链接它
if(REALSENSE_LIB)
  target_link_libraries(test_calib_analysis ${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
  target_link_libraries(test_calib_analysis ${REALSENSE_LIBRARIES})
endif()

# 安装标定分析测试程序
install(TARGETS test_calib_analysis
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 添加标定应用模块测试程序
add_executable(test_calib_application test/test_calib_application.cpp)

ament_target_dependencies(test_calib_application
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
)

target_link_libraries(test_calib_application
  hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 如果找到了realsense库，则链接它
if(REALSENSE_LIB)
  target_link_libraries(test_calib_application ${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
  target_link_libraries(test_calib_application ${REALSENSE_LIBRARIES})
endif()

# 安装标定应用测试程序
install(TARGETS test_calib_application
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 添加机械臂坐标管理模块测试程序
add_executable(test_calib_robot_pos_mgr test/test_calib_robot_pos_mgr.cpp)

ament_target_dependencies(test_calib_robot_pos_mgr
  rclcpp
  aruco_alg
  visualization
  log_system
  data_handler
  bas_operate
)

target_link_libraries(test_calib_robot_pos_mgr
  hand_eye_calib_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${JSON_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  data_handler::data_handler
  bas_operate::bas_operate
)

# 如果找到了realsense库，则链接它
if(REALSENSE_LIB)
  target_link_libraries(test_calib_robot_pos_mgr ${REALSENSE_LIB})
elseif(REALSENSE_FOUND)
  target_link_libraries(test_calib_robot_pos_mgr ${REALSENSE_LIBRARIES})
endif()

# 安装机械臂坐标管理测试程序
install(TARGETS test_calib_robot_pos_mgr
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 添加gtest依赖
find_package(ament_cmake_gtest REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()