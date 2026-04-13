cmake_minimum_required(VERSION 3.8)
project(cam_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cam_manage REQUIRED)
find_package(OpenCV REQUIRED)
find_package(log_system REQUIRED)

find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(sensor_msgs REQUIRED)      # 添加 sensor_msgs
find_package(std_msgs REQUIRED)         # 添加 std_msgs
find_package(cv_bridge REQUIRED)        # 添加 cv_bridge
find_package(image_transport REQUIRED)  # 添加 image_transport
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

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

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${ament_index_cpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${cam_manage_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${log_system_INCLUDE_DIRS}
  #../cam_manage/src/include
  /usr/local/include
)

# 源文件
set(SOURCES
    src/cam_demo.cpp
)

add_definitions(${PCL_DEFINITIONS})

add_executable(cam_demo 
  src/src/cam_demo.cpp
)

find_library(ORBBEC_LIB NAMES OrbbecSDK libOrbbecSDK.so)
if(ORBBEC_LIB)
    target_link_libraries(cam_demo ${ORBBEC_LIB})
endif()

find_library(REALSENSE_LIB NAMES librealsense2.so realsense2)
if(REALSENSE_LIB)
    #target_link_libraries(cam_test ${REALSENSE_LIB})
    target_link_libraries(cam_demo /usr/local/lib/librealsense2.so)
else()
    pkg_check_modules(REALSENSE REQUIRED realsense2)
    if(REALSENSE_FOUND)
        target_link_libraries(cam_demo ${REALSENSE_LIBRARIES})
        target_include_directories(cam_demo PRIVATE ${REALSENSE_INCLUDE_DIRS})
    endif()
endif()
target_link_libraries(cam_demo
  ${ament_index_cpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${cam_manage_LIBRARIES}
  ${log_system_LIBRARIES}
)

ament_target_dependencies(cam_demo 
  rclcpp
  rclcpp_components
  visualization_msgs
  PCL
  pcl_conversions
  sensor_msgs
  std_msgs
  cam_manage
  cv_bridge
  image_transport
  yaml-cpp
  realsense2
  log_system
)

target_link_libraries(cam_demo
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  /usr/local/lib/librealsense2.so
)

# Install targets
install(TARGETS cam_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_index_cpp)
ament_export_dependencies(rclcpp)
ament_export_dependencies(yaml-cpp) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()