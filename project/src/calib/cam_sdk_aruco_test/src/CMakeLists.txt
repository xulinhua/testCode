cmake_minimum_required(VERSION 3.8)
project(cam_sdk_aruco_test)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 如果在ROS环境中构建
if(DEFINED ENV{AMENT_PREFIX_PATH})
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(cam_manage REQUIRED)
  find_package(visualization REQUIRED)
  find_package(aruco_alg REQUIRED)
  find_package(comm_alg REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(custom_msgs_comm REQUIRED)
  # 查找log_system包
  find_package(log_system REQUIRED)
  find_package(bas_operate REQUIRED)
  
  # 添加包含目录
  include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${log_system_INCLUDE_DIRS}
    ${cam_manage_INCLUDE_DIRS}
  )
  
  # 添加可执行文件
  add_executable(${PROJECT_NAME}
    src/main.cpp
    src/cam_sdk_aruco_test.cpp
  )
  
  # 添加ROS环境编译定义
  target_compile_definitions(${PROJECT_NAME} PRIVATE __ROS_ENV__)
  
  # 链接库
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    stdc++fs  # filesystem库
  )
  
  # 导出依赖
  ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
    "std_msgs"
    "sensor_msgs"
    "cv_bridge"
    "OpenCV"
    "yaml_cpp_vendor"
    "cam_manage"
    "visualization"
    "aruco_alg"
    "comm_alg"
    "ament_index_cpp"
    "custom_msgs_comm"
    "log_system"
    "bas_operate"
  )
  
  # 安装include目录
  install(DIRECTORY include/
    DESTINATION include
  )
  
  # 安装规则
  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  
  # 安装配置文件
  install(DIRECTORY ../config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  
  ament_export_include_directories(include)
  ament_export_libraries(${PROJECT_NAME})
  ament_export_dependencies(OpenCV yaml_cpp_vendor cam_manage visualization aruco_alg)
  
  ament_package()
else()
  # 非ROS环境下的构建配置
  find_package(OpenCV REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(PkgConfig REQUIRED)
  # 查找log_system包
  find_package(log_system REQUIRED)
  find_package(bas_operate REQUIRED)
  
  # 查找相机管理模块
  find_package(cam_manage REQUIRED)
  if(NOT cam_manage_FOUND)
    message(FATAL_ERROR "cam_manage package not found")
  endif()
  
  # 查找可视化模块
  find_package(visualization REQUIRED)
  if(NOT visualization_FOUND)
    message(FATAL_ERROR "visualization package not found")
  endif()
  
  # 查找aruco算法模块
  find_package(aruco_alg REQUIRED)
  if(NOT aruco_alg_FOUND)
    message(FATAL_ERROR "aruco_alg package not found")
  endif()
  
  # 添加包含目录
  include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${log_system_INCLUDE_DIRS}
    ${cam_manage_INCLUDE_DIRS}
  )
  
  # 添加可执行文件
  add_executable(${PROJECT_NAME}
    src/main.cpp
    src/cam_sdk_aruco_test.cpp
  )
  
  # 链接库
  target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    yaml-cpp
    cam_manage::cam_manage
    visualization::visualization
    aruco_alg::aruco_alg
    stdc++fs  # filesystem库
    pthread  # 线程支持
  )
  
  # 安装规则
  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
  )
  
  # 安装配置文件
  install(DIRECTORY ../config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()