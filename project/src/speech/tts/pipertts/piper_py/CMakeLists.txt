cmake_minimum_required(VERSION 3.8)
project(piper_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python executables
ament_python_install_package(piper_py)  # 修改为正确的包名

# Install scripts
install(PROGRAMS
  run_demo.py
  test_tts.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install models and other resources from the parent pipertts directory
install(DIRECTORY
  ../resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 确保Python依赖被安装
if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
  # 在虚拟环境中，使用pip安装依赖
  message(STATUS "在虚拟环境中，将使用pip安装Python依赖")
  execute_process(
    COMMAND pip3 install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
    RESULT_VARIABLE PIP_INSTALL_RESULT
    OUTPUT_VARIABLE PIP_INSTALL_OUTPUT
    ERROR_VARIABLE PIP_INSTALL_ERROR
  )
  if(NOT PIP_INSTALL_RESULT EQUAL 0)
    message(WARNING "pip安装失败: ${PIP_INSTALL_ERROR}")
  endif()
else()
  # 在系统环境中，提示用户手动安装依赖
  message(STATUS "不在虚拟环境中，提示用户手动安装Python依赖")
  message(STATUS "请运行: pip3 install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")
endif()

ament_package()