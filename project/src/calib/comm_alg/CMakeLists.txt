cmake_minimum_required(VERSION 3.8)
project(comm_alg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)

# CUDA 版本兼容性处理：指定使用 CUDA 12.4 以匹配 OpenCV 编译时的版本
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.4")
set(CUDA_HOME "/usr/local/cuda-12.4")
set(CUDA_VERSION "12.4")

find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME}
  src/marker_detect_base.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install configuration files
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# Install export targets
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Avoid using lint tools that require catkin_pkg
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()