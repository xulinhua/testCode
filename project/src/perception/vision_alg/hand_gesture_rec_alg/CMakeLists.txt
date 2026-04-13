cmake_minimum_required(VERSION 3.8)
project(hand_gesture_rec_alg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# 为共享库添加-fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

enable_language(CUDA)

# 查找并包含TensorRT
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS /usr/include/aarch64-linux-gnu
    /usr/include
    /usr/local/TensorRT-10.8.0.43/include
    DOC "TensorRT include path")

find_library(TENSORRT_LIBRARY nvinfer
    PATHS /usr/lib/aarch64-linux-gnu
    /usr/lib
    /usr/local/TensorRT-10.8.0.43/lib
    DOC "TensorRT library")

if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY)
    set(TENSORRT_FOUND TRUE)
    message(STATUS "Found TensorRT includes: ${TENSORRT_INCLUDE_DIR}")
    message(STATUS "Found TensorRT library: ${TENSORRT_LIBRARY}")
else()
    message(FATAL_ERROR "TensorRT not found. Please install TensorRT.")
endif()

# Include directories
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${TENSORRT_INCLUDE_DIR})

# 设置CUDA编译选项，添加-fPIC和更多选项
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fPIC --expt-relaxed-constexpr")
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# Add library with CUDA support
add_library(${PROJECT_NAME} SHARED
  src/hand_detector.cpp
  src/hand_pose.cpp
  src/hand_pipeline.cpp
  src/utils.cpp
  src/process.cu
  src/gesture_detector.cpp
)

# 设置目标属性
set_target_properties(${PROJECT_NAME} PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
    LINKER_LANGUAGE "CXX"
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME}
  "OpenCV"
)

# 链接库
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${CUDA_cudadevrt_LIBRARY}
    ${TENSORRT_LIBRARY}
    ${CUDA_cudart_LIBRARY}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install export targets
install(EXPORT ${PROJECT_NAME}_targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()