cmake_minimum_required(VERSION 3.8)
project(yolo_obb_tensorrt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(vision_base REQUIRED)
find_package(log_system REQUIRED)
find_package(bas_operate REQUIRED)

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
include_directories(${vision_base_INCLUDE_DIRS})
include_directories(${log_system_INCLUDE_DIRS})
include_directories(${bas_operate_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${TENSORRT_INCLUDE_DIR})

# 添加CUDA架构设置（根据GPU型号调整）
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3 -Xcompiler=-fPIC")

# 收集源文件
set(SOURCES
    src/yolo_obb_det_tensorrt.cpp
)

set(CUDA_SOURCES
    src/process.cu
)

# 编译CUDA源文件
cuda_add_library(yolo_obb_tensorrt_cuda STATIC
    ${CUDA_SOURCES}
)

target_include_directories(yolo_obb_tensorrt_cuda PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
)

target_link_libraries(yolo_obb_tensorrt_cuda
    ${TENSORRT_LIBRARY}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
)

# 编译主库
add_library(yolo_obb_tensorrt SHARED
    ${SOURCES}
)

target_link_libraries(yolo_obb_tensorrt
    yolo_obb_tensorrt_cuda
    ${TENSORRT_LIBRARY}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
)

# 链接vision_base（获取接口定义）
ament_target_dependencies(yolo_obb_tensorrt
    vision_base
    log_system
    bas_operate
)

# 安装
install(TARGETS yolo_obb_tensorrt yolo_obb_tensorrt_cuda
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY config/
    DESTINATION config
)

ament_export_include_directories(include)
ament_export_libraries(yolo_obb_tensorrt yolo_obb_tensorrt_cuda)

ament_package()
