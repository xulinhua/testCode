cmake_minimum_required(VERSION 3.10)
project(easy_deploy_tools)

# Find CUDA
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find TensorRT
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
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/deploy_core/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inference_core/trt_core/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/deploy_utils)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${TENSORRT_INCLUDE_DIR})

# Pre-built library paths
set(DEPLOY_CORE_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdeploy_core.so)
set(TRT_CORE_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/libtrt_core.so)
set(IMAGE_PROCESSING_UTILS_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/lib/libimage_processing_utils.so)

# Create imported libraries for ament_export
add_library(deploy_core SHARED IMPORTED)
set_target_properties(deploy_core PROPERTIES
    IMPORTED_LOCATION ${DEPLOY_CORE_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/deploy_core/include"
)

add_library(trt_core SHARED IMPORTED)
set_target_properties(trt_core PROPERTIES
    IMPORTED_LOCATION ${TRT_CORE_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/inference_core/trt_core/include;${CMAKE_CURRENT_SOURCE_DIR}/deploy_core/include"
)

add_library(image_processing_utils SHARED IMPORTED)
set_target_properties(image_processing_utils PROPERTIES
    IMPORTED_LOCATION ${IMAGE_PROCESSING_UTILS_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/deploy_utils"
)

# Link dependencies
set_target_properties(deploy_core PROPERTIES
    INTERFACE_LINK_LIBRARIES "${CUDA_LIBRARIES};${CUDA_cudart_LIBRARY};${TENSORRT_LIBRARY};${OpenCV_LIBS}"
)

set_target_properties(trt_core PROPERTIES
    INTERFACE_LINK_LIBRARIES "${CUDA_LIBRARIES};${CUDA_cudart_LIBRARY};${TENSORRT_LIBRARY}"
)

# Install libraries from lib folder (these are pre-built libraries)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so"
)

message(STATUS "easy_deploy_tool configured successfully")
