cmake_minimum_required(VERSION 3.10)
project(foundationpose_alg)

# Set CMAKE_CUDA_ARCHITECTURES before project() to avoid policy warning
set(CMAKE_CUDA_ARCHITECTURES "75" CACHE STRING "CUDA architectures")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)

enable_language(CUDA)

# Find glog
find_package(glog REQUIRED)

# Find assimp
find_package(assimp REQUIRED)

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
else()
    message(FATAL_ERROR "TensorRT not found. Please install TensorRT.")
endif()

# ppl.cv - bundled static library
set(PPLCV_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ppl.cv/include)
set(PPLCV_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/ppl.cv/lib/libpplcv_static.a)
set(PPLCOMMON_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/ppl.cv/lib/libpplcommon_static.a)

# Include directories
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${TENSORRT_INCLUDE_DIR})
include_directories(${PPLCV_INCLUDE_DIR})

# Set CUDA compiler
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fPIC --expt-relaxed-constexpr")
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# Add subdirectory for easy_deploy_tool (uses pre-built libraries)
set(ENABLE_TENSORRT ON CACHE BOOL "Enable TensorRT support")
set(ENABLE_ORT OFF CACHE BOOL "Enable ONNX Runtime support")
add_subdirectory(easy_deploy_tool)

# Add include paths for easy_deploy_tool
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/easy_deploy_tool/deploy_core/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/easy_deploy_tool/inference_core/trt_core/include)

# Source files
set(source_file
  src/nvdiffrast/common/cudaraster/impl/Buffer.cpp
  src/nvdiffrast/common/cudaraster/impl/CudaRaster.cpp
  src/nvdiffrast/common/cudaraster/impl/RasterImpl.cpp
  src/nvdiffrast/common/cudaraster/impl/RasterImpl.cu
  src/nvdiffrast/common/common.cpp
  src/nvdiffrast/common/interpolate.cu
  src/nvdiffrast/common/rasterize.cu
  src/nvdiffrast/common/texture.cu
  src/foundationpose_render.cpp
  src/foundationpose_render.cu
  src/foundationpose_sampling.cpp
  src/foundationpose_sampling.cu
  src/foundationpose_utils.cu
  src/foundationpose_decoder.cu
  src/foundationpose.cpp
  src/mesh_loader/assimp_mesh_loader.cpp
)

# Add library with CUDA support
add_library(${PROJECT_NAME} SHARED ${source_file})

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
    LINKER_LANGUAGE "CXX"
)

# Link libraries (deploy_core and trt_core are pre-built libraries from easy_deploy_tool)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${OpenCV_LIBS}
  glog::glog
  assimp::assimp
  ${PPLCV_LIBRARY}
  ${PPLCOMMON_LIBRARY}
  ${CUDA_LIBRARIES}
  ${CUDA_cudart_LIBRARY}
  ${CMAKE_CURRENT_SOURCE_DIR}/easy_deploy_tool/lib/libdeploy_core.so
  ${CMAKE_CURRENT_SOURCE_DIR}/easy_deploy_tool/lib/libtrt_core.so
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install easy_deploy_tool headers (deploy_core and trt_core)
install(DIRECTORY easy_deploy_tool/deploy_core/include/deploy_core/
  DESTINATION include/deploy_core
)

install(DIRECTORY easy_deploy_tool/inference_core/trt_core/include/trt_core/
  DESTINATION include/trt_core
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}_Targets"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install pre-built libraries from easy_deploy_tool
install(DIRECTORY easy_deploy_tool/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)

# Install ppl.cv libraries (needed at runtime)
install(DIRECTORY ppl.cv/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.a" PATTERN "*.so"
)

# Export targets
ament_export_targets(${PROJECT_NAME}_Targets)
ament_export_dependencies(OpenCV Eigen3 glog assimp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
