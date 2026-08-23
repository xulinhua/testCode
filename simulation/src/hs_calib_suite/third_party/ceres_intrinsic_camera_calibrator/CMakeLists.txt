cmake_minimum_required(VERSION 3.8)
project(tier4_ceres_intrinsic CXX)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

add_library(tier4_ceres_intrinsic STATIC
  src/ceres_camera_intrinsics_optimizer.cpp
  src/ceres_intrinsic_calibrate.cpp
)

target_include_directories(tier4_ceres_intrinsic PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(tier4_ceres_intrinsic PUBLIC
  Ceres::ceres
  ${OpenCV_LIBS}
)

target_compile_features(tier4_ceres_intrinsic PUBLIC cxx_std_17)
set_target_properties(tier4_ceres_intrinsic PROPERTIES POSITION_INDEPENDENT_CODE ON)
