cmake_minimum_required(VERSION 3.8)
project(chessboard_alg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(comm_alg REQUIRED)

# Include directories
include_directories(src/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/include)

# Add library
add_library(${PROJECT_NAME} SHARED
  src/src/chessboard_pose_detector.cpp
)

# Target dependencies
ament_target_dependencies(${PROJECT_NAME} 
  "OpenCV"
  "Eigen3"
  "comm_alg"
)

# Install configuration files
install(DIRECTORY src/config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install header files
install(DIRECTORY src/include/
  DESTINATION include
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Add executable for chessboard pose example
add_executable(chessboard_pose_example src/test/example_chessboard_pose.cpp)
target_link_libraries(chessboard_pose_example ${PROJECT_NAME} ${OpenCV_LIBS})

# Add executable for camera calibration example
add_executable(camera_calibration_example src/test/example_camera_calibration.cpp)
target_link_libraries(camera_calibration_example ${PROJECT_NAME} ${OpenCV_LIBS})
target_compile_features(camera_calibration_example PRIVATE cxx_std_17)

install(TARGETS chessboard_pose_example camera_calibration_example
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install export targets
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Avoid using lint tools that require catkin_pkg
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()