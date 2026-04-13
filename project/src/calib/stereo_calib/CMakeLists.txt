cmake_minimum_required(VERSION 3.8)
project(stereo_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(src/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/src/stereo_calib.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "OpenCV"
  "Eigen3"
)

install(DIRECTORY src/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY src/include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(stereo_calib_example src/test/example_stereo_calib.cpp)
target_link_libraries(stereo_calib_example ${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)
target_compile_features(stereo_calib_example PRIVATE cxx_std_17)

add_executable(stereo_calibrate src/test/stereo_calibrate.cpp)
target_link_libraries(stereo_calibrate ${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)
target_compile_features(stereo_calibrate PRIVATE cxx_std_17)

add_executable(stereo_calib_test src/test/stereo_calib_test.cpp)
target_link_libraries(stereo_calib_test ${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)
target_compile_features(stereo_calib_test PRIVATE cxx_std_17)

add_executable(stereo_process src/test/stereo_process.cpp)
target_link_libraries(stereo_process ${PROJECT_NAME} ${OpenCV_LIBS} yaml-cpp)
target_compile_features(stereo_process PRIVATE cxx_std_17)

install(TARGETS stereo_calib_example stereo_calibrate stereo_calib_test stereo_process
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
