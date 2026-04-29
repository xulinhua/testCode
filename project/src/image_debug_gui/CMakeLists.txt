cmake_minimum_required(VERSION 3.8)
project(image_debug_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

add_library(image_debug_core SHARED
  include/image_debug_gui/main_window.hpp
  include/image_debug_gui/image_view_widget.hpp
  src/main_window.cpp
  src/image_view_widget.cpp
)

target_include_directories(image_debug_core PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(image_debug_core
  ${OpenCV_LIBS}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

ament_target_dependencies(image_debug_core
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(image_debug_app
  src/main.cpp
)

target_link_libraries(image_debug_app
  image_debug_core
)

ament_target_dependencies(image_debug_app
  rclcpp
)

install(TARGETS
  image_debug_core
  image_debug_app
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(image_debug_core)
ament_export_dependencies(rclcpp sensor_msgs cv_bridge OpenCV)
ament_package()
