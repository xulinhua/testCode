cmake_minimum_required(VERSION 3.8)
project(piper_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)

# Install Python executables
install(PROGRAMS
  piper_ros/tts_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tts_node
)

install(PROGRAMS
  tests/test_tts_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_tts_node
)

install(PROGRAMS
  tests/test_imports.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_imports
)

install(PROGRAMS
  tests/verify_tts_functionality.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verify_tts_functionality
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource files (models, configs, etc.) from the parent pipertts directory
install(DIRECTORY
  ../resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()