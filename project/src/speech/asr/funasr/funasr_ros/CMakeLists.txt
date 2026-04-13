cmake_minimum_required(VERSION 3.8)
project(funasr_ros)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find Python
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# Include Python directories
include_directories(${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})

# Install Python modules
install(PROGRAMS
  scripts/asr_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules (if any)
# install(DIRECTORY
#   scripts/
#   DESTINATION lib/${PROJECT_NAME}
#   USE_SOURCE_PERMISSIONS
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()