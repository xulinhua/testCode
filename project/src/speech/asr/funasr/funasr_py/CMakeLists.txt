cmake_minimum_required(VERSION 3.8)
project(funasr_py)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python packages
ament_python_install_package(${PROJECT_NAME})

# Install Python modules
install(DIRECTORY
  asr_core
  tests
  DESTINATION share/${PROJECT_NAME}
)

# Install resource files (models, configs, etc.) from the parent funasr directory
install(DIRECTORY
  ../resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

# Install Python executables
install(PROGRAMS
  tests/test_model_download.py
  tests/test_model_export.py
  tests/test_model_loading.py
  tests/test_asr_recognition.py
  tests/test_audio_processing.py
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_package()