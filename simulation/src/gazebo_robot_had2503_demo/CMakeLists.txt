cmake_minimum_required(VERSION 3.8)
project(gazebo_robot_had2503_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config launch worlds moveit rviz
  DESTINATION share/${PROJECT_NAME}/
  PATTERN ".linked" EXCLUDE
)

# model://gazebo_robot_had2503_demo/... 需要包目录下有 model.config
install(FILES model.config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/had2503_debug_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
