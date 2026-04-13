cmake_minimum_required(VERSION 3.8)
project(x-trainer_asm-0226)  # 保留后缀，但需处理特殊字符

# 声明 C++标准（ROS2 要求）
set(CMAKE_CXX_STANDARD 17)

# 查找 ROS2 核心依赖
find_package(ament_cmake REQUIRED)

# 安装资源文件（自动处理含后缀的包名）
foreach(dir config launch meshes urdf)
  install(
    DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir}  # PROJECT_NAME 自动包含后缀
  )
endforeach()

# 必须的 ROS2 指令
ament_package()