#!/bin/bash

# Gazebo仿真启动脚本

echo "正在启动Gazebo仿真..."

# 设置ROS2环境
source /home/hs/testCode/simulation/install/setup.sh

# 启动Gazebo仿真
ros2 launch nova_sim gazebo.launch.py

echo "Gazebo仿真已关闭"
