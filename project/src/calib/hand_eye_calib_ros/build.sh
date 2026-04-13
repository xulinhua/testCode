#!/bin/bash

# hand_eye_calib_ros 构建脚本
# 用于在 ROS 2 Humble 环境下构建 hand_eye_calib_ros 包

echo "开始构建 hand_eye_calib_ros 包..."

# 检查是否在 ROS 2 环境中
if ! command -v ros2 &> /dev/null
then
    echo "错误: 未找到 ROS 2 环境，请先 source ROS 2 setup.bash"
    exit 1
fi

# 获取当前目录
CURRENT_DIR=$(pwd)
echo "当前目录: $CURRENT_DIR"

# 检查是否在 ROS 2 工作空间中
if [ ! -f "src/calib/hand_eye_calib_ros/package.xml" ]; then
    echo "警告: 似乎不在 ROS 2 工作空间根目录中"
    echo "请确保在包含 src 目录的工作空间根目录中运行此脚本"
fi

# 构建包
echo "正在构建 hand_eye_calib_ros..."
colcon build --packages-select hand_eye_calib_ros

# 检查构建结果
if [ $? -eq 0 ]; then
    echo "✅ hand_eye_calib_ros 构建成功!"
    echo "请运行以下命令来设置环境:"
    echo "source install/setup.bash"
else
    echo "❌ hand_eye_calib_ros 构建失败!"
    exit 1
fi