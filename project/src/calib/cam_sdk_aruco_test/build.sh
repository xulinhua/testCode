#!/bin/bash

# 相机SDK Aruco测试项目构建脚本

# 设置项目根目录
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

# 创建构建目录
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# 检查是否在ROS环境中
if [ -n "$AMENT_PREFIX_PATH" ]; then
    echo "Detected ROS environment, using colcon build"
    # echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
    cd "${PROJECT_ROOT}/../.."
    colcon build --packages-select cam_sdk_aruco_test
else
    echo "Building standalone project"
    # echo "AMENT_PREFIX_PATH not set"
    cd "${BUILD_DIR}"
    cmake ..
    make -j$(nproc)
    
    if [ $? -eq 0 ]; then
        echo "Build successful!"
        echo "Executable located at: ${BUILD_DIR}/cam_sdk_aruco_test"
    else
        echo "Build failed!"
        exit 1
    fi
fi