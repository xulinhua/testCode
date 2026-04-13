# ArUco Algorithm Demo Application

这是一个完整的ArUco标记检测演示应用程序，展示了如何使用aruco_alg库进行实时标记检测。

## 功能特性

### 摄像头支持
- ✅ 普通USB摄像头支持
- ✅ Intel RealSense深度相机支持
- ✅ 多摄像头设备选择
- ✅ 实时视频流处理

### 检测功能
- ✅ 实时ArUco标记检测
- ✅ 3D位姿估计
- ✅ 可视化显示
- ✅ 控制台输出
- ✅ 截图保存功能

### 用户交互
- ⌨️ 键盘控制（退出、保存）
- 🖥️ 实时可视化界面
- 📊 详细的信息显示
- 🎛️ 多种配置选项

## 项目结构

```
aruco_alg_demo/
├── src/
│   └── aruco_demo_node.cpp    # 主程序
├── launch/
│   └── aruco_demo.launch.py   # 启动文件
├── include/aruco_alg_demo/    # 头文件目录（预留）
├── package.xml                # 包配置
├── CMakeLists.txt             # 构建配置
└── README.md                  # 本文件
```

## 依赖要求

### 必需依赖
- CMake >= 3.8
- OpenCV >= 4.0
- aruco_alg库

### 可选依赖
- Intel RealSense SDK 2.0 (用于深度相机功能)

## 编译和安装

### 方法1：使用colcon
```bash
colcon build --packages-select aruco_alg_demo
```

### 方法2：使用CMake
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make install
```

## 使用方法

### 基本使用

#### 运行程序
```bash
# 编译后运行
./install/aruco_alg_demo/lib/aruco_alg_demo/aruco_alg_demo_node

# 或使用launch文件
ros2 launch aruco_alg_demo aruco_demo.launch.py
```

#### 命令行参数
```bash
aruco_alg_demo_node [选项]

选项：
  --camera-id <id>     摄像头设备ID (默认: 0)
  --realsense          使用RealSense相机 (默认: false)
  --no-display         禁用显示窗口
  --no-console         禁用控制台输出
  --marker-length <m>  标记长度，单位米 (默认: 0.1)
  --enable-scaling     启用图像缩放以提高性能
  --scale-factor <f>   图像缩放因子 (默认: 0.5)
  --help, -h           显示帮助信息
```

### 使用示例

#### 1. 基本检测
```bash
# 使用默认摄像头
aruco_alg_demo_node

# 使用摄像头1
aruco_alg_demo_node --camera-id 1
```

#### 2. 高精度检测
```bash
# 使用更大的标记
aruco_alg_demo_node --marker-length 0.15

# 启用性能优化
aruco_alg_demo_node --enable-scaling --scale-factor 0.3
```

#### 3. 深度相机
```bash
# 使用RealSense相机
aruco_alg_demo_node --realsense
```

#### 4. 静默模式
```bash
# 只显示，不输出到控制台
aruco_alg_demo_node --no-console

# 只输出到控制台，不显示
aruco_alg_demo_node --no-display
```

### 键盘控制

程序运行时的键盘操作：

- **'q' 或 'ESC'**：退出程序
- **'s'**：保存当前帧为图片文件

### 输出显示

#### 可视化窗口
程序会在窗口中显示：
- 🔴 ArUco标记边界框
- 🎯 标记ID号
- 📐 3D坐标系轴
- 📍 位置和旋转信息
- 📏 距离信息
- 🎯 2D中心点坐标

#### 控制台输出
程序会在控制台显示每个检测到的标记的详细信息：
```
=== Marker 0 ===
Position (m): X:0.234 Y:-0.156 Z:0.500
Rotation (deg): X:12.345 Y:-5.678 Z:23.456
Distance: 0.589m
Center (px): X:320.0 Y:240.0
```

## 配置文件

### 相机标定文件
程序会自动查找`camera_calibration.yaml`文件，如果没有找到，将使用默认相机参数。

**相机标定文件格式：**
```yaml
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ fx, 0.0, cx,
           0.0, fy, cy,
           0.0, 0.0, 1.0 ]

distortion_coefficients: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ k1, k2, p1, p2, k3 ]
```

### 创建相机标定文件

您可以使用OpenCV的相机标定工具来生成标定文件：

```python
import cv2
import numpy as np

# 标定参数
square_size = 0.025  # 25mm
pattern_size = (9, 6) # 9x6 角点

# 运行标定程序...
```

## 摄像头设置

### 普通摄像头
- 确保摄像头连接正常
- 检查设备ID：Linux下使用`ls /dev/video*`
- 确保摄像头没有被其他程序占用

### Intel RealSense摄像头
```bash
# 安装RealSense SDK
sudo apt install librealsense2-dev  # Ubuntu
# 或使用vcpkg安装
vcpkg install realsense2:x64-windows  # Windows
```

## 性能优化

### 图像处理优化
```bash
# 启用图像缩放（提高速度）
aruco_alg_demo_node --enable-scaling --scale-factor 0.5

# 更激进的缩放（更快但精度降低）
aruco_alg_demo_node --enable-scaling --scale-factor 0.3
```

### 系统优化
- 使用较快的CPU
- 确保足够的内存
- 使用SSD存储
- 关闭不必要的后台程序

## 故障排除

### 常见问题

**1. 相机无法打开**
```bash
# 检查摄像头设备
# Linux
ls /dev/video*

# 检查权限
sudo chmod 666 /dev/video0

# Windows
# 检查设备管理器中的摄像头
```

**2. 检测不到标记**
- 确保光照条件良好
- 使用高质量的Aruco标记打印
- 检查标记是否在摄像头视野内
- 验证相机标定参数

**3. 程序崩溃**
- 检查相机标定文件格式
- 确保所有依赖库正确安装
- 使用调试模式编译获取详细信息

**4. 性能问题**
- 启用图像缩放
- 降低摄像头分辨率
- 使用更快的硬件

### 调试模式

程序支持多种调试选项：

```bash
# 启用详细输出
# 在代码中设置 detector->setPrintDebugInfo(true)

# 保存调试图片
# 按's'键保存当前帧
```

## 集成到您的项目

### 作为库使用
```cpp
#include "aruco_demo_node.hpp"

// 创建演示节点
ArucoDemoNode demo;

// 设置参数
demo.parseArguments(argc, argv);

// 运行
demo.run();
```

### 自定义启动配置

修改`launch/aruco_demo.launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_alg_demo',
            executable='aruco_alg_demo_node',
            name='aruco_demo',
            output='screen',
            parameters=[
                {'camera_id': 0},
                {'marker_length': 0.1},
                {'enable_scaling': False}
            ]
        ),
    ])
```

## 示例场景

### 机器人导航
```bash
# 使用大标记进行远距离检测
aruco_alg_demo_node --marker-length 0.2 --enable-scaling

# 使用RealSense获取精确深度
aruco_alg_demo_node --realsense --marker-length 0.15
```

### 增强现实
```bash
# 高精度模式
aruco_alg_demo_node --no-console --marker-length 0.1

# 保存截图用于分析
aruco_alg_demo_node --no-display
```

### 质量检测
```bash
# 生产线上快速检测
aruco_alg_demo_node --enable-scaling --scale-factor 0.4 --no-console
```

## 扩展功能

### 添加新的相机支持
```cpp
// 在aruco_demo_node.cpp中添加新摄像头类型
// 实现相应的初始化和帧获取代码
```

### 自定义可视化
```cpp
// 修改drawArucoResults函数
// 添加自定义的绘制逻辑
```

### 数据记录
```cpp
// 添加数据保存功能
// 记录标记位置和位姿数据
```

## 许可证

MIT License

## 支持

如有问题，请查看：
1. aruco_alg库文档
2. OpenCV ArUco教程
3. Intel RealSense文档
4. 项目GitHub Issues

开始使用ArUco标记检测吧！🚀