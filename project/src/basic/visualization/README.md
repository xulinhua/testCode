# Visualization 可视化管理模块

## 简介

Visualization是一个通用的可视化管理模块，提供跨平台的图像显示和图形绘制功能。该模块基于OpenCV开发，支持实时图像显示、图形绘制和用户交互功能。

## 功能特性

### 1. 图像显示
- 支持实时图像窗口创建和管理
- 可自定义窗口大小和标题
- 支持多种图像格式显示

### 2. 用户交互
- 键盘事件处理
- 鼠标事件处理
- 窗口关闭事件处理

## 项目结构

```
visualization/
├── CMakeLists.txt                 # CMake构建配置
├── package.xml                    # ROS包描述文件
├── README.md                      # 项目说明文档
├── include/visualization/         # 头文件目录
│   └── visualization_manager.hpp  # 可视化管理器头文件
└── src/                           # 源文件目录
    └── visualization_manager.cpp  # 可视化管理器实现
```

## 依赖项

- OpenCV 4.x
- C++14 或更高版本

## 编译方法

### 在ROS环境下编译

```bash
# 进入ROS工作空间
cd ~/ros2_ws

# 将项目复制到src目录
cp -r /path/to/basic/visualization src/

# 编译
colcon build --packages-select visualization

# 设置环境变量
source install/setup.bash
```

### 独立编译

```bash
# 进入项目目录
cd basic/visualization

# 创建构建目录
mkdir build && cd build

# 配置项目
cmake ..

# 编译
make
```

## 使用方法

### 基本用法

```cpp
#include "visualization/visualization_manager.hpp"

using namespace visualization;

// 创建可视化管理器实例
VisualizationManager vis_manager;

// 初始化可视化窗口
if (vis_manager.initialize("My Window", 1280, 720)) {
    std::cout << "Visualization window created successfully!" << std::endl;
}

// 显示图像
cv::Mat image = cv::imread("test.jpg");
vis_manager.showImage(image);

// 关闭窗口
vis_manager.closeWindows();
```

## API说明

### VisualizationManager 类

#### 构造函数
```cpp
VisualizationManager();
```

#### 初始化函数
```cpp
bool initialize(const std::string& window_name = "Visualization", 
               int width = 1280, int height = 720);
```
初始化可视化管理器，创建显示窗口。

参数:
- `window_name`: 窗口标题
- `width`: 窗口宽度
- `height`: 窗口高度

返回值:
- `true`: 初始化成功
- `false`: 初始化失败

#### 图像显示函数
```cpp
void showImage(const cv::Mat& image);
```
在窗口中显示图像。

参数:
- `image`: 要显示的图像

#### 窗口关闭函数
```cpp
void closeWindows();
```
关闭所有可视化窗口。

#### 窗口状态检查函数
```cpp
bool isWindowOpen() const;
```
检查窗口是否打开。

返回值:
- `true`: 窗口已打开
- `false`: 窗口未打开

## 注意事项

1. 确保系统已安装OpenCV开发包
2. 在多线程环境中使用时需要注意线程安全
3. 图像显示需要GUI环境支持

## 故障排除

### 编译问题

1. **找不到OpenCV**: 确保已安装OpenCV开发包
   ```bash
   sudo apt install libopencv-dev
   ```

### 运行问题

1. **无法创建窗口**: 检查GUI环境是否正常
2. **图像无法显示**: 检查图像数据是否有效