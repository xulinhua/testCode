# 相机SDK Aruco测试项目

## 项目简介

相机SDK Aruco测试项目(cam_sdk_aruco_test)是一个用于相机SDK实时采图进行Aruco码识别，或加载本地图片文件列表进行Aruco码识别的C++应用程序。该项目支持两种工作模式：
1. 实时相机模式：通过相机SDK实时采集图像并进行Aruco码识别
2. 本地文件模式：加载本地图片文件进行Aruco码识别，并支持键盘切换图片

## 项目架构

```
cam_sdk_aruco_test/
├── config/                      # 配置文件目录
│   └── aruco_show_config.yaml   # 主配置文件
├── src/                         # 源代码目录
│   ├── include/                 # 头文件目录
│   │   └── cam_sdk_aruco_test/  # 项目头文件
│   │       └── cam_sdk_aruco_test.hpp
│   └── src/                     # 源文件目录
│       ├── cam_sdk_aruco_test.cpp
│       └── main.cpp
├── CMakeLists.txt               # CMake构建文件
└── package.xml                  # ROS包描述文件
```

## 功能特性

### 1. 双模式支持
- **相机实时模式**：连接并控制真实相机设备，实时采集图像进行Aruco码识别
- **本地文件模式**：加载本地图片文件进行Aruco码识别，支持键盘切换图片

### 2. 可视化显示
- 实时显示采集或加载的图像
- 叠加显示Aruco码识别结果
- 支持交互式操作

### 3. 键盘交互控制
- **ESC键**：退出程序
- **↑/↓/←/→键**：在文件模式下切换上一张/下一张图片

### 4. 配置驱动
- 通过YAML配置文件控制工作模式和参数
- 支持相机参数配置
- 支持图片文件夹路径配置

## 项目结构

### 核心类说明

#### CamSdkArucoTest
主控制类，负责整个应用程序的初始化、运行和资源管理。

主要功能：
- 加载和解析配置文件
- 根据工作模式初始化相机或文件系统
- 控制主循环运行
- 处理键盘输入事件
- 管理资源释放

### 模块依赖关系

```
cam_sdk_aruco_test
├── cam_manage        # 相机管理模块（相机SDK接口）
├── visualization     # 可视化显示模块
├── aruco_alg         # Aruco算法检测模块
├── OpenCV            # 图像处理库
└── yaml-cpp/yaml_cpp_vendor # YAML配置文件解析库
```

## 编译说明

### 独立编译（不依赖ROS）

1. 确保系统已安装依赖库：
   ```bash
   sudo apt update
   sudo apt install libopencv-dev libyaml-cpp-dev cmake build-essential
   ```

2. 编译项目：
   ```bash
   cd src/
   mkdir build
   cd build
   cmake ..
   make
   ```

### ROS环境编译

在ROS工作空间中编译：
```bash
cd ~/your_ros_workspace/
colcon build --packages-select cam_sdk_aruco_test
```

## 运行方法

### 配置文件说明

配置文件位于`config/aruco_show_config.yaml`：

```yaml
# 工作模式配置
work_mode: "camera"  # 可选: "camera"(实时相机) 或 "file"(本地图片文件)

# 相机配置 (仅在work_mode为"camera"时有效)
camera:
  # 使用的相机配置文件路径
  config_file: "../../../sys_config/cam_config.yaml"

# 文件配置 (仅在work_mode为"file"时有效)
file:
  # 图片文件夹路径
  image_folder: "./images"
  # 支持的图片格式
  image_extensions: [".jpg", ".jpeg", ".png", ".bmp"]
```

### 运行程序

1. 独立运行：
   ```bash
   ./cam_sdk_aruco_test [配置文件路径]
   ```

2. ROS环境下运行：
   ```bash
   ros2 run cam_sdk_aruco_test cam_sdk_aruco_test [配置文件路径]
   ```

### 使用示例

1. 相机实时模式：
   ```bash
   # 修改配置文件中的work_mode为"camera"
   ./cam_sdk_aruco_test ../config/aruco_show_config.yaml
   ```

2. 本地文件模式：
   ```bash
   # 修改配置文件中的work_mode为"file"
   # 并设置正确的image_folder路径
   ./cam_sdk_aruco_test ../config/aruco_show_config.yaml
   ```

## 项目实现原理

### 1. 工作模式切换机制

项目通过配置文件中的`work_mode`参数决定运行模式：
- `camera`模式：初始化相机管理器，通过相机SDK获取实时图像流
- `file`模式：扫描指定文件夹中的图片文件，构建图片列表

### 2. 图像处理流程

```
图像获取 → Aruco检测 → 结果可视化 → 显示输出
    ↑                                    ↓
    └────────── 键盘事件处理 ←─────────────┘
```

### 3. 相机控制流程（相机模式）

1. 读取相机配置文件
2. 初始化相机管理器
3. 配置相机参数
4. 启动图像采集循环
5. 实时获取图像帧
6. 进行Aruco检测
7. 显示结果

### 4. 文件处理流程（文件模式）

1. 扫描图片文件夹
2. 构建支持格式的图片列表
3. 按文件名排序
4. 加载当前图片
5. 进行Aruco检测
6. 显示结果
7. 等待键盘事件切换图片

## 依赖说明

### 核心依赖

1. **OpenCV**：图像处理和可视化显示
2. **yaml-cpp/yaml_cpp_vendor**：YAML配置文件解析（根据环境自动选择）
3. **cam_manage**：相机SDK接口封装
4. **visualization**：通用可视化管理模块
5. **aruco_alg**：Aruco码检测算法模块

### 系统依赖

- Ubuntu 22.04 LTS
- C++17编译器
- CMake 3.8+

## 注意事项

1. 在相机模式下，确保相机设备已正确连接并安装了相应的SDK驱动
2. 在文件模式下，确保图片文件夹路径正确且包含支持格式的图片文件
3. 程序运行时可以通过ESC键退出
4. 在文件模式下，可以通过方向键切换图片
5. Aruco检测结果会实时显示在图像上

## 故障排除

### 常见问题

1. **相机无法连接**
   - 检查相机是否正确连接
   - 确认相机驱动是否安装正确
   - 检查相机配置文件参数是否正确

2. **图片无法加载**
   - 检查图片文件夹路径是否正确
   - 确认图片格式是否在支持列表中
   - 检查文件权限是否正确

3. **Aruco检测无结果**
   - 确认图片中包含Aruco码
   - 检查Aruco码类型是否与检测器配置匹配
   - 调整图像亮度和对比度

### 日志信息

程序会在控制台输出运行状态和错误信息，可根据输出信息进行问题定位。