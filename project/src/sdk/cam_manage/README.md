# cam_manage - 相机管理库开发指南

## 项目概述

cam_manage是一个统一的相机管理库，专门用于封装和管理多种相机的接口，包括Orbbec和RealSense相机。本库提供了统一的API接口，简化了多相机系统的开发和集成。

## 项目结构说明

### 整体项目结构
```
project/                           # 项目根目录
├── src/                          # 源代码目录
│   ├── sdk/                      # SDK目录
│   │   ├── cam_manage/          # 相机管理核心库
│   │   │   ├── src/             # cam_manage源码目录
│   │   │   │   ├── CMakeLists.txt   # cam_manage构建配置
│   │   │   │   ├── package.xml      # cam_manage包配置
│   │   │   │   ├── include/         # 头文件目录
│   │   │   │   │   └── cam_manage/  # 相机管理头文件
│   │   │   │   │       ├── cam_base.hpp       # 相机基类
│   │   │   │   │       ├── cam_manage.hpp     # 相机管理器
│   │   │   │   │       ├── cam_orbbec.hpp     # Orbbec相机接口
│   │   │   │   │       ├── cam_realsense.hpp  # RealSense相机接口
│   │   │   │   │       └── cam_com_struct.hpp # 通用数据结构
│   │   │   │   ├── src/             # 实现文件目录
│   │   │   │   │   ├── cam_base.cpp        # 相机基类实现
│   │   │   │   │   ├── cam_manage.cpp      # 相机管理器实现
│   │   │   │   │   ├── cam_orbbec.cpp      # Orbbec相机实现
│   │   │   │   │   └── cam_realsense.cpp   # RealSense相机实现
│   │   │   │   ├── test/            # 测试文件目录
│   │   │   │   │   └── cam_test.cpp         # 测试程序
│   │   │   │   └── config/          # 配置文件目录
│   │   │   └── cam_demo/           # 演示项目（调用方）
│   │   │       └── src/             # cam_demo源码目录
│   │   │           ├── CMakeLists.txt  # cam_demo构建配置
│   │   │           ├── package.xml     # cam_demo包配置
│   │   │           ├── README.md       # 项目说明文档
│   │   │           └── src/            # 源代码目录
│   │   │               └── cam_demo.cpp # 演示程序主文件
├── install/                     # 安装目录（构建后生成）
│   └── cam_manage/             # cam_manage安装目录
│       ├── include/            # 安装的头文件
│       └── lib/                # 安装的库文件
└── build/                      # 构建目录（构建后生成）
    └── cam_manage/             # cam_manage构建中间文件
```

### cam_manage详细文件说明
```
cam_manage/src/                    # cam_manage项目根目录
├── CMakeLists.txt              # CMake构建配置文件
│   - 配置依赖查找（OpenCV、PCL、相机SDK等）
│   - 设置编译选项和C++标准
│   - 配置静态库构建规则
│   - 配置测试程序构建
│   - 设置头文件和库文件安装规则
├── package.xml                 # ROS包配置文件
│   - 声明项目元数据
│   - 定义构建工具依赖
│   - 配置测试依赖
│   - 设置导出规则
├── README.md                   # 项目说明文档
│   - 开发和使用指南
│   - API接口说明
│   - 构建和安装步骤
│   - 故障排除方法
├── include/                    # 头文件目录
│   └── cam_manage/            # 公共接口头文件
│       ├── cam_base.hpp       # 相机基类定义
│       ├── cam_manage.hpp     # 相机管理器接口
│       ├── cam_orbbec.hpp     # Orbbec相机接口
│       ├── cam_realsense.hpp  # RealSense相机接口
│       └── cam_com_struct.hpp # 通用数据结构定义
├── src/                        # 实现文件目录
│   ├── cam_base.cpp           # 相机基类实现
│   ├── cam_manage.cpp         # 相机管理器实现
│   ├── cam_orbbec.cpp         # Orbbec相机实现
│   └── cam_realsense.cpp      # RealSense相机实现
├── test/                       # 测试文件目录
│   └── cam_test.cpp           # 功能测试程序
└── config/                     # 配置文件目录
    └── 相机配置文件
```

### 关键文件作用
- **CMakeLists.txt**: 配置构建系统，确保正确查找和链接所有依赖库
- **package.xml**: 声明包信息和依赖关系
- **cam_base.hpp/cpp**: 定义相机基类接口，实现通用功能
- **cam_manage.hpp/cpp**: 实现相机管理器，提供统一的管理接口
- **cam_orbbec.hpp/cpp**: Orbbec相机的具体实现
- **cam_realsense.hpp/cpp**: RealSense相机的具体实现
- **cam_com_struct.hpp**: 定义相机通信相关的数据结构
- **README.md**: 完整的开发和使用指南

## 核心功能模块

### 1. 相机基类 (cam_base)
- 提供所有相机的通用接口定义
- 实现基础的相机操作功能
- 定义虚拟函数供子类实现

### 2. 相机管理器 (cam_manage)
- 实现单例模式，全局统一管理
- 支持多相机的发现、连接和管理
- 提供相机配置和参数管理功能
- 统一的相机数据获取接口

### 3. Orbbec相机模块 (cam_orbbec)
- 封装Orbbec SDK接口
- 支持Orbbec相机的深度和彩色数据获取
- 实现ROI配置和流管理
- 提供相机内参和标定功能

### 4. RealSense相机模块 (cam_realsense)
- 封装Intel RealSense SDK
- 支持RealSense相机的多种数据流
- 实现高级功能如深度滤波、对齐等
- 提供设备管理和配置功能

## 依赖库和环境要求

### 必需依赖
- **CMake**: 3.8+
- **C++17**: 支持现代C++特性
- **OpenCV**: 4.x版本（图像处理）
- **PCL**: 1.10+（点云处理）

### 相机SDK依赖
- **OrbbecSDK**: Orbbec SDK 2.4.11
- **librealsense2**: Intel RealSense SDK 2.0

### 系统要求
- Ubuntu 20.04/22.04 LTS
- 支持USB3.0的硬件接口
- 足够的计算资源处理实时图像数据

## 构建和安装步骤

### 1. 环境准备
```bash
# 安装ROS2依赖
sudo apt update
sudo apt install ros-humble-rclcpp ros-humble-ament-cmake

# 安装OpenCV和PCL
sudo apt install libopencv-dev libpcl-dev

# 安装相机SDK（根据实际相机类型）
##### Orbbec SDK #####
sudo apt update
sudo apt install -y git cmake build-essential libudev-dev libusb-1.0-0-dev libv4l-dev libopencv-dev libgoogle-glog-dev libgflags-dev
cd Downloads/
cd OrbbecSDK_v2-2.4.11/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
cd ../scripts/
sudo bash install_udev_rules.sh
# 创建udev规则文件
sudo tee /etc/udev/rules.d/99-obsensor-libusb.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
EOF
# 重新加载udev规则并触发
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev $USER

##### 下载并编译最新版 RealSense SDK #####
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
# 检查最新发布版本
git tag -l | sort -V | tail -5
# 切换到最新稳定版本（例如 2.57.1）
git checkout v2.57.1
# 创建构建目录
mkdir build && cd build
# 配置编译选项
cmake .. \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=false
# 编译并安装
make -j$(nproc)
sudo make install
# 设置库路径
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/realsense.conf
sudo ldconfig
```

### 2. 构建顺序
```bash
# 构建cam_manage库
colcon build --packages-select cam_manage

```

### 3. 环境配置
```bash
# 源化工作空间
source install/setup.bash

# 验证安装
ros2 pkg list | grep cam_manage
```

## API使用示例

### 基本使用流程
```cpp
#include "cam_manage/cam_manage.hpp"

// 获取相机管理器实例
CameraManager& cam_manager = CameraManager::get_instance();

//初始化并打开相机
CamConfigInfo1D configs;
CamConfigInfo config;

config.cam_index = 0;
config.cam_type = CamType::CAM_TYPE_OB;
config.depth_para.fps = 30;
config.color_para.fps = 30;
config.color_para.width = 640;
config.color_para.height = 480;
config.depth_para.width = 640;
config.depth_para.height = 480;
configs.push_back(config);
RtnType rtn = cam_manager_->init_all_camera(configs);

//获取相机内参
CamIntrinsics intrinsics;
CamStreamType stream_type =  CamStreamType::STREAM_COLOR；
RtnType rtn = cam_manager_->get_cam_intrinsics(0, stream_type, intrinsics); // 获取相机内参
//获取一帧图像/点云
cv::Mat *color_image = nullptr;
PointCloudXYZPtr cloud = nullptr;
cam_manager_->get_one_frame_color(0, color_image);
cam_manager_->get_one_frame_depth(0, cloud);

```

## 常见问题和解决方案

### 1. 构建问题
**问题**: 找不到OrbbecSDK或librealsense2库
**解决**: 确保SDK正确安装并设置库路径
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### 2. 相机连接问题
**问题**: 无法发现相机设备
**解决**: 检查USB连接和权限
```bash
# 添加用户到plugdev组
sudo usermod -a -G plugdev $USER
# 重新登录或重启
```

### 3. 运行时问题
**问题**: 程序运行时崩溃或无数据
**解决**: 检查相机是否被其他程序占用，确保相机驱动正常

## 开发和扩展

### 添加新的相机类型
1. 继承`CamBase`类
2. 实现所有虚函数
3. 在`CameraManage`中注册新相机类型
4. 更新CMakeLists.txt添加新依赖

## 测试和验证

### 运行测试程序
```bash
# 构建测试
colcon build --packages-select cam_manage

# 运行测试
ros2 run cam_manage cam_test
```

### 功能验证
- 相机发现和连接
- 图像数据获取
- 点云数据生成
- 参数配置和保存
- 多相机并发操作
