# Camera Configuration Manager (cam_config_mgr)

## 简介

`cam_config_mgr` 是一个不依赖 ROS 的相机配置管理库，用于加载、保存和管理不同型号相机的配置参数。该库支持 RealSense 和 Orbbec 相机，并提供了简单易用的 API 接口。

## 功能特性

- 支持 YAML 格式的配置文件
- 支持多种相机类型配置（RealSense、Orbbec等）
- 提供配置加载和保存功能
- 不依赖 ROS，可独立使用
- 支持多相机配置管理

## 依赖项

- yaml-cpp
- file_operate (项目内的基础库)

## 构建

```bash
# 在项目根目录下执行
colcon build --packages-select cam_config_mgr
```

## 使用方法

### 1. 配置文件格式

```yaml
default_camera:
  camera_type: "realsense"
  device_id: ""
  resolution: "1280x720"
  fps: 30
  auto_exposure: true
  auto_white_balance: true
  enable_save: true
  save_path: "./images"
  save_interval_ms: 1000
  enable_display: false

orbbec_camera:
  camera_type: "orbbec"
  device_id: "A00123456"
  resolution: "640x480"
  fps: 15
  auto_exposure: false
  auto_white_balance: false
  enable_save: false
  save_path: "./orbbec_images"
  save_interval_ms: 2000
  enable_display: true
```

### 2. 代码示例

```cpp
#include "cam_config_mgr/camera_config_manager.hpp"

int main() {
    cam_mgr::CamConfigMgr config_manager;
    
    // 加载配置文件
    if (config_manager.load_config("config.yaml")) {
        // 获取相机配置
        auto config = config_manager.get_camera_config("default_camera");
        std::cout << "相机类型: " << config.camera_type << std::endl;
        std::cout << "分辨率: " << config.width << "x" << config.height << std::endl;
    }
    
    return 0;
}
```

## API 说明

### CameraConfig 结构体

包含以下字段：
- `camera_type`: 相机类型 (realsense, orbbec)
- `device_id`: 设备ID
- `resolution`: 分辨率字符串 (如: "1280x720")
- `fps`: 帧率
- `auto_exposure`: 自动曝光
- `auto_white_balance`: 自动白平衡
- `enable_save`: 启用保存
- `save_path`: 保存路径
- `save_interval_ms`: 保存间隔(毫秒)
- `enable_display`: 启用显示
- `width`: 宽度（自动解析）
- `height`: 高度（自动解析）

### CamConfigMgr 类

#### 构造函数
```cpp
CamConfigMgr();
```

#### load_config
加载配置文件
```cpp
bool load_config(const std::string& config_file);
```

#### save_config
保存配置到文件
```cpp
bool save_config(const std::string& config_file);
```

#### get_camera_config
获取相机配置
```cpp
CameraConfig get_camera_config(const std::string& camera_name);
```

#### set_camera_config
设置相机配置
```cpp
void set_camera_config(const std::string& camera_name, const CameraConfig& config);
```

#### parse_resolution
解析分辨率字符串
```cpp
void parse_resolution(const std::string& resolution, int& width, int& height);
```

#### get_camera_names
获取所有相机名称
```cpp
std::vector<std::string> get_camera_names();
```