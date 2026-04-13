# UDP通信项目 (C++版本)

## 项目概述

本项目是UDP通信系统的C++实现版本，完全复制了Python版本的功能，用于在x86主板和Jetson主板之间进行数据通信和命令控制。

## 项目结构

```
udp_comm/
├── custom_msgs_comm/              # 自定义消息包
│   ├── msg/                       # 消息定义
│   │   ├── AICoordinateData.msg
│   │   ├── CalibrationCommand.msg
│   │   ├── CalibrationResponse.msg
│   │   ├── LaserScanData.msg
│   │   ├── PointCloudData.msg
│   │   ├── ProjectCommand.msg
│   │   └── ProjectStatus.msg
│   ├── srv/                       # 服务定义
│   │   ├── ProjectControl.srv
│   │   └── ProjectStatus.srv
│   ├── CMakeLists.txt
│   └── package.xml
├── udp_comm_client/               # UDP客户端包
│   ├── include/udp_comm_client/   # 头文件
│   │   ├── project_manager.hpp
│   │   └── udp_client_node.hpp
│   ├── src/                       # 源文件
│   │   ├── main.cpp
│   │   ├── project_manager.cpp
│   │   └── udp_client_node.cpp
│   ├── config/                    # 配置文件
│   │   └── udp_client_params.yaml
│   ├── launch/                    # 启动文件
│   │   └── udp_client.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
├── udp_comm_server/               # UDP服务器包
│   ├── include/udp_comm_server/   # 头文件
│   │   ├── project_control.hpp
│   │   └── udp_server_node.hpp
│   ├── src/                       # 源文件
│   │   ├── main.cpp
│   │   ├── project_control.cpp
│   │   └── udp_server_node.cpp
│   ├── config/                    # 配置文件
│   │   └── udp_server_params.yaml
│   ├── launch/                    # 启动文件
│   │   └── udp_server.launch.py
│   ├── CMakeLists.txt
│   └── package.xml
└── README.md
```

## 功能特性

### 1. 数据通信
- **AI坐标数据**: 传输目标检测的坐标信息
- **激光扫描数据**: 传输激光雷达扫描数据
- **点云数据**: 传输3D点云数据
- **标定数据**: 支持相机-机器人标定流程

### 2. 项目控制
- **项目启动/停止**: 远程控制项目运行状态
- **状态监控**: 实时监控项目运行状态
- **自动启动**: 支持系统启动时自动运行项目

### 3. 标定功能
- **自动标定**: 支持自动标定流程
- **标定数据管理**: 标定数据的存储和加载
- **标定结果验证**: 标定结果的验证和优化

## 构建说明

### 环境要求
- Ubuntu 22.04
- ROS 2 Humble
- C++17 编译器
- CMake 3.8+

### 构建步骤

1. **创建工作空间**
```bash
mkdir -p ~/udp_comm_ws/src
cd ~/udp_comm_ws/src
```

2. **复制项目代码**
```bash
cp -r /path/to/udp_comm/* .
```

3. **构建项目**
```bash
cd ~/udp_comm_ws
colcon build --packages-select custom_msgs_comm udp_comm_client udp_comm_server
```

4. **配置环境**
```bash
source install/setup.bash
```

## 使用方法

### 启动UDP服务器
```bash
ros2 launch udp_comm_server udp_server.launch.py
```

### 启动UDP客户端
```bash
ros2 launch udp_comm_client udp_client.launch.py
```

### 项目控制命令
```bash
# 启动相机项目
ros2 service call /udp_comm_server/project_control custom_msgs_comm/srv/ProjectControl "{command_type: 0, project_name: 'camera'}"

# 查询项目状态
ros2 service call /udp_comm_server/project_status custom_msgs_comm/srv/ProjectStatus "{project_name: 'camera'}"
```

## 配置参数

### UDP客户端配置 (udp_client_params.yaml)
```yaml
udp_client:
  ros__parameters:
    server_ip: "192.168.10.30"
    server_port: 8888
    client_port: 8889
    bShowRunInfo: true
    log_interval: 10
    udp_blocking_mode: false
    auto_start_enabled: false
    auto_start_camera: false
    auto_start_pcl2laser: false
    auto_start_detection: false
    auto_start_delay: 3.0
```

### UDP服务器配置 (udp_server_params.yaml)
```yaml
udp_server:
  ros__parameters:
    udp_port: 8888
    bShowRunInfo: true
    log_interval: 10
    udp_blocking_mode: false
    client_ip: "192.168.10.30"
    client_port: 8889
```

## 消息类型

| 消息类型 | 描述 | 数据格式 |
|---------|------|----------|
| COMMAND | 控制命令 | JSON |
| AI_COORDINATE | AI坐标数据 | JSON |
| LASERSCAN | 激光扫描数据 | 二进制 |
| POINTCLOUD | 点云数据 | 二进制 |
| CALIBRATION_COMMAND | 标定指令 | JSON |
| CALIBRATION_RESPONSE | 标定响应 | JSON |
| PROJECT_COMMAND | 项目控制指令 | JSON |
| PROJECT_STATUS | 项目状态 | JSON |

## 故障排除

### 常见问题

1. **UDP连接失败**
   - 检查防火墙设置
   - 确认IP地址和端口配置正确
   - 验证网络连接

2. **项目启动失败**
   - 检查依赖包是否安装
   - 确认可执行文件权限
   - 查看系统日志获取详细信息

3. **数据丢失**
   - 调整UDP缓冲区大小
   - 优化网络配置
   - 增加重传机制

### 日志查看

```bash
# 查看UDP客户端日志
ros2 topic echo /udp_comm_client/log

# 查看UDP服务器日志
ros2 topic echo /udp_comm_server/log
```

# UDP通信模块

UDP通信模块用于在x86主板（Ubuntu22.04 + ROS Humble）和Jetson主板（Ubuntu22.04 + ROS Humble）之间通过UDP协议传输命令和数据。

## 功能概述

该模块提供以下核心功能：
1. UDP客户端和服务端实现
2. 项目控制命令传输
3. 状态信息反馈
4. 数据可靠传输机制

## 项目架构

```
graph TD
    subgraph "x86主板"
        A[UDP服务端]
    end
    
    subgraph "Jetson主板"
        B[UDP客户端]
    end
    
    subgraph "通用接口模块"
        C[custom_msgs_comm]
    end
    
    A --- C
    B --- C
```

## 依赖关系

本项目依赖以下模块：
- ROS 2 Humble环境
- custom_msgs_comm：通用消息接口模块

## 消息类型

本项目使用[custom_msgs_comm](file:///192.168.10.61/testCode/project_udp/src/custom_msgs_comm/src/include/custom_msgs_comm)模块中定义的消息类型：
- AICoordinateData.msg：AI坐标数据消息
- CalibrationCommand.msg：标定指令消息
- CalibrationResponse.msg：标定响应消息
- LaserScanData.msg：激光扫描数据消息
- PointCloudData.msg：点云数据消息
- ProjectCommand.msg：项目控制指令消息
- ProjectStatus.msg：项目状态消息

## 服务类型

本项目使用[custom_msgs_comm](file:///192.168.10.61/testCode/project_udp/src/custom_msgs_comm/src/include/custom_msgs_comm)模块中定义的服务类型：
- ProjectControl.srv：项目控制服务

## 配置文件

系统配置文件位于`sys_config`目录下：
- comm_network.yaml：UDP网络配置参数

## 构建和运行

```bash
# 构建项目
colcon build --packages-select udp_comm_server udp_comm_client

# 运行UDP服务端
ros2 launch udp_comm_server udp_server.launch.py

# 运行UDP客户端
ros2 launch udp_comm_client udp_client.launch.py
```

## 实现原理

1. 通过UDP协议实现跨主机通信
2. 使用统一的消息接口确保数据一致性
3. 实现心跳机制保证连接有效性
4. 支持数据重传机制确保可靠性
