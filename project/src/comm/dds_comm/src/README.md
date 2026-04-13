# DDS通信项目

## 项目概述

DDS通信项目是一个基于Ubuntu 22.04 + ROS Humble框架的分布式数据服务系统，专为机器人分布式系统设计。项目采用现代C++开发，提供高性能、高可靠性的通信服务，支持多项目生命周期管理和实时数据发布。

### 核心特性
- **统一通信框架**: 基于DDS标准实现跨平台、跨语言的通信能力
- **项目生命周期管理**: 完整的项目启动、停止、重启、状态监控功能
- **实时数据发布**: 支持多种数据类型的高效发布和订阅
- **错误恢复机制**: 自动错误检测和恢复，确保系统高可用性
- **模块化设计**: 清晰的接口定义和松耦合架构设计

## 系统架构

### 整体架构图
```
┌─────────────────┐    ┌─────────────────┐
│   客户端设备     │    │   服务端设备     │
│   (x86主板)     │    │   (Jetson主板)   │
│                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │DDS客户端节点│◄─────►│ │DDS服务端节点│ │
│ │             │ │    │ │             │ │
│ │- 命令发送   │ │    │ │- 命令处理   │ │
│ │- 状态接收   │ │    │ │- 状态发布   │ │
│ │- 项目控制   │ │    │ │- 项目管理   │ │
│ └─────────────┘ │    │ └─────────────┘ │
│                 │    │                 │
│ ┌─────────────┐ │    │ ┌─────────────┐ │
│ │项目控制工具 │ │    │ │项目生命周期 │ │
│ │             │ │    │ │管理器       │ │
│ └─────────────┘ │    │ └─────────────┘ │
└─────────────────┘    └─────────────────┘
```

### 核心组件

#### 1. DDS服务核心 (DdsService)
- **位置**: `src/include/dds_comm/dds_service.hpp`
- **功能**: 提供DDS通信的核心服务，包括命令发布、状态订阅、项目管理等
- **设计模式**: 观察者模式、状态模式、工厂模式
- **线程安全**: 使用std::mutex确保多线程环境安全

#### 2. 项目管理器 (ProjectManager)
- **位置**: `src/include/dds_comm/project_manager.hpp`
- **功能**: 管理项目生命周期，包括注册、启动、停止、状态监控
- **状态管理**: 完整的状态机实现（UNREGISTERED→REGISTERED→STARTING→RUNNING→STOPPING→ERROR→RECOVERING）

#### 3. 命令管理器 (CommandManager)
- **位置**: `src/include/dds_comm/command_manager.hpp`
- **功能**: 处理各种类型的命令，支持命令注册和回调机制
- **命令类型**: START, STOP, RESTART, STATUS, CONFIGURE, PAUSE, RESUME, EMERGENCY_STOP

#### 4. 数据发布器 (DataPublisher)
- **位置**: `src/include/dds_comm/data_publisher.hpp`
- **功能**: 提供高效的数据发布能力，支持多种数据类型
- **配置灵活**: 可配置发布频率、QoS策略等参数

## 功能特性

### 1. 通信能力
- **双向通信**: 支持命令下发和状态上报的双向通信
- **多播支持**: 可配置多播组实现一对多通信
- **QoS策略**: 支持可靠性、持久性、实时性等QoS配置
- **网络容错**: 自动重连和错误恢复机制

### 2. 项目管理
- **动态注册**: 支持运行时项目注册和注销
- **状态监控**: 实时监控项目运行状态和健康状态
- **自动恢复**: 支持项目异常时的自动恢复机制
- **批量操作**: 支持批量项目的启动、停止等操作

### 3. 数据发布
- **多种格式**: 支持字符串、JSON、二进制等多种数据格式
- **频率控制**: 可配置发布频率和间隔时间
- **统计信息**: 提供发布统计和性能监控
- **内存管理**: 高效的内存使用和资源回收

### 4. 错误处理
- **异常捕获**: 全面的异常捕获和处理机制
- **错误日志**: 详细的错误日志记录和分析
- **恢复策略**: 多级恢复策略确保系统稳定性
- **健康检查**: 定期健康检查和状态报告

## 技术实现

### 编程语言和框架
- **语言**: C++17标准，面向对象设计
- **框架**: ROS2 Humble Hawksbill
- **构建系统**: CMake + colcon
- **日志系统**: 集成log_system进行统一日志管理

### 设计原则
- **单一职责**: 每个类有明确的单一职责
- **开闭原则**: 通过接口抽象支持扩展
- **依赖倒置**: 高层模块不依赖低层模块
- **接口隔离**: 细粒度的接口设计

### 性能优化
- **内存池**: 使用对象池减少内存分配开销
- **异步处理**: 异步消息处理提高吞吐量
- **缓存优化**: 合理使用缓存提高访问效率
- **锁优化**: 细粒度锁减少竞争

## 消息格式

### 命令消息格式
``cpp
struct CommandMessage {
    CommandType command_type;    // 命令类型
    ProjectType project_type;    // 项目类型  
    int64_t timestamp;           // 时间戳（毫秒）
    std::string source;          // 命令来源
    std::string data;            // 附加数据（JSON格式）
};
```

### 状态消息格式
``cpp
struct StatusMessage {
    ProjectType project_type;    // 项目类型
    std::string status;          // 状态描述
    bool is_running;            // 是否运行中
    int uptime;                 // 运行时间（秒）
    std::string last_error;     // 最后错误信息
    int64_t timestamp;          // 时间戳（毫秒）
};
```

## 支持的平台和项目

### 硬件平台
- **服务端**: NVIDIA Jetson系列（ARM架构）
- **客户端**: x86架构主板
- **网络**: 千兆以太网，支持无线网络

### 支持的项目类型
```cpp
enum class ProjectType {
    DDS_COMMUNICATION,  // DDS通信项目
    HAND_EYE_CALIB,     // 手眼标定项目
    CAMERA,             // 相机项目
    PCL2LASERSCAN,      // 点云转激光扫描项目
    YOLO_DET,           // 目标检测项目
    NAVIGATION,         // 导航项目
    SLAM,               // SLAM项目
    MOTION_CONTROL,     // 运动控制项目
    SENSOR_FUSION       // 传感器融合项目
};
```

## 用户使用指南

### 系统环境要求

#### 硬件平台
- **x86主板（客户端）**: Ubuntu 22.04 + ROS Humble
- **Jetson主板（服务端）**: Ubuntu 22.04 + ROS Humble
- **网络**: 两台设备在同一局域网，能够互相ping通

#### 软件依赖
```bash
# 在两台设备上都需要安装
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-pip
pip3 install rclpy
```

### 快速开始

#### 步骤1：项目构建
在两台设备上分别构建DDS通信包：

```bash
# 进入项目工作空间
cd ~/testCode/project

# 构建DDS通信包
colcon build --packages-select dds_comm

# 加载环境
source install/setup.bash
```

#### 步骤2：启动服务端（Jetson主板）
在Jetson主板上启动DDS服务端：

```bash
# 使用统一启动文件（推荐）
ros2 launch dds_comm dds_service.launch.py mode:=server

# 或直接运行可执行文件
ros2 run dds_comm dds_service --server
```

#### 步骤3：启动客户端（x86主板）
在x86主板上启动DDS客户端：

```bash
# 使用统一启动文件（推荐）
ros2 launch dds_comm dds_service.launch.py mode:=client

# 或直接运行可执行文件
ros2 run dds_comm dds_service --client
```

### 命令发送示例

#### 1. 使用项目控制节点发送命令
```bash
# 启动项目（例如：相机项目）
ros2 run dds_comm project_control_node --start camera

# 停止项目
ros2 run dds_comm project_control_node --stop camera

# 查询项目状态
ros2 run dds_comm project_control_node --status camera

# 重启项目
ros2 run dds_comm project_control_node --restart camera

# 列出所有支持的项目
ros2 run dds_comm project_control_node --list
```

#### 2. 使用ROS2话题直接发送命令
```bash
# 监听命令话题（在Jetson端查看接收的命令）
ros2 topic echo /dds/command

# 监听状态话题（在x86端查看接收的状态）
ros2 topic echo /dds/status

# 手动发送命令（示例）
ros2 topic pub /dds/command dds_comm/msg/Command '
command_type: 1  # START命令
project_type: 2  # CAMERA项目
project_name: "camera_calibration"
data: "{\"config\": \"default\"}"
'
```

#### 3. 发送配置命令（JSON数据）
```bash
# 发送手眼标定配置
ros2 run dds_comm project_control_node --configure hand_eye_calib '{
  "calibration_type": "eye_to_hand",
  "camera_matrix": [1000, 0, 320, 0, 1000, 240, 0, 0, 1],
  "distortion_coeffs": [0.1, -0.2, 0.001, 0.002, 0]
}'

# 发送运动控制命令
ros2 run dds_comm project_control_node --configure motion_control '{
  "target_pose": {
    "x": 1.5,
    "y": 2.3, 
    "z": 0.8,
    "roll": 0.1,
    "pitch": 0.2,
    "yaw": 0.3
  },
  "velocity": 0.5,
  "acceleration": 0.2
}'
```

### 数据发布和订阅

#### 1. 发布传感器数据（Jetson端）
```bash
# 发布相机图像数据（示例话题）
ros2 topic pub /dds/data/camera sensor_msgs/msg/Image '
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "camera_frame"
height: 480
width: 640
encoding: "rgb8"
is_bigendian: 0
step: 1920
data: [255, 255, 255, ...]  # 图像数据
'

# 发布激光雷达数据
ros2 topic pub /dds/data/lidar sensor_msgs/msg/LaserScan '
header:
  frame_id: "laser_frame"
angle_min: -3.14159
angle_max: 3.14159
angle_increment: 0.0174533
range_min: 0.1
range_max: 30.0
ranges: [1.0, 1.1, 1.2, ...]  # 距离数据
'
```

#### 2. 订阅数据（x86端）
```bash
# 订阅相机数据
ros2 topic echo /dds/data/camera

# 订阅激光雷达数据  
ros2 topic echo /dds/data/lidar

# 订阅所有DDS数据话题
ros2 topic list | grep dds
```

### 高级使用场景

#### 1. 批量命令发送
```bash
# 使用脚本批量发送命令
#!/bin/bash
PROJECTS=("camera" "pcl2laser" "detection" "slam")

for project in "${PROJECTS[@]}"; do
    echo "启动项目: $project"
    ros2 run dds_comm project_control_node --start $project
    sleep 2
done
```

#### 2. 定时状态监控
```bash
# 定时查询所有项目状态
while true; do
    echo "=== 项目状态监控 $(date) ==="
    ros2 run dds_comm project_control_node --status camera
    ros2 run dds_comm project_control_node --status pcl2laser
    ros2 run dds_comm project_control_node --status detection
    sleep 10
done
```

#### 3. 网络故障恢复测试
```bash
# 模拟网络中断和恢复
echo "测试网络故障恢复..."

# 断开网络连接（临时）
sudo iptables -A OUTPUT -p tcp --dport 7400 -j DROP

# 尝试发送命令（应该失败）
ros2 run dds_comm project_control_node --status camera

# 恢复网络连接
sudo iptables -D OUTPUT -p tcp --dport 7400 -j DROP

# 重试命令（应该成功）
ros2 run dds_comm project_control_node --status camera
```

## 详细代码结构

### 项目根目录结构
```
src/dds_comm/
├── CMakeLists.txt                    # CMake构建配置文件
├── package.xml                       # ROS2包元数据定义
├── README.md                         # 项目根README文档
├── src/                             # 源代码目录
│   ├── include/dds_comm/   # 公共头文件目录
│   ├── src/                         # 源文件实现目录
│   ├── msg/                         # ROS2消息定义文件
│   ├── launch/                      # ROS2启动文件
│   ├── config/                      # 运行时配置文件
│   ├── test/                        # 测试代码目录
│   └── docs/                        # 项目文档目录
└── scripts/                         # 工具脚本目录
```

### 核心头文件详解 (`src/include/dds_comm/`)

#### 1. 核心服务类
- **`dds_service.hpp`** - DDS通信服务核心类
  - 提供命令发布、状态订阅、项目管理等核心功能
  - 实现服务状态管理（INITIALIZING→RUNNING→STOPPING→STOPPED→ERROR→RECOVERING）
  - 支持批量命令处理和发布间隔控制

- **`dds_communication_node.hpp`** - 服务端主节点类（Jetson端）
  - 运行在Jetson主板，作为DDS通信服务端
  - 接收来自x86主板的命令并管理本地项目
  - 包含各项目类型的命令处理器和状态查询器

- **`dds_client_node.hpp`** - 客户端节点类（x86端）
  - 运行在x86主板，作为DDS通信客户端
  - 向Jetson主板发送命令并接收状态反馈
  - 支持定时命令发送和状态监控

#### 2. 管理组件类
- **`project_manager.hpp`** - 项目管理器类
  - 管理项目生命周期状态（UNREGISTERED→REGISTERED→STARTING→RUNNING→STOPPING→ERROR→RECOVERING）
  - 支持项目动态注册、状态监控、自动恢复
  - 提供健康检查和错误计数功能

- **`command_manager.hpp`** - 命令管理器类
  - 处理各种命令类型（START, STOP, RESTART, STATUS, CONFIGURE等）
  - 支持命令注册和回调机制
  - 提供命令执行和结果处理

- **`data_publisher.hpp`** - 数据发布器类
  - 提供高效的数据发布能力
  - 支持多种数据类型（字符串、JSON、二进制）
  - 可配置发布频率和QoS策略

#### 3. 基础类型定义
- **`command_types.hpp`** - 命令和项目类型定义
  - 定义`CommandType`枚举（START, STOP, RESTART, STATUS等）
  - 定义`ProjectType`枚举（DDS_COMMUNICATION, HAND_EYE_CALIB, CAMERA等）
  - 提供类型转换和验证功能

### 源文件实现 (`src/src/`)

#### 1. 核心服务实现
- **`dds_service.cpp`** - DDS服务核心实现
  - 集成log_system进行统一日志管理
  - 实现命令解析、状态构建、错误处理
  - 提供服务初始化、健康检查、恢复机制

- **`main.cpp`** - 程序主入口
  - 解析命令行参数和配置文件
  - 创建并启动DDS服务节点
  - 处理信号和优雅关闭

#### 2. 管理组件实现
- **`project_manager.cpp`** - 项目管理器实现
  - 实现项目状态转换和验证逻辑
  - 提供项目信息查询和统计功能
  - 实现自动恢复和清理机制

- **`command_manager.cpp`** - 命令管理器实现
  - 实现命令执行和结果处理
  - 支持命令重试和超时控制
  - 提供命令统计和性能监控

- **`data_publisher.cpp`** - 数据发布器实现
  - 实现数据序列化和发布逻辑
  - 支持发布频率控制和统计
  - 提供内存管理和资源回收

#### 3. 工具节点实现
- **`project_control_node.cpp`** - 项目控制节点
  - 提供命令行接口进行项目控制
  - 支持项目启动、停止、重启、状态查询
  - 集成信号处理和帮助信息

### 消息定义 (`src/msg/`)

#### 1. 核心消息类型
- **`Command.msg`** - 命令消息格式
  ```msg
  uint8 command_type        # 命令类型枚举
  uint8 project_type        # 项目类型枚举  
  string project_name       # 项目名称
  string data               # 附加数据（JSON格式）
  ```

- **`ProjectStatus.msg`** - 项目状态消息格式
  ```msg
  uint8 project_type        # 项目类型
  string project_name       # 项目名称
  uint8 status              # 状态：0=停止，1=运行，2=错误，3=暂停
  string status_message     # 状态信息
  uint64 timestamp         # 时间戳
  ```

- **`RobotPose.msg`** - 机器人位姿消息格式
  ```msg
  float64 x                 # X坐标
  float64 y                 # Y坐标  
  float64 z                 # Z坐标
  float64 roll              # 滚转角
  float64 pitch             # 俯仰角
  float64 yaw               # 偏航角
  ```

### 启动文件 (`src/launch/`)

#### 1. 统一启动文件
- **`dds_service.launch.py`** - DDS服务统一启动文件
  - 支持服务端和客户端模式启动
  - 通过`mode`参数控制运行模式（server/client）
  - 支持配置文件路径参数传递

### 配置文件 (`src/config/`)

#### 1. 主配置文件
- **`dds_config.yaml`** - DDS服务主配置
  - 话题名称配置（命令话题、状态话题、数据话题）
  - 发布间隔配置（命令、状态、数据发布间隔）
  - 重试和超时设置（最大重试次数、超时时间）
  - 项目特定配置（自动启动、自动恢复、健康检查）

#### 2. 日志配置
- **`log_config.yaml`** - 日志系统配置
  - 日志级别设置（DEBUG, INFO, WARN, ERROR）
  - 日志输出格式和文件路径
  - 日志轮转和归档配置

### 性能指标

#### 通信性能
- **延迟**: < 20ms（局域网环境）
- **吞吐量**: > 200 消息/秒
- **可靠性**: > 99.9% 消息送达率
- **并发**: 支持100+并发连接

#### 资源使用
- **内存**: < 100MB（服务端）
- **CPU**: < 30%（正常负载）
- **网络**: 自适应带宽使用

### 故障排除

#### 1. 连接问题排查
```bash
# 检查网络连通性
ping <jetson_ip_address>

# 检查ROS2节点发现
ros2 node list

# 检查话题通信
ros2 topic list | grep dds
ros2 topic info /dds/command
```

#### 2. 服务状态检查
```bash
# 检查DDS服务是否运行
ps aux | grep dds_service

# 查看服务日志
ros2 run dds_comm dds_service --server --log-level debug

# 检查系统资源使用
top -p $(pgrep dds_service)
```

#### 3. 常见错误解决
- **"节点未发现"**: 确保两台设备在同一ROS域中（设置ROS_DOMAIN_ID）
- **"话题不存在"**: 检查服务端是否正确启动和话题名称配置
- **"命令无响应"**: 检查网络连接和服务端日志
- **"权限拒绝"**: 确保有足够的权限访问网络端口

## 相关文档

- [DDS测试指南](docs/DDS_TEST_GUIDE.md) - 详细的功能测试流程
- [配置指南](docs/CONFIG_GUIDE.md) - 系统配置说明
- [日志系统指南](docs/LOG_SYSTEM_GUIDE.md) - 日志系统使用说明

## 版本信息

- **当前版本**: v2.0
- **ROS2版本**: Humble Hawksbill
- **C++标准**: C++17
- **最后更新**: 2025年10月11日

## 贡献指南

欢迎提交Issue和Pull Request来改进项目。请确保：
1. 代码符合项目编码规范
2. 包含相应的单元测试
3. 更新相关文档
4. 通过所有现有测试

## 许可证

本项目采用MIT许可证。详见LICENSE文件。

---

*本文档提供了DDS通信项目的整体概述、架构说明和使用指南。详细的功能测试和配置说明请参考相关文档。*
```

## 代码修改建议
```
# DDS通信服务模块

DDS通信服务模块用于在x86主板（Ubuntu22.04 + ROS Humble）和Jetson主板（Ubuntu22.04 + ROS Humble）之间传输命令和数据。

## 功能概述

该模块提供以下核心功能：
1. 项目控制：启动、停止、暂停、恢复项目
2. 状态监控：实时监控项目运行状态
3. 数据发布：发布项目产生的数据

## 项目架构

```
graph TD
    subgraph "x86主板"
        A[DDS服务端]
        B[项目控制节点]
    end
    
    subgraph "Jetson主板"
        C[DDS客户端]
    end
    
    subgraph "通用接口模块"
        D[custom_msgs_comm]
    end
    
    A --- D
    B --- D
    C --- D
```

## 依赖关系

本项目依赖以下模块：
- ROS 2 Humble环境
- custom_msgs_comm：通用消息接口模块
- log_system：日志系统模块

## 消息类型

本项目使用[custom_msgs_comm](file:///192.168.10.61/testCode/project_udp/src/custom_msgs_comm/src/include/custom_msgs_comm)模块中定义的消息类型：
- Command.msg：DDS命令消息
- ProjectStatus.msg：项目状态消息
- RobotPose.msg：机器人位姿消息

## 配置文件

系统配置文件位于`sys_config`目录下：
- comm_network.yaml：DDS网络配置参数

## 构建和运行

```bash
# 构建项目
colcon build --packages-select dds_comm

# 运行DDS服务
ros2 launch dds_comm dds_service.launch.py
```

## 实现原理

1. 通过DDS协议实现跨主机通信
2. 使用统一的消息接口确保数据一致性
3. 集成日志系统便于调试和监控
4. 支持多种项目类型的控制和监控
