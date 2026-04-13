# Common Interfaces

通用消息和服务接口模块，用于在不同项目间共享数据结构定义。

## 功能概述

该模块提供了一套标准化的消息和服务接口定义，旨在消除不同项目间重复定义相同数据结构的问题。通过将通用的数据结构集中管理，确保了项目间数据交互的一致性和可维护性。

本模块专为以下场景设计：
- x86主板（Ubuntu22.04 + ROS Humble）与Jetson主板（Ubuntu22.04 + ROS Humble）之间的命令和数据传输
- AI识别目标坐标的统一数据格式
- 手眼标定等工业应用场景的数据交换
- 项目间通用控制指令和状态反馈

## 消息类型 (Messages)

### AICoordinateData.msg
AI识别坐标数据消息，用于传输目标检测结果：
- object_class: 物体类别
- position: 位置坐标 [x, y, z]
- orientation: 方向四元数 [x, y, z, w]
- confidence: 置信度
- frame_id: 坐标系标识
- stamp: 时间戳

### CalibrationCommand.msg
标定指令消息，用于手眼标定等场景：
- command_type: 指令类型 (CALIB_START, CALIB_NEXT_POINT, CALIB_FINISHED)
- robot_pose: 机械手位姿 [x, y, z, roll, pitch, yaw]

### CalibrationResponse.msg
标定响应消息：
- success: 标定是否成功
- message: 响应消息
- calibration_matrix: 4x4标定矩阵（展平）
- timestamp: 响应时间戳

### LaserScanData.msg
激光扫描数据消息，兼容sensor_msgs/LaserScan格式。

### PointCloudData.msg
点云数据消息，兼容sensor_msgs/PointCloud2格式。

### ProjectCommand.msg
项目控制指令消息：
- command_type: 指令类型 (PROJECT_START, PROJECT_STOP, PROJECT_STATUS)
- project_name: 项目名称

### ProjectStatus.msg
项目状态消息：
- status: 状态码 (STATUS_STOPPED, STATUS_RUNNING, STATUS_STARTING, STATUS_STOPPING, STATUS_ERROR)
- project_name: 项目名称
- message: 状态信息

### Command.msg
DDS命令消息格式：
- command_type: 命令类型 (START, STOP, STATUS, PAUSE, RESUME)
- project_type: 项目类型 (HAND_EYE_CALIB, CAMERA, PCL2LASERSCAN, YOLO_DET)
- project_name: 项目名称
- data: 附加数据（JSON格式）

### RobotPose.msg
机器人位姿消息格式：
- position: 位置 (geometry_msgs/Point)
- orientation: 方向 (geometry_msgs/Quaternion)
- frame_id: 坐标系标识
- timestamp: 时间戳

## 服务类型 (Services)

### ProjectControl.srv
项目控制服务：
- Request:
  - project_name: 项目名称
  - command: 控制命令 (START, STOP, RESTART, STATUS)
- Response:
  - success: 操作是否成功
  - message: 响应消息
  - status: 当前项目状态

## 使用方法

在您的项目CMakeLists.txt中添加依赖：

```cmake
find_package(custom_msgs_comm REQUIRED)
```

在您的package.xml中添加依赖：

```xml
<depend>custom_msgs_comm</depend>
```

在代码中包含需要的消息类型：

```cpp
#include "custom_msgs_comm/msg/ai_coordinate_data.hpp"
```

或在Python中：

```python
from custom_msgs_comm.msg import AICoordinateData
```

## 架构设计

```
graph TD
    subgraph "通用接口模块"
        A[AICoordinateData]
        B[ProjectCommand]
        C[ProjectStatus]
        D[CalibrationCommand]
        E[CalibrationResponse]
        F[Command]
        G[RobotPose]
        H[ProjectControl]
    end
    
    subgraph "使用项目"
        I[DDS通信项目]
        J[UDP通信项目]
        K[YOLO检测项目]
        L[手眼标定项目]
    end
    
    I --- A
    J --- A
    K --- A
    L --- D
    L --- E
```

## 配置文件

系统配置文件位于`sys_config/network_config/comm_network.yaml`，包含网络通信参数等系统级配置。

## 构建要求

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.8+

## 维护

该模块作为基础数据结构定义模块，应保持向后兼容性。新增消息类型时需要评估对现有项目的影响。

## 版本历史

v1.0.0 (2024-01-01)
- 初始版本发布
- 包含AI识别坐标、项目控制、标定相关消息和服务

# 重新构建
colcon build --packages-select custom_msgs_comm