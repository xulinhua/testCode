# DDS通信测试指南

## 概述

本文档详细说明DDS通信项目的三个测试脚本的使用方法，包括功能测试、数据发布测试和机器人场景测试。

## 测试文件说明

### 1. test_dds_client.py - 基础客户端功能测试
**文件位置**: `src/dds_comm/src/test/test_dds_client.py`

**功能**: 测试DDS客户端的基本通信功能，包括命令发送、状态接收、项目生命周期管理等。

**测试内容**:
- 项目启动/停止/状态查询命令
- 手眼标定、相机、点云转激光、目标检测等项目测试
- 定时命令发送和状态响应验证
- JSON消息格式解析测试

**使用方法**:
```bash
# 在x86客户端设备上执行
ros2 run dds_comm test_dds_client.py

# 或直接运行Python脚本
python3 src/dds_comm/src/test/test_dds_client.py
```

**预期输出**:
```
[INFO] DDS客户端测试节点启动
[INFO] 发送命令: 项目=hand_eye_calib, 命令=start
[INFO] 接收到状态响应: 项目=hand_eye_calib, 状态=running
[INFO] 发送命令: 项目=hand_eye_calib, 命令=status
[INFO] 所有项目测试完成
```

### 2. test_data_publisher.cpp - 数据发布功能测试
**文件位置**: `src/dds_comm/src/test/test_data_publisher.cpp`

**功能**: 测试DataPublisher类的数据发布功能，包括多种数据类型发布、频率控制、统计信息等。

**测试内容**:
- 字符串数据发布测试
- JSON数据发布测试  
- 二进制数据发布测试
- 发布者创建和管理测试
- 发布统计信息获取
- 发布频率控制测试

**编译和运行**:
```bash
# 编译测试程序（需要先构建DDS通信包）
cd ~/ros2_ws
colcon build --packages-select dds_comm

# 运行测试程序
./build/dds_comm/test_data_publisher

# 或使用ROS2运行
ros2 run dds_comm test_data_publisher
```

**预期输出**:
```
=== DataPublisher功能测试 ===
✓ 字符串数据发布者创建成功
✓ 字符串数据发布成功
✓ JSON数据发布成功
✓ 二进制数据发布成功
✓ 发布统计获取成功 - 总发布数: 1
✓ 发布者存在性检查成功
✓ 发布状态: active
=== 测试完成 ===
```

### 3. robot_dds_test.py - 机器人分布式通讯测试
**文件位置**: `src/dds_comm/src/test/robot_dds_test.py`

**功能**: 综合测试DDS通信在机器人分布式系统中的应用效果，包括性能测量和场景验证。

**测试内容**:
- 运动控制命令测试（关节运动、笛卡尔运动、夹爪控制）
- 传感器数据测试（相机、激光雷达、IMU数据采集）
- 导航控制测试（路径规划、避障、目标导航）
- 系统监控测试（健康检查、状态报告、紧急停止）
- 通信延迟和吞吐量性能测量
- 网络稳定性验证

**使用方法**:
```bash
# 在x86客户端设备上执行
python3 src/dds_comm/src/test/robot_dds_test.py
```

**预期输出**:
```
[INFO] 机器人分布式DDS通信测试节点启动
[INFO] === 测试基本通信功能 ===
[INFO] 发送机器人命令: connection_test - ping
[SUCCESS] ✓ 基本通信测试通过
[INFO] === 测试机器人分布式场景 ===
[INFO] 测试场景: 运动控制
[INFO] 发送机器人命令: 运动控制 - move_joint
[INFO] 通信延迟: 23.45ms
[INFO] 吞吐量: 150.2 消息/秒
[SUCCESS] ✓ 测试成功率: 95.0%
```

## 运行环境要求

- **操作系统**: Ubuntu 22.04 + ROS Humble
- **硬件架构**: 
  - x86主板（客户端）- 运行测试脚本
  - Jetson主板（服务端）- 运行DDS服务端
- **网络**: 两台设备在同一网络，能够互相ping通
- **软件依赖**: 
  - ROS2 Humble
  - Python 3.8+
  - rclpy Python包

## 完整测试流程

### 步骤1：环境准备
```bash
# 在两台设备上分别构建DDS通信包
cd ~/ros2_ws
colcon build --packages-select dds_comm
source install/setup.bash
```

### 步骤2：启动服务端（Jetson设备）
```bash
# 在Jetson设备上启动DDS服务端
ros2 run dds_comm dds_communication_node --server
```

### 步骤3：运行测试（x86设备）

#### 选项1：运行基础功能测试
```bash
ros2 run dds_comm test_dds_client.py
```

#### 选项2：运行数据发布测试
```bash
ros2 run dds_comm test_data_publisher
```

#### 选项3：运行机器人分布式通讯测试
```bash
python3 src/dds_comm/src/test/robot_dds_test.py
```

#### 选项4：运行完整测试套件（按顺序）
```bash
# 1. 基础功能测试
ros2 run dds_comm test_dds_client.py

# 2. 数据发布测试  
ros2 run dds_comm test_data_publisher

# 3. 机器人分布式通讯测试
python3 src/dds_comm/src/test/robot_dds_test.py
```

## 性能基准要求

| 测试项目 | 优秀指标 | 良好指标 | 需优化 |
|---------|----------|----------|--------|
| 通信延迟 | < 20ms | < 50ms | > 100ms |
| 命令吞吐量 | > 200 msg/s | > 100 msg/s | < 50 msg/s |
| 数据发布吞吐量 | > 5 MB/s | > 2 MB/s | < 1 MB/s |
| 系统可用性 | > 99.9% | > 99% | < 95% |

## 问题排查指南

### 常见问题1：测试脚本无法运行
**症状**: `ModuleNotFoundError: No module named 'rclpy'`
**解决**: 
```bash
# 安装rclpy Python包
pip3 install rclpy

# 或使用ROS2提供的Python环境
source /opt/ros/humble/setup.bash
```

### 常见问题2：服务端连接失败
**症状**: 测试脚本无法连接到Jetson服务端
**排查**:
```bash
# 检查网络连接
ping <jetson_ip_address>

# 检查ROS2节点发现
ros2 node list

# 检查DDS服务端是否运行
ros2 node info /dds_server
```

### 常见问题3：命令无响应
**症状**: 发送命令后没有状态返回
**排查**:
```bash
# 监听状态话题
ros2 topic echo /dds/server/status

# 检查命令话题
ros2 topic echo /dds/client/command_out

# 查看服务端日志
ros2 run dds_comm dds_communication_node --server --log-level debug
```

### 常见问题4：性能不达标
**症状**: 延迟过高或吞吐量低
**优化建议**:
- 检查网络带宽和延迟
- 优化消息序列化格式
- 调整QoS配置（可靠性vs实时性）
- 减少消息大小和频率

## 文件结构
```
src/dds_comm/src/test/
├── test_dds_client.py          # 基础客户端功能测试
├── test_data_publisher.cpp     # C++数据发布功能测试  
└── robot_dds_test.py           # 机器人分布式通讯测试
```

**文档版本**: v2.0  
**最后更新**: 2025年10月11日