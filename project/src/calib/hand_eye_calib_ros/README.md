## 项目概述

本项目是 hand_eye_calib 库的 ROS 封装器，专门用于在 ROS Humble 环境下运行手眼标定功能。该封装器实现了与 marker_detect_ros 和机械臂项目的通信接口，控制标定流程并调用 hand_eye_calib 库的核心功能。

为简化使用，项目支持两种运行模式：
1. **手动模式**：启动节点后，通过调用服务手动启动标定流程
2. **自动模式**：节点启动后自动开始标定流程

## 功能特性

1. **数据采集功能**：
   - 订阅 marker_detect_ros 发布的 Aruco 码识别结果
   - 订阅机械臂项目发布的机械臂位姿数据

2. **标定控制功能**：
   - 手动启动标定流程（通过服务调用）
   - 自动启动标定流程（配置参数控制）
   - 控制机械臂按预定轨迹移动
   - 自动采集标定点数据

3. **标定计算功能**：
   - 调用 hand_eye_calib 库进行标定矩阵计算
   - 保存标定结果
   - 分析标定质量

## 项目架构

```
graph TD
    A[marker_detect_ros] --> B[hand_eye_calib_ros]
    C[机械臂项目] --> B
    B --> C
    B --> D[hand_eye_calib库]
```

## 启动项目

### 环境准备

在启动项目之前，请确保以下依赖已经正确安装和配置：

1. ROS 2 Humble 环境
2. hand_eye_calib 库
3. marker_detect_ros 包
4. 机械臂控制项目
5. custom_msgs_comm 包
6. log_system 包

### 构建项目

1. 确保在 ROS 2 工作空间根目录中
2. 运行构建脚本：
   ```bash
   ./src/calib/hand_eye_calib_ros/build.sh
   ```
   或者使用 colcon 构建：
   ```bash
   colcon build --packages-select hand_eye_calib_ros
   ```
3. 构建成功后，设置环境变量：
   ```bash
   source install/setup.bash
   ```

### 启动节点

有两种方式启动 hand_eye_calib_ros 节点：

#### 方法一：使用 launch 文件启动（推荐）

```bash
ros2 launch hand_eye_calib_ros hand_eye_calib.launch.py
```

可以通过参数控制是否自动启动标定流程：
```bash
# 手动模式（默认）
ros2 launch hand_eye_calib_ros hand_eye_calib.launch.py

# 自动模式
ros2 launch hand_eye_calib_ros hand_eye_calib.launch.py auto_start:=true
```

#### 方法二：直接运行节点

```bash
ros2 run hand_eye_calib_ros hand_eye_calib_node
```

同样可以通过参数控制自动启动：
```bash
# 手动模式（默认）
ros2 run hand_eye_calib_ros hand_eye_calib_node

# 自动模式
ros2 run hand_eye_calib_ros hand_eye_calib_node --ros-args -p auto_start_calibration:=true
```

## 话题接口

### 订阅的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/aruco_detection/result` | `custom_msgs/msg/AICoordinateData` | 标定marker识别结果 |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | 机械臂当前位姿 |
| `/robot_status` | `std_msgs/msg/String` | 机械臂状态信息 |

### 发布的话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/robot_target` | `geometry_msgs/msg/PoseStamped` | 机械臂目标位姿 |
| `/calib_control` | `std_msgs/msg/String` | 标定控制命令 |
| `/calib_status` | `std_msgs/msg/String` | 标定状态信息 |

## 服务接口

### 提供的服务

| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `/start_calibration` | `std_srvs/srv/Trigger` | 启动手动标定流程 |

### 调用的服务

| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `/robot_control` | `custom_msgs/srv/ProjectControl` | 控制机械臂项目 |
| `/aruco_detection/get_result` | `custom_msgs/srv/GetArucoDetection` | 获取标定marker检测结果 |

## 参数配置

| 参数名称 | 类型 | 默认值 | 描述 |
|---------|------|--------|------|
| `config_file` | string | `"config/calib_config.yaml"` | hand_eye_calib配置文件路径 |
| `aruco_marker_topic` | string | `"/aruco_detection/result"` | 标定marker话题名称 |
| `robot_pose_topic` | string | `"/robot_pose"` | 机械臂位姿话题名称 |
| `robot_run_state_topic` | string | `"/robot_run_status"` | 机械臂运行状态话题名称 |
| `robot_target_pos_topic` | string | `"/robot_target_pos"` | 机械臂目标位姿话题名称 |
| `robot_control_service` | string | `"/robot_control"` | 机械臂控制服务名称 |
| `detect_res_service` | string | `"/aruco_detection/get_result"` | Aruco检测服务名称 |
| `start_calib_service` | string | `"/start_calibration"` | 启动标定服务名称 |
| `calib_src_img_service` | string | `"/aruco_detection/get_aruco_src_img"` | 标定源图像服务名称 |
| `timer_period_ms` | int | `100` | 定时器周期（毫秒） |

## 标定流程

1. **启动节点**：
   - 通过 `ros2 launch` 或 `ros2 run` 启动 hand_eye_calib_ros 节点
   - 节点开始监听话题和服务

2. **启动标定**：
   - 手动模式：用户通过调用`/start_calibration`服务启动标定流程
   - 自动模式：节点自动启动标定流程
   - 节点通过`/robot_control`服务向机械臂项目发送准备标定指令

3. 机械臂准备：
   - 机械臂项目接收到启动命令后进行准备
   - 准备完成后发布"ready"状态到`/robot_status`话题

4. 移动到标定点：
   - hand_eye_calib_ros接收到"ready"状态后，从hand_eye_calib库获取第一个标定点位姿
   - 发布目标位姿到`/robot_target`话题给机械臂项目
   - 发布暂停相机采图命令到`/calib_control`话题
   - 机械臂移动到位后发布"moved"状态到`/robot_status`话题

5. 数据采集：
   - hand_eye_calib_ros接收到"moved"状态后，发布恢复相机采图命令到`/calib_control`话题
   - 通过`/aruco_detection/get_result`服务请求标定marker检测项目获取标记识别结果
   - 采集当前标定点的机械臂位姿和标定marker位置数据

6. 异常处理：
   - 如果机械臂位姿数据无效或标定marker识别失败，则舍弃该数据点
   - 直接发送下一个标定点的目标位姿给机械臂项目

7. 重复步骤3-6：
   - 直到完成所有标定点的数据采集

8. 标定计算：
   - 如果成功采集的标定点数量满足配置要求，则调用hand_eye_calib库进行标定计算
   - 保存标定结果并分析质量
   - 发送标定结束命令到`/calib_control`话题

## 依赖关系

- ROS 2 Humble
- hand_eye_calib 库
- marker_detect_ros 包
- 机械臂控制项目
- std_srvs 包
- custom_msgs 包

## 构建问题解决

如果遇到构建问题，请检查以下几点：

1. 确保所有依赖包都已正确安装
2. 确保hand_eye_calib库已正确构建并安装
3. 确保custom_msgs_comm包已正确构建并安装
4. 确保log_system包已正确构建并安装