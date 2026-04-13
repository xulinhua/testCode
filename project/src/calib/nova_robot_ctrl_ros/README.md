# nova_robot_ctrl_ros

ROS Humble封装器项目，用于控制越疆Nova机械臂，与hand_eye_calib_ros项目配合完成手眼标定任务。

## 项目概述

`nova_robot_ctrl_ros` 是一个ROS 2节点，它封装了`nova_robot_ctrl`库的功能，提供了ROS接口来控制越疆Nova机械臂。该节点与`hand_eye_calib_ros`项目协同工作，实现机械臂的手眼标定功能。

## 项目架构

```
nova_robot_ctrl_ros/
├── CMakeLists.txt                    # 构建配置文件
├── package.xml                       # 包描述文件
├── config/                          # 配置文件目录
│   └── nova_robot_ctrl_ros.yaml     # 参数配置文件
├── include/nova_robot_ctrl_ros/     # 头文件目录
│   └── nova_robot_ctrl_node.hpp     # 节点头文件
├── launch/                          # 启动文件目录
│   └── nova_robot_ctrl.launch.py    # 启动文件
├── src/                             # 源代码目录
│   ├── main.cpp                     # 主函数文件
│   └── nova_robot_ctrl_node.cpp     # 节点实现文件
└── README.md                        # 项目说明文档
```

## 功能特性

1. **机械臂控制接口**：
   - 连接和使能机械臂
   - 移动机械臂到指定位置
   - 移动机械臂到标准位置
   - 获取机械臂当前位姿

2. **ROS通信接口**：
   - 订阅目标位姿话题（`/target_pose`）
   - 发布当前位姿话题（`/right_arm_cartesian_pose`）
   - 发布机械臂运行状态话题（`/robot_run_state`）
   - 提供机械臂控制服务（`/robot_control`）

3. **与hand_eye_calib_ros协作**：
   - 接收hand_eye_calib_ros发送的标定点坐标
   - 移动机械臂到指定标定点
   - 移动到位后发布机械臂状态和坐标
   - 支持完整的标定流程控制

## 项目结构详解

### nova_robot_ctrl_node.hpp

这是NovaRobotCtrlNode类的声明文件，定义了ROS节点的接口和内部状态管理。

主要组件包括：
- ROS订阅器：接收目标位姿消息
- ROS发布器：发布当前位姿和机械臂状态
- ROS服务服务器：提供机械臂控制服务
- 定时器：定期发布机械臂状态
- nova_robot_ctrl对象：底层机械臂控制接口

### nova_robot_ctrl_node.cpp

这是NovaRobotCtrlNode类的实现文件，包含了所有的业务逻辑。

主要功能包括：
- 参数初始化和配置加载
- ROS通信接口的初始化
- 机械臂连接和使能
- 位姿获取和移动控制
- 与hand_eye_calib_ros的交互逻辑

### 配置文件

配置文件`nova_robot_ctrl_ros.yaml`定义了节点运行时的参数：
- 机械臂IP地址
- ROS话题和服务名称
- 定时器周期

### 启动文件

启动文件`nova_robot_ctrl.launch.py`提供了便捷的节点启动方式，自动加载配置文件。

## 运行方法

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- 已正确安装和配置的ROS工作空间
- 越疆Nova机械臂

### 构建项目

在ROS工作空间根目录下执行以下命令：

```bash
# 进入工作空间根目录
cd /path/to/your/ros/workspace

# 构建项目
colcon build --packages-select nova_robot_ctrl_ros

# 设置环境变量
source install/setup.bash
```

### 运行节点

#### 方法一：使用ros2 run命令

```bash
ros2 run nova_robot_ctrl_ros nova_robot_ctrl_node
```

#### 方法二：使用launch文件

```bash
ros2 launch nova_robot_ctrl_ros nova_robot_ctrl.launch.py
```

### 配置参数

可以通过修改`config/nova_robot_ctrl_ros.yaml`文件来配置参数，也可以在启动时通过命令行参数覆盖：

```bash
ros2 run nova_robot_ctrl_ros nova_robot_ctrl_node --ros-args -p robot_ip:=192.168.1.100
```

## 与hand_eye_calib_ros的协作流程

1. **初始化阶段**：
   - nova_robot_ctrl_ros启动并连接机械臂
   - 机械臂使能并移动到标准位置
   - 发布机械臂准备就绪状态

2. **标定启动阶段**：
   - hand_eye_calib_ros调用nova_robot_ctrl_ros的服务通知开始标定
   - nova_robot_ctrl_ros确认收到请求并准备移动

3. **标定点移动阶段**：
   - hand_eye_calib_ros发布第一个标定点坐标到`/target_pose`
   - nova_robot_ctrl_ros接收坐标并控制机械臂移动
   - 机械臂移动到位后，发布当前位姿和准备就绪状态

4. **循环执行**：
   - 重复步骤3，直到所有标定点处理完毕

5. **标定结束**：
   - hand_eye_calib_ros完成标定计算
   - nova_robot_ctrl_ros回到待机状态

## 接口说明

### 订阅的话题

- `/target_pose` (geometry_msgs/msg/PoseStamped)
  - 接收目标位姿信息
  - 由hand_eye_calib_ros发布

### 发布的话题

- `/right_arm_cartesian_pose` (geometry_msgs/msg/PoseStamped)
  - 发布机械臂当前位姿
  - 供hand_eye_calib_ros订阅

- `/robot_run_state` (std_msgs/msg/Bool)
  - 发布机械臂运行状态
  - true表示准备就绪，false表示正在移动
  - 供hand_eye_calib_ros订阅

### 提供的服务

- `/robot_control` (std_srvs/srv/Trigger)
  - 机械臂控制服务
  - 由hand_eye_calib_ros调用以启动标定流程

## 注意事项

1. 确保机械臂电源已开启并与网络连接正常
2. 确保ROS环境已正确设置
3. 在运行前检查配置文件中的机械臂IP地址是否正确
4. 机械臂周围应保持安全距离，避免碰撞
5. 如遇紧急情况，请立即按下机械臂急停按钮

## 故障排除

1. **无法连接机械臂**：
   - 检查机械臂IP地址配置
   - 确认网络连接正常
   - 检查防火墙设置

2. **机械臂无法使能**：
   - 检查机械臂是否处于正常状态
   - 确认急停按钮未被按下

3. **ROS节点无法启动**：
   - 检查ROS环境变量是否正确设置
   - 确认依赖包是否已正确安装

4. **机械臂移动异常**：
   - 检查目标坐标是否在机械臂工作范围内
   - 查看日志信息定位问题

## 日志输出

项目使用log_system进行日志记录，日志级别包括：
- DEBUG：调试信息
- INFO：一般信息
- WARN：警告信息
- ERROR：错误信息

可通过配置log_config.yaml文件调整日志输出级别和格式。