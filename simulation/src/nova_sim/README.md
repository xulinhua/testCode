# Nova Sim - 机械臂Gazebo仿真项目

## 项目概述

Nova Sim是一个基于ROS2和Gazebo的机械臂仿真项目，专为Ubuntu 22.04和ROS2 Humble设计。项目使用C++17标准，提供完整的机械臂仿真环境。

## 项目结构

```
nova_sim/
├── src/
│   ├── CMakeLists.txt          # 构建配置
│   ├── package.xml            # 包描述文件
│   ├── include/nova_sim/      # 头文件目录
│   ├── src/                   # 源代码目录
│   ├── launch/                # 启动文件目录
│   └── config/                # 配置文件目录
└── README.md                  # 项目说明
```

## 环境要求

- **操作系统**: Ubuntu 22.04
- **ROS2版本**: Humble
- **C++标准**: C++17
- **仿真工具**: Gazebo

## 快速开始

### 1. 编译项目

```bash
cd /home/hs/testCode/simulation/nova_sim
colcon build --packages-select nova_sim
source install/setup.bash
```

### 2. 项目状态

当前项目为空项目，包含基本的ROS2包结构。可以根据需要添加以下功能：

- 机械臂URDF模型
- Gazebo仿真环境
- 机械臂控制节点
- 轨迹规划算法
- 视觉感知模块

## 开发指南

### 添加新功能

1. **添加头文件**到 `include/nova_sim/` 目录
2. **添加源文件**到 `src/` 目录
3. **更新CMakeLists.txt**添加编译目标
4. **添加启动文件**到 `launch/` 目录
5. **添加配置文件**到 `config/` 目录

### 依赖管理

在 `package.xml` 中添加所需的依赖包：

```xml
<depend>rclcpp</depend>
<depend>gazebo_ros</depend>
<depend>urdf</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf2_ros</depend>
```

## 许可证

MIT License

## 联系方式

如有问题请联系项目维护者。