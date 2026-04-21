# Nova Robot Tools

ROS2 机器人辅助工具集 - 参考 calib_sim 架构实现

## 功能

- 系统监控（CPU/GPU/内存）
- ArUco 码生成器（开发中）
- 坐标转换器（开发中）
- 手眼标定（开发中）
- 运动学求解器（开发中）

## 编译

```bash
cd /home/hs/testCode/simulation
colcon build --packages-select nova_robot_tools
source install/setup.bash
```

## 运行

```bash
# 方法 1: ros2 run
ros2 run nova_robot_tools nova_robot_tools_ui

# 方法 2: ros2 launch
ros2 launch nova_robot_tools nova_robot_tools.launch.py
```

## 架构说明

本项目完全参考 calib_sim 的架构模式：

- **不使用 Q_OBJECT 和 AUTOMOC**
- **所有 Qt UI 代码在 .cpp 文件中实现**
- **使用 QObject::connect 的 lambda 语法**
- **CUDA 12.4 路径配置匹配本地 OpenCV 编译环境**

## 项目结构

```
nova_robot_tools/
├── CMakeLists.txt
├── package.xml
├── include/nova_robot_tools/
│   └── nova_robot_tools_ui.hpp
├── src/
│   ├── nova_robot_tools_ui.cpp          # Qt UI 实现
│   └── nova_robot_tools_ui_node.cpp     # ROS2 节点入口
├── config/
└── launch/
```

## 开发说明

添加新功能时：
1. 在 `nova_robot_tools_ui.cpp` 中实现 UI
2. 不使用 Q_OBJECT 宏
3. 使用 QObject::connect + lambda 连接信号槽
4. 参考 calib_sim 项目的实现模式
