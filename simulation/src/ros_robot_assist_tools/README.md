# ROS Robot Assist Tools

ROS2 Humble / Ubuntu 22.04 机器人视觉辅助工具（C++17 + Qt）

## 功能

- 主页面实时监控（CPU/GPU/内存/显存）
- 单目内参标定（实时/离线，UI切换 + YAML）
- 双目标定（实时/离线，UI切换 + YAML）
- 手眼标定（Eye-in-hand / Eye-to-hand，实时/离线，算法切换）
- ArUco 生成（全字典 + 图片导出 + DAE导出）
- ArUco 识别
- RPY 与四元数转换

## 编译

```bash
cd /home/hs/testCode/simulation
colcon build --packages-select ros_robot_assist_tools
source install/setup.bash
```

## 运行

```bash
ros2 run ros_robot_assist_tools ros_robot_assist_tools_ui
```

## 说明

- 当前版本优先保证工程结构清晰与可编译。
- 实时采图、标定流程、机器人控制接口已留出统一适配层，后续可按具体机器人协议补齐。

## 项目结构

```
ros_robot_assist_tools/
├── CMakeLists.txt
├── package.xml
├── include/ros_robot_assist_tools/
│   └── ros_robot_assist_tools_ui.hpp
├── src/
│   ├── ros_robot_assist_tools_ui.cpp          # Qt UI 实现
│   └── ros_robot_assist_tools_ui_node.cpp     # ROS2 节点入口
└── config/
```

## 开发说明

添加新功能时：
1. 在 `ros_robot_assist_tools_ui.cpp` 中实现 UI
2. 不使用 Q_OBJECT 宏
3. 使用 QObject::connect + lambda 连接信号槽
4. 参考 calib_sim 项目的实现模式
