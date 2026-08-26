# ROS Robot Assist Tools

ROS2 Humble / Ubuntu 22.04 机器人辅助工具（C++17 + Qt）

## 功能

- 系统状态（CPU/GPU/内存、进程/节点/话题监控）
- 图像查看（ROS Image 话题）
- 标定板生成与导出（ArUco / 棋盘格 / 圆点 / ChArUco / Kalibr AprilGrid）
- 姿态转换（欧拉角 / 四元数 / 旋转矩阵等）
- 运动学计算（机械臂 KDL/MoveIt、差速、阿克曼）
- TF 查看

> 相机内参 / 双目 / 手眼 / TCP 等标定流程请使用 `hs_calib_suite`。

## 编译

```bash
cd /path/to/simulation
colcon build --packages-select ros_robot_assist_tools
source install/setup.bash
```

## 运行

```bash
ros2 run ros_robot_assist_tools ros_robot_assist_tools_ui
```

## 项目结构

```
ros_robot_assist_tools/
├── CMakeLists.txt
├── package.xml
├── cmake/AdaptiveOpenCV.cmake
├── config/                 # 默认 YAML（运动学等）
├── assets/                 # 帮助文档与运动学示意图
├── include/ros_robot_assist_tools/
│   ├── ui/                 # 各功能页 Widget
│   ├── module/             # 业务逻辑
│   ├── manage/             # YAML 配置读写（运动学）
│   └── kinematics/
└── src/
    ├── ros_robot_assist_tools_ui.cpp       # 主窗口与导航
    ├── ros_robot_assist_tools_ui_node.cpp  # ROS2 入口
    └── ...
```

## 开发说明

1. 新功能以独立 Widget + Module 放在 `ui/`、`module/`，在 `ros_robot_assist_tools_ui.cpp` 注册导航页
2. 不使用 `Q_OBJECT` / AUTOMOC；信号槽用 `QObject::connect` + lambda
3. OpenCV 通过 `cmake/AdaptiveOpenCV.cmake` 按本机 CUDA toolkit 自适应选择
