# ROS Robot Workbench

ROS2 Humble / Ubuntu 22.04 机器人综合工作台（C++17 + Qt5）

在 `ros_robot_assist_tools` 基础上扩展，按 **Kit** 组织多类机器人开发与调试工具。

## 编译

默认编译全部 Kit。可按 **预设** 或 **Kit/能力开关** 裁剪模块（编译与左侧导航同步）。

```bash
cd ~/testCode/simulation

# 全量（默认）
colcon build --packages-select ros_robot_workbench

# 最小可用集：通用 Kit（系统状态、话题、Rosbag、图像查看），无 MoveIt/KDL/OpenCV
colcon build --packages-select ros_robot_workbench --cmake-args -DWORKBENCH_PRESET=minimal

# 通用运维：系统状态 + 话题/Rosbag/图像 + 运动学 + 仿真（无 MoveIt/OpenCV）
colcon build --packages-select ros_robot_workbench --cmake-args -DWORKBENCH_PRESET=general

# 机械臂开发：general + 标定 + 机械臂 + MoveIt
colcon build --packages-select ros_robot_workbench --cmake-args -DWORKBENCH_PRESET=arm

# 显式关闭某 Kit / 能力
colcon build --packages-select ros_robot_workbench --cmake-args \
  -DWORKBENCH_KIT_ARM=OFF -DWORKBENCH_WITH_MOVEIT=OFF

source install/setup.bash
```

### CMake 选项

| 类型 | 选项 | 说明 |
|------|------|------|
| 预设 | `WORKBENCH_PRESET` | `minimal` / `general` / `arm` / `full` |
| Kit | `WORKBENCH_KIT_GENERAL` 等 11 项 | 控制对应 Kit 源码与导航 |
| 能力 | `WORKBENCH_WITH_MOVEIT` | MoveIt 调试、运动学 MoveIt 后端 |
| 能力 | `WORKBENCH_WITH_KDL` | URDF/KDL 运动学（Kit 运动学依赖） |
| 能力 | `WORKBENCH_WITH_OPENCV` | 标定板、手眼标定 |

缺少 `moveit_msgs` / OpenCV 时，对应能力会自动关闭并给出 CMake 警告。

## 运行

```bash
ros2 run ros_robot_workbench ros_robot_workbench_ui
```

## Kit 模块一览

| Kit | 模块 |
|-----|------|
| Kit | 模块 |
|-----|------|
| **通用** | 系统状态、Rosbag工作台、话题调试、图像查看 |
| **运动学** | 姿态转换、运动学计算、TF查看 |
| **标定** | 标定板生成、内参/双目/多传感器/手眼/TCP |
| **机械臂** | 关节监视、MoveIt调试、抓取姿态、多TCP管理 |
| **移动机器人** | 里程计分析、轮速标定、Nav2状态 |
| **机器狗** | 足端监视、IMU姿态、RL策略监视 |
| **人形** | 全身关节、平衡面板 |
| **路径规划** | 路径对比、障碍编辑 |
| **3D感知** | 点云查看、深度分析、点云投影 |
| **仿真** | 仿真控制、Sim Time、Sim2Real对比、USD转换 |
| **深度学习** | 推理监视、检测叠加 |

## 架构

每个模块独立四层：

```
config/{module}.yaml
include/.../manage/{module}_data_manager.hpp
include/.../module/{module}_module.h
include/.../ui/{module}_widget.h
```

导航注册：`src/ui/workbench_module_registry.cpp`

新增模块脚手架：

```bash
python3 src/ros_robot_workbench/scripts/generate_kit_modules.py
```

## 说明

- 原 `ros_robot_assist_tools` 保持不变；本包为扩展副本。
- 部分新模块为可运行骨架，后续可按 Kit 逐步接入 ROS 接口与算法。
- 已具备基础 ROS 能力的模块：关节监视、Rosbag校验、点云统计、Sim Time、里程计分析。
- **USD转换**（仿真 Kit）：URDF / OBJ / STL 转 USD，后端可选 OpenUSD / urdf-usd-converter / Isaac Sim
- **仿真 Kit V1**：
  - **Sim Time**：自动刷新 sim time、RTF（读 `/clock`）
  - **仿真控制**：Play/Pause/Step/Reset/Load Scene；后端可选 ROS 话题 / Gazebo gz service / Isaac 脚本
  - **Sim2Real对比**：仿真 vs 实机 `JointState` 按关节对齐，显示误差与 RMSE
