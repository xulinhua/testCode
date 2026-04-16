# nova_sim

`nova_sim` 是当前开发/调试用的 ROS2 仿真包，包含：
- Gazebo + RViz 启动
- `ros2_control` 位置控制链路
- MoveIt2 IK 执行节点（C++）
- C++ Qt 控制界面（关节控制 + 位姿控制 + 夹爪控制）

## 环境

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic（`gazebo_ros`）

## 包结构

```text
nova_sim/
├── launch/
│   ├── nova_position.launch.py     # 主入口（Gazebo/RViz/MoveIt/UI）
│   └── gazebo_only.launch.py       # 仅仿真与模型生成
├── urdf/
│   └── nova_robot_position.urdf    # 机器人模型（含传感器与夹爪约束）
├── config/
│   ├── nova_position.yaml          # ros2_control 控制配置
│   └── nova_sim.rviz              # RViz 默认配置
├── src/
│   ├── moveit2_arm_executor.cpp    # IK 求解与命令下发
│   └── nova_control_ui_qt.cpp      # Qt 控制界面
└── README.md
```

## 快速编译

在工作空间根目录执行：

```bash
cd /home/hs/testCode/simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select nova_sim
source install/setup.bash
```

## 快速运行

完整模式（Gazebo + RViz + MoveIt + 控制工具）：

```bash
ros2 launch nova_sim nova_position.launch.py with_moveit:=true with_control_tools:=true
```

纯仿真模式（不启 MoveIt / UI）：

```bash
ros2 launch nova_sim nova_position.launch.py with_moveit:=false with_control_tools:=false
```

常用启动参数：

- `world`：Gazebo world 文件路径
- `urdf_file`：URDF 路径
- `spawn_z`：模型初始高度（默认 `0.06`，用于避免初始穿地）
- `rvizconfig`：RViz 配置路径
- `with_moveit`：是否启动 `move_group`
- `with_control_tools`：是否启动执行器与 Qt UI

## 主要节点与话题

- 控制命令：`/arm_controller/commands` (`std_msgs/Float64MultiArray`)
- 当前关节：`/joint_states`
- IK 输入：
  - `/nova_arm_id` (`std_msgs/Int32`)
  - `/nova_target_pose` (`geometry_msgs/PoseStamped`)
  - `/nova_gripper_goal` (`std_msgs/String`, `open|close|width:<m>`)
- IK 服务：`/compute_ik`
- UI 日志回传：`/nova_pose_log` (`std_msgs/String`)

## 控制指令示例

直接发布 28 轴关节控制：

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]}"
```

Pose 控制链路（arm + pose）：

```bash
ros2 topic pub --once /nova_arm_id std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /nova_target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.35, y: -0.10, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

夹爪控制：

```bash
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: open}"
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: close}"
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: width:0.03}"
```

## Qt UI 说明

- `Joint Control`
  - 目标值实时发布（可开关）
  - 显示当前关节值（只读）
  - 支持“复制当前值到目标值”
- `Pose Control`
  - 支持 `RPY / Quaternion` 两种输入模式
  - 显示四个机械臂末端当前位姿（只读）
  - 支持“复制当前位姿到目标位姿”
  - 右侧 `Pose Log` 显示前端校验日志与后端 IK 结果日志

## 夹爪约束（当前版本）

- `J1_7`: close=-0.04, open=0.02
- `J1_8`: close=0.04, open=-0.02
- `J2_7`: close=-0.04, open=0.02
- `J2_8`: close=0.04, open=-0.02
- `width` 插值范围：`[0.0, 0.06]`

## 常见问题

- `IK failed ... code=-31`
  - 一般是目标位姿不可达，或参考系/姿态设置不合理
  - 优先尝试：先复制当前位姿，再小步调整目标
- `frame_id=world` 不可用
  - 确认已通过 launch 发布 `world -> base_link` 静态 TF
- 点击“复制当前位姿到目标”失败
  - 查看 `Pose Log` 是否存在 `cannot transform ref <- ee` 提示
  - 确认 TF 树稳定、所选 `frame_id` 与当前 `arm_id` 有可用变换
- RViz 显示异常
  - 确认 `rvizconfig` 指向存在文件，`Fixed Frame` 与 TF 根一致（推荐 `base_link`）
- 底座看起来“左低右高”或轻微漂移
  - 常见根因是 `base_link` 使用复杂网格作为碰撞体，地面接触点不稳定
  - 当前版本已将 `base_link` 的碰撞改为简化平底 box（视觉仍保留 STL）
  - 如仍有穿地/悬空，可微调 `spawn_z`（建议范围 `0.05~0.08`）

## 备注

`nova_sim` 主要用于功能迭代验证；稳定版本请同步到 `nova_sim`。