# nova_grasp_moveit

**独立** MoveIt2 抓取包：内嵌 MoveIt 配置与 `move_group`、自带消息、Qt UI、IK 执行器。
与 Isaac（`nova_robot_graspnet`）**仅通过 ROS 话题通讯**，不依赖工作区其它包（含 `nova_moveit_config` / `nova_test`）。

## 包内组件

| 可执行 / Launch             | 作用                                                     |
| --------------------------- | -------------------------------------------------------- |
| `grasp_stack.launch.py`   | **推荐**：本包 `move_group` + 执行器 + Qt UI     |
| `move_group.launch.py`    | 仅启动内嵌`/compute_ik`                                |
| `grasp_qt_ui_node`        | Qt：关节/末端、夹爪、抓取规划与执行                      |
| `grasp_arm_executor_node` | `/nova_target_arm_pose` → `/compute_ik` → 关节指令 |

| 配置目录                     | 说明                                                      |
| ---------------------------- | --------------------------------------------------------- |
| `config/moveit/`           | URDF（运动学，无 mesh）/ SRDF / KDL / OMPL / joint_limits |
| `config/grasp_moveit.yaml` | 抓取业务参数                                              |

## 运行（Isaac 联调）

1. Isaac：`nova_robot_graspnet` Load + Play（发 `/box_pose`、`/joint_states`、`/tf`）
2. 本机：

```bash
cd /home/hs/testCode/simulation
colcon build --packages-select nova_grasp_moveit --symlink-install
source install/setup.bash
ros2 launch nova_grasp_moveit grasp_stack.launch.py
```

无需再单独启动其它 MoveIt 包。UI 中 **MoveIt IK** 应变为就绪，再「计算抓取」→「执行抓取」。

## ROS 接口

### 订阅

| 话题              | 类型                          | 来源  |
| ----------------- | ----------------------------- | ----- |
| `/box_pose`     | `geometry_msgs/PoseStamped` | Isaac |
| `/joint_states` | `sensor_msgs/JointState`    | Isaac |
| `/tf`           | TF                            | Isaac |

### 发布

| 话题                      | 类型                          | 说明                                            |
| ------------------------- | ----------------------------- | ----------------------------------------------- |
| `/joint_command`        | `sensor_msgs/JointState`    | 关节/夹爪/位姿 IK 结果（Isaac 主通道）          |
| `/nova_target_arm_pose` | `nova_grasp_moveit/ArmPose` | 位姿目标 → executor → IK →`/joint_command` |
| `/nova_gripper_goal`    | `std_msgs/String`           | `open` / `close`                            |
| `/nova_grasp/status`    | `std_msgs/String`           | 抓取状态                                        |

### 本包自提供

| 服务            | 类型                          | 说明                                         |
| --------------- | ----------------------------- | -------------------------------------------- |
| `/compute_ik` | `moveit_msgs/GetPositionIK` | `grasp_stack` / `move_group` launch 启动 |

## 编译

```bash
colcon build --packages-select nova_grasp_moveit --symlink-install
source install/setup.bash
```

## 参数

见 `config/grasp_moveit.yaml`。

## 常见问题

### MoveIt IK 未就绪

请用 `grasp_stack.launch.py`（会起本包 `move_group`），不要只用 `grasp_qt_ui.launch.py`。
确认：`ros2 service list | grep compute_ik`。

### `libnova_grasp_moveit__rosidl_typesupport_*.so` 找不到

运行前 `source install/setup.bash`；改消息后需重新 `colcon build`。

### 末端位姿无反应

旧版 executor 只发 `/arm_controller/commands`（MuJoCo），Isaac 不订阅。现已同时发 `/joint_command`。请用 `grasp_stack.launch.py` 启动，并确认日志有 `Pose command sent`。

UI「复制当前」：把 TF 当前 xyz/rpy 填入目标控件（不下发）；「应用位姿」或实时模式才会 IK 下发。

### 全开/全闭带动机械臂

旧逻辑把整臂实测角写进指令。现「全开/全闭/应用夹爪」只改 J7/J8，臂杆保持上次指令。日志应出现 `[gripper] ... (arm joints held)`。

### 改 J1 时 J2 也跟着漂

旧逻辑把另一臂的**实测角**写进 `/joint_command` 作保持，静差会被反复下发并放大。现已改为保持**上次指令目标**。请重新 `colcon build --packages-select nova_grasp_moveit` 后再开 UI。

### 肩关节 J*_1「当前」与「目标」差几度

开环 force PD 无积分项；Isaac Load 时会只加强 `J1_1`/`J2_1` 刚度。请 **重新 Load scene**；若仍差 2–3°，可再加大 `scene_loader.py` 里 `_boost_arm_drives` 的 `stiffness` / `maxForce`。
