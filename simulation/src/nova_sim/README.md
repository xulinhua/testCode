# nova_sim

双机械臂（龙门 + 双臂）在 **Gazebo Classic** 下的 **ROS 2 Humble** 仿真包：URDF 模型、`ros2_control` 位置控制、**MoveIt2** IK、C++ 轨迹执行节点、与 **calib_sim** 对接的桥接节点，以及 **Qt** 关节/位姿/夹爪控制界面。

---

## 环境要求

| 项目 | 版本 / 说明 |
|------|-------------|
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 **Humble** |
| 仿真 | Gazebo Classic（`gazebo_ros`、`gazebo_ros2_control`） |
| 可选 | MoveIt2（默认 launch 调用 `nova_moveit_config`） |
| 依赖包 | 本仓库 **`calib_sim`**（标定桥接与消息类型） |

---

## 仓库与工作空间

本包位于仿真工作空间内，例如：

```text
/home/hs/testCode/simulation/
├── src/
│   ├── nova_sim/          # 本包
│   ├── calib_sim/         # 手眼标定（见 calib_sim README）
│   └── pt_tf_trans/       # 可选：点云光学系→camera_link
├── install/
└── build/
```

在 **`simulation` 根目录** 编译（可同时编译依赖）：

```bash
cd /home/hs/testCode/simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select nova_sim calib_sim
source install/setup.bash
```

仅改 `nova_sim` 时也可：

```bash
colcon build --packages-select nova_sim
```

---

## 包结构

```text
nova_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── nova_position.launch.py   # 主入口：Gazebo + RViz + MoveIt + 控制工具
│   ├── gazebo_only.launch.py     # 仅 Gazebo + 机器人生成 + 控制接口
│   └── display.launch.py         # 轻量显示（无完整仿真时用）
├── urdf/
│   └── nova_robot_position.urdf  # 整机：龙门、双臂、夹爪、固定/腕部相机、标定板挂载等
├── config/
│   ├── nova_position.yaml        # ros2_control：控制器、关节组
│   └── nova_sim.rviz             # RViz 默认配置
├── worlds/
│   └── simple.world
├── meshes/                       # 视觉/碰撞网格
├── src/
│   ├── moveit2_arm_executor.cpp  # 订阅目标位姿 + arm_id，调用 /compute_ik 并下发轨迹
│   ├── calib_sim_bridge_node.cpp # 话题桥接：robot_pose / target / 标定控制等
│   ├── all_joints_reset_node.cpp
│   └── nova_control_ui_qt.cpp    # Qt：关节、位姿、夹爪、日志
└── README.md
```

---

## 快速运行

### 完整开发模式（推荐）

Gazebo + RViz + MoveIt + IK 执行器 + **标定桥接** + 关节复位辅助 + Qt 控制界面：

```bash
source install/setup.bash
ros2 launch nova_sim nova_position.launch.py with_moveit:=true with_control_tools:=true
```

### 精简模式

不需要 MoveIt / UI 时：

```bash
ros2 launch nova_sim nova_position.launch.py with_moveit:=false with_control_tools:=false
```

### Launch 常用参数

| 参数 | 含义 |
|------|------|
| `world` | Gazebo world 路径 |
| `urdf_file` | URDF 路径（默认本包 `urdf/nova_robot_position.urdf`） |
| `spawn_z` | 模型生成高度补偿（默认 `0.06`，减轻穿地/悬空） |
| `rvizconfig` | RViz2 配置路径 |
| `moveit_launch_package` | 默认 `nova_moveit_config` |
| `moveit_launch_file` | 默认 `move_group.launch.py` |
| `with_moveit` | 是否启动 MoveIt |
| `with_control_tools` | 是否启动 `moveit2_arm_executor`、`calib_sim_bridge_node`、`all_joints_reset_node`、`nova_control_ui_qt` |

---

## 主要节点与话题

### 控制与状态

- **`/arm_controller/commands`**（`std_msgs/Float64MultiArray`）  
  全关节位置指令（维度与 `nova_position.yaml` 中关节列表一致）。

- **`/joint_states`**（`sensor_msgs/JointState`）  
  仿真关节反馈。

### 位姿 / IK 链路（与 Qt、标定共用）

- **`/nova_arm_id`**（`std_msgs/Int32`）  
  选择臂 `0` 或 `1`。

- **`/nova_target_pose`**（`geometry_msgs/PoseStamped`）  
  目标末端位姿（`frame_id` 建议 `base_link`）。

- **`/compute_ik`**（MoveIt 提供的 IK 服务，由执行器调用）

- **`/nova_gripper_goal`**（`std_msgs/String`）  
  示例：`open`、`close`、`width:0.03`。

### 标定桥接（`calib_sim_bridge_node`）

将仿真侧话题映射为 **`calib_sim`** 期望的名称（如 `/robot_pose`、`/robot_target_pose`、`/robot_reached` 等，以桥接源码为准）。  
手眼标定请另起：

```bash
ros2 run calib_sim calib_sim_node
```

详见 **`calib_sim/README.md`**。

### UI 日志

- **`/nova_pose_log`**（`std_msgs/String`）  
  Qt 位姿页校验与 IK 结果摘要。

---

## 坐标系与传感器（摘录）

- **`base_link`**：整机基座；静态 TF 中常见 `world` → `base_link`（以 launch 为准）。
- **末端**：`J1_6`、`J2_6` 等，与 **`calib_sim`** 中 `tf_ee_frame_arm0` / `arm1` 配置保持一致。
- **固定相机（眼在手外）**：如 `camera0_link`、`camera0_optical_frame`；Gazebo RGB 插件可能使用 `camera0_link` 作为 `frame_name`，光学系为子坐标系（零平移 + 固定 RPY）。
- **腕部相机（眼在手上）**：`camera1_optical_frame`、`camera2_optical_frame` 等与 URDF 一致。

标定输出 `T_cam_base` 的参考矩阵应与 **`tf2_echo base_link <光学 frame>`** 一致。

---

## 控制示例

### 全零关节（示例数组长度请与实际关节数一致）

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]}"
```

### 单臂目标位姿（arm 0）

```bash
ros2 topic pub --once /nova_arm_id std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /nova_target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.35, y: -0.10, z: 0.25}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### 夹爪

```bash
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: open}"
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: close}"
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: width:0.03}"
```

---

## Qt 控制界面说明

- **Joint Control**  
  目标关节值可连续发布；显示当前关节；支持「复制当前 → 目标」。

- **Pose Control**  
  支持 **RPY / 四元数**；只读显示多臂末端位姿；支持复制当前位姿；右侧 **Pose Log** 汇总前端检查与 IK 反馈。

---

## 夹爪关节约定（当前 URDF / UI）

| 关节 | close 方向（约） | open 方向（约） |
|------|------------------|-----------------|
| `J1_7` / `J2_7` | -0.04 | 0.02 |
| `J1_8` / `J2_8` | 0.04 | -0.02 |

`width` 插值约 **`[0.0, 0.06]`** m（以执行器实现为准）。

---

## 常见问题

| 现象 | 处理建议 |
|------|----------|
| `IK failed ... code=-31` | 目标不可达或姿态不合理；先「复制当前位姿」再小步修改 |
| `frame_id=world` 不可用 | 确认 launch 已发布 `world` → `base_link`（或改用 `base_link`） |
| 复制位姿失败 / `cannot transform` | 检查 TF 树、`nova_arm_id` 与 `frame_id` 是否匹配 |
| RViz 异常 | 检查 `rvizconfig` 路径；**Fixed Frame** 建议 `base_link` |
| 底座倾斜、漂移、穿地 | `base_link` 碰撞已尽量简化；可调 **`spawn_z`**（例如 `0.05~0.08`） |
| 手眼结果与 RViz 不一致 | 核对光学 frame 与 `camera_info`；用 `tf2_echo` 对比 `calib_sim` 中 URDF 参考 |

---

## 相关包

- **`calib_sim`**：ArUco 手眼标定、Qt 标定界面、结果 YAML。  
- **`pt_tf_trans`**（可选）：深度点云从光学系变换到 `camera_link`，话题默认为 `/camera/depth_pcl` → `/camera/pt_transformed_out`；若与 `nova_sim` 相机命名不一致需在源码中改常量后重编译。

---

## 备注

`nova_sim` 用于功能迭代与算法验证；部署到真机或发布环境前请同步更新 URDF、安全限位与控制参数。
