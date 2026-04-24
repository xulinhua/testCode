# nova_sim_mujoco

`nova_sim_mujoco` 是 `nova_sim` 的 MuJoCo 迁移包，用于双臂/多臂机器人仿真、控制与标定联调。  
原则上只改 `nova_sim_mujoco` 与 `calib_sim_mujoco`，不改原 `nova_sim`。

---

## 1. 项目结构（核心文件）

```text
nova_sim_mujoco/
├── launch/
│   ├── mujoco_only.launch.py          # MuJoCo + ros2_control 最小链路
│   ├── nova_position.launch.py        # 全量入口（MuJoCo + RViz + MoveIt + UI + bridge）
│   └── display.launch.py
├── config/
│   ├── mujoco_inputs.xml              # MuJoCo 物理参数、actuator kp/dampratio
│   ├── nova_position.yaml             # ros2_control 控制器配置（28 轴）
│   ├── calib_sim_bridge.yaml          # calib_sim 与 nova_sim 的桥接参数
│   └── nova_sim.rviz
├── urdf/
│   ├── nova_robot_position.urdf       # 主 URDF（含 ArUco 板）
│   └── aruco_cali.xacro
├── meshes/
│   ├── aruco_6x6_1000_id0.dae
│   ├── aruco_6x6_1000_id1.dae
│   ├── aruco_6x6_1000_id2.dae
│   └── aruco_6x6_1000_id3.dae
├── src/
│   ├── moveit2_arm_executor.cpp       # IK -> /arm_controller/commands 执行器
│   ├── calib_sim_bridge_node.cpp      # 标定话题桥接
│   ├── nova_control_ui_qt.cpp         # Qt 控制 UI
│   ├── nova_control_ui.cpp            # CLI 控制 UI
│   └── all_joints_reset_node.cpp      # 一键归零
├── scripts/
│   ├── start_mujoco.sh
│   ├── nova_control_ui.py             # Tk UI（备用）
│   └── regenerate_aruco_board_daes.py
├── CMakeLists.txt
└── package.xml
```

---

## 2. 环境与编译

```bash
cd /home/hs/testCode/simulation
source /opt/ros/humble/setup.bash
# 如你本机有 MuJoCo / moveit 叠加工作区，也先 source
# source /home/hs/ws_mujoco/install/setup.bash

colcon build --packages-select nova_sim_mujoco calib_sim_mujoco
source install/setup.bash
```

---

## 3. 启动方式

### 3.1 最小仿真（仅 MuJoCo + ros2_control）

```bash
ros2 launch nova_sim_mujoco mujoco_only.launch.py
```

用途：
- 验证 URDF->MJCF 转换
- 验证控制器与 `arm_controller` 收发
- 排除 MoveIt/UI/bridge 干扰

### 3.2 全量入口（推荐日常联调）

```bash
ros2 launch nova_sim_mujoco nova_position.launch.py
```

默认行为：
- `start_moveit:=true`
- `start_motion_stack:=true`
- 有 `DISPLAY` 时自动起 `nova_control_ui_qt`，否则回退 `nova_control_ui_cpp`

可选参数示例：

```bash
# 只调关节，不起 MoveIt 与 motion stack
ros2 launch nova_sim_mujoco nova_position.launch.py start_moveit:=false start_motion_stack:=false

# 自定义 URDF
ros2 launch nova_sim_mujoco nova_position.launch.py urdf_file:=/abs/path/to/xxx.urdf
```

---

## 4. 控制与话题链路（重要）

### 4.1 关节控制主链路

- 控制话题：`/arm_controller/commands`  
- 消息类型：`std_msgs/msg/Float64MultiArray`  
- 维度：28（与 `config/nova_position.yaml` 一致）

### 4.2 位姿控制主链路（已修正为原子消息）

- 推荐话题：`/nova_target_arm_pose`
- 消息类型：`calib_sim_mujoco/msg/ArmPose`
- 内容：`arm_id + PoseStamped` 同消息发送，避免 arm_id 与 pose 分话题乱序

兼容话题（仅保留给历史工具）：
- `/nova_arm_id`
- `/nova_target_pose`

> `moveit2_arm_executor_cpp` 以 `/nova_target_arm_pose` 为主链路，避免出现“控制 arm0 时 arm1 跟着动”的错序问题。

### 4.3 位姿 IK 依赖

- 服务：`/compute_ik`（由 MoveIt `move_group` 提供）
- 未启动 MoveIt 时，Pose Control 会显示 IK unavailable 或无法执行

---

## 5. 常见问题、原因与排查

### 问题 A：控制 arm0 时 arm1 也动

可能原因：
- 历史链路里 `arm_id` 与 `pose` 分开发布导致回调乱序
- 额外节点同时向 `/arm_controller/commands` 发全量 28 轴命令

排查步骤：
```bash
ros2 topic info /nova_target_arm_pose
ros2 topic echo /nova_target_arm_pose --once
ros2 topic info /arm_controller/commands
ros2 topic hz /arm_controller/commands
```

确认点：
- 只保留一个主要发送源（UI/bridge/脚本不要同时发）
- 位姿控制优先走 `/nova_target_arm_pose`

---

### 问题 B：Pose Control 可见但点 Send Pose 无动作

可能原因：
- `start_moveit:=false`，`/compute_ik` 不存在
- MoveIt 启动失败

排查步骤：
```bash
ros2 service list | rg compute_ik
ros2 node list | rg move_group
```

---

### 问题 C：抖动、爆振或不稳定

可能原因：
- `mujoco_inputs.xml` 里刚度过高、阻尼不足
- 自碰撞策略不当
- 多节点反复覆盖同一关节命令

排查步骤：
```bash
ros2 topic hz /arm_controller/commands
ros2 topic echo /arm_controller/commands --once
```

建议参数入口：
- `config/mujoco_inputs.xml`：
  - `<joint damping armature>`
  - `<actuator ... kp dampratio>`
  - `<default class="collision">` 中 `contype/conaffinity/condim/friction`

---

### 问题 D：归零后关节有残余误差

可能原因：
- 重力静差（P 控制下常见）
- 对应关节 kp 偏低或载荷/惯量偏大

排查步骤：
```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"
ros2 topic echo /joint_states --once
```

---

### 问题 E：无图形界面

可能原因：
- 环境未设置 `DISPLAY`

行为说明：
- `nova_position.launch.py` 自动回退到 `nova_control_ui_cpp`（CLI）

排查：
```bash
echo $DISPLAY
ros2 node list | rg nova_control_ui
```

---

## 6. 手动调试指令（可直接复制）

### 6.1 查看关键节点与控制器

```bash
ros2 node list
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### 6.2 关节命令（全 28 轴）

```bash
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]}"
```

### 6.3 位姿命令（推荐：原子 arm+pose）

```bash
ros2 topic pub --once /nova_target_arm_pose calib_sim_mujoco/msg/ArmPose \
'{arm_id: 0, pose: {header: {frame_id: "base_link"}, pose: {position: {x: 0.25, y: 0.00, z: 0.45}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
```

### 6.4 夹爪命令

```bash
ros2 topic pub --once /nova_arm_id std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: open}"
# 或
ros2 topic pub --once /nova_gripper_goal std_msgs/msg/String "{data: width:0.0400}"
```

### 6.5 标定链路检查

```bash
ros2 topic echo /robot_target_pose --once
ros2 topic echo /robot_pose --once
ros2 topic echo /robot_reached --once
ros2 topic echo /calib_sim/reach_error --once
```

---

## 7. 参数调优建议（MuJoCo）

主要文件：`config/mujoco_inputs.xml`

- 先稳后快：先提高阻尼/armature 抑制颤振，再逐步提升 `kp`
- 对易下垂关节（如 J1/J2 的 2~4 轴）单独提 `kp + dampratio`
- 任何参数改动后固定做三步验证：
  1. 全零归位
  2. 小步阶正反向
  3. 连续多次重复动作观察是否漂移

---

## 8. 与 calib_sim_mujoco 联调

推荐顺序：

1. 启动 `nova_sim_mujoco`（`nova_position.launch.py`）
2. 确认 `/compute_ik`、`/nova_target_arm_pose`、`/arm_controller/commands` 正常
3. 启动 `calib_sim_mujoco` 节点
4. 观察 `robot_target_pose -> nova_target_arm_pose -> arm_controller/commands` 链路

---

## 9. 维护边界说明

- MuJoCo 相关改动放在：
  - `src/nova_sim_mujoco/`
  - `src/calib_sim_mujoco/`
- 不回改 `src/nova_sim/` 原包，避免影响 Gazebo/历史流程。
