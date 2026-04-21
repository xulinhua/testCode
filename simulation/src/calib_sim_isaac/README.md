# calib_sim_isaac

面向 **Gazebo / 仿真或实机** 的 **手眼标定** ROS2 功能包：基于 ArUco 单 marker 的 PnP、眼在手上 / 眼在手外两种模式、可选「已知板子安装」直接求固定相机外参、Qt 可视化界面与结果落盘。

**ROS 发行版**：ROS 2 Humble（Ubuntu 22.04）  
**主要依赖**：`rclcpp`、`sensor_msgs`、`geometry_msgs`、`tf2_ros`、`OpenCV`、`Qt5::Widgets`

---

## 功能概览

| 能力 | 说明 |
|------|------|
| 眼在手上（eye-in-hand） | 腕部相机；求解 `T_cam→gripper`，并与 URDF 中 `camera*_optical_frame` 参考对比 |
| 眼在手外（eye-to-hand） | 固定相机；求解 `T_cam→base`（`^{base}T_{cam}`，`p_base = T · p_cam`） |
| 已知板子安装 | `use_known_target_mount: true` 时，用眼在手外链式闭合 + `target_to_gripper_pose` 直接融合多帧外参（并搜索 flip / 小位移补偿） |
| OpenCV 手眼 | 关闭已知安装时，眼在手上走 `calibrateHandEye` 多算法选优；眼在手外可走 PARK 等 |
| 多 ArUco ID | 画面中多个 ID 时 **仅对 `target_marker_id` 做 PnP**，避免误用其它板 |
| TF 同步采样 | `use_tf_for_sample_pose: true` 时末端位姿与图像时间戳对齐 |
| 质量指标 | 链式平移/旋转残差、角点重投影、与 URDF 参考差等 |

---

## 包内可执行文件

| 可执行文件 | 作用 |
|------------|------|
| `calib_sim_node` | **推荐**：标定节点 `CalibNode` + Qt UI（`calib_unified.yaml`），多模式一键切换 |
| `calib_qt_ui_node` | 仅 Qt 订阅 `/calib_sim_isaac/*`；需另起标定节点时使用 |
| `calib_sim_ui_node` | 旧版 UI（`ui.yaml`），依赖 `CalibSimUiNode` |

标定逻辑与话题均在 **`CalibNode`**（`calib_node.cpp`）中实现。

---

## 编译

在 **simulation** 工作空间根目录：

```bash
cd /home/hs/testCode/simulation   # 按你本机路径调整
source /opt/ros/humble/setup.bash
colcon build --packages-select calib_sim_isaac
source install/setup.bash
```

> 若 CMake 中指向了本机 CUDA/OpenCV 路径，请以你仓库内 `CMakeLists.txt` 为准；缺少 Qt 开发包时安装 `qtbase5-dev`。

---

## 运行方式

### 统一模式（推荐）

```bash
source install/setup.bash
ros2 run calib_sim_isaac calib_sim_node
```

启动后在窗口中通过下拉框切换 **眼在手外 eth0/eth1**、**眼在手上 eih0/eih1**；参数来自安装目录下的 `share/calib_sim_isaac/config/calib_unified.yaml`（各模式带前缀，由 `config_data_manager` 加载）。

### 单 YAML 节点（调试）

也可自行指定参数文件启动同一 `CalibNode` 逻辑（需自行准备入口或沿用项目内其它 main），例如仅眼在手上：

```bash
ros2 run calib_sim_isaac <你的节点> --ros-args --params-file install/calib_sim_isaac/share/calib_sim_isaac/config/eye_in_hand.yaml
```

（具体可执行文件名以 `CMakeLists.txt` 中 `add_executable` 为准；统一体验请用 `calib_sim_node`。）

---

## 主要话题

### 订阅（随配置变化）

- 图像：`image_topic`（如 `/camera0_rgb_sensor/image_raw`）
- 相机内参：`camera_info_topic`
- 机械臂状态：`robot_pose_topic`（默认 `sensor_msgs/msg/JointState` 的 `/joint_states`），或启用 TF 时由 `tf_base_frame` → `tf_ee_frame_arm*` 查询
- 到达标志：`robot_state_topic`（`std_msgs/Bool`）
- 控制：`/calib_sim_isaac/control`（`std_msgs/String`，如 `cmd:set_mode`、`init`、`step` 等，与 Qt/桥接约定一致）

### 发布

- `/calib_sim_isaac/status`、`/calib_sim_isaac/log`、`/calib_sim_isaac/result_text`
- `/calib_sim_isaac/raw_image`、`/calib_sim_isaac/result_image`（叠加检测与坐标轴）
- `robot_target_topic`：下发关节命令（默认 `sensor_msgs/msg/JointState` 的 `/joint_command`）
- 可选：`nova_all_joints_reset_topic`（`std_msgs/Empty`）等，见各 yaml

完整话题名以 **`eye_in_hand.yaml` / `eye_to_hand.yaml` / `calib_unified.yaml`** 为准。

---


## Isaac Joint 接口

`CalibNode` 已内置 Isaac Sim 风格 joint 接口转换（不需要新增节点）：

- `robot_pose_topic`：默认 `/joint_states`，类型 `sensor_msgs/msg/JointState`
- `robot_target_topic`：默认 `/joint_command`，类型 `sensor_msgs/msg/JointState`
- `kinematics_pose_goal_topic`：默认 `/nova_target_pose`，发送位姿目标给运动学/IK
- `kinematics_joint_command_topic`：默认 `/ik_joint_command`，接收 IK 输出关节命令

可选回退：

- `enable_legacy_pose_fallback: true` 时，当 IK 命令不可用会回退发布 `legacy_pose_command_topic`（`ArmPose`）

## 配置文件说明

| 文件 | 用途 |
|------|------|
| `config/calib_unified.yaml` | Qt 统一入口：多模式前缀参数（eth0/eth1/eih0/eih1） |
| `config/eye_in_hand.yaml` | 单模式：腕部相机 + 对应话题与采样网格 |
| `config/eye_to_hand.yaml` | 单模式：固定相机 + `target_to_gripper_pose`、龙门 camera0 等 |
| `config/ui.yaml` | 旧版 UI 节点专用 |

**常用参数要点**：

- `target_marker_id`：只解算该 ArUco ID。
- `marker_length_m`、`aruco_dict_id`：须与打印板 / 纹理一致。
- `use_tf_for_sample_pose`、`tf_base_frame`、`tf_ee_frame_arm0` / `arm1`：与 `nova_sim` 中 `J1_6` / `J2_6` 等一致。
- `use_known_target_mount`、`target_to_gripper_pose`：眼在手外板子刚接在末端时，安装位姿（目标系相对末端）；与 URDF 中板子 joint 一致时链式残差可达毫米级。
- `known_mount_quality_max_m`：已知安装路径下的质量门限。

---

## 坐标系约定（重要）

- PnP / OpenCV 输出按 **光学系**（Z 前向等）理解时，与 URDF **`camera*_optical_frame`** 对齐；Gazebo 中 RGB 可能以 `camera*_link` 为 `frame_id`，光学关节常为 **零平移 + 固定旋转**，原点一般重合。
- 眼在手上参考外参见代码中 `getUrdfReferenceTcamGripperForEyeInHand`；眼在手外参考见 `getUrdfReferenceTcamBaseForArm`（`^{base}T_{optical}`，应与 `tf2_echo base_link camera0_optical_frame` 一致，避免手写矩阵 **平移符号反号**）。
- 眼在手外已知安装公式：`T_cam_base = T_gripper_to_base · T_target_to_gripper · T_target_to_cam^{-1}`（与 `computeHandEyeChainResiduals` 中锚点检验一致）。

---

## 输出结果

默认写入参数 `output_dir` 下（如工作空间内 `calib_output_isaac/`），每次 run 带时间戳子目录，包含：

- `calib_result_eye_in_hand.yaml` 或 `calib_result_eye_to_hand.yaml`
- `sample_manifest.csv`（若启用）
- 文本结果经 `/calib_sim_isaac/result_text` 同步到 Qt

YAML 中含 `T_cam_base`、`T_base_cam`、链式残差、`mean_corner_reprojection_error_px`、与 URDF 对比等字段。

---

## 与 nova_sim 联调

仿真下通常先启动 `nova_sim`（Gazebo、桥接、`moveit2_arm_executor`、`/joint_states` 等），再启动 **`calib_sim_node`**。桥接节点名将 `nova_sim` 侧话题对齐到标定节点期望的名称（见 `nova_sim` README）。

---

## 常见问题

- **链式平移残差很大**：先检查眼在手外矩阵乘法是否已更新为上述公式；再查 `target_to_gripper`、TF 时间同步、`target_marker_id`。
- **`T_cam_base_vs_urdf` 极大但链式很好**：多为参考矩阵与当前 TF 不一致，用 `tf2_echo` 核对后更新 `getUrdfReferenceTcamBaseForArm` 或改为在线查 TF。
- **重投影很大**：检查 `marker_length_m`、字典 ID、畸变与 `camera_info`。
- **多 ID 误检**：已仅解目标 ID；日志中 `multi_marker using_id=...` 为提示。

---

## 目录结构（摘录）

```text
calib_sim_isaac/
├── CMakeLists.txt
├── package.xml
├── msg/ArmPose.msg
├── config/
│   ├── calib_unified.yaml
│   ├── eye_in_hand.yaml
│   ├── eye_to_hand.yaml
│   └── ui.yaml
├── include/calib_sim_isaac/
└── src/
    ├── calib_node.cpp          # 核心：检测、采样、手眼、落盘
    ├── config_data_manager.cpp
    ├── calib_qt_ui.cpp         # Qt 界面
    ├── calib_ui_node.cpp
    ├── calib_sim_node.cpp
    ├── calib_qt_ui_node.cpp
    └── calib_sim_ui_node.cpp
```
