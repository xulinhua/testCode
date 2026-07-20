# nova_grasp_moveit

独立的 ROS 2 / MoveIt 2 双臂抓取包。包内包含 MoveIt 配置、`move_group`、
Qt 控制界面、抓取路点生成器和 IK 执行器。它与 Isaac Sim 插件
`nova_robot_graspnet` 只通过 ROS 话题和 TF 通信，不依赖工作区中的其它 MoveIt
配置包。

## 1. 系统架构

推荐运行链路：

```text
/box_pose 或 /yolo_graspnet/collision_free_grasps
                │
                ▼
grasp_qt_ui_node
  Qt UI + GraspRosNode + GraspExecutor
                │  /nova_target_arm_pose
                │  /nova_gripper_goal
                ▼
grasp_arm_executor_node
                │  /compute_ik
                ▼
MoveIt move_group
                │  IK joint solution
                ▼
/joint_command ───────────────► Isaac ArticulationController
```

`grasp_qt_ui_node` 使用两个线程：

- Qt 主线程负责界面和按钮事件。
- ROS spin 线程负责订阅、TF、服务响应和状态更新。
- 耗时的候选筛选、IK 检查和执行请求通过 `QtConcurrent` 运行，避免阻塞界面。

核心源码：

| 文件/组件                | 职责                                            |
| ------------------------ | ----------------------------------------------- |
| `grasp_planner.*`      | 盒子顶抓路点、TCP 到腕部换算、路径日志          |
| `grasp_executor.*`     | 连续执行、单步执行、到位等待                    |
| `grasp_ros_node.*`     | ROS 接口、GraspNet 候选转换与选择、线程安全快照 |
| `grasp_arm_executor.*` | 调用 MoveIt IK，并将解发布为关节命令            |
| `grasp_qt_ui.*`        | 三个 Tab 页、实时状态、参数和异步操作           |

## 2. 编译与运行

### 2.1 编译

```bash
cd /home/hs/testCode/simulation
colcon build --packages-select nova_grasp_moveit --symlink-install
source install/setup.bash
```

### 2.2 与 Isaac Sim 联调

1. 在 Isaac Sim 中加载 `nova_robot_graspnet` 场景并点击 Play。
2. 确认 Isaac 正在发布 `/joint_states`、`/tf` 和需要的目标话题。
3. 启动完整抓取栈：

```bash
ros2 launch nova_grasp_moveit grasp_stack.launch.py
```

该 launch 同时启动：

- 本包的 `move_group`；
- `grasp_arm_executor_node`；
- `grasp_qt_ui_node`。

UI 中应显示 `/joint_states`、TF 和 MoveIt IK 均正常。

### 2.3 其它启动方式

| Launch/脚本 | 作用 |
| --- | --- |
| `grasp_stack.launch.py` | 推荐；启动完整抓取栈 |
| `move_group.launch.py` | 只启动本包 MoveIt `/compute_ik` |

## 3. Qt 界面

### 3.1 机械臂状态

- 显示 J1/J2 两臂的 J1–J8 当前值和目标值。
- 支持手动关节、末端位姿和夹爪控制。
- “复制当前”只把 TF/关节当前值填入控件，不发送命令。
- “实时模式”会周期性发送当前控件目标；执行自动抓取前会自动关闭。
- 下方日志显示 IK 请求、IK 解和关节发送结果。

### 3.2 抓取测试

使用 `/box_pose` 的位置计算固定顶抓：

```text
open → raise → move_xy → reorient → descend → close → lift
```

盒子姿态不会直接控制夹爪 RPY。顶抓姿态由
`canonical_top_down_quat()` 和 `grasp_yaw_offset_deg` 生成。

### 3.3 GraspNet 抓取

- 订阅 `geometry_msgs/PoseArray` 候选。
- 话题名可在界面中修改，默认 `/yolo_graspnet/collision_free_grasps`。
- 点击“应用”或在话题输入框中按回车，会重新订阅并清空旧候选。
- 进入本页时，页签右上角显示第三方发布状态：1 秒内有消息为正常，
  1–3 秒为延迟，超过 3 秒为中断；持续收到空数组会显示“在线 / 无位姿”。
- 点击“选择最优并计算”时冻结最新一帧，不再受后续连续发布影响。
- “单步”执行冻结规划中的一个步骤。
- “抓取最新最优位姿”会重新冻结最新一帧、选姿态并直接执行。

## 4. ROS 接口

### 4.1 订阅

| 话题                     | 类型                          | 说明                              |
| ------------------------ | ----------------------------- | --------------------------------- |
| `/box_pose`            | `geometry_msgs/PoseStamped` | 盒子位姿；盒子抓取只使用位置      |
| `/yolo_graspnet/collision_free_grasps` | `geometry_msgs/PoseArray` | GraspNet 候选；话题名可运行时修改 |
| `/joint_states`        | `sensor_msgs/JointState`    | IK 种子、通信状态和当前关节角     |
| `/tf`、`/tf_static`  | TF                            | 相机到`base_link`、腕部当前位姿 |
| `/nova_grasp/status`   | `std_msgs/String`           | 执行器状态回读                    |
| `/nova_pose_log`       | `std_msgs/String`           | IK 和关节命令调试日志             |

### 4.2 发布

| 话题                         | 类型                           | 说明                                         |
| ---------------------------- | ------------------------------ | -------------------------------------------- |
| `/nova_target_arm_pose`    | `nova_grasp_moveit/ArmPose`  | 指定机械臂的 J1_6/J2_6 腕部目标              |
| `/nova_arm_id`             | `std_msgs/Int32`             | 当前目标机械臂，0=J1，1=J2                   |
| `/nova_gripper_goal`       | `std_msgs/String`            | 兼容通道：`open` 或 `close`              |
| `/nova_grasp/status`       | `std_msgs/String`            | `EXECUTE`、步骤、完成或错误状态            |
| `/joint_command`           | `sensor_msgs/JointState`     | Isaac 主关节命令通道                         |
| `/arm_controller/commands` | `std_msgs/Float64MultiArray` | MuJoCo/旧控制器兼容通道                      |
| `/nova_pose_log`           | `std_msgs/String`            | `grasp_arm_executor_node` 发布 IK 调试信息 |

夹爪精确开口由 UI 后端直接发布 J7/J8 子集到 `/joint_command`；不会覆盖另一条臂。

### 4.3 服务

| 服务            | 类型                          | 说明                                |
| --------------- | ----------------------------- | ----------------------------------- |
| `/compute_ik` | `moveit_msgs/GetPositionIK` | 由本包`move_group.launch.py` 提供 |

## 5. 坐标约定

### 5.1 公共参考系

- 所有执行路点最终转换到 `base_link`。
- GraspNet 必须填写 `PoseArray.header.frame_id`。
- 转换优先使用消息时间戳；时间外推失败时回退到最新 TF。
- MoveIt IK link 是 `J1_6` 或 `J2_6`，不是指尖中心。

### 5.2 TCP 与腕部

`ee_tcp_z_offset` 表示腕部 J*_6 到两指中心沿 TCP 局部 `+Z` 的距离。
当前默认值为 `0.20 m`：

```text
p_wrist = p_tcp - R_tcp * [0, 0, ee_tcp_z_offset]
```

### 5.3 GraspNet 姿态

约定 GraspNet：

- 局部 `+X`：朝向物体的接近方向；
- Pose 原点：两指中心。

机器人 TCP 约定局部 `+Z` 从腕部指向两指中心，因此固定轴映射为：

```text
X_tcp = Y_grasp
Y_tcp = Z_grasp
Z_tcp = X_grasp
```

## 6. 路径计算

### 6.1 盒子顶抓

设盒子目标中心为 `p`：

```text
pregrasp = p + [0, 0, pregrasp_z_offset]
grasp    = p
lift     = p + [0, 0, lift_z_offset]
```

下降和抬升使用规划姿态并锁定当前腕部 XY，避免实际 TF 小偏差导致斜走。
下降前可通过 `grasp_yaw_offset_deg` 在盒心上方旋转。

### 6.2 GraspNet 抓取

设转换后的 TCP 抓取中心为 `p_g`，TCP 接近单位向量为 `a`：

```text
pregrasp = p_g - a * pregrasp_distance
grasp    = p_g
lift     = p_g + [0, 0, lift_z_offset]
```

GraspNet 路点设置 `preserve_waypoints=true`，执行器不会用盒子顶抓规则覆盖姿态。
预抓取沿接近轴后退，抬升固定沿 `base_link +Z`。

需要注意：当前每个路点独立调用 IK，再把关节目标发给 Isaac。路点之间由关节驱动运动，
不是 MoveIt 笛卡尔插值，因此端点按上述公式计算，但中间 TCP 轨迹不保证严格直线。

## 7. GraspNet 候选选择

1. 点击按钮时复制最新 `PoseArray`。
2. 过滤 NaN、Inf 和无效四元数。
3. 通过 TF 转换到 `base_link`。
4. 计算局部 `+X` 与 `base_link -Z` 的夹角。
5. 小于 `graspnet_top_max_angle_deg` 的候选标记为顶部抓取。
6. 顶部抓取优先；同类候选按接近角从小到大排序。
7. 对候选的 `pregrasp` 和 `grasp` 分别检查 J1、J2 IK。
8. 同一候选两臂都可达时，选择关节移动代价较小者：

```text
cost = Σ wrap_to_pi(q_solution - q_current)²
```

找到排序后的第一个可达候选后停止。当前 `PoseArray` 不包含分数，因此不使用网络评分。

## 8. 参数

参数文件：`config/grasp_moveit.yaml`。

| 参数                           |                    默认值 | 说明                               |
| ------------------------------ | ------------------------: | ---------------------------------- |
| `box_topic`                  |             `/box_pose` | 盒子位姿话题                       |
| `graspnet_topic` | `/yolo_graspnet/collision_free_grasps` | GraspNet PoseArray 话题；UI 可修改 |
| `graspnet_top_max_angle_deg` |                  `30.0` | 顶部抓取最大接近倾角               |
| `pose_frame`                 |             `base_link` | 无 frame 时的兼容默认值            |
| `target_arm_pose_topic`      | `/nova_target_arm_pose` | 腕部位姿命令                       |
| `arm_id_topic`               |         `/nova_arm_id` | 夹爪兼容命令当前机械臂             |
| `gripper_topic`              |    `/nova_gripper_goal` | 夹爪兼容命令                       |
| `status_topic`               |    `/nova_grasp/status` | 抓取状态                           |
| `pose_log_topic`             |        `/nova_pose_log` | IK 调试日志                        |
| `arm_split_x`                |                  `0.40` | 盒子抓取按 X 选择臂的分界          |
| `pregrasp_z_offset`          |                  `0.15` | 预抓取距离                         |
| `lift_z_offset`              |                  `0.15` | 抓取后抬升距离                     |
| `box_grasp_z_offset`         |                   `0.0` | 盒子目标 Z 修正                    |
| `ee_tcp_z_offset`            |                  `0.20` | 腕部到两指中心距离                 |
| `min_approach_clearance`     |                  `0.15` | 预抓取最小安全距离                 |
| `grasp_yaw_offset_deg`       |                  `90.0` | 盒子顶抓绝对 yaw                   |
| `step_settle_sec`            |                   `2.0` | 位姿步骤仿真稳定时间               |
| `gripper_settle_sec`         |                   `0.8` | 夹爪开闭等待时间                   |
| `gripper_open_m`             |                  `0.08` | 张开距离                           |
| `gripper_close_m`            |                  `0.02` | 闭合后保留距离                     |
| `ee_frame_arm0`              |                  `J1_6` | J1 腕部 TF frame                   |
| `ee_frame_arm1`              |                  `J2_6` | J2 腕部 TF frame                   |
| `ref_frame`                  |             `base_link` | 腕部显示参考系                     |
| `joint_command_topic`        |        `/joint_command` | Isaac 关节命令话题                 |
| `joint_command_burst_count`  |                     `5` | 同一命令重复发送次数               |
| `publish_mujoco_joint_array` |                  `true` | 是否发布旧控制器兼容数组           |

## 9. 日志

- `[PATH]`：规划后的腕部路点，蓝色显示。
- `[STEP] BEFORE/PLAN/AFTER/ERR`：单步执行前后和误差。
- `[graspnet]`：候选数量、选择下标、机械臂和接近角。
- `[pose_log]`：IK 种子、请求、结果和关节命令。
- `[ERROR][NG]`：IK 超时、无解或命令拒绝。

## 10. 常见问题

### MoveIt IK 未就绪

使用 `grasp_stack.launch.py`，并确认：

```bash
ros2 service list | rg /compute_ik
```

### GraspNet 一直等待候选

检查：

```bash
ros2 topic type /yolo_graspnet/collision_free_grasps
ros2 topic echo --once /yolo_graspnet/collision_free_grasps
ros2 run tf2_ros tf2_echo base_link <camera_frame>
```

消息类型应为 `geometry_msgs/msg/PoseArray`，且 `header.frame_id` 必须存在。若使用其它话题，
在 GraspNet Tab 输入话题名后点击“应用”。

### 候选存在但全部 IK 失败

- 检查相机外参和长度单位是否为米。
- 检查 Pose 原点是否确实是夹爪中心。
- 检查 GraspNet 局部 `+X` 是否为接近方向。
- 检查 `ee_tcp_z_offset` 是否与模型一致。
- 查看 `[graspnet]` 和 `[pose_log]` 日志。

### 末端位姿无反应

确认日志出现 `Pose command sent`，并确认 Isaac 订阅 `/joint_command`。
`/arm_controller/commands` 只是兼容通道。

### 全开/全闭带动机械臂

夹爪命令应只包含当前臂的 J7/J8。日志应出现
`[gripper] ... (arm joints held)`；如果仍带动臂杆，检查是否运行了旧 build。

### `libnova_grasp_moveit__rosidl_typesupport_*.so` 找不到

修改消息或切换终端后重新编译并执行：

```bash
source install/setup.bash
```

