# gazebo_robot_had2503_demo

ROS2 Humble：HAD2503-D-DEX1 在 **Gazebo Classic** 中仿真。

- **底盘**：差速驱动（`/diff_drive_controller/cmd_vel_unstamped`）
- **双臂**：MoveIt2（`left_arm` / `right_arm`，末端 `left_wrist_roll_link` / `right_wrist_roll_link`）
- 原始 SolidWorks 导出保留在 `HAD2503-D-DEX1/`（**视觉 mesh 原样**；Gazebo/MoveIt **碰撞** 用 AABB 盒 / 驱动轮圆柱，避免高面数 STL 碰撞导致飞车与 OOM）

## 编译

```bash
cd /home/hs/testCode/simulation
colcon build --packages-select gazebo_robot_had2503_demo --symlink-install
source install/setup.bash
```

> **注意**
> 1. 启动本包 `gazebo_robot_had2503_demo`，不要用 `ros2_ws` 的 `had2503_description`。
> 2. **不覆盖** `GAZEBO_MASTER_URI` / `ROS_DOMAIN_ID`，完全用你在终端里的设置。
> 3. 与 nova 并开时：两边端口和 domain 都要不同。
> 4. Mesh 已改为 `file://` 绝对路径；launch 仍会设置干净的 `GAZEBO_MODEL_PATH`（避免 `/opt/ros/humble/share` 刷屏）。
> 5. RViz 里大半 link “No transform” = 还没有 `/joint_states`（controller_manager / joint_state_broadcaster 未起）。

## 启动

```bash
# 例：与 nova（默认 11345 / domain 0）隔离时，在本终端先 export
export ROS_DOMAIN_ID=10
export GAZEBO_MASTER_URI=http://127.0.0.1:11346

cd /home/hs/testCode/simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gazebo_robot_had2503_demo bringup.launch.py
```

其它入口：

```bash
ros2 launch gazebo_robot_had2503_demo gazebo.launch.py
ros2 launch gazebo_robot_had2503_demo debug_ui.launch.py
ros2 launch gazebo_robot_had2503_demo display.launch.py
```

## 调试 UI

窗口 **HAD2503 调试**：底盘 / 双臂 / 夹爪 / 躯干。

## 底盘控制（命令行）

```bash
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

## MoveIt

```bash
ros2 service list | grep compute_ik
```

## 说明

- 驱动轮高度关节在仿真 URDF 中锁为 `fixed`（mesh 未改）。
- `wheel_separation` / `wheel_radius` 在 `config/controllers.yaml`；落地用 `spawn_z`。
