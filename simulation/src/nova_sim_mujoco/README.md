# nova_sim_mujoco

`nova_sim_mujoco` 是基于 `nova_sim` 复制并迁移的 MuJoCo 版本，原 `nova_sim` 不改动。

## 主要内容

- MuJoCo + `mujoco_ros2_control` 仿真链路
- 双臂/龙门 URDF 模型与 `ros2_control` 控制器
- 轨迹执行节点、Qt 控制界面、标定桥接节点
- 与 `calib_sim_mujoco` 联调用于标定流程

## 快速启动

```bash
cd /home/hs/testCode/simulation
source /opt/ros/humble/setup.bash
source /home/hs/ws_mujoco/install/setup.bash
colcon build --packages-select nova_sim_mujoco calib_sim_mujoco
source install/setup.bash

# MuJoCo only
ros2 launch nova_sim_mujoco mujoco_only.launch.py

# 全量入口（RViz/MoveIt/控制工具）
ros2 launch nova_sim_mujoco nova_position.launch.py
```

## 关键文件

- `launch/mujoco_only.launch.py`：MuJoCo 主启动
- `config/mujoco_inputs.xml`：MuJoCo 转换/执行参数
- `config/nova_position.yaml`：控制器配置
- `src/calib_sim_bridge_node.cpp`：与 `calib_sim_mujoco` 话题桥接

## 说明

- 本包已清理 Gazebo 专用启动与 world 文件。
- 若要用于标定，可先起 `nova_sim_mujoco`，再运行 `ros2 run calib_sim_mujoco calib_sim_node`。
