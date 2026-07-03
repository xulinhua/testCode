# hs.robot.nova_robot_graspnet

Nova 双臂 + 不锈钢工作台 + 抓取盒子的 **Isaac Sim 5** 扩展，用于 GraspNet 等算法的仿真数据出流（color / depth / 点云 / CameraInfo / joint_states / TF）。

本插件**完全自包含**：机器人 USD、URDF、mesh 均拷贝在 `data/` 内，不修改、不依赖其他 simulation 包。

---

## 环境要求

| 项 | 说明 |
|----|------|
| Isaac Sim | 5.x（已在 5.0.0-rc.45 验证） |
| Python 环境 | `isaac_env`（含 `isaacsim`） |
| ROS 2 | Humble（Isaac 内置 bridge） |
| 网络 | 首次 Load 需拉取 Omniverse `rsd455.usd` payload |

---

## 快速开始

```bash
conda activate isaac_env
cd ~/testCode   # 或你的工作区根目录

bash simulation/src/nova_robot_graspnet/hs.robot.nova_robot_graspnet/scripts/start_isaac.sh
```

1. 打开 **Window → Hs Robot Nova Robot GraspNet**
2. 点击 **Load scene**
3. 点击 **Timeline Play** 或 **Start ROS stream**
4. 另开终端验证（**必须与 Isaac 同一 `ROS_DOMAIN_ID=31`**）：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=31   # ~/.bashrc 与 start_isaac.sh 默认均为 31
ros2 topic list | grep camera
ros2 topic hz /camera0_rgb_sensor/image_raw
ros2 topic echo /joint_states --once
```

---

## 场景内容

| 元素 | 说明 |
|------|------|
| 机械臂 | Nova 双臂，Load 后关节零位 |
| 相机 | 3× Intel RSD455（cam0 基座 / cam1 J1_6 / cam2 J2_6） |
| 工作台 | 2×1×0.6 m 不锈钢桌（程序化生成） |
| 盒子 | 8×26×16 cm 动态刚体（占位立方体，可换 `data/box/` 模型） |

---

## 默认 ROS 话题

### 相机（可在 UI 每路独立修改）

| 相机 | Color | Depth | PointCloud2 | CameraInfo |
|------|-------|-------|-------------|------------|
| cam0 | `/camera0_rgb_sensor/image_raw` | `/camera0_depth_sensor/depth/image_raw` | `/camera0_depth_sensor/depth/points` | `/camera0_rgb_sensor/camera_info` |
| cam1 | `/camera1_rgb_sensor/image_raw` | `/camera1_depth_sensor/depth/image_raw` | `/camera1_depth_sensor/depth/points` | `/camera1_rgb_sensor/camera_info` |
| cam2 | `/camera2_rgb_sensor/image_raw` | `/camera2_depth_sensor/depth/image_raw` | `/camera2_depth_sensor/depth/points` | `/camera2_rgb_sensor/camera_info` |

默认 frame_id：`camera0_pseudo_depth` / `camera1_pseudo_depth` / `camera2_pseudo_depth`（UI 可改）。

### 机器人

| 类型 | 默认话题 |
|------|----------|
| JointState | `/joint_states` |
| TF | `/tf` |
| 静态 TF | `/tf_static`（map → world） |

每路相机可单独勾选 **color / depth / points / CameraInfo**，分辨率（宽/高）在 UI 配置。

---

## 目录结构

```text
hs.robot.nova_robot_graspnet/
├── config/extension.toml          # Isaac 扩展清单
├── hs/robot/nova_robot_graspnet/  # ★ 唯一源码（Isaac 直接加载）
│   ├── __init__.py
│   ├── global_variables.py
│   ├── paths.py
│   └── impl/
│       ├── extension.py           # 扩展入口
│       ├── ui_builder.py          # 面板 UI
│       ├── session_controller.py  # Load / 出流状态机
│       ├── scene_loader.py        # 桌子 + 机器人 + 盒子
│       └── ros_io/                # OmniGraph ROS 出流
├── data/
│   ├── robot/
│   │   ├── nova_robot_prepared.usda   # 预处理后（优先加载）
│   │   ├── nova_robot.usda            # 自 nova_isaac_sim/2.usda 拷贝
│   │   ├── nova_robot_source.usda     # 首次 prepare 备份
│   │   ├── nova_robot_position.urdf
│   │   └── meshes/*.stl
│   └── box/                       # 自定义盒子模型（待替换）
├── scripts/
│   ├── start_isaac.sh             # 启动 Isaac + 本扩展
│   ├── prepare_robot_usd.py       # 生成 nova_robot_prepared.usda
│   └── prepare_urdf.py            # 清理 URDF 路径 / 去 ArUco
└── docs/README.md                 # 与本文档同步（extension.toml 引用）
```

---

## 开发与修改

**只维护一份代码**：直接编辑 `hs/robot/nova_robot_graspnet/`，改完后在 Isaac 中 **Reload Extension** 或重启，**无需 sync**。

手动启用扩展（不用 start 脚本时）：

```bash
isaacsim .../isaacsim.exp.full.kit \
  --ext-folder /path/to/simulation/src/nova_robot_graspnet \
  --enable hs.robot.nova_robot_graspnet
```

---

## 资源预处理

`start_isaac.sh` 会自动执行；也可手动：

```bash
EXT=simulation/src/nova_robot_graspnet/hs.robot.nova_robot_graspnet

python3 $EXT/scripts/prepare_urdf.py
python3 $EXT/scripts/prepare_robot_usd.py
```

`prepare_robot_usd.py` 会：

- 去掉内置 `ActionGraph`（避免与插件 ROS 图冲突）
- **彻底删除标定板**：`aruco_board*` mesh/link、`Looks/aruco_mat*` 材质及 `6x6_1000-*.png` 贴图引用
- 为 cam0/1/2 设置 `xformOp:resetXformStack`、关闭 RSD455 刚体/碰撞

Load 时还会再扫一遍，删除 stage 上名称含 `aruco` 的 prim（双保险）。

---

## 常见问题

### Load 后 Isaac 段错误

旧版本在 Load 中同步 `app.update()` 上百次会导致 Vulkan `NOT_READY` 崩溃。当前版本已改为 `stage.Load` + 延迟设关节零位。请 **Reload 扩展** 后再试。

### 控制台 PhysX：RSD455 missing xformstack reset

说明仍在用旧代码或未生成 `nova_robot_prepared.usda`。运行 `prepare_robot_usd.py` 并 Reload。

### 相机话题无数据

1. 确认已 **Play** 或 **Start ROS stream**
2. 检查 UI 中对应相机的 color/depth 勾选
3. 确认 RSD455 在线 payload 可访问（需网络）

### 看不到机器人

查看控制台 `SceneLoader:` 日志；确认 `data/robot/nova_robot_prepared.usda` 存在。

---

## 替换盒子模型

1. 将 OBJ/USD 放入 `data/box/`
2. 修改 `scene_loader.py` 中 `_create_box()`（当前为 8×26×16 cm 占位 Cube）

---

## 许可证与来源

- 机器人模型：自 `nova_isaac_sim` **拷贝**至本仓库，不在原包内修改
- RSD455：Isaac Sim Omniverse 官方 sensor asset（在线 payload）
