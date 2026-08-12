# Hs Robot Spot Room Sim

Isaac Sim Kit 扩展：在 Simple Room 里跑 Boston Dynamics Spot，前方 RGB-D / 点云，用于障碍建图测试。

## 功能

- 场景：本地 Simple Room（不走 Nucleus 在线拉资产）
- 杂物：Load 时自动放带**碰撞+质量**的圆柱/凳子（Dynamic，Play 后受重力落地）及固定靠背，便于障碍建图
- 机器人：Spot + 官方 SpotFlatTerrainPolicy 步态
- 控制：订阅 `/cmd_vel`（`linear.x/y` + `angular.z`）
- 相机：`color` / `depth` / `points` / `camera_info`（头前视，正立）
- 状态：`/odom`、`/tf`、`/tf_static`、`/joint_states`

## 环境

- Conda：`isaac_env`（Isaac Sim 5.0）
- 默认 `ROS_DOMAIN_ID=31`

## 一键启动

```bash
cd /home/hs/testCode/simulation/src/spot_room_sim/hs.robot.spot_room_sim
bash scripts/start_isaac.sh
```

启动脚本会：

1. 检查本地资产，缺失则自动下载
2. 压缩 Simple Room 贴图（lite），加快 Load
3. 启动 Isaac 并启用 `hs.robot.spot_room_sim`

## 首次准备资产（可选手动）

```bash
cd /home/hs/testCode/simulation/src/spot_room_sim/hs.robot.spot_room_sim
python3 scripts/download_assets.py
python3 scripts/make_lite_textures.py
```

下载目录：

- `data/scenes/simple_room/`
- `data/robots/spot/`
- `data/policies/spot/`

## 使用步骤

1. 顶栏打开 **Hs Robot Spot Room Sim**
2. 点 **Load**（异步初始化 World，状态会显示 LOADING）
3. Timeline **Play**
4. 面板 **Drive** 区**按住** Fwd/Back/Left/Right 遥控（松开即停；也可点 **Stop**）
5. 或另一终端（同一 ROS_DOMAIN_ID）发 `/cmd_vel`：

```bash
export ROS_DOMAIN_ID=31
ros2 topic list
ros2 topic echo /tf_static --once
ros2 run tf2_tools view_frames
```

前进示例：

```bash
export ROS_DOMAIN_ID=31
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

键盘遥控：

```bash
ROS_DOMAIN_ID=31 ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 话题

| Topic | 说明 |
|-------|------|
| `/cmd_vel` | Twist，控制（订阅） |
| `/odom` | 里程计 |
| `/tf` | `odom -> base_link` + Spot 连杆树 |
| `/tf_static` | `base_link→body`、`base_link→cam_0_link`、以及 `*_frame→*_optical_frame` |
| `/joint_states` | 关节状态 |
| `/cam_0/color/image_raw` | 彩色图（frame: `cam_0_color_optical_frame`） |
| `/cam_0/depth/image_raw` | 深度图（frame: `cam_0_depth_optical_frame`） |
| `/cam_0/depth/points` | 点云（同上光学系） |
| `/cam_0/depth/camera_info` | 相机内参 |

坐标系约定（与实车 `cam_mgr_ros` / Gazebo 一致）：

| Frame | 轴向 | 说明 |
|-------|------|------|
| `cam_0_link` | X 前、Y 左、**Z 上** | REP-103 机体式安装系 |
| `cam_0_*_optical_frame` | X 右、Y 下、**Z 前** | REP-103 光学系；图像/点云挂此 frame |
| link→optical | `rpy(-π/2, 0, -π/2)` | 与 Gazebo `camera_optical_joint` 相同 |

建图固定系用 `odom`，障碍分析 `target_frame` 用 `base_link`。

## 默认位姿

- Spot 出生点：`(-1.8, 1.6, 0.55)`（避开房间中央低桌）
- 相机相对 body：USD 平移 `(0.35, 0, 0.12)`，RPY 度 `(90, 0, -90)`（渲染镜头朝向机器狗前方 +X）；ROS `cam_0_link` 为机体式（Z 上）+ 光学系 `rpy(-90°,0,-90°)`（光学 +Z 同为前方）
- 物理步长：`1/500` s

## 目录

```
spot_room_sim/
  README.md                          # 本文件
  hs.robot.spot_room_sim/
    config/extension.toml
    docs/README.md                   # 扩展包内说明（与本文件同步）
    scripts/start_isaac.sh
    scripts/download_assets.py
    scripts/make_lite_textures.py
    data/                            # 本地场景 / Spot / policy
    hs/robot/spot_room_sim/          # 扩展 Python 代码
```

## Common issues

**Viewport FPS very low (~2)**

- Spot needs 500Hz physics; without a catch-up cap, one slow frame schedules hundreds of physics steps
- Defaults: camera **640x480**, color/depth/points **on**, full-leg TF on, RTX ecoMode, max 8 physics steps/frame
- Restart via `scripts/start_isaac.sh`, then Load -> Play
- Point cloud + higher res can drop viewport FPS; turn off points in the UI if needed

**Load is slow**

- 确认已跑过 `make_lite_textures.py`（贴图从约 100MB 压到约 0.5MB）
- 不要走 Nucleus 在线拉 Simple Room

**Play 后 Spot 翻倒 / Stop 后再 Play**

- Timeline **Stop → Play** 会丢弃失效的 articulation view，并强制复位到加载时的出生点 `SPOT_SPAWN_POS`
- 控制台应出现 `hard re-init → standing @ ...` 与 `settle done`
- **Unload → Load** 会清空 World 单例后再建场景；若仍报 `is_homogeneous` / `create_articulation_view`，先 Stop，再 Unload → Load → Play

**RViz Map「No map」/ MessageFilter 丢 `base_link` 时间戳**

- 详见障碍侧文档：`project/src/slam/obstacle/doc/known_regressions.md`（§A/§B）
- `/tf`、`/odom` 用 `ReadSimTime`（`resetOnStop=False`）；相机必须同为 `resetSimulationTimeOnStop=False`
- Stop/Play 后若相机戳归零而 TF 累加 → RViz Fixed Frame=`odom` 丢图
- 改插件后优先 **重启 Isaac** 再建 OmniGraph；Unload→Load 见下条 Spot 残留问题

**Load 后看不到机器狗 / 障碍物（Status 变回 IDLE）**

- 根因：二次 Load 时 `/World/Spot` 删不干净 → `define_prim` 报 `A prim already exists`，只留下空房间
- 现已：Unload/Load 强制清 Spot/Room/Clutter，并复用已存在 prim
- 热更代码后请再点一次 **Load**；成功时 Status 应为 **LOADED**，控制台有 `SceneClutter: spawned` / `SceneLoader ready`
- 若仍失败：重启 Isaac 再 Load → Play

**图像是横的 / TF 不全**

- 相机应变为正立；TF 应同时有 `/tf` 与 `/tf_static`
- 用 `ros2 run tf2_tools view_frames` 检查是否连到 `cam_0_depth_optical_frame`

**hydra primvar 警告**

- Simple Room 资产自带的 UV 问题，建图可忽略；日志里已尽量压到 error

## 环境变量（可选）

| 变量 | 默认 | 含义 |
|------|------|------|
| `ROS_DOMAIN_ID` | `31` | ROS2 domain |
| `SPOT_KILL_EXISTING` | `1` | 启动前结束旧 Isaac 进程 |
| `SPOT_SKIP_ASSET_CHECK` | `0` | 跳过资产检查 |
| `SPOT_SKIP_LITE_TEXTURES` | `0` | 跳过贴图压缩 |
| `SPOT_ROOM_SIM_ENABLE` | `1` | 是否 `--enable` 本扩展 |
