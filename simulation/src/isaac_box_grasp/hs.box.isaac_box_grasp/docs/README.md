# hs.box.isaac_box_grasp

料盒识别抓取 Isaac Sim 扩展：Load 加载机器人/场景（可选）与相机；Timeline Play 出 ROS 流与 cmd_vel 差速；Start 订阅位姿驱动左臂抓取；面板手眼标定采样。

## 启用

```bash
conda activate isaac_env

isaacsim \
  /home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit \
  --ext-folder /home/hs/testCode/simulation/src/isaac_box_grasp \
  --enable hs.box.isaac_box_grasp
```

菜单：**Hs Box Isaac Box Grasp**

## 使用流程

1. **Load** — 加载 `data/robots/` 下机器人 USD（默认 `HAD2503-D-DEX1.usd`）；`Scene USD` 可空（场景未就绪时仅机器人+相机）
2. **Timeline Play** — 发布相机话题、TF，并订阅 `/box_grasp/cmd_vel`
3. **Start** — 订阅 `/box_grasp/pose_world`，左臂执行抓取序列
4. **Calibration** — 采样外参 / Export JSON 到 `calib_output/`

扩展**启动时不自动 Load**。

## 资源目录

```text
data/scenes/              # 场景 USD（可选，稍后提供）
data/robots/              # 机器人 USD（含 configuration/ 子目录）
robot_urdf/               # 源 URDF（离线转 USD 用 scripts/urdf_to_usd.py）
calib_output/             # 标定导出
```

## 默认 ROS 话题

| 话题 | 类型 | 方向 |
|------|------|------|
| `/box_grasp/camera/color/image_raw` | Image | 发布 |
| `/box_grasp/camera/depth/image_raw` | Image | 发布 |
| `/box_grasp/camera/depth/points` | PointCloud2 | 发布 |
| `/box_grasp/pose_world` | PoseStamped | 订阅（world，左臂目标） |
| `/box_grasp/cmd_vel` | Twist | 订阅（差速底盘） |
| TF `world` → `camera_optical_frame` | | 发布 |

详见 [vision_integration.md](vision_integration.md)。

## URDF → USD

```bash
conda activate isaac_env
python hs.box.isaac_box_grasp/scripts/urdf_to_usd.py
```

输出默认在 `data/robots/HAD2503-D-DEX1.usd`。

## 开发

源码在 `python/`。改后同步：

```bash
bash hs.box.isaac_box_grasp/scripts/sync_runtime.sh
```
