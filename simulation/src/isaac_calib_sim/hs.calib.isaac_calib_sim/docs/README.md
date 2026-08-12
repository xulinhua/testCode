# Hs Calib Isaac Calib Sim

独立 Isaac Sim **UI 扩展**（不改动 `hs_calib_suite` / 其它 Isaac 工程）。

> **UI 文案仅英文**（Isaac 字体对中文易乱码）。本文件为中文使用说明。

## 功能

- 单相机 + 可切换标定板（**实体几何**，不依赖贴图）
- 板型：
  - 平面：Chessboard / Circles / ChArUco / ArUco
  - **直角三面（3 faces）：** chessboard / **ChArUco（推荐，DICT_4X4_250，面 ID 0/100/200）** / ArUco·Tag
  - 三面为正方形板 + 约 1 格白边；`Squares X/Y` 表示**内角点**（ChArUco 方格 = n+1）
- Play 后：距离与仰角振荡、方位角旋转；相机朝向板心（含轻微 look-at 漂移与 roll，增加姿态多样性）
- 无物理 / 无碰撞；地面为纯视觉网格
- ROS2：默认约 **30 FPS** 发布 `/calib_sim/camera/image_raw` 与 `camera_info`（分辨率默认 1280×720，可在面板改；灯光保持默认）

## 启动

```bash
cd /home/hs/testCode/simulation/src/isaac_calib_sim/hs.calib.isaac_calib_sim
chmod +x scripts/start_isaac.sh
./scripts/start_isaac.sh
```

面板：`Window → Hs Calib Isaac Calib Sim`

1. 选择板型 / 参数 → **Load scene**（视口对准标定板）
   - 三面标定请选 **Trihedral — ChArUco (3 faces)**，Squares 用内角点（默认 8→9×9 方格），字典固定 DICT_4X4_250
2. 可选：改板型 → **Apply board params**
3. **Start ROS stream** 或 Timeline **Play**
4. `hs_calib_suite` GUI 选 `trihedral_oneshot` / `trihedral_charuco` / `DICT_4X4_250`，订阅图像话题

场景应包含 `/World/CalibSim/Table`、`CalibBoard`、以及 `camera_link/.../camera`。

## 目录布局

```text
simulation/src/isaac_calib_sim/
  hs.calib.isaac_calib_sim/     # Kit 扩展根
    config/extension.toml
    hs/calib/isaac_calib_sim/   # Python 包
    scripts/start_isaac.sh
    data/textures/              # 可选遗留贴图（板面建模未使用）
```
