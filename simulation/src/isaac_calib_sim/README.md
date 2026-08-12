# Isaac Calib Sim

独立的 Isaac Sim **标定板 UI 扩展**（不修改 `hs_calib_suite` 及其它 `isaac_*` 工程）。

> 面板与控件文案为 **English only**（避免 Isaac 字体对 CJK 乱码）；本 README 用中文说明用法。  
> 详细说明见 [`hs.calib.isaac_calib_sim/docs/README.md`](hs.calib.isaac_calib_sim/docs/README.md)。

## 功能概要

- 单相机 + 可切换标定板（实体几何建模，不依赖贴图）
- 平面：棋盘 / 圆点 / ChArUco / ArUco
- 直角三面：**正方形面板** + 四周约 1 格**白边**；开口朝上（chess / ChArUco / ArUco）
- 视觉场景：无 PhysX / 碰撞；相机轨道由 app 更新驱动
- ROS2 约 **30 FPS** 发布图像与 CameraInfo（默认分辨率不变）
- 话题命名空间默认 `calib_sim`，可在面板修改

## 启动

```bash
cd /home/hs/testCode/simulation/src/isaac_calib_sim/hs.calib.isaac_calib_sim
chmod +x scripts/start_isaac.sh
./scripts/start_isaac.sh
```

面板：`Window → Hs Calib Isaac Calib Sim`

1. 选择板型与参数 → **Load scene**
2. 可选：改参数 → **Apply board params**
3. **Start ROS stream** 或 Timeline **Play**
4. 在 `hs_calib_suite` GUI 订阅对应图像话题采集

## 目录

```text
simulation/src/isaac_calib_sim/
  hs.calib.isaac_calib_sim/     # Kit 扩展根目录
    config/extension.toml
    hs/calib/isaac_calib_sim/   # Python 包
    scripts/start_isaac.sh
    docs/README.md
```
