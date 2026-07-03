# Isaac Table Scene 扩展

桌面 + 长方体 + 俯视相机场景。Timeline **Play** 后通过 ROS2 发布 color、depth、PointCloud2。

## 启动

```bash
/home/hs/anaconda3/envs/isaac_env/bin/isaacsim \
  /home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit \
  --ext-folder /home/hs/testCode/simulation/src/isaac_table_scene \
  --enable hs.table.isaac_table_scene
```

菜单打开 **Hs Table Isaac Table Scene** 面板 → **Load scene** → **Play**。

## RViz

- **Fixed Frame** 设为 `map`（与默认 TF 父坐标系一致）
- Color: `/table_scene/camera/color/image_raw`，Encoding 选 `rgb8`
- Depth: `/table_scene/camera/depth/image_raw`

| 类型 | 话题 |
|------|------|
| Color | `/table_scene/camera/color/image_raw` |
| Depth | `/table_scene/camera/depth/image_raw` |
| PointCloud2 | `/table_scene/camera/depth/points` |

## 运行时 6D 位姿

- **Camera 6D Pose**：平移 X/Y/Z + Roll/Pitch/Yaw（度，世界系）
- **Cuboid 6D Pose**：同上
- 勾选 **Live apply** 后在 Play 期间改字段即实时生效
- **Apply poses now**：立即应用一次
- **Sync from Stage**：从 Stage 读回当前位姿到 UI

可选：将 `data/raw_data/mz/mz.usd` 放入扩展目录以使用真实桌子模型（否则使用程序生成的盒子桌子）。
