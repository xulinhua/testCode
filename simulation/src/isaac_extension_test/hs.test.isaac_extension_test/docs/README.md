# hs.test.isaac_extension_test

独立的 Isaac Sim 抓取场景数据采集扩展，与同仓库其他扩展无运行时依赖。

## 目录

```text
hs.test.isaac_extension_test/
  config/extension.toml
  data/raw_data/          # Load 使用的 USD/OBJ/贴图
  data/icon.png, preview.png
  data_log/               # Start 采集输出（运行时生成，已 gitignore）
  python/                 # 源码（构建时同步到 hs/test/...）
  hs/test/isaac_extension_test/   # Python 包路径
```

## 启用

```bash
isaacsim .../isaacsim.exp.full.kit \
  --ext-folder /path/to/simulation/src/isaac_extension_test \
  --enable hs.test.isaac_extension_test
```

菜单：**Hs Test Isaac Extension Test**

## 依赖

- Isaac Sim 5.0
- 扩展声明：`isaacsim.gui.components`、`isaacsim.core.api`
- Python：`open3d`（LxWriter 输出 PCD）
- 界面中文：系统需安装 CJK 字体，例如 `sudo apt install fonts-noto-cjk`（否则中文可能显示为 `?`）

## 使用

1. **Load** — 桌子 + 随机物体  
2. **Play** — 物理落稳  
3. **Start** — 写入 `data_log/<时间戳>/`（RGB、深度、语义分割、点云、`info.json`、`camera_params.json`）
4. **Clear data_log** — 删除 `data_log/` 下全部采集结果，完成后弹窗提示

## 相机 UI（Camera 折叠面板）

| 字段 | 默认 | 说明 |
|------|------|------|
| Image width / height | 640 / 480 | 分辨率 |
| Focal length (mm) | 24 | 焦距 |
| Horiz / Vert aperture (mm) | 20.955 / 15.2908 | 孔径，用于计算 fx/fy |
| Init cam X/Y/Z | 0 / 0 / 1 | Load 时 Replicator 相机初始位置 |
| Camera FPS | 30 | 记录到 `camera_params.json`（采集元数据） |
| Render subframes | 8 | 每帧 `orchestrator.step_async` 子帧数 |

每次采集会在输出目录生成 **`camera_params.json`**（内参、UI 配置、每帧外参位姿）。

## 资源维护

模型放在 `data/raw_data/<类别>/`，文件名与 `global_variables.class_names` 一致。  
缺少 USD 时从 zip 或 Asset Converter 导入后保存到对应目录即可。
