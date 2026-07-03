# IsaacsimExtGrasping 抓取场景数据采集插件说明文档

适用项目：`/home/hs/testCode/simulation/src/data_collect_isaac/IsaacsimExtGrasping`

适用环境：Isaac Sim 5.0.0，Python 3.11，conda 环境 `isaac_env`

## 1. 项目概述

### 1.1 插件用途

`IsaacsimExtGrasping` 是一个 Isaac Sim UI 扩展，用于在仿真环境中搭建桌面抓取数据采集场景。插件可以自动加载桌子、地面、灯光、相机和随机物体，并通过 Isaac Sim Replicator 采集多视角数据。

本插件主要用于生成抓取、识别、分割、点云处理等任务所需的仿真数据。

### 1.2 适用场景

适合以下任务：

- 桌面物体抓取场景构建。
- 多类别物体随机摆放。
- RGB 图像采集。
- 深度图采集。
- 语义分割和实例分割采集。
- 点云 NPY 和 PCD 文件导出。
- 物体世界位姿记录。

### 1.3 整体工作流程

整体流程如下：

```text
启动 Isaac Sim
  -> 加载 IsaacsimExtGrasping 扩展
  -> 打开 Ext_Grasing_Scene UI
  -> 设置物体数量、类别、生成范围
  -> 点击 Load 创建场景
  -> 物体物理下落并稳定
  -> 设置相机采样和采集帧数
  -> 点击 Start 开始采集
  -> 写入 data_log/<timestamp>/ 输出目录
```

## 2. 从零搭建 Isaac Sim 扩展

### 2.1 Isaac Sim 扩展基本组成

一个 Isaac Sim 扩展通常包含以下部分：

```text
extension_root/
  config/
    extension.toml
  python_module/
    __init__.py
    extension.py
```

其中：

- `config/extension.toml` 用于声明扩展的名称、版本、依赖、Python 模块入口。
- `__init__.py` 用于让 Python 能识别当前目录为模块。
- `extension.py` 通常定义一个继承 `omni.ext.IExt` 的类。
- `on_startup()` 是扩展加载时执行的入口。
- `on_shutdown()` 是扩展卸载时执行的清理入口。

本项目是在 Isaac Sim 扩展模板基础上开发的，扩展名称为 `IsaacsimExtGrasping`，UI 标题为 `Ext_Grasing_Scene`。

### 2.2 扩展加载机制

Isaac Sim 启动时可以通过参数指定扩展目录和启用扩展：

```bash
--ext-folder <扩展父目录>
--enable <扩展目录名>
```

本项目的扩展目录是：

```text
/home/hs/testCode/simulation/src/data_collect_isaac/IsaacsimExtGrasping
```

因此 `--ext-folder` 应该指向它的父目录：

```text
/home/hs/testCode/simulation/src/data_collect_isaac
```

`--enable` 应该填写扩展目录名：

```text
IsaacsimExtGrasping
```

Isaac Sim 加载扩展后，会读取：

```text
IsaacsimExtGrasping/config/extension.toml
```

然后根据 `[[python.module]]` 中配置的模块名导入 Python 模块：

```toml
[[python.module]]
name = "Ext_Grasing_Scene_python"
```

### 2.3 本插件的搭建思路

本插件的开发思路可以拆成三层：

第一层是 Isaac Sim 扩展层：

- 注册扩展。
- 创建菜单项。
- 创建 UI 窗口。
- 监听 Stage、Timeline、Physics 事件。

第二层是场景构建层：

- 创建地面。
- 创建灯光。
- 创建相机和 render product。
- 加载桌子。
- 随机加载物体。
- 给物体添加语义标签、刚体、质量和碰撞。

第三层是数据采集层：

- 根据 UI 参数生成随机相机位姿。
- 使用 Replicator 逐帧渲染。
- 使用自定义 `LxWriter` 写出 RGB、深度、分割、点云和 PCD。
- 保存当前物体位姿到 `info.json`。

## 3. 项目目录结构

### 3.1 根目录说明

项目目录：

```text
IsaacsimExtGrasping/
  config/
  data/
  data_log/
  docs/
  Ext_Grasing_Scene_python/
```

### 3.2 `config/` 配置目录

```text
config/
  extension.toml
```

`extension.toml` 是 Isaac Sim 扩展的核心配置文件。

### 3.3 `Ext_Grasing_Scene_python/` 代码目录

```text
Ext_Grasing_Scene_python/
  __init__.py
  extension.py
  ui_builder.py
  lx_writer.py
  util.py
  global_variables.py
  scenario.py
  layouts.py
```

该目录是插件的 Python 主体代码目录。

### 3.4 `data/` 资源目录

```text
data/
  icon.png
  preview.png
  objects/
  raw_data/
```

`raw_data/` 下存放真实参与加载的模型资源。

### 3.5 `data_log/` 输出目录

采集结果会输出到：

```text
IsaacsimExtGrasping/data_log/
```

每次点击 `Start`，会自动创建一个时间戳目录。

### 3.6 `docs/` 文档目录

```text
docs/
  README.md
  CHANGELOG.md
```

这是扩展模板自带的文档目录。

## 4. 配置文件说明

### 4.1 `extension.toml` 文件作用

文件位置：

```text
IsaacsimExtGrasping/config/extension.toml
```

该文件告诉 Isaac Sim：

- 这个扩展是否可热重载。
- 扩展的标题、版本、描述。
- 扩展需要哪些依赖。
- Python 模块入口在哪里。

### 4.2 `[core]` 配置说明

当前配置：

```toml
[core]
reloadable = true
order = 0
```

字段说明：

- `reloadable = true`：表示扩展支持热重载。开发时修改代码后可以在 Extension Manager 中重载。
- `order = 0`：表示扩展加载顺序。数值越小越早加载。

### 4.3 `[package]` 配置说明

当前配置中包含：

```toml
[package]
version = "1.0.1"
category = "Simulation"
title = "Ext_Grasing_Scene"
description = "Grasping Scene Data Capture"
authors = ["NVIDIA"]
changelog = "docs/CHANGELOG.md"
readme = "docs/README.md"
preview_image = "data/preview.png"
icon = "data/icon.png"
```

主要字段：

- `version`：扩展版本。
- `category`：在 Extension Manager 中显示的分类。
- `title`：扩展 UI 和菜单标题。
- `description`：扩展说明。
- `changelog`：更新日志路径。
- `readme`：说明文档路径。
- `preview_image` 和 `icon`：扩展显示图标。

### 4.4 `[dependencies]` 依赖说明

当前依赖：

```toml
[dependencies]
"omni.kit.uiapp" = {}
"isaacsim.gui.components" = {}
"isaacsim.core.api" = {}
```

含义：

- `omni.kit.uiapp`：提供 Kit UI 应用基础能力。
- `isaacsim.gui.components`：提供 Isaac Sim UI 组件，例如按钮、输入框、折叠面板等。
- `isaacsim.core.api`：提供 World、Scene、Prim、物理世界等 Isaac Sim 核心 API。

代码中还使用了 Replicator、USD、PhysX 等模块。这些模块由 Isaac Sim 环境提供。

### 4.5 `[[python.module]]` 入口模块说明

当前配置：

```toml
[[python.module]]
name = "Ext_Grasing_Scene_python"
```

Isaac Sim 会导入：

```python
import Ext_Grasing_Scene_python
```

因此该目录必须包含：

```text
Ext_Grasing_Scene_python/__init__.py
```

## 5. Python 代码结构说明

### 5.1 `__init__.py` 模块入口

文件位置：

```text
Ext_Grasing_Scene_python/__init__.py
```

作用：

- 声明当前目录是 Python 包。
- 导入扩展类。

典型逻辑：

```python
from .extension import *
```

Isaac Sim 加载 `Ext_Grasing_Scene_python` 模块后，会通过这里进一步导入 `extension.py`。

### 5.2 `extension.py` 扩展生命周期逻辑

`extension.py` 是 Isaac Sim 扩展生命周期入口，核心类是：

```python
class Extension(omni.ext.IExt):
```

主要函数：

- `on_startup(self, ext_id)`：扩展启动时执行。
- `on_shutdown(self)`：扩展关闭或卸载时执行。
- `_menu_callback(self)`：点击菜单时打开或关闭窗口。
- `_build_ui(self)`：构建窗口内容。
- `_on_timeline_event(self, event)`：监听播放、暂停、停止事件。
- `_on_physics_step(self, step)`：监听物理步进。
- `_on_stage_event(self, event)`：监听 Stage 打开、关闭事件。

核心逻辑：

```text
on_startup()
  -> 创建 ScrollingWindow
  -> 注册菜单项
  -> 实例化 UIBuilder
  -> 准备 timeline、stage、physics 事件接口
```

### 5.3 `ui_builder.py` UI 和主业务逻辑

`ui_builder.py` 是本插件最重要的文件，主要负责：

- 创建 UI 控件。
- 读取 UI 参数。
- 加载和清理场景。
- 生成桌子和随机物体。
- 添加物理属性。
- 添加语义标签。
- 创建相机和 render product。
- 调用采集逻辑。
- 写出 `info.json`。

核心类：

```python
class UIBuilder:
```

核心函数：

- `build_ui()`：创建 UI。
- `load_world()`：加载场景。
- `apply_collision_to_mesh_children()`：给 mesh 子节点添加碰撞。
- `set_obj_position()`：设置 Prim 位置。
- `get_current_item_info()`：读取当前物体位姿。
- `collect_data()`：执行采集。
- `generate_frames()`：逐帧移动相机并触发采集。
- `collection_on_button_click()`：Start 按钮回调。

### 5.4 `lx_writer.py` 数据写入逻辑

`lx_writer.py` 定义自定义 Replicator Writer：

```python
class LxWriter(Writer):
```

作用：

- 启用需要的 annotator。
- 处理 Replicator 每帧输出的数据。
- 写出 RGB、深度、分割、点云等文件。
- 使用 Open3D 将点云保存为 PCD。

文件末尾通过：

```python
WriterRegistry.register(LxWriter)
```

注册 writer，因此在采集时可以通过：

```python
rep.WriterRegistry.get("LxWriter")
```

获取并使用。

### 5.5 `util.py` 相机位姿生成工具

`util.py` 中主要函数：

```python
generate_camera_pose(cylinder_radius, cylinder_height, upper_cylinder_height, obj_xy, obj_radius)
```

作用：

- 在圆柱区域内随机生成相机位置。
- 在物体附近随机生成目标点。
- 计算相机朝向目标点的旋转矩阵。
- 输出相机位置、目标点、四元数和欧拉角。

### 5.6 `global_variables.py` 全局配置变量

该文件定义全局变量：

```python
EXTENSION_TITLE = "Ext_Grasing_Scene"
EXTENSION_DESCRIPTION = "Grasping Scene Data Capture"
num_classes = 4
class_names = ['bwb', 'sqm', 'yida', 'dingshuji']
```

其中：

- `class_names` 决定 UI 中显示哪些类别。
- `num_classes` 决定类别数量。
- 类别名称需要和 `data/raw_data/` 下的目录名对应。

### 5.7 `scenario.py` 模板场景逻辑

`scenario.py` 是模板生成时自带的示例场景逻辑，当前项目主要业务并不依赖它。

它展示了如何用 Isaac Sim API 控制 articulation 和物体运动。

### 5.8 `layouts.py` UI 辅助布局

`layouts.py` 是 UI 布局辅助文件，目前不是核心业务逻辑。

## 6. 插件启动逻辑

### 6.1 Isaac Sim 加载扩展

启动命令中包含：

```bash
--ext-folder /home/hs/testCode/simulation/src/data_collect_isaac
--enable IsaacsimExtGrasping
```

Isaac Sim 会在 `--ext-folder` 中查找 `IsaacsimExtGrasping/config/extension.toml`。

### 6.2 Python 模块导入流程

导入链路：

```text
Ext_Grasing_Scene_python
  -> __init__.py
  -> extension.py
  -> ui_builder.py
  -> lx_writer.py
  -> util.py / global_variables.py
```

如果其中任何一个 import 失败，扩展都会启动失败。

例如之前出现过：

```text
ModuleNotFoundError: No module named 'open3d'
```

原因是 `ui_builder.py` 导入 `lx_writer.py`，而 `lx_writer.py` 中导入了 `open3d`。

### 6.3 `Extension.on_startup()` 执行流程

`on_startup()` 主要完成：

```text
保存 ext_id
获取 USD context
创建 ScrollingWindow
注册菜单 action
添加菜单项
实例化 UIBuilder
获取 PhysX、timeline 接口
```

### 6.4 菜单项和窗口创建

扩展会创建一个标题为：

```text
Ext_Grasing_Scene
```

的窗口，并注册到 Isaac Sim 菜单中。

点击菜单后执行 `_menu_callback()`：

```python
self._window.visible = not self._window.visible
self.ui_builder.on_menu_callback()
```

### 6.5 `UIBuilder` 初始化

`UIBuilder.__init__()` 中初始化：

- UI frame 列表。
- timeline 接口。
- stage。
- 按钮和输入框字典。
- 相机、render product、地面、灯光、物体列表。
- 类别列表。
- `World(physics_dt=1.0 / 60.0)`。
- writer。

## 7. UI 构建逻辑

### 7.1 UI 总体结构

UI 由 `build_ui()` 创建，主要包含一个折叠面板：

```python
selection_controls_frame = CollapsableFrame("Select Plane", collapsed=False)
```

面板中放置：

- 物体数量输入框。
- 类别复选框。
- 物体生成范围输入框。
- Load 按钮。
- 采集帧数输入框。
- 相机采样参数输入框。
- Start 按钮。

### 7.2 物体数量配置

UI 字段：

```text
Number of objects
```

默认值：

```text
10
```

代码读取：

```python
item_count = self._line_edit['num_obj'].get_value_as_int()
```

它决定生成几个随机物体：

```text
/World/obj_0
/World/obj_1
...
```

### 7.3 类别复选框配置

类别来自：

```python
class_names = ['bwb', 'sqm', 'yida', 'dingshuji']
```

UI 中每个类别对应一个 checkbox。勾选状态会更新：

```python
self._classes_select
```

物体随机生成时，只会从 `self._classes_select` 对应的目录中选择 USD 文件。

### 7.4 物体生成范围配置

UI 字段：

```text
Spawn x min
Spawn x max
Spawn y min
Spawn y max
```

这些参数决定物体初始生成位置的 XY 随机范围。

代码逻辑：

```python
x = random.uniform(x_min, x_max)
y = random.uniform(y_min, y_max)
z = 2
self.set_obj_position(prim_path, (x, y, z))
```

物体初始放在 `z=2`，后续依靠物理下落。

### 7.5 `Load` 按钮逻辑

Load 按钮绑定：

```python
on_clicked_fn = self.load_world
```

点击后执行：

```text
load_world()
  -> 创建地面
  -> 创建灯光
  -> 创建相机
  -> 创建桌子
  -> 随机创建物体
  -> 添加语义和物理属性
  -> world.reset()
```

### 7.6 采集帧数配置

UI 字段：

```text
Number of shots
```

默认值：

```text
20
```

代码读取：

```python
shots_count = self._line_edit['num_of_shots'].get_value_as_int()
```

该值决定采集多少帧。

### 7.7 相机采样参数配置

UI 字段：

```text
cylinder_radius
cylinder_height
upper_cylinder_height
obj_x
obj_y
obj_radius
```

这些参数传给：

```python
generate_camera_pose()
```

用于随机生成每一帧的相机位姿。

### 7.8 `Start` 按钮逻辑

Start 按钮绑定：

```python
on_clicked_fn = self.collection_on_button_click
```

点击后会异步执行：

```text
collection_on_button_click()
  -> _run_collection()
  -> collect_data()
```

代码中使用 `_is_collecting` 防止重复点击。

## 8. 场景加载逻辑

### 8.1 `load_world()` 总体流程

`load_world()` 是创建场景的核心函数。

总体流程：

```text
清理 UI 包装元素
创建默认地面
给地面添加语义标签
获取当前 stage
创建灯光
创建相机和 render product
加载桌子
读取 UI 物体数量和生成范围
清理旧物体
收集类别 USD 路径
随机生成物体
添加语义、刚体、质量、碰撞
world.reset()
```

### 8.2 创建地面

地面创建代码：

```python
self.ground = self.world.scene.add_default_ground_plane()
```

然后添加语义标签：

```python
add_update_semantics(self.ground.prim, "ground", "select_classes")
```

这里的语义类型是：

```text
select_classes
```

### 8.3 创建灯光

灯光路径：

```text
/World/DefaultLight
```

灯光类型：

```python
UsdLux.DistantLight
```

设置内容：

- intensity：3000
- color：略偏暖色
- angle：0.53
- rotation：由代码计算欧拉角后写入

### 8.4 创建相机和 Render Product

相机创建：

```python
self.camera = rep.create.camera(position=(0, 0, 1))
```

Render Product 创建：

```python
self.render_product = rep.create.render_product(self.camera, (640, 480))
```

当前采集分辨率是：

```text
640 x 480
```

如需修改分辨率，可改这里。

### 8.5 加载桌子模型 `obj_mz`

桌子 Prim 路径：

```text
/World/obj_mz
```

当前加载资源：

```text
data/raw_data/mz/mz.usd
```

代码使用相对插件目录计算绝对路径：

```python
parent_folder = os.path.split(__file__)[0]
table_usd_path = os.path.abspath(os.path.join(parent_folder, "..", "data", "raw_data", "mz", "mz.usd"))
```

桌子加载后添加语义：

```python
add_update_semantics(added_obj, "mz", "select_classes")
```

桌子缩放：

```python
xform.AddScaleOp().Set(Gf.Vec3d(2.5, 2.5, 2.5))
```

### 8.6 随机加载物体 `obj_0 ~ obj_N`

物体资源从以下目录中选择：

```text
data/raw_data/bwb/
data/raw_data/sqm/
data/raw_data/yida/
data/raw_data/dingshuji/
```

代码会遍历所选类别目录，收集 `.usd`、`.usda`、`.usdc` 文件。

生成物体 Prim 路径：

```python
prim_path = f"/World/obj_{i}"
```

加载资源：

```python
added_obj = add_reference_to_stage(usd_path=obj_path, prim_path=prim_path)
```

### 8.7 添加语义标签

物体的语义标签来自 USD 文件名：

```python
semantic_label = obj_path.split("/")[-1].rsplit('.', 1)[0]
```

例如：

```text
data/raw_data/sqm/sqm.usd -> semantic_label = "sqm"
```

如果该标签在当前选择类别中，则添加：

```python
add_update_semantics(added_obj, semantic_label, "select_classes")
```

### 8.8 添加刚体和质量属性

对每个物体应用刚体：

```python
UsdPhysics.RigidBodyAPI.Apply(obj_prim)
```

应用质量：

```python
mass_api = UsdPhysics.MassAPI.Apply(obj_prim)
mass_attr.Set(0.1)
```

桌子的质量设置为：

```python
mass_attr.Set(10)
```

### 8.9 添加碰撞属性

碰撞添加函数：

```python
apply_collision_to_mesh_children(parent_prim)
```

该函数递归遍历 parent prim 下所有 mesh/cube/sphere/cylinder/capsule 子节点，并应用：

```python
UsdPhysics.CollisionAPI.Apply(child_prim)
UsdPhysics.MeshCollisionAPI.Apply(child_prim)
meshcollsion_api.GetApproximationAttr().Set(meshOptions[2])
```

`meshOptions[2]` 对应：

```text
convexHull
```

### 8.10 重置物理世界

场景构建完成后执行：

```python
self.world.reset()
```

这会让 Isaac Sim 初始化刚体和碰撞状态。

## 9. 资源组织与类别管理

### 9.1 `data/raw_data/` 目录结构

当前目录：

```text
data/raw_data/
  bwb/
  sqm/
  yida/
  dingshuji/
  mz/
```

其中：

- `bwb`、`sqm`、`yida`、`dingshuji` 是随机物体类别。
- `mz` 是桌子资源。

### 9.2 物体类别目录说明

每个类别目录中通常包含：

```text
<class_name>.usd
<class_name>.obj
material.mtl
textures/
```

插件目前主要加载 `.usd` 文件。

### 9.3 USD、OBJ、贴图文件关系

USD 文件可能引用 OBJ、材质和贴图。为了项目可迁移，USD 中应尽量使用相对路径，不要使用旧机器的绝对路径。

正确示例：

```text
textures/material_0.jpeg
```

不推荐：

```text
/home/yz/ycl/exts/IsaacsimExtGrasping/...
```

### 9.4 `class_names` 和类别目录对应关系

`global_variables.py` 中：

```python
class_names = ['bwb', 'sqm', 'yida', 'dingshuji']
```

应与目录名对应：

```text
data/raw_data/bwb
data/raw_data/sqm
data/raw_data/yida
data/raw_data/dingshuji
```

### 9.5 新增类别的文件要求

新增类别时，建议满足：

- 目录名等于类别名。
- USD 文件名等于类别名。
- USD 内部子 Prim 名称包含类别名。
- 贴图路径使用相对路径。

## 10. 物理与碰撞逻辑

### 10.1 Isaac Sim 物理世界

插件创建 World：

```python
self.world = World(physics_dt=1.0 / 60.0)
```

物理步长为：

```text
1 / 60 秒
```

### 10.2 `RigidBodyAPI` 作用

`RigidBodyAPI` 让一个 Prim 参与刚体物理仿真。

如果物体没有刚体属性，它不会正常受重力和碰撞影响。

### 10.3 `MassAPI` 作用

`MassAPI` 用于设置质量。

质量影响物理仿真的运动、碰撞响应和稳定性。

### 10.4 `CollisionAPI` 作用

`CollisionAPI` 让几何体具备碰撞能力。

只给外层 Prim 添加碰撞通常不够，因为真实 mesh 可能在子 Prim 下，所以代码递归处理所有子节点。

### 10.5 `MeshCollisionAPI` 和 `convexHull`

`MeshCollisionAPI` 用于给 mesh 指定碰撞近似方式。

当前设置为：

```text
convexHull
```

这会用凸包近似 mesh 形状，通常比完整三角网格碰撞更稳定、更快。

### 10.6 物体下落与稳定过程

随机物体初始设置：

```python
z = 2
```

也就是说物体会从高处掉落到桌面上。

建议操作：

```text
Load 后点击 Play
等待物体稳定
停止或保持仿真
再点击 Start 采集
```

## 11. 相机位姿生成逻辑

### 11.1 相机采样区域定义

相机位置在一个圆柱体区域中随机生成。

圆柱体参数：

- 半径：`cylinder_radius`
- 基础高度：`cylinder_height`
- 额外高度：`upper_cylinder_height`

### 11.2 圆柱体随机采样参数

相机 XY 坐标由：

```python
random_point_in_circle(cylinder_radius)
```

随机生成。

相机 Z 坐标由：

```python
np.random.uniform(cylinder_height, cylinder_height + upper_cylinder_height)
```

随机生成。

### 11.3 目标点随机生成

目标点在物体附近圆形区域中随机生成：

```python
target_xy = random_point_in_circle(obj_radius)
target_xy = target_xy + obj_xy
target_point = np.array([target_xy[0], target_xy[1], 0.0])
```

### 11.4 相机朝向计算

相机方向向量：

```python
direction = target_point - camera_position
direction_normalized = direction / np.linalg.norm(direction)
```

代码构造旋转矩阵，使相机朝向目标点。

### 11.5 四元数和欧拉角输出

`generate_camera_pose()` 返回：

```python
{
    "camera_position": camera_position,
    "target_point": target_point,
    "quaternion": quaternion,
    "euler_angles": euler_angles
}
```

采集时实际使用欧拉角，并做了一个偏移：

```python
rot = pose['euler_angles'].copy()
rot[1] -= 90
rep.modify.pose(position=pose['camera_position'], rotation=rot)
```

## 12. 数据采集逻辑

### 12.1 点击 `Start` 后的执行链路

执行链路：

```text
Start 按钮
  -> collection_on_button_click()
  -> asyncio.ensure_future(self._run_collection())
  -> _run_collection()
  -> collect_data()
  -> generate_frames()
```

### 12.2 `collection_on_button_click()`

该函数用于防止重复采集：

```python
if getattr(self, "_is_collecting", False):
    print("正在采集中，请勿重复点击")
    return
```

如果当前没有采集，则设置：

```python
self._is_collecting = True
```

然后异步执行 `_run_collection()`。

### 12.3 `_run_collection()`

该函数保证采集结束后恢复状态：

```python
try:
    await self.collect_data()
finally:
    self._is_collecting = False
```

### 12.4 `collect_data()`

`collect_data()` 是采集主函数。

主要步骤：

```text
检查是否有物体
检查 work_dir
读取采集帧数
读取当前物体位姿
创建时间戳输出目录
写 info.json
初始化 LxWriter
绑定 render product
读取相机采样参数
调用 generate_frames()
detach writer
```

### 12.5 输出目录创建

输出目录由时间戳生成：

```python
folder_name = time.strftime("%Y_%m_%d_%H_%M_%S")
abs_folder_path = self.collector_folder_prep(folder_name)
```

最终路径示例：

```text
IsaacsimExtGrasping/data_log/2026_05_14_09_44_05/
```

### 12.6 `info.json` 写入

写入内容包括：

```python
info['camera_info'] = camera_info
info['objects'] = current_info
```

`objects` 中记录每个随机物体的：

- 名称。
- 世界坐标 translation。
- 四元数 quat。

注意：该列表不包含桌子 `obj_mz`。

### 12.7 Writer 初始化和绑定

代码：

```python
self.writter = rep.WriterRegistry.get("LxWriter")
self.writter.initialize(
    output_dir=abs_folder_path,
    pointcloud=True,
    rgb=True,
    distance_to_camera=True,
    semantic_segmentation=True,
    semantic_types=["select_classes"]
)
self.writter.attach([self.render_product])
```

其中 `semantic_types=["select_classes"]` 必须和前面添加语义标签时的类型一致。

### 12.8 `generate_frames()` 逐帧采集

每一帧都会：

```text
生成随机相机位姿
修改相机 pose
调用 rep.orchestrator.step_async()
Writer 写出当前帧数据
```

关键调用：

```python
await rep.orchestrator.step_async(
    rt_subframes=8,
    delta_time=None,
    pause_timeline=False
)
```

## 13. Writer 写数据逻辑

### 13.1 `LxWriter` 注册机制

`lx_writer.py` 继承 Isaac Replicator 的 `Writer`：

```python
class LxWriter(Writer):
```

文件末尾注册：

```python
WriterRegistry.register(LxWriter)
```

注册后才能通过：

```python
rep.WriterRegistry.get("LxWriter")
```

获取。

### 13.2 Annotator 配置

`LxWriter.__init__()` 根据参数启用 annotator：

- `rgb`
- `distance_to_camera`
- `semantic_segmentation`
- `instance_id_segmentation`
- `pointcloud`
- 其他可选 annotator

每个 annotator 会在每一帧返回对应类型数据。

### 13.3 RGB 图像写入

函数：

```python
_write_rgb()
```

输出文件：

```text
rgb_0000.png
rgb_0001.png
...
```

### 13.4 深度数据写入

函数：

```python
_write_distance_to_camera()
```

输出：

```text
distance_to_camera_0000.npy
```

该文件保存的是 NumPy 数组。

### 13.5 语义分割写入

函数：

```python
_write_semantic_segmentation()
```

输出：

```text
semantic_segmentation_0000.png
semantic_segmentation_labels_0000.json
```

JSON 文件记录 ID 到标签的映射。

### 13.6 实例分割写入

函数：

```python
_write_instance_id_segmentation()
```

输出：

```text
instance_id_segmentation_0000.png
instance_id_segmentation_mapping_0000.json
```

### 13.7 点云 NPY 写入

函数：

```python
_write_pointcloud()
```

输出：

```text
pointcloud_0000.npy
pointcloud_rgb_0000.npy
pointcloud_normals_0000.npy
pointcloud_semantic_0000.npy
pointcloud_instance_0000.npy
```

### 13.8 PCD 文件写入

函数：

```python
lxSavePCD()
```

使用 Open3D：

```python
pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(...)
pc.colors = o3d.utility.Vector3dVector(...)
o3d.io.write_point_cloud(full_path, pc)
```

输出：

```text
lx_data_0000.pcd
```

## 14. 输出数据结构说明

### 14.1 输出目录命名规则

每次采集一个目录：

```text
data_log/YYYY_MM_DD_HH_MM_SS/
```

示例：

```text
data_log/2026_05_14_09_44_05/
```

### 14.2 `info.json` 内容说明

`info.json` 结构：

```json
{
  "camera_info": [[...], [...], [...]],
  "objects": [
    {
      "name": "sqm",
      "translation": [x, y, z],
      "quat": [w, x, y, z]
    }
  ]
}
```

注意：

- `translation` 是世界坐标。
- `quat` 是世界旋转四元数。
- 记录的是随机生成物体，不记录桌子。

### 14.3 `rgb_*.png`

RGB 彩色图像。

命名：

```text
rgb_0000.png
rgb_0001.png
...
```

### 14.4 `distance_to_camera_*.npy`

相机深度数据，NumPy 格式。

### 14.5 `semantic_segmentation_*.png`

语义分割图像。

### 14.6 `semantic_segmentation_labels_*.json`

语义 ID 和标签映射文件。

### 14.7 `instance_id_segmentation_*.png`

实例 ID 分割图。

### 14.8 `instance_id_segmentation_mapping_*.json`

实例 ID 映射文件。

### 14.9 `pointcloud_*.npy`

点云 XYZ 坐标。

### 14.10 `pointcloud_rgb_*.npy`

点云每个点对应的 RGB 颜色。

### 14.11 `pointcloud_semantic_*.npy`

点云每个点对应的语义类别。

### 14.12 `pointcloud_instance_*.npy`

点云每个点对应的实例 ID。

### 14.13 `lx_data_*.pcd`

PCD 格式点云文件，可使用 Open3D、PCL、CloudCompare 等工具打开。

## 15. Stage 结构说明

### 15.1 `/World` 根节点

插件创建的主要对象都位于：

```text
/World
```

### 15.2 `/World/obj_mz` 桌子模型

`obj_mz` 是桌子。

它不属于随机物体列表，不会写入 `info.json` 的 `objects`。

### 15.3 `/World/obj_0 ~ obj_N` 随机物体容器

`obj_0` 到 `obj_N` 是随机生成物体的外层容器。

例如设置 `Number of objects = 10` 时，会生成：

```text
/World/obj_0
/World/obj_1
...
/World/obj_9
```

### 15.4 子 Prim 和真实 Mesh

真实模型通常在 `obj_i` 的子节点下。

也就是说：

```text
/World/obj_0
```

可能只是外层引用容器，真实 mesh 可能在：

```text
/World/obj_0/sqm
```

或类似路径中。

### 15.5 语义标签所在位置

代码对加载返回的 prim 添加语义标签：

```python
add_update_semantics(added_obj, semantic_label, "select_classes")
```

点云和分割数据会根据语义标签生成。

### 15.6 Stage 面板中节点与采集数据的关系

Stage 面板中看到的外层 `obj_i` 不一定等于 `info.json` 中记录位姿的实际节点。

`get_current_item_info()` 会在 `obj_i` 的子节点中查找名称包含类别名的 Prim，然后计算该子 Prim 的 world transform。

## 16. 坐标系与位姿说明

### 16.1 Isaac Sim 世界坐标系

Isaac Sim 使用三维世界坐标系。

本插件中的物体、桌子、相机都位于同一个世界坐标系中。

### 16.2 Z-up 和单位米

Isaac Sim 默认使用 Z-up：

```text
X/Y：水平平面
Z：高度方向
```

单位是米。

### 16.3 外层 Prim 坐标

`/World/obj_0` 这类外层 Prim 的坐标通常是引用容器坐标。

它不一定代表真实几何中心。

### 16.4 子 Prim 世界坐标

真实 mesh 子节点可能有自己的局部 transform。

`info.json` 中更接近记录的是子 Prim 的世界坐标。

### 16.5 模型原点和几何中心差异

模型原点由建模或导出时决定，可能在：

- 几何中心。
- 底部中心。
- 角落。
- 扫描模型的任意参考位置。

所以坐标和肉眼看到的“物体中心”可能不一致。

### 16.6 相机坐标与世界坐标差异

采集图像和点云由相机生成。

相机坐标系和世界坐标系不同，需要通过相机外参转换。

当前 `info.json` 保存了相机内参，但没有完整逐帧外参记录。

### 16.7 点云坐标解释

Replicator pointcloud annotator 输出的点云坐标需要结合 Isaac Sim annotator 的定义理解。使用时要确认它是相机坐标还是世界坐标输出。

如果要和 `info.json` 中的物体世界坐标对齐，需要明确坐标系转换关系。

### 16.8 `info.json` 中物体位姿解释

`info.json` 中每个物体：

```json
{
  "name": "...",
  "translation": [...],
  "quat": [...]
}
```

表示插件找到的物体子 Prim 的世界位姿。

如果和 Stage 面板中外层 `obj_i` 不一致，应展开 `obj_i` 查看子 Prim。

## 17. 插件启动和使用流程

### 17.1 启动 Isaac Sim

当前机器推荐命令：

```bash
/home/hs/anaconda3/envs/isaac_env/bin/isaacsim \
  /home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit \
  --ext-folder /home/hs/testCode/simulation/src/data_collect_isaac \
  --enable IsaacsimExtGrasping
```

### 17.2 加载扩展

启动命令中已经包含：

```text
--enable IsaacsimExtGrasping
```

因此 Isaac Sim 启动时会自动加载扩展。

### 17.3 打开插件窗口

在菜单中点击：

```text
Ext_Grasing_Scene
```

打开插件面板。

### 17.4 设置物体参数

设置：

- `Number of objects`
- 类别复选框
- `Spawn x min/max`
- `Spawn y min/max`

### 17.5 点击 `Load` 加载场景

点击 `Load` 后，插件生成桌子和随机物体。

### 17.6 等待物理稳定

点击 Isaac Sim 的 Play，让物体落到桌面并稳定。

### 17.7 设置采集参数

设置：

- `Number of shots`
- `cylinder_radius`
- `cylinder_height`
- `upper_cylinder_height`
- `obj_x`
- `obj_y`
- `obj_radius`

### 17.8 点击 `Start` 开始采集

点击 `Start` 后，插件开始逐帧移动相机并采集数据。

### 17.9 查看采集结果

输出目录：

```text
/home/hs/testCode/simulation/src/data_collect_isaac/IsaacsimExtGrasping/data_log/
```

## 18. 当前机器运行环境

本项目当前验证和日常使用的 Isaac Sim 是安装在 conda 环境中的 Isaac Sim 5.0，不是 `Downloads` 目录下的 Isaac Sim standalone 解压包。

环境路径记录如下：

| 项 | 记录值 |
| --- | --- |
| 实际 Isaac Sim 环境根目录 | `/home/hs/anaconda3/envs/isaac_env` |
| Isaac Sim 启动入口 | `/home/hs/anaconda3/envs/isaac_env/bin/isaacsim` |
| Python 解释器 | `/home/hs/anaconda3/envs/isaac_env/bin/python` |
| Isaac Sim app 配置文件 | `/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit` |
| Isaac Sim Python 包目录 | `/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim` |
| 说明 | `/home/hs/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64` 是下载/解压包，不作为本项目实际安装和运行路径。 |

### 18.1 Isaac Sim 版本

当前实际使用版本：

```text
Isaac Sim 5.0.0
```

### 18.2 Isaac Sim 安装位置

实际运行环境位于：

```text
/home/hs/anaconda3/envs/isaac_env
```

### 18.3 conda 环境路径

Python 环境：

```text
/home/hs/anaconda3/envs/isaac_env/bin/python
```

Isaac Sim 启动入口：

```text
/home/hs/anaconda3/envs/isaac_env/bin/isaacsim
```

### 18.4 启动命令

见第 17.1 节。

### 18.5 Python 依赖版本

当前已验证：

```text
open3d 0.19.0
scipy 1.15.3
psutil 5.9.8
```

### 18.6 `open3d` 安装说明

安装命令：

```bash
/home/hs/anaconda3/envs/isaac_env/bin/python -m pip install open3d
```

### 18.7 `psutil` 版本注意事项

Isaac Sim 5.0 依赖：

```text
psutil==5.9.8
```

如果被 pip 升级，需要降回：

```bash
/home/hs/anaconda3/envs/isaac_env/bin/python -m pip install psutil==5.9.8
```

## 19. 本地路径与资源修复说明

### 19.1 旧绝对路径问题

项目曾存在旧机器路径：

```text
/home/yz/ycl/exts/IsaacsimExtGrasping/...
/home/ubuntu/Desktop/cl/Proj/exts/IsaacsimExtGrasping/...
```

这些路径在当前机器上不存在，会导致 USD 或贴图加载失败。

### 19.2 桌子模型路径修复

代码中桌子路径已改为基于当前文件位置计算：

```python
parent_folder = os.path.split(__file__)[0]
table_usd_path = os.path.abspath(os.path.join(parent_folder, "..", "data", "raw_data", "mz", "mz.usd"))
```

### 19.3 `mz.usd` 贴图路径修复

`mz.usd` 中贴图路径已改为相对路径：

```text
textures/GenericClassicTable001_color.png
textures/GenericClassicTable001_normal.png
textures/GenericClassicTable001_roughness.png
```

这样项目移动到其他机器后，只要目录结构不变，贴图仍可解析。

### 19.4 相对路径组织原则

建议资源引用使用相对路径：

```text
textures/xxx.png
../textures/xxx.png
```

避免使用：

```text
/home/用户名/...
```

### 19.5 移植到其他机器的注意事项

迁移时确认：

- Isaac Sim 版本兼容。
- Python 依赖已安装。
- `data/raw_data/` 资源完整。
- USD 内部引用是相对路径。
- 启动命令中的 `--ext-folder` 路径正确。

## 20. 常见问题与排查

### 20.1 扩展无法加载

检查 Isaac Sim 控制台错误。

常见原因：

- Python import 报错。
- 缺少依赖包。
- `extension.toml` 配置错误。
- Isaac Sim 版本不匹配。

### 20.2 缺少 `open3d`

错误：

```text
ModuleNotFoundError: No module named 'open3d'
```

解决：

```bash
/home/hs/anaconda3/envs/isaac_env/bin/python -m pip install open3d
```

### 20.3 `psutil` 版本冲突

如果安装依赖后 Isaac Sim 报 `psutil` 版本问题，执行：

```bash
/home/hs/anaconda3/envs/isaac_env/bin/python -m pip install psutil==5.9.8
```

### 20.4 桌子贴图找不到

检查日志中是否仍出现：

```text
/home/yz/ycl/exts/...
```

如果出现，说明 USD 内部仍有旧路径。

### 20.5 点击 `Load` 后没有物体

检查：

- 是否勾选了类别。
- `data/raw_data/<class>/` 下是否存在 USD 文件。
- 控制台是否提示未找到 USD 模型。

### 20.6 点击 `Start` 后没有数据

检查：

- 是否已经点击 `Load`。
- `self.object` 是否为空。
- `data_log/` 是否有新时间戳目录。
- 控制台是否报错。

### 20.7 坐标和实物不对应

可能原因：

- 查看的是外层 `obj_i`，不是子 Prim。
- 模型原点不在几何中心。
- 物理仿真后位姿发生变化。
- 世界坐标和相机坐标混用。

### 20.8 分割标签不正确

检查：

- 语义类型是否为 `select_classes`。
- 类别名是否和目录名、USD 文件名一致。
- 是否在 `class_names` 中添加了对应类别。

### 20.9 点云为空或异常

检查：

- 相机是否对准物体。
- render product 是否创建成功。
- pointcloud annotator 是否启用。
- 物体是否有语义标签。

## 21. 新增物体类别

### 21.1 新建类别目录

例如新增类别 `apple`：

```text
data/raw_data/apple/
```

### 21.2 放置 USD 和贴图资源

建议：

```text
data/raw_data/apple/apple.usd
data/raw_data/apple/textures/
```

### 21.3 修改 `global_variables.py`

修改：

```python
num_classes = 5
class_names = ['bwb', 'sqm', 'yida', 'dingshuji', 'apple']
```

### 21.4 检查文件名和语义标签

当前语义标签来自 USD 文件名。

如果文件名是：

```text
apple.usd
```

则语义标签为：

```text
apple
```

### 21.5 在 UI 中显示新类别

UI 类别复选框由 `class_names` 自动生成。

修改后重载扩展，新类别会出现在 UI 中。

### 21.6 验证采集输出

采集后检查：

- `semantic_segmentation_labels_*.json`
- `pointcloud_semantic_*.npy`
- `info.json`

确认新类别出现。

## 22. 修改采集内容

### 22.1 修改相机分辨率

位置：

```python
self.render_product = rep.create.render_product(self.camera, (640, 480))
```

修改为：

```python
self.render_product = rep.create.render_product(self.camera, (1024, 1024))
```

### 22.2 修改默认 UI 参数

默认值在 `build_ui()` 中设置，例如：

```python
"default_val": "10"
```

### 22.3 修改采集帧数

UI 默认采集帧数：

```python
"label": "Number of shots",
"default_val": "20"
```

### 22.4 修改 Writer 输出内容

位置：

```python
self.writter.initialize(...)
```

可以控制是否启用：

- `rgb`
- `pointcloud`
- `distance_to_camera`
- `semantic_segmentation`
- `instance_id_segmentation`

### 22.5 添加新的 Annotator

在 `LxWriter.__init__()` 中增加 annotator，并在 `write()` 中处理对应输出。

### 22.6 修改 PCD 输出逻辑

PCD 写入函数：

```python
lxSavePCD()
```

如果需要改变点云颜色、过滤无效点、改坐标系，可以在这里处理。

## 23. 修改记录

### 23.1 路径修复记录

已将桌子模型加载路径改为项目内相对计算路径。

### 23.2 依赖安装记录

已在 `isaac_env` 中安装：

```text
open3d 0.19.0
```

并保持：

```text
psutil 5.9.8
```

### 23.3 数据采集验证记录

已验证采集数据输出到：

```text
IsaacsimExtGrasping/data_log/<timestamp>/
```

示例：

```text
2026_05_14_09_44_05
```

### 23.4 后续维护建议

建议后续重点维护：

- 统一资源相对路径。
- 给每帧记录相机外参。
- 明确点云坐标系。
- 清理未使用模板代码。
- 将 UI 参数和采集参数配置化。
