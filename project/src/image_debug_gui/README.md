# image_debug_gui

基于 Qt + C++17 + ROS2 的多窗口图像调试工具，支持彩色/深度图查看、像素值读取、同步查看、截图保存和 rosbag 录制。

## 1. 项目结构

```text
image_debug_gui/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── image_debug_gui/
│       ├── image_view_widget.hpp   # 图像控件：缩放、平移、像素取值、同步提示
│       └── main_window.hpp         # 主窗口：菜单、多窗口布局、工具栏、话题与录制逻辑
└── src/
    ├── main.cpp                    # 程序入口（Qt + rclcpp 初始化）
    ├── main_window.cpp             # 主界面与业务逻辑
    └── image_view_widget.cpp       # 图像交互与绘制逻辑
```

## 2. 主要功能

- 多窗口图像查看（1~4 路）
- 布局规则：
  - 1 路：单窗口
  - 2 路：1x2
  - 3/4 路：2x2 宫格
- ROS2 话题下拉选择 + 手动刷新
- 图像交互：
  - 滚轮缩放
  - 左键拖拽平移
  - 左键双击还原（Fit To Window）
- 像素信息查看（按住 `Ctrl` 移动鼠标）：
  - 彩色图：显示 `R/G/B`
  - 深度图：显示真实值（`mm`）
- 同步查看（Tool 菜单开关）：
  - 同步像素信息提示
  - 同步缩放/平移/双击还原
  - 条件：窗口图像尺寸一致
- 右上角浮动工具栏（ToolBox 开关）：
  - 保存图像（带默认文件名）
  - 放大 / 缩小
  - rosbag 录制开始/停止

## 3. 构建方法

在工作区根目录执行：

```bash
colcon build --packages-select image_debug_gui
```

## 4. 运行方法

```bash
source install/setup.bash
ros2 run image_debug_gui image_debug_app
```

## 5. 使用说明

### 5.1 选择话题

1. 在每个窗口上方的 `ROS2 Image Topic` 下拉框选择话题
2. 点击 `Refresh` 可刷新当前可选话题列表

### 5.2 查看像素值

1. 将鼠标移动到图像区域
2. 按住 `Ctrl`
3. 移动鼠标，查看对应像素信息

### 5.3 同步查看

1. 菜单 `Tool -> 同步查看` 勾选
2. 在任意窗口执行缩放/平移/双击还原或 `Ctrl` 像素查看
3. 其他同尺寸窗口会同步显示

### 5.4 保存图像

1. 菜单 `Tool -> ToolBox` 勾选，显示右上角小图标工具栏
2. 点击保存图标
3. 在弹窗中选择目录并确认文件名

默认文件名格式示例：

```text
color_20260425193151121.bmp
depth_20260425193151121_v2.bmp
```

### 5.5 rosbag 录制

1. 确保已选择至少一个图像话题
2. 点击录制图标（红点）开始
3. 在弹窗中选择保存路径
4. 再次点击录制图标（停止方块）结束录制

> 说明：录制命令为 `ros2 bag record`，请确保运行环境已正确 `source` ROS2。

## 6. 依赖

- ROS2（`rclcpp`、`sensor_msgs`、`cv_bridge`）
- OpenCV
- Qt5（Core / Gui / Widgets）
- C++17
