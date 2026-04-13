# debug_viewer

## 描述
Debug viewer 是一个用于 ROS2 应用程序的调试查看器包，基于 C++17 开发。

## 特性
- 提供 ROS2 节点的调试功能
- 可视化数据流以进行开发和测试

## 依赖项
- ROS2 Humble 或更高版本
- rclcpp
- std_msgs

## 构建
要构建此包，请运行：

```bash
colcon build --packages-select debug_viewer
```

## 使用方法
构建完成后，激活工作区并运行：

```bash
source install/setup.bash
# ros2 run debug_viewer debug_viewer_node
```

注意：需要先添加实际的节点实现才能运行。

## 开发指南
- 所有源代码应放在 `src/` 目录中
- 头文件应放在 `include/debug_viewer/` 目录中
- 测试文件应放在 `test/` 目录中
- 使用 C++17 标准进行开发