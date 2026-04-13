# pt_tf_trans - Point Cloud Coordinate Transformer

## 功能描述

`pt_tf_trans`是一个基于ROS2的功能包，用于将仿真环境输出的点云从光学坐标系转换到ROS标准坐标系。该功能包实现了与Gazebo仿真中`<origin xyz="0 0 0" rpy="-1.57 0 -1.57" />`相同的坐标变换。

## 技术实现

- **C++标准**：C++17
- **构建系统**：ament_cmake
- **核心依赖**：
  - rclcpp
  - sensor_msgs
  - tf2
  - tf2_ros
  - pcl_conversions
  - pcl_ros

## 节点信息

### 订阅话题
- `/camera/depth_pcl` (sensor_msgs/PointCloud2)：输入的原始点云数据

### 发布话题
- `/camera/pt_transformed_out` (sensor_msgs/PointCloud2)：转换后的点云数据

### 坐标系
- 输入坐标系：光学坐标系
- 输出坐标系：`camera_link`

## 变换参数

使用以下旋转参数进行坐标系转换：
- 绕X轴旋转：-1.57 rad (-90°)
- 绕Y轴旋转：0 rad
- 绕Z轴旋转：-1.57 rad (-90°)

## 构建与运行

### 构建

```bash
cd /home/hs/testCode/point_tf
colcon build
```

### 运行

1. 启动ROS2环境

2. 运行节点：
   ```bash
   source install/setup.bash
   ros2 run pt_tf_trans point_cloud_transformer
   ```

3. 发布点云数据到`/camera/depth_pcl`话题

4. 从`/camera/pt_transformed_out`话题接收转换后的点云数据

## 代码结构

```
pt_tf_trans/
├── CMakeLists.txt          # 构建配置文件
├── package.xml             # 包信息和依赖项
├── src/
│   └── point_cloud_transformer.cpp  # 点云转换节点实现
└── include/
    └── pt_tf_trans/        # 头文件目录
```

## 实现原理

1. 订阅原始点云数据
2. 使用PCL库将ROS点云消息转换为PCL点云对象
3. 对每个点应用旋转变换
4. 将转换后的PCL点云转换回ROS点云消息
5. 发布转换后的点云数据

## 配置参数

节点的配置参数位于`src/point_cloud_transformer.cpp`文件的开头，方便调试和修改：

```cpp
// Configuration parameters for easy debugging and modification
const std::string INPUT_TOPIC_NAME = "/camera/depth_pcl";
const std::string OUTPUT_TOPIC_NAME = "/camera/pt_transformed_out";
const std::string OUTPUT_FRAME_ID = "camera_link";
```

## 注意事项

- 确保PCL库版本与系统兼容
- 运行前需要设置正确的ROS2环境变量
- 点云数据格式应符合sensor_msgs/PointCloud2规范
- 修改配置参数后需要重新构建功能包
