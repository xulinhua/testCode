# pcl2laserscan_trans - 点云转激光扫描转换工具

## 项目概述

`pcl2laserscan_trans` 是一个ROS2功能包，用于将3D点云数据转换为2D激光扫描数据。支持相机和激光雷达两种模式，提供灵活的配置选项和高效的转换算法。

## 功能特性

### 相机模式
- **输入**: 3D相机点云数据
- **处理**: 坐标系转换、点云滤波、高度过滤、统计滤波
- **输出**: 2D激光扫描数据（LaserScan）
- **支持**: Realsense、Gemini等各类深度相机

### 激光雷达模式  
- **输入**: 3D激光雷达点云数据
- **处理**: 坐标系转换、倾斜补偿、点云滤波、高度过滤
- **输出**: 2D激光扫描数据（LaserScan）
- **支持**: Hesai、Livox等各类激光雷达

## 项目结构

```
pcl2laserscan_trans/
├── config/
│   └── pcl_cloud_config.yaml      # 主配置文件
├── include/pcl2laserscan_trans/
│   ├── pcl_config_struct.hpp      # 配置数据结构
│   ├── pcl_config_manager.hpp     # 配置管理器
│   ├── camera_pcl2laserscan.h     # 相机节点头文件
│   └── pointcloud_to_laserscan.h  # 雷达节点头文件
├── launch/
│   └── pcl2laserscan_trans.launch.py  # 启动文件
├── src/
│   ├── camera_pcl2laserscan.cpp   # 相机节点实现
│   ├── pointcloud_to_laserscan_node.cpp  # 雷达节点实现
│   └── pcl_config_manager.cpp     # 配置管理器实现
└── CMakeLists.txt
```

## 快速开始

### 1. 环境要求
- ROS2 Humble
- PCL 1.12
- YAML-CPP
- tf2、sensor_msgs等ROS2基础包

### 2. 编译安装
```bash
cd /home/hs/testCode/project/src/slam/pcl2laserscan_trans
colcon build --packages-select pcl2laserscan_trans
source install/setup.bash
```

### 3. 运行方式

#### 仅启动相机模式
```bash
ros2 launch pcl2laserscan_trans pcl2laserscan_trans.launch.py mode:=camera_only
```

#### 仅启动激光雷达模式
```bash
ros2 launch pcl2laserscan_trans pcl2laserscan_trans.launch.py mode:=lidar_only
```

#### 同时启动相机和雷达模式（推荐）
```bash
ros2 launch pcl2laserscan_trans pcl2laserscan_trans.launch.py mode:=both
```

### 4. 配置相机驱动

#### Realsense相机
```bash
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  enable_sync:=true \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=15 \
  color_width:=640 \
  color_height:=480 \
  color_fps:=15
```

#### Gemini相机
```bash
ros2 launch orbbec_camera gemini2.launch.py \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=15 \
  color_width:=640 \
  color_height:=480 \
  color_fps:=15 \
  depth_decimation_filter:=true \
  enable_hardware_noise_removal:=false \
  enable_point_cloud:=true
```

#### Livox激光雷达
```bash
cd ~/livox_ws
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

## 配置说明

### 主要配置参数

#### 启动参数（launch）
- `default_mode`: 默认启动模式（camera_only, lidar_only, both）
- `camera_input_topic`: 相机输入点云话题
- `camera_output_topic`: 相机输出激光扫描话题
- `lidar_input_topic`: 雷达输入点云话题
- `lidar_output_topic`: 雷达输出激光扫描话题

#### 相机参数（camera）
- `target_frame`: 目标坐标系（默认：camera_link）
- `min_height`/`max_height`: 高度过滤范围（米）
- `range_min`/`range_max`: 距离范围（米）
- `angle_min`/`angle_max`: 扫描角度范围（弧度）
- `voxel_leaf_size`: 体素滤波大小（米）
- `sor_mean_k`/`sor_stddev_mul_thresh`: 统计滤波参数

#### 雷达参数（lidar）
- `target_frame`: 目标坐标系（默认：lidar_link）
- `min_height`/`max_height`: 高度过滤范围（米）
- `range_min`: 最小距离（米）
- `tilt_compensation_angle`: 倾斜补偿角度（弧度）
- `filter_mean_k`/`filter_stddev`: 统计滤波参数
- `voxel_leaf_size`: 体素滤波大小（米）

#### TF变换参数（tf）
- `publish_camera_tf`: 是否发布相机TF变换
- `publish_lidar_tf`: 是否发布雷达TF变换
- 相机TF参数：parent_frame→child_frame变换
- 雷达TF参数：parent_frame→child_frame变换

### 配置文件位置
主配置文件：`config/pcl_cloud_config.yaml`

## 数据流说明

### 相机数据流
1. **输入**: 相机原始点云（如：`/camera/pt_transformed_out`）
2. **处理**: 
   - 坐标系转换到目标frame
   - 体素滤波降采样
   - 统计滤波去噪
   - 高度过滤地面和障碍物
   - 投影到2D平面生成激光扫描
3. **输出**: 2D激光扫描（如：`/camera/cam_laser_scan_Trans`）

### 雷达数据流
1. **输入**: 雷达原始点云（如：`/lidar_pcl`）
2. **处理**:
   - 坐标系转换到目标frame
   - 倾斜补偿校正
   - 统计滤波去噪
   - 高度过滤地面和障碍物
   - 投影到2D平面生成激光扫描
3. **输出**: 2D激光扫描（如：`/lidar/lidar_laserscan_Trans`）

## 调试与监控

### 查看转换结果
在RViz中添加以下显示类型：
- **LaserScan**: 查看2D激光扫描数据
- **PointCloud2**: 查看原始和滤波后的3D点云

### 调试话题
- `/trans_cloud`: 相机滤波后的点云
- `/cloud_heightfilted`: 雷达滤波后的点云
- 调试信息输出到终端（可通过配置关闭）

## 常见问题

### 1. 坐标系问题
- 确保TF树中存在从源坐标系到目标坐标系的变换
- 检查配置文件中`target_frame`设置是否正确
- 使用`ros2 run tf2_ros tf2_echo source_frame target_frame`验证TF变换

### 2. 数据不显示
- 确认输入话题是否正确发布数据
- 检查配置文件中话题名设置
- 查看节点启动日志确认参数加载

### 3. 性能优化
- 调整`voxel_leaf_size`控制点云密度
- 调整滤波参数平衡去噪效果和计算开销
- 根据实际场景调整高度过滤范围

## 开发说明

### 代码结构
- **配置管理**: 使用YAML配置文件，支持动态参数调整
- **模块化设计**: 相机和雷达处理逻辑分离，便于扩展
- **异常处理**: 完善的错误处理和日志记录

### 扩展新传感器
1. 在配置文件中添加新传感器参数
2. 更新配置数据结构（`pcl_config_struct.hpp`）
3. 实现对应的处理节点
4. 在launch文件中添加启动逻辑

## 许可证

本项目基于MIT许可证开源。

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 联系方式

如有问题请联系项目维护者。