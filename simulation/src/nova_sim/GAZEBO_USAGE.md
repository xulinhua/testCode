# Gazebo仿真使用说明

## 启动Gazebo仿真

### 方法1：使用启动脚本
```bash
cd /home/hs/testCode/simulation
source install/setup.sh
ros2 run nova_sim start_gazebo.sh
```

### 方法2：使用launch文件
```bash
cd /home/hs/testCode/simulation
source install/setup.sh
ros2 launch nova_sim gazebo.launch.py
```

## 启动流程

1. **Gazebo服务器**：启动Gazebo仿真环境
2. **robot_state_publisher**：发布机器人描述
3. **joint_state_publisher**：发布关节状态
4. **spawn_entity**：在Gazebo中生成机器人模型（延迟5秒启动）

## 机器人模型

- **底座**：base_link
- **四个机械臂**：J1, J2, J3, J4
- **龙门架**：左侧立柱、横梁、右侧立柱
- **三个相机**：
  - Camera0：龙门架连接件
  - Camera1：J1-6
  - Camera2：J2-6

## 相机话题

每个相机提供两个话题：

### RGB相机
- Camera0: `/camera0_rgb_sensor/image_raw`
- Camera1: `/camera1_rgb_sensor/image_raw`
- Camera2: `/camera2_rgb_sensor/image_raw`

### 深度相机
- Camera0: `/camera0_depth_sensor/image_raw`
- Camera1: `/camera1_depth_sensor/image_raw`
- Camera2: `/camera2_depth_sensor/image_raw`

## 常见问题

### 1. 模型不显示
- 检查Gazebo是否完全启动（等待5秒）
- 检查终端输出是否有错误信息
- 确认URDF文件路径正确

### 2. 相机图像不显示
- 确认Gazebo仿真正在运行
- 检查相机话题是否发布：`ros2 topic list`
- 查看相机图像：`ros2 run rqt_image_view rqt_image_view`

### 3. 启动失败
- 检查ROS2环境是否正确设置
- 确认所有依赖包已安装
- 查看终端错误日志

## 调试命令

### 查看机器人描述
```bash
ros2 topic echo /robot_description
```

### 查看TF树
```bash
ros2 run tf2_tools view_frames
```

### 查看所有话题
```bash
ros2 topic list
```

### 查看所有节点
```bash
ros2 node list
```
