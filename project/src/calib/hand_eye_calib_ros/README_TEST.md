# 手眼标定测试模块使用说明

## 概述

本测试模块用于验证手眼标定的精度，包含以下功能：
1. 订阅Aruco码识别结果
2. 控制机械臂移动到指定位置
3. 计算并输出标定精度偏差

## 功能特性

### 1. Aruco码识别结果订阅
- 实时订阅 `/aruco_detection/result` 话题
- 获取Aruco码在机械臂坐标系中的位置

### 2. 机械臂位置控制
- 'S'/'s' 键：移动机械臂到Aruco码识别的位置
- 实时订阅 `/right_arm_cartesian_pose` 话题获取机械臂当前位置

### 3. 标定精度验证
- 记录移动前的Aruco码位置和移动后的机械臂实际位置
- 计算并输出X、Y、Z三个方向的偏差值

## 启动测试模块

### 方式1：使用ros2 run命令
```bash
ros2 run hand_eye_calib_ros calib_test_node
```

### 方式2：使用launch文件
```bash
ros2 launch hand_eye_calib_ros calib_test.launch.py
```

## 操作说明

启动测试模块后，可以使用以下键盘快捷键进行操作：

| 按键 | 功能说明 |
|------|----------|
| S/s  | 移动到Aruco检测位置 |
| P/p  | 显示当前位置 |
| Q/q  | 退出程序 |

## 输出信息

测试模块会输出以下信息：

1. 机械臂当前位置（定期输出）
2. Aruco码检测结果
3. 标定精度偏差计算结果

## 标定精度偏差计算

当按下'S'键时，系统会：
1. 记录当前Aruco码在机械臂坐标系中的位置（目标位置）
2. 控制机械臂移动到该位置
3. 等待机械臂停止移动后，记录机械臂的实际位置
4. 计算并输出三个方向的偏差值：
   - X轴偏差
   - Y轴偏差
   - Z轴偏差
   - 总体偏差（欧几里得距离）

## 配置参数

测试模块的配置参数位于 `config/calib_test_node.yaml` 文件中：

```yaml
calib_test_node:
  ros__parameters:
    # 话题名称配置
    robot_pose_topic: "/right_arm_cartesian_pose"
    aruco_detection_topic: "/aruco_detection/result"
    
    # 服务名称配置
    gripper_control_service: "/gripper_control"
    
    # 定时器周期（毫秒）
    timer_period_ms: 100
```

## 依赖项

- ROS 2 Humble
- hand_eye_calib_ros 包
- marker_detect_ros 包
- nova_robot_ctrl_ros 包