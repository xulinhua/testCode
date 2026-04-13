# 标定参数处理模块

## 概述

本模块提供了一种新的方式来处理手眼标定参数，使用ROS 2原生的`add_on_set_parameters_callback`回调机制，实现一次回调即可获取所有参数的完整、一致的标定数据，解决了原有实现中每个参数变化都会触发单独回调的问题。

## 文件结构

- `include/hand_eye_calib_ros/calib_param_handler.h` - 头文件，声明参数处理类
- `src/calib_param_handler.cpp` - 实现文件，实现参数处理逻辑

## 主要功能

### 1. 标定参数处理器类 (CalibParamHandler)

`CalibParamHandler`类提供了以下功能：

#### 构造函数
```cpp
explicit CalibParamHandler(const rclcpp::Node::SharedPtr& node);
```

#### 注册参数回调处理器
```cpp
void registerCalibParamCallback(std::function<void(const handeyecalib::CamCalibInfoList&)> callback);
```

使用`registerCalibParamCallback`函数注册回调，当任何相关的标定参数发生变化时，会一次性接收到完整的数据结构。

#### 从参数服务器读取数据
```cpp
bool getCalibDatFromServer(const std::string& param_prefix, handeyecalib::CamCalibInfoList& calib_data);
```

使用`getCalibDatFromServer`函数从参数服务器读取完整的标定数据结构。

#### 获取最新的标定数据
```cpp
const handeyecalib::CamCalibInfoList& getLatestCalibData() const;
```

## 集成到HandEyeCalibNode

在`HandEyeCalibNode`中，我们已经集成了`CalibParamHandler`：

1. 在构造函数中初始化参数处理器：
```cpp
calib_param_handler_ = std::make_unique<CalibParamHandler>(shared_from_this());
```

2. 注册标定参数回调：
```cpp
calib_param_handler_->registerCalibParamCallback(
    [this](const handeyecalib::CamCalibInfoList& calib_data) {
        this->calibDatChangedCallback(calib_data);
    });
```

3. 从参数服务器读取初始标定数据：
```cpp
handeyecalib::CamCalibInfoList initial_calib_data;
if (calib_param_handler_->getCalibDatFromServer("sys_cam_calib_list", initial_calib_data)) {
    // 处理初始标定数据
    calibDatChangedCallback(initial_calib_data);
}
```

## 使用流程

### 1. 启动bas_sys_config_ros节点
首先启动`bas_sys_config_ros`节点，它会将系统配置和标定数据发布到参数服务器：
```bash
ros2 run bas_sys_config_ros sys_config_ros_node
```

### 2. 启动hand_eye_calib_ros节点
然后启动`hand_eye_calib_ros`节点，它会自动从参数服务器读取标定数据并监听后续的更新：
```bash
ros2 run hand_eye_calib_ros hand_eye_calib_node
```

### 3. 标定数据更新
当`bas_sys_config_ros`中的标定数据发生变化时，`hand_eye_calib_ros`会自动接收到更新通知，并通过回调函数处理新的标定数据。

## 优势

1. **数据一致性**：使用ROS 2原生回调机制，确保所有相关参数一次性到达，避免了数据不一致的问题。
2. **减少回调次数**：相比原有实现，只需要一次回调就能获取完整的数据结构。
3. **易于使用**：提供了简单易用的API接口。
4. **高性能**：减少了不必要的回调开销，提高了系统性能。

## 配置参数

在配置文件中，确保设置了正确的参数前缀：
```yaml
hand_eye_calib_node:
  ros__parameters:
    # 其他参数...
```

## 日志输出

模块会输出详细的日志信息，帮助调试和监控标定数据的处理过程：
- 初始化信息
- 参数读取状态
- 数据更新通知
- 错误和警告信息

## 故障排除

### 常见问题

1. **无法读取标定数据**
   - 检查`bas_sys_config_ros`节点是否正常运行
   - 确认参数服务器中是否存在相应的参数
   - 检查参数前缀是否正确

2. **回调未触发**
   - 确认参数处理器已正确初始化
   - 检查回调函数是否正确注册
   - 验证参数名称是否匹配

3. **数据不一致**
   - 确保使用的是ROS 2原生回调机制
   - 检查参数解析逻辑是否正确
   - 验证数据结构定义是否一致