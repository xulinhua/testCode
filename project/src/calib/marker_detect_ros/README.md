# marker_detect_ros

基于ROS2 Humble的Marker码检测节点，使用marker_alg模块进行Marker标记检测。

## 项目概述

本项目实现了一个ROS2节点，用于订阅图像话题、检测Marker标记，并发布检测结果。项目参照marker_alg_demo的结构设计，完全符合ROS2规范。

## 核心特性

- ✅ **ROS2 Humble兼容**: 基于ROS2 Humble开发
- ✅ **marker_alg集成**: 使用marker_alg模块进行Marker检测
- ✅ **OpenCV 4.10**: 编译时使用OpenCV 4.10版本
- ✅ **话题通信**: 支持订阅图像话题和发布检测结果
- ✅ **完整的可执行文件**: 生成marker_detect_node可执行文件
- ✅ **可靠的标定数据同步机制**: 采用推拉结合策略，确保数据一致性
  - 推模式：通过参数事件实时响应
  - 拉模式：通过版本号检查兜底保障
  - 防抖机制：避免多次快速变化导致重复更新
  - 原子写入：保证标定数据结构完整性

## 项目结构

```
marker_detect_ros/
├── CMakeLists.txt              # CMake构建配置
├── package.xml                 # ROS包配置文件
├── README.md                   # 本文档
├── include/
│   └── marker_detect_ros/
│       └── marker_detect_node.hpp   # 节点头文件
├── src/
│   └── marker_detect_node.cpp       # 节点实现文件
├── launch/
│   └── marker_detect.launch.py      # ROS2启动文件
└── config/
    └── marker_detect_params.yaml    # 参数配置文件
```

## 编译项目

使用colcon构建系统编译项目:

```bash
cd /home/user/project
source install/setup.sh
colcon build --packages-select marker_detect_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 运行节点

### 方式1: 使用ros2 run

```bash
source install/setup.sh
ros2 run marker_detect_ros marker_detect_node
```

### 方式2: 使用launch文件

```bash
source install/setup.sh
ros2 launch marker_detect_ros marker_detect.launch.py
```

### 方式3: 指定配置文件路径运行

```bash
source install/setup.sh
# 使用默认配置文件
ros2 run marker_detect_ros marker_detect_node

# 启用标定模式
ros2 run marker_detect_ros marker_detect_node --ros-args -p usecalib:=true

# 指定marker字典类型和标记长度      
ros2 run marker_detect_ros marker_detect_node --ros-args -p marker_dict_type:=5 -p marker_length:=0.1
```

## 配置文件

节点支持从命令行指定配置文件路径。如果没有指定，则使用默认配置文件：

- **默认配置文件路径**: `/home/user/project/sys_config/marker_detect_ros/marker_detect_params.yaml`

配置文件在编译时会自动打包到项目目录的 `sys_config` 文件夹中。

### 使用自定义配置文件

```bash
ros2 run marker_detect_ros marker_detect_node /path/to/your/custom_config.yaml
```

### 启用标定模式

``bash
# 启用标定模式（从默认配置文件加载标定参数）
ros2 run marker_detect_ros marker_detect_node --ros-args -p usecalib:=true
```

### 配置文件格式

配置文件使用YAML格式，包含以下参数：

```yaml
marker_detect_node:
  ros__parameters:
    image_topic: "/camera/camera/color/image_raw"    # 图像话题
    marker_length: 0.1                              # ArUco标记实际长度(米)
    marker_dict_type: 10                             # ArUco字典类型
    show_src_image: true                            # 是否显示源图像
    show_result_image: true                         # 是否显示结果图像
```

### ArUco字典类型

| 类型值 | 字典类型 | 描述 |
|--------|----------|------|
| 0 | DICT_4X4_50 | 4x4 bits, 50 markers |
| 1 | DICT_4X4_100 | 4x4 bits, 100 markers |
| 2 | DICT_4X4_250 | 4x4 bits, 250 markers |
| 3 | DICT_4X4_1000 | 4x4 bits, 1000 markers |
| 4 | DICT_5X5_50 | 5x5 bits, 50 markers |
| 5 | DICT_5X5_100 | 5x5 bits, 100 markers |
| 6 | DICT_5X5_250 | 5x5 bits, 250 markers |
| 7 | DICT_5X5_1000 | 5x5 bits, 1000 markers |
| 8 | DICT_6X6_50 | 6x6 bits, 50 markers |
| 9 | DICT_6X6_100 | 6x6 bits, 100 markers |
| 10 | DICT_6X6_250 | 6x6 bits, 250 markers |
| 11 | DICT_6X6_1000 | 6x6 bits, 1000 markers |
| 12 | DICT_7X7_50 | 7x7 bits, 50 markers |
| 13 | DICT_7X7_100 | 7x7 bits, 100 markers |
| 14 | DICT_7X7_250 | 7x7 bits, 250 markers |
| 15 | DICT_7X7_1000 | 7x7 bits, 1000 markers |

## 节点接口

### 订阅话题

- **`/camera/camera/color/image_raw`** (`sensor_msgs/Image`)
  - 输入图像数据，用于Marker标记检测

### 发布话题

- **`/marker_detection/result`** (`vision_msgs/Detection2D`)
  - 检测到的Marker标记位置和姿态信息

## 标定数据同步机制

### 推拉结合策略

本项目采用可靠的推拉结合机制来确保标定数据的一致性：

#### 推模式（实时响应）
- **参数事件订阅**：订阅 `/parameter_events` 话题
- **实时响应**：标定数据变化时立即触发回调
- **防抖机制**：100ms延迟，避免多次快速变化导致重复更新

#### 拉模式（兜底保障）
- **版本号检查**：每3秒检查一次版本号一致性
- **自动修复**：版本号不匹配时自动同步最新数据
- **强制同步**：启动时确保获取最新数据

### 版本号机制
- 标定数据更新时自动生成时间戳版本号
- 版本号存储在参数服务器中，与标定数据一起原子写入
- 版本号比较确保数据最终一致性

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `marker_length` | double | 0.1 | ArUco标记实际长度(米) |
| `marker_dict_type` | int | 10 | ArUco字典类型 (DICT_5X5_100) |
| `usecalib` | bool | false | 是否启用标定模式 |
| `marker_result_type` | string | "center" | ArUco检测结果类型("center"或"corner") |

### Marker字典类型

- 0: DICT_4X4_50
- 1: DICT_4X4_100
- 2: DICT_4X4_250
- 3: DICT_4X4_1000
- 4: DICT_5X5_50
- 5: DICT_5X5_100
- 6: DICT_5X5_250
- 7: DICT_5X5_1000
- 8: DICT_6X6_50
- 9: DICT_6X6_100
- 10: DICT_6X6_250 (默认)
- 11: DICT_6X6_1000
- 12: DICT_7X7_50
- 13: DICT_7X7_100
- 14: DICT_7X7_250
- 15: DICT_7X7_1000

### 标定数据同步参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `cam_id` | int | 1 | 相机ID |
| `arm_id_list` | array | `[1]` | 机械臂ID列表 |
| `VERSION_CHECK_INTERVAL` | int | 3 | 版本号检查间隔（秒） |
| `DEBOUNCE_DELAY` | int | 100 | 防抖延迟（毫秒） |

## 核心实现

### 头文件 (`marker_detect_node.hpp`)

包含核心成员变量:
```cpp
std::unique_ptr<marker_alg::MarkerDetector> marker_detector_;  ///< Marker检测器
```

### 图像回调函数

在回调函数中使用`detectAndProcessMarkers`进行检测:
```cpp
void MarkerDetectNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // 转换ROS图像为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    // 使用Marker检测器检测标记
    auto result = marker_detector_->detectAndProcessMarkers(image, nullptr, true, true);

    // 发布检测结果
    if (result.found && !result.markers_info.empty()) {
        // 创建Detection2D消息
        auto detection_msg = vision_msgs::msg::Detection2D();
        detection_msg.header.stamp = this->now();
        detection_msg.header.frame_id = frame_id_;
        
        // 设置检测结果
        auto detect_res = vision_msgs::msg::ObjectHypothesisWithPose();
        detect_res.hypothesis.class_id = "marker_marker";
        detect_res.hypothesis.score = 1.0;
        
        // 设置位置坐标
        detection_msg.bbox.center.position.x = result.markers_info[0].position.x;
        detection_msg.bbox.center.position.y = result.markers_info[0].position.y;
        
        detect_res.pose.pose.position.x = result.markers_info[0].position.x;
        detect_res.pose.pose.position.y = result.markers_info[0].position.y;
        detect_res.pose.pose.position.z = result.markers_info[0].position.z;
        
        // 设置方向（单位四元数）
        detect_res.pose.pose.orientation.x = 0.0;
        detect_res.pose.pose.orientation.y = 0.0;
        detect_res.pose.pose.orientation.z = 0.0;
        detect_res.pose.pose.orientation.w = 1.0;
        
        detection_msg.results.push_back(detect_res);
        
        // 发布Detection2D消息
        detection_pub_->publish(detection_msg);
    }
}
```

## 多线程同步机制详解

### 需求分析

根据用户需求，marker_detect_ros项目需要实现以下功能：
1. 去除项目中的use_service参数
2. 订阅获取到图像后，默认进行Marker码识别处理，无论是否识别到都显示图像窗口
3. 收到服务通讯消息时，不单独获取图像，而是使用持续进行的识别结果
4. 结果获取刷新时，做好多线程防护

### 整体架构设计

#### 线程模型
项目中有两个主要的并发操作线程：
1. **图像订阅线程**：通过imageCallback函数持续接收图像并进行Marker检测
2. **服务回调线程**：通过markerDetectionService函数响应外部服务请求

#### 数据流设计
```
图像订阅线程:
  图像输入 -> Marker检测 -> 更新共享结果 -> 设置更新标志

服务回调线程:
  接收请求 -> 清除旧结果 -> 等待新结果 -> 返回最新结果
```

### 详细实现流程

#### 初始化阶段

1. 在类构造函数中初始化所有成员变量：
   - result_mutex_：保护共享数据的互斥锁
   - result_updated_：原子布尔变量，标记是否有新结果
   - latest_detection_result_：存储最新的检测结果
   - max_retry_attempts_：从配置文件读取的最大重试次数

2. 移除use_service参数相关的代码逻辑

#### 图像订阅线程实现(imageCallback)

1. 接收图像消息并转换为OpenCV格式
2. 调用MarkerDetector进行标记检测
3. 使用VisualizationMgr显示检测结果
4. **关键同步步骤**：
   - 获取result_mutex_锁
   - 更新latest_detection_result_
   - 设置result_updated_为true
   - 通知等待的线程(result_cv_.notify_all())
   - 释放锁

#### 服务回调线程实现(markerDetectionService)

1. 接收服务请求
2. **清除旧结果**：
   - 获取result_mutex_锁
   - 清空latest_detection_result_
   - 设置result_updated_为false
   - 释放锁

3. **等待新结果**：
   - 循环max_retry_attempts_次尝试获取新结果
   - 每次循环中：
     * 获取result_mutex_锁
     * 等待条件变量(result_cv_.wait_for())最多100ms
     * 检查result_updated_标志
     * 如果有新结果则跳出循环
     * 否则继续等待

4. **返回结果**：
   - 将latest_detection_result_复制到response中（注意数据类型转换）
   - 发送响应

### 多线程同步机制详解

#### 互斥锁(std::mutex)的作用

互斥锁用于保护共享资源latest_detection_result_，确保同一时间只有一个线程可以访问：
- imageCallback线程需要写入新结果
- markerDetectionService线程需要读取结果或清空结果

#### 条件变量(std::condition_variable)的作用

条件变量用于线程间通信：
- imageCallback线程在更新结果后调用result_cv_.notify_all()通知等待的线程
- markerDetectionService线程调用result_cv_.wait()等待新结果的通知

#### 原子变量(std::atomic<bool>)的作用

原子布尔变量result_updated_用于标记是否有新结果，避免复杂的锁操作：
- 读写操作都是原子的，不需要额外的锁保护
- 比互斥锁更轻量级

### 数据类型转换处理

#### 问题背景
在实现过程中，我们遇到了数据类型不匹配的问题：
- AICoordinateData消息使用float32[]类型存储位置和方向数据
- GetMarkerDetection服务响应使用float64[]类型存储位置和方向数据

#### 解决方案
在服务回调函数中，我们需要进行显式的数据类型转换：
```
// 类型转换：从float32[]转换为float64[]
response->position.clear();
response->position.reserve(latest_detection_result_.position.size());
for (const auto& val : latest_detection_result_.position) {
    response->position.push_back(static_cast<double>(val));
}

response->orientation.clear();
response->orientation.reserve(latest_detection_result_.orientation.size());
for (const auto& val : latest_detection_result_.orientation) {
    response->orientation.push_back(static_cast<double>(val));
}
```

### JSON配置文件解析增强

#### 问题背景
在解析相机内参和标定结果JSON文件时，原来的字符串解析方式容易出现类型转换错误。

#### 解决方案
使用nlohmann::json库进行结构化解析：
```
// 相机内参解析
double fx = j["intrinsics"]["fx"];
double fy = j["intrinsics"]["fy"];
double ppx = j["intrinsics"]["ppx"];
double ppy = j["intrinsics"]["ppy"];

// 畸变系数解析
auto coeffs = j["distortion"]["coefficients"];
dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
for (int i = 0; i < 5 && i < coeffs.size(); ++i) {
    dist_coeffs_.at<double>(0, i) = coeffs[i];
}

// 变换矩阵解析
auto transform_data = j["cam_to_base_transform"];
camera_to_base_transform_ = cv::Mat::eye(4, 4, CV_64F);

// 检查变换矩阵数据是否有效
if (transform_data.is_array() && transform_data.size() >= 4) {
    for (int i = 0; i < 4 && i < transform_data.size(); ++i) {
        auto row = transform_data[i];
        if (row.is_array() && row.size() >= 4) {
            for (int j = 0; j < 4 && j < row.size(); ++j) {
                camera_to_base_transform_.at<double>(i, j) = row[j];
            }
        }
    }
}
```

#### 异常处理
增加了完善的异常处理机制，当JSON解析失败时不再使用默认参数，而是直接返回错误：

```
catch (const std::exception& e) {
    LOG_ERROR("加载相机内参失败: %s", e.what());
    // 出错时不再使用默认相机内参，直接返回false
    LOG_ERROR("无法加载相机内参，程序将无法正常运行");
    return false;  // 返回错误而不是继续运行
}
```

## 关键代码解析

### imageCallback中的同步代码
```
{
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_detection_result_ = detection_result;
    result_updated_ = true;
}
result_cv_.notify_all();  // 通知等待的线程
```

### markerDetectionService中的等待逻辑
```
for (int attempt = 0; attempt < max_retry_attempts_; ++attempt) {
    std::unique_lock<std::mutex> lock(result_mutex_);
    // 等待最多100ms或者直到result_updated_变为true
    result_cv_.wait_for(lock, std::chrono::milliseconds(100), 
                [this]() { return result_updated_; });
    
    if (result_updated_) {
        // 进行数据类型转换后赋值
        response->success = true;
        response->message = "成功检测到Marker标记";
        response->object_class = latest_detection_result_.object_class;
        response->confidence = latest_detection_result_.confidence;
        response->frame_id = latest_detection_result_.frame_id;
        response->stamp = latest_detection_result_.stamp;
        
        // 类型转换：从float32[]转换为float64[]
        response->position.clear();
        response->position.reserve(latest_detection_result_.position.size());
        for (const auto& val : latest_detection_result_.position) {
            response->position.push_back(static_cast<double>(val));
        }
        
        response->orientation.clear();
        response->orientation.reserve(latest_detection_result_.orientation.size());
        for (const auto& val : latest_detection_result_.orientation) {
            response->orientation.push_back(static_cast<double>(val));
        }
        
        result_updated_ = false;  // 重置标志
        return true;
    }
}
```

## 为什么选择100ms等待时间

选择100ms作为等待时间是基于以下考虑：
1. 图像处理通常很快，在100ms内应该能完成一次检测
2. 不会太短导致频繁超时重试
3. 不会太长影响服务响应速度
4. 给了系统足够的处理时间，同时保持响应性

## 流程图示

```
graph TD
    A[开始] --> B[图像订阅线程]
    A --> C[服务回调线程]
    
    B --> D[接收图像]
    D --> E[Marker检测]
    E --> F[更新共享结果]
    F --> G[设置更新标志]
    G --> H[通知等待线程]
    H --> B
    
    C --> I[接收服务请求]
    I --> J[清除旧结果]
    J --> K{达到最大重试次数?}
    K -->|否| L[获取互斥锁]
    L --> M[等待新结果]
    M --> N{有新结果?}
    N -->|是| O[返回结果]
    N -->|否| P[释放锁]
    P --> K
    K -->|是| Q[返回空结果]
    O --> R[结束]
    Q --> R
```

## 依赖项

- **ROS2 Humble**
- **OpenCV 4.10** (支持Marker模块)
- **marker_alg** (Marker检测算法库)
- **log_system** (日志系统)
- **cv_bridge** (ROS-OpenCV桥接)

## 编译输出

成功编译后,生成以下文件:

- 可执行文件: `install/marker_detect_ros/lib/marker_detect_ros/marker_detect_node`
- 配置文件: `install/marker_detect_ros/share/marker_detect_ros/config/marker_detect_params.yaml`
- 启动文件: `install/marker_detect_ros/share/marker_detect_ros/launch/marker_detect.launch.py`

## 测试验证

查看可执行文件:
```bash
ls -lh install/marker_detect_ros/lib/marker_detect_ros/marker_detect_node
```

检查节点信息:
```bash
source install/setup.sh
ros2 pkg executables marker_detect_ros
```

## 测试Marker识别结果服务响应

### 1. 启动marker_detect_node节点

首先启动marker_detect_node节点，确保它能够正常接收图像并进行Marker检测:

```bash
source install/setup.sh
ros2 run marker_detect_ros marker_detect_node
```

或者使用launch文件启动:

```bash
source install/setup.sh
ros2 launch marker_detect_ros marker_detect.launch.py
```

### 2. 检查服务是否可用

在另一个终端中，检查Marker检测服务是否已正确发布:

```bash
source install/setup.sh
ros2 service list | grep marker
```

应该能看到类似以下的输出:
```
/marker_detection/get_result
```

### 3. 查看服务类型

查看服务的详细信息:

```bash
source install/setup.sh
ros2 service type /marker_detection/get_result
```

应该输出:
```
custom_msgs_comm/srv/GetMarkerDetection
```

### 4. 调用服务进行测试

使用ros2 service call命令调用服务进行测试:

```bash
source install/setup.sh
ros2 service call /marker_detection/get_result custom_msgs_comm/srv/GetMarkerDetection "{request_id: 'test_request'}"
```

### 5. 使用Python脚本进行自动化测试

项目中包含一个Python测试脚本[test_marker_service.py](file:///\\192.168.11.129\testCode\project\src\calib\marker_detect_ros\test_marker_service.py)，可以用于自动化测试Marker服务响应。

首先确保脚本具有执行权限:

```bash
chmod +x test_marker_service.py
```

然后运行测试脚本:

```bash
source install/setup.sh
python3 test_marker_service.py
```

### 6. 预期响应结果

如果一切正常，您应该会收到类似以下的响应:

```
success: true
message: "成功检测到Marker标记"
object_class: "marker_marker"
position: [0.123, -0.045, 0.567]
orientation: [0.0, 0.0, 0.0, 1.0]
confidence: 1.0
frame_id: "camera_frame"
stamp:
  sec: 1634567890
  nanosec: 123456789
```

如果未检测到Marker标记，响应会是:

```
success: false
message: "未能获取到有效的Marker检测结果"
object_class: ""
position: []
orientation: []
confidence: 0.0
frame_id: "test_request"
stamp:
  sec: 1634567890
  nanosec: 123456789
```

### 7. 使用ROS2工具查看详细信息

您也可以使用ROS2的其他工具来查看服务和话题的详细信息:

```bash
# 查看服务详细信息
ros2 service info /marker_detection/get_result

# 查看节点信息
ros2 node info /marker_detect_node

# 查看发布的检测结果话题
ros2 topic echo /marker_detection/result
```

### 8. 调试日志查看

如果需要查看更详细的调试信息，可以在启动节点时调整日志级别:

```bash
source install/setup.sh
ros2 run marker_detect_ros marker_detect_node --ros-args --log-level DEBUG
```

## 许可证

MIT License

## 作者

Developer

## 使用示例

### 标定数据更新流程

1. 在 `sys_config_ros_node` 中更新标定数据
   ```bash
   # 更新机械臂1的标定数据
   ros2 service call /sys_config/set_arm_calib_info custom_msgs_comm/srv/SetArmCalibInfo "{arm_id: 1, cam_id: 1, arm_calib_info: {...}}"
   ```

2. `marker_detect_ros` 节点会自动检测到变化：
   - 推模式：通过参数事件实时响应
   - 拉模式：通过版本号检查兜底

3. 验证数据同步：
   ```bash
   # 检查版本号
   ros2 param get /marker_detect_node cam_calib_list.cam_1.version
   ```

## 启动节点

### 方式1: 使用ros2 run

```bash
source install/setup.sh
ros2 run marker_detect_ros marker_detect_node
```

### 方式2: 使用launch文件

```bash
source install/setup.sh
ros2 launch marker_detect_ros marker_detect.launch.py
```

## 许可证

MIT License

## 作者

Developer

## 版本

1.0.0