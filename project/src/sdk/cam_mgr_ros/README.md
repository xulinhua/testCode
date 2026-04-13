# cam_mgr_ros 相机管理器

## 项目概述

cam_mgr_ros 是一个基于 ROS2 的相机管理系统，提供统一的接口管理多种类型的相机设备，包括 RealSense、Orbbec 和 CSI 相机。该系统采用模块化设计，支持相机的自动发现、配置管理、进程控制、数据获取和图像保存。

## 核心功能

### 多相机支持
- **RealSense 相机**：支持 Intel RealSense 深度相机系列
- **Orbbec 相机**：支持 Orbbec 深度相机系列  
- **CSI 相机**：支持 CSI 接口相机

### 设备管理
- **自动设备发现**：自动枚举并识别连接的相机设备
- **设备信息管理**：收集和保存相机设备的详细信息
- **进程管理**：自动启动和停止相机进程，确保资源正确释放
- **进程监控**：实时监控相机进程状态，自动清理僵尸进程

### 配置系统
- **多源配置**：支持从 YAML 文件或 ROS2 参数服务器加载配置
- **场景参数**：支持为每个相机配置多个场景参数
- **动态配置**：运行时支持场景参数切换

### 数据管理
- **内参获取**：自动获取相机内参并转发
- **话题管理**：统一管理相机图像和内参话题
- **数据可视化**：支持实时图像显示
- **图像保存**：支持保存彩色、深度图像到本地文件
- **点云保存**：支持保存点云数据为 PCD 格式
- **订阅管理**：根据配置动态启用/禁用图像和点云订阅
- **场景切换双模式**：
  - **模式 0**：停流设置参数再启用流（默认，快速切换）
  - **模式 1**：先关闭相机再重新开启（稳定，确保参数应用）

### 服务接口
- **相机控制服务**：提供统一的相机控制接口
- **内参获取服务**：提供相机内参查询接口
- **图像保存服务**：提供图像保存控制接口

### 性能优化
- **线程池处理**：使用独立线程池处理图像保存和显示，避免阻塞主回调
- **队列管理**：限制显示队列大小，避免显示滞后
- **异步处理**：所有耗时操作异步执行，保证高帧率
- **窗口清理优化**：关闭相机时完全销毁可视化窗口，避免控件残留

## 项目结构

```
cam_mgr_ros/
├── include/cam_mgr_ros/
│   ├── cam_mgr.hpp        # 相机管理器主类头文件
│   ├── cam_base.hpp       # 相机抽象基类
│   ├── cam_rs.hpp         # RealSense相机实现
│   ├── cam_ob.hpp         # Orbbec相机实现
│   ├── cam_csi.hpp        # CSI相机实现
│   └── utils.hpp          # 工具函数头文件
├── src/
│   ├── cam_mgr.cpp        # 相机管理器实现
│   ├── cam_base.cpp       # 相机基类实现
│   ├── cam_rs.cpp         # RealSense相机实现
│   ├── cam_ob.cpp         # Orbbec相机实现
│   ├── cam_csi.cpp        # CSI相机实现
│   ├── utils.cpp          # 工具函数实现
│   └── cam_mgr_node.cpp   # 节点主入口
├── config/
│   └── log_config.yaml    # 日志配置
├── CMakeLists.txt         # CMake配置
├── package.xml            # 包配置
└── README.md              # 项目说明
```

## 系统架构

### 核心类

1. **CamMgrRos**：相机管理器主类，负责整体协调
2. **CamBase**：相机抽象基类，定义统一接口（包含通用成员：`cam_info_map_`, `sensor_roi_fps_map_` 等）
3. **CamRs**：RealSense 相机实现
4. **CamOb**：Orbbec 相机实现
5. **CamCsi**：CSI 相机实现

### 数据流

1. **配置加载**：从文件或参数服务器加载相机配置
2. **设备枚举**：发现并识别连接的相机设备
3. **进程管理**：启动/停止相机进程
4. **话题订阅**：订阅相机图像和内参话题
5. **数据处理**：处理和转发相机数据
6. **服务响应**：响应相机控制、内参查询和图像保存请求

### 线程模型

- **主线程**：处理服务请求和话题订阅
- **保存线程池**：异步处理图像保存任务（默认 2 个线程）
- **显示线程池**：异步处理图像显示任务（默认 1 个线程）
- **监控线程**：监控相机进程状态，支持场景切换标志 (`is_switching_scene`) 避免误判掉线
- **重试线程**：尝试重新打开失败的相机

## 依赖

- **ROS2 Humble** 或更高版本
- **librealsense2** (RealSense相机支持)
- **Orbbec SDK** (Orbbec相机支持)
- **yaml-cpp** (配置文件解析)
- **OpenCV** (图像处理和显示)
- **cv_bridge** (ROS图像转换)
- **cam_config_mgr** (配置管理)
- **log_system** (日志系统)
- **visualization** (可视化显示)
- **custom_msgs_comm** (自定义消息和服务)

## 安装与编译

### 前提条件

确保已安装以下依赖：
- ROS2 Humble 或更高版本
- 相应相机的SDK（RealSense/Orbbec）
- 必要的系统库

### 编译步骤

```bash
# 进入项目目录
cd /home/user/testCode/project

# 源码工作空间
source install/setup.sh

# 编译cam_mgr_ros包
colcon build --packages-select cam_mgr_ros

# 重新加载环境
source install/setup.sh
```

## 运行

### 启动相机管理器

```bash
# 启动相机管理器节点
ros2 run cam_mgr_ros cam_mgr_node
```

## API 接口

### 相机控制服务

- **服务名称**: `/camera_control`
- **服务类型**: `custom_msgs_comm/srv/CameraControl`
- **请求参数**:
  - `cam_id`: 相机 ID
  - `operate_type`: 操作类型
  - `sence_id`: 场景 ID 或操作子类型
- **响应参数**:
  - `success`: 操作是否成功
  - `message`: 操作结果信息

**操作类型说明**:
- `1`: 开启相机
- `2`: 关闭相机（正常关闭时完全销毁可视化窗口，掉线关闭时保留实例以便重连）
- `3`: 切换场景参数

### 内参获取服务

- **服务名称**: `/get_cam_intr`
- **服务类型**: `custom_msgs_comm/srv/GetCamIntr`
- **功能**: 获取指定相机的内参信息

### 图像保存服务

- **服务名称**: `/set_image_save`
- **服务类型**: `custom_msgs_comm/srv/SetImageSave`
- **功能**: 开启或关闭图像保存功能

**请求参数**:
- `operate_type`: 操作类型
  - `0`: 停止保存图像
  - `1`: 开始保存图像
  - `2`: 开始录制 ROS 包
  - `3`: 停止录制 ROS 包并解析
- `cam_ids`: 相机 ID 数组
  - `[-1]`: 所有相机
  - `[0, 1]`: 指定相机
- `stream_types`: 流类型数组
  - `[-1]`: 所有流
  - `[0]`: 彩色流（COLOR）
  - `[1]`: 深度流（DEPTH）
  - `[2]`: 红外流（IR）
  - `[3]`: 点云流（CLOUD）
- `sence_ids`: 场景 ID 数组
  - `[-1]`: 所有场景
  - `[0, 1]`: 指定场景
- `save_path`: 保存路径名（可选）
  - `""` (空字符串): 使用时间戳作为目录名（默认行为），格式为 `YYYYMMDDHHMMSSmmm`
  - `"custom_name"`: 使用自定义名称作为目录名
  - **适用**: 图像保存和 ROS 包录制均支持此参数

**响应参数**:
- `success`: 操作是否成功
- `message`: 操作结果信息

## 服务调用示例

### 相机控制

#### 启动相机
```bash
ros2 service call /camera_control custom_msgs_comm/srv/CameraControl "{cam_id: 0, operate_type: 1, sence_id: 0}"
```

#### 停止相机
```bash
ros2 service call /camera_control custom_msgs_comm/srv/CameraControl "{cam_id: 0, operate_type: 2}"
```

#### 切换场景
```bash
ros2 service call /camera_control custom_msgs_comm/srv/CameraControl "{cam_id: 0, operate_type: 3, sence_id: 1}"
```

### 图像保存

#### 开启所有相机存图（使用时间戳目录）
``bash
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [-1], stream_types: [-1], sence_ids: [-1], save_path: ''}"
```

#### 关闭所有相机存图
```bash
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 0, cam_ids: [-1], stream_types: [-1], sence_ids: [-1], save_path: ''}"
```

#### 开启指定相机存图（使用时间戳目录）
```
# 保存相机 0 的彩色和深度图像
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [0], stream_types: [-1], sence_ids: [-1], save_path: ''}"

# 保存相机 0 的彩色图像
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [0], stream_types: [0], sence_ids: [-1], save_path: ''}"

# 保存相机 0 的深度图像
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [0], stream_types: [1], sence_ids: [-1], save_path: ''}"
```

#### 开启指定相机存图（使用自定义目录）
```
# 使用自定义目录名保存相机 0 的图像
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [0], stream_types: [-1], sence_ids: [-1], save_path: 'my_test_images'}"

# 保存到特定项目目录
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [0], stream_types: [-1], sence_ids: [-1], save_path: 'project_alpha/calib_data'}"
```

#### 保存多个相机
```
# 保存相机 0 和相机 1 的图像（使用时间戳目录）
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [0, 1], stream_types: [-1], sence_ids: [-1], save_path: ''}"
```

#### 保存指定场景
```
# 保存场景 0 的图像
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 1, cam_ids: [0], stream_types: [-1], sence_ids: [0], save_path: ''}"
```

### ROS 包录制

#### 开启 ROS 包录制（使用时间戳目录）
```bash
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 2, cam_ids: [-1], stream_types: [-1], sence_ids: [-1], save_path: ''}"
```

#### 开启 ROS 包录制（使用自定义目录）
```bash
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 2, cam_ids: [-1], stream_types: [-1], sence_ids: [-1], save_path: 'bag_record_2024'}"
```

#### 停止 ROS 包录制并解析
```bash
ros2 service call /set_image_save custom_msgs_comm/srv/SetImageSave "{operate_type: 3, cam_ids: [-1], stream_types: [-1], sence_ids: [-1], save_path: ''}"
```

## 图像保存

### 保存目录结构

图像保存在 `install/项目路径/Images/` 目录下，按以下结构组织：

```
Images/
├── cam_0/
│   ├── sence_0/
│   │   ├── color/
│   │   │   ├── image_20260309_143052_123.bmp
│   │   │   └── ...
│   │   └── depth/
│   │       ├── image_20260309_143052_456.bmp
│   │       └── ...
│   └── sence_1/
│       └── ...
├── cam_1/
│   └── ...
```

### 文件命名规则

- **格式**: `image_YYYYMMDD_HHMMSS_mmm.bmp`
- **示例**: `image_20260309_143052_123.bmp`
- **说明**:
  - `YYYYMMDD`: 年月日
  - `HHMMSS`: 时分秒
  - `mmm`: 毫秒

### 保存规则

- **彩色图像**: 保存到 `color/` 子目录，BMP格式
- **深度图像**: 保存到 `depth/` 子目录，BMP格式
- **红外图像**: 不保存
- **点云数据**: 保存到 `cloud/` 子目录，PCD格式
- **文件格式**: 图像为BMP格式，点云为PCD格式

### 性能优化

- **异步保存**: 使用独立线程池处理图像保存，不阻塞主回调
- **目录检查**: 每次保存前检查目录是否存在，确保目录被删除后能重新创建
- **队列管理**: 限制显示队列大小，避免显示滞后

## 配置管理

### 配置文件结构

相机配置文件位于 `bas_config_data/cam_config/` 目录下，每个相机有独立的配置文件夹，每个场景参数单独保存：

```
bas_config_data/cam_config/
├── sys_cam_config.yaml          # 系统相机总配置
├── all_cameras_info.yaml        # 所有相机信息
├── cam_0/                       # 相机 ID 为 0 的配置
│   ├── cam_0_info.yaml          # 相机基本信息
│   ├── cam_0_device_info.yaml   # 相机设备信息
│   ├── sence_0/                 # 场景 0 参数
│   │   └── cam_0_sence_0_info.yaml
│   ├── sence_1/                 # 场景 1 参数
│   │   └── cam_0_sence_1_info.yaml
│   └── cam0_arm0/               # 机械臂标定参数
│       └── tcp_offset_cam0_arm0.yaml
├── cam_1/                       # 相机 ID 为 1 的配置
│   ├── cam_1_info.yaml
│   ├── sence_0/
│   │   └── cam_1_sence_0_info.yaml
│   └── sence_1/
│       └── cam_1_sence_1_info.yaml
└── ...
```

### 配置参数

**相机基本信息**:
- `cam_id`: 相机 ID
- `cam_type`: 相机类型 (0: 无，1: RealSense, 2: Orbbec, 3: CSI)
- `enable`: 是否启用
- `cam_usr_name`: 用户定义的相机名称
- `serial_number`: 相机序列号

**场景参数**:
- `enable_color_stream`: 是否启用彩色流
- `enable_depth_stream`: 是否启用深度流
- `enable_ir_stream`: 是否启用红外流
- `color_para`: 彩色相机参数 (width, height, fps)
- `depth_para`: 深度相机参数 (width, height, fps)
- `ir_para`: 红外相机参数 (width, height, fps)
- `show_topic_image`: 是否显示图像

---

### 相机配置模式

相机配置支持两种启动方式：

#### 方式一：参数服务器 + 本地配置文件

**前提条件**：需要提前启动 `bas_sys_config_ros` 节点
```bash
ros2 run bas_sys_config_ros sys_config_ros_node
```

**1. 配置系统相机参数**
- **文件路径**：`bas_config/bas_config_data/cam_config/sys_cam_config.yaml`
- **配置参数**：
  ```yaml
  cam_num: 2                  # 要启动的相机数量

  cam_0:
    is_enable: true             # 是否开启该相机
    serial_number: "xxx"       # 相机序列号
  ```

**2. 配置相机基本信息**（以 cam0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/cam_0_info.yaml`
- **配置参数**：
  ```yaml
  show_topic_image: true        # 是否启用控件显示已开启的图像
  sence_num: 2                  # 场景数量
  ```

**3. 配置场景参数**（以 cam0_sence0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/sence_0/cam_0_sence_0_info.yaml`
- **配置参数**：
  ```yaml
  enable_color_stream: true     # 是否开启彩色图像流
  enable_depth_stream: true     # 是否开启深度流
  enable_ir_stream: false       # 是否开启红外流
  enable_cloud_stream: true     # 是否开启点云流
  enable_publish_intrinsics: true  # 是否开启内参转发
  enable_save_intrinsics: false    # 是否保存内参到本地
  
  # 图像流配置参数（width, height, fps, exposure, gain）
  color_para: 
  
  # 深度流配置参数（width, height, fps, exposure, gain）
  depth_para: 
  
  ```

#### 方式二：纯本地配置文件启动

**无需启动参数服务器节点**，直接从配置文件加载所有参数。

**1. 配置相机基本信息**（以 cam0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/cam_0_info.yaml`
- **配置参数**：
  ```yaml
  cam_id: 0                     # 相机 ID
  enable: true                  # 是否开启该相机
  show_topic_image: true        # 是否启用控件显示已开启的图像
  cam_usr_name: "Front_Camera"  # 相机自定义名称
  serial_number: "SN123456"     # 序列号
  cam_type: 0                   # 相机类型：realsense/orbbec/csi 或 0 1 2
  cam_model: "D435i"            # 相机型号
  sence_num: 2                  # 场景数量
  ```

**2. 配置场景参数**（以 cam0_sence0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/sence_0/cam_0_sence_0_info.yaml`
- **配置参数**：
  ```yaml
  enable_color_stream: true     # 是否开启彩色图像流
  enable_depth_stream: true     # 是否开启深度流
  enable_ir_stream: false       # 是否开启红外
  enable_cloud_stream: true     # 是否开启点云流
  enable_publish_intrinsics: true  # 是否开启内参转发
  enable_save_intrinsics: false    # 是否保存内参到本地
  
  # 图像流配置参数
  color_para: 
  
  # 深度流配置参数
  depth_para: 
  ```

---

### BAG 包解析工具

新增 BAG 包数据解析功能，支持将 ROS2 bag 包中的图像和点云数据导出为常用格式。

**接口定义**：
```cpp
/**
 * @brief 解析 ROS2 bag 包，导出图像和点云数据
 * @param bag_path bag 包的绝对路径
 * @return bool 解析是否成功
 */
bool parse_rosbag(const std::string& bag_path);
```

**解析规则**：

**1. 图像数据导出**：
- 为每个图像话题创建对应的文件夹
- 图像文件名使用时间戳命名（毫秒级）
- 图像格式：BMP
- 示例输出结构：
  ```
  output_dir/
  ├── cam_0_color/
  │   ├── 1711234567890.bmp
  │   ├── 1711234567923.bmp
  │   └── ...
  └── cam_0_depth/
      ├── 1711234567890.bmp
      └── ...
  ```

**2. 点云数据导出**：
- 为每个点云话题创建对应的文件夹
- 点云文件名使用时间戳命名（毫秒级）
- 点云格式：PCD
- 示例输出结构：
  ```
  output_dir/
  └── cam_0_pointcloud/
      ├── 1711234567890.pcd
      ├── 1711234567923.pcd
      └── ...
  ```

**使用方法**：
```bash
# 通过服务调用解析 bag 包
ros2 service call /parse_rosbag custom_msgs_comm/srv/ParseRosBag "{bag_path: '/path/to/your.bag'}"
```

---

### 配置参数详解

#### 1. 图像与点云订阅管理

**图像订阅**（参考点云订阅逻辑）：
- **启用条件**：仅在配置中开启对应的 color/depth 数据流时才启用订阅
- **取消订阅**：当数据流关闭时，同步取消对应图像的订阅
- **显示空间管理**：
  - 开启订阅时：创建对应的图像显示控件
  - 取消订阅时：删除对应的显示控件，避免资源泄漏
- **应用场景**：
  - 相机打开时：根据配置自动启用/禁用订阅
  - 相机关闭时：清理所有订阅和显示资源
  - 切换场景时：动态调整订阅状态和显示控件

**点云保存功能**：
- **保存格式**：PCD 格式（Point Cloud Data）
- **异步处理**：使用线程池异步保存，避免阻塞主线程
- **启用方式**：与图像保存类似，通过配置参数控制
- **保存路径**：可配置保存目录，默认保存在 `~/ros2_logs/pcl_data/`

#### 2. FPS 自适应调节接口

新增 FPS 查找接口，用于在图像分辨率不变的情况下查找支持的最低帧率：

```
/**
 * @brief 查找支持的最低 FPS（下限）
 * @param cam_id 相机 ID
 * @param stream_type 数据流类型（COLOR/DOWNLOAD/IR）
 * @param current_width 当前宽度（保持不变）
 * @param current_height 当前高度（保持不变）
 * @param current_fps 当前 FPS
 * @param min_width 返回支持的最小宽度
 * @param min_height 返回支持的最小高度
 * @param min_fps 返回支持的最低 FPS
 * @return bool 是否找到合适的配置
 */
bool find_minimum_roi_fps(
    int cam_id, 
    CamMgr::CamStreamType stream_type, 
    int current_width, 
    int current_height, 
    int current_fps,
    int& min_width, 
    int& min_height, 
    int& min_fps);
```

**使用场景**：带宽不足时自动降低帧率，保持分辨率不变。

#### 3. 掉线重连增强功能

**掉线判断机制**（新增）：
- **原有机制**：检测相机进程是否存在
- **新增机制**：
  - 当前相机配置的图像长时间未订阅到消息（超时 10 秒）
  - 当前相机配置的点云长时间未订阅到消息（超时 10 秒）

**智能重连策略**：
- **重连间隔检测**：如果同一相机两次重连的时间间隔 < 5 秒
- **原因分析**：判断为带宽不足
- **降帧重连**：第二次重连时使用 `find_minimum_roi_fps` 接口查找最低帧率
- **记录机制**：
  - 每个相机独立记录最近 3 条掉线原因
  - 掉线原因类型使用枚举定义（见 `cam_com_struct.hpp`）
  - 方便后续统计分析和故障诊断

**掉线原因枚举**（定义于 `cam_com_struct.hpp`）：
```cpp
enum class DisconnectReason {
    PROCESS_CRASH,           // 进程崩溃
    IMAGE_TIMEOUT,          // 图像消息超时
    POINTCLOUD_TIMEOUT,     // 点云消息超时
    BANDWIDTH_INSUFFICIENT, // 带宽不足
    HARDWARE_ERROR,         // 硬件故障
    USB_CONNECTION_LOST     // USB 连接丢失
};
```

---

### 相机配置模式

相机配置支持两种启动方式：

#### 方式一：参数服务器 + 本地配置文件

**前提条件**：需要提前启动 `bas_sys_config_ros` 节点
```bash
ros2 run bas_sys_config_ros sys_config_ros_node
```

**1. 配置系统相机参数**
- **文件路径**：`bas_config/bas_config_data/cam_config/sys_cam_config.yaml`
- **配置参数**：
  ```yaml
  cam_num: 2                    # 要启动的相机数量
  is_enable: [true, true]       # 是否开启该相机（数组，对应每个相机）
  serial_number: ["xxx1", "xxx2"] # 相机序列号（数组）
  ```

**2. 配置相机基本信息**（以 cam0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/cam_0_info.yaml`
- **配置参数**：
  ```yaml
  show_topic_image: true        # 是否启用控件显示已开启的图像
  sence_num: 2                  # 场景数量
  save_pcd_cloud: true          # 是否保存点云为 PCD 格式
  ```

**3. 配置场景参数**（以 cam0_sence0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/sence_0/cam_0_sence_0_info.yaml`
- **配置参数**：
  ```yaml
  enable_color_stream: true     # 是否开启彩色图像流
  enable_depth_stream: true     # 是否开启深度流
  enable_ir_stream: false       # 是否开启红外流
  enable_cloud_stream: true     # 是否开启点云流
  enable_publish_intrinsics: true  # 是否开启内参转发
  enable_save_intrinsics: false    # 是否保存内参到本地
  
  # 图像流配置参数（width, height, fps）
  color_para: [640, 480, 30]
  
  # 深度流配置参数（width, height, fps）
  depth_para: [640, 480, 30]
  
  # 红外流配置参数（如有）
  ir_para: [640, 480, 30]
  ```

#### 方式二：纯本地配置文件启动

**无需启动参数服务器节点**，直接从配置文件加载所有参数。

**1. 配置相机基本信息**（以 cam0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/cam_0_info.yaml`
- **配置参数**：
  ```yaml
  cam_id: 0                     # 相机 ID
  enable: true                  # 是否开启该相机
  show_topic_image: true        # 是否启用控件显示已开启的图像
  cam_usr_name: "Front_Camera"  # 相机自定义名称
  serial_number: "SN123456"     # 序列号
  cam_type: "rs"                # 相机类型：rs/orbbec/csi
  cam_model: "D435i"            # 相机型号
  sence_num: 2                  # 场景数量
  save_pcd_cloud: true          # 是否保存点云为 PCD 格式
  ```

**2. 配置场景参数**（以 cam0_sence0 为例）
- **文件路径**：`bas_config/bas_config_data/cam_config/cam_0/sence_0/cam_0_sence_0_info.yaml`
- **配置参数**：
  ```yaml
  enable_color_stream: true     # 是否开启彩色图像流
  enable_depth_stream: true     # 是否开启深度流
  enable_ir_stream: false       # 是否开启红外流
  enable_cloud_stream: true     # 是否开启点云流
  enable_publish_intrinsics: true  # 是否开启内参转发
  enable_save_intrinsics: false    # 是否保存内参到本地
  
  # 图像流配置参数
  color_para: [640, 480, 30]
  
  # 深度流配置参数
  depth_para: [640, 480, 30]
  ```

---

### BAG 包解析工具

新增 BAG 包数据解析功能，支持将 ROS2 bag 包中的图像和点云数据导出为常用格式。

**接口定义**：
```cpp
/**
 * @brief 解析 ROS2 bag 包，导出图像和点云数据
 * @param bag_path bag 包的绝对路径
 * @return bool 解析是否成功
 */
bool parse_rosbag(const std::string& bag_path);
```

**解析规则**：

**1. 图像数据导出**：
- 为每个图像话题创建对应的文件夹
- 图像文件名使用时间戳命名（毫秒级）
- 图像格式：BMP
- 示例输出结构：
  ```
  output_dir/
  ├── cam_0_color/
  │   ├── 1711234567890.bmp
  │   ├── 1711234567923.bmp
  │   └── ...
  └── cam_0_depth/
      ├── 1711234567890.bmp
      └── ...
  ```

**2. 点云数据导出**：
- 为每个点云话题创建对应的文件夹
- 点云文件名使用时间戳命名（毫秒级）
- 点云格式：PCD
- 示例输出结构：
  ```
  output_dir/
  └── cam_0_pointcloud/
      ├── 1711234567890.pcd
      ├── 1711234567923.pcd
      └── ...
  ```

**使用方法**：
```bash
# 通过服务调用解析 bag 包
ros2 service call /parse_rosbag custom_msgs_comm/srv/ParseRosBag "{bag_path: '/path/to/your.bag'}"
```

## 高级功能

### 相机接口统一

所有相机类型（RealSense、Orbbec、CSI）使用统一的接口命名：
- `open_cam()` - 启动相机
- `close_cam()` - 关闭相机
- `switch_cam_sence()` - 切换场景
- `get_camera_info_topic()` - 获取内参话题名
- `get_camera_image_topic()` - 获取图像话题名
- `find_minimum_roi_fps()` - 查找支持的最低 FPS
- `parse_rosbag()` - 解析 ROS2 bag 包

### 相机监控与智能恢复

系统会自动监控相机状态，实现智能重连和故障恢复：

**监控机制**：
- **进程监控**：定期检查已打开相机的进程状态
- **消息超时检测**：
  - 图像消息接收超时检测（10 秒）
  - 点云消息接收超时检测（10 秒）
- **掉线原因追踪**：每个相机循环记录最近 3 条掉线原因

**智能重连策略**：
- **重试线程**：后台线程持续尝试重新打开失败的相机
- **重连间隔分析**：
  - 如果两次重连间隔 < 5 秒 → 判定为带宽不足
  - 自动触发降帧重连流程
- **自适应帧率调节**：
  - 调用 `find_minimum_roi_fps()` 查找最低可用帧率
  - 在保持分辨率不变的前提下降低帧率
  - 优先保证数据完整性，而非实时性

**防误判机制**：
- **场景切换保护**：通过 `is_switching_scene` 标志区分正常切换和异常掉线
- **状态同步**：场景切换期间暂停掉线检测，避免误触发重连

**进程管理**：
- **僵尸进程清理**：自动检测并清理相机相关的僵尸进程
- **资源释放**：确保 USB 设备和内存正确释放

### 内参管理

- **自动获取**：从相机流中自动获取内参并发布到 ROS 话题
- **保存到文件**：将内参保存为 JSON 格式文件，便于离线使用
- **转发功能**：将内参转发到指定话题，供其他节点订阅
- **可选配置**：通过 `enable_publish_intrinsics` 和 `enable_save_intrinsics` 控制

### 点云数据处理

**点云保存**：
- **保存格式**：PCD（Point Cloud Data）标准格式
- **异步处理**：使用独立线程池异步保存，不阻塞主线程
- **配置控制**：通过 `save_pcd_cloud` 参数启用/禁用
- **保存路径**：可配置，默认为 `~/ros2_logs/pcl_data/`

**点云订阅优化**：
- **按需订阅**：仅在 `enable_cloud_stream=true` 时订阅点云话题
- **资源管理**：取消订阅时同步释放相关资源
- **超时检测**：10 秒未收到点云消息触发掉线判断

### 可视化显示

支持实时显示相机图像，可在图像上叠加相机信息和话题名称：

**显示管理**：
- **队列限制**：最多保留 3 张图像，避免显示滞后
- **异步处理**：使用独立线程池处理显示任务，不阻塞数据采集
- **动态更新**：优先显示最新图像，自动丢弃过期图像
- **窗口清理**：关闭相机或取消订阅时完全销毁窗口，避免控件残留

**配置参数**：
- `show_topic_image`: 是否启用图像显示控件
- 可通过运行时服务动态开关

### 工具函数

系统提供了一系列工具函数，封装了常用功能：

- **相机类型判断**：`CamBase::get_cam_type_by_str(string)`
- **设备类型判断**：`CamBase::get_device_type_by_str(string)`
- **进程管理**：查找和杀死相机进程
- **设备信息打印**：`print_camera_device_info()`
- **流类型转换**：`get_sensor_type_by_stream_type()`
- **时间戳获取**：`get_current_timestamp()`
- **FPS 查找**：`find_minimum_roi_fps()` - 查找支持的最低帧率
- **BAG 解析**：`parse_rosbag()` - 解析 ROS2 bag 包

## 注意事项

1. **权限问题**：确保用户有访问相机设备的权限
2. **资源占用**：多相机同时运行会增加系统资源占用
3. **带宽限制**：高分辨率和高帧率会占用更多网络带宽
4. **相机冲突**：避免多个进程同时访问同一相机
5. **依赖安装**：确保所有依赖项正确安装和配置
6. **磁盘空间**：长时间存图会占用大量磁盘空间，注意监控磁盘使用情况
7. **显示性能**：开启显示会影响存图速率，建议测试时关闭显示

## 故障排除

### 常见问题

1. **相机无法启动**
   - 检查相机是否正确连接
   - 检查是否有其他进程占用相机
   - 检查相机驱动是否正确安装
   - 检查相机序列号是否正确

2. **内参获取失败**
   - 确保相机已成功启动
   - 检查相机是否支持内参输出
   - 检查话题订阅是否正常

3. **图像显示问题**
   - 确保环境支持GUI显示
   - 检查OpenCV是否正确安装
   - 检查相机图像流是否正常

4. **服务调用失败**
   - 检查服务是否已正确启动
   - 检查参数是否正确
   - 检查相机ID是否存在

5. **存图失败**
   - 检查磁盘空间是否充足
   - 检查目录是否有写入权限
   - 检查相机是否正常发布图像

6. **存图速率低**
   - 关闭图像显示功能
   - 检查磁盘IO性能
   - 降低图像分辨率或帧率


## 扩展与定制

### 添加新相机类型

1. 继承 `CamBase` 类
2. 实现必要的纯虚函数
3. 在 `CamMgrRos::create_camera_instance` 中添加相机类型支持

### 自定义功能

- **添加新服务**：在 `custom_msgs_comm` 项目添加新的服务定义
- **扩展配置**：在配置文件中添加自定义参数
- **修改行为**：重写相应的虚函数

## 版本历史

- **v1.0.0**：初始版本，支持 RealSense 和 Orbbec 相机
- **v1.1.0**：添加 CSI 相机支持
- **v1.2.0**：增加内参自动保存和转发功能
- **v1.3.0**：优化相机监控和自动恢复机制
- **v1.4.0**：添加图像保存功能，支持多相机、多场景、多流类型
- **v1.5.0**：性能优化，使用线程池处理图像保存和显示
- **v1.6.0**：添加工具函数封装，代码结构优化
- **v1.7.0**：添加点云保存功能，支持 PCD 格式
- **v1.8.0**：实现图像订阅动态管理，基于配置启用/禁用订阅
- **v1.9.0**：增强掉线重连功能，支持消息超时检测和带宽不足判断
- **v2.0.0**：添加最低 FPS 查找接口，优化重连逻辑
- **v2.1.0**：接口命名统一化（去除 `_single` 后缀），基类重构（通用成员移至基类）
- **v2.2.0**：场景切换双模式支持，掉线误判修复，可视化窗口清理优化

## 许可证

本项目采用 MIT 许可证，详情请参阅 LICENSE 文件。

## 联系方式

如有问题或建议，请联系项目维护者。
