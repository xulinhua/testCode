# bas_sys_config_ros 系统配置共享模块

## 项目概述

bas_sys_config_ros 是一个基于 ROS 2 Humble 的系统配置共享模块，主要用于提供系统级配置信息的统一管理和访问。该模块为其他 ROS 项目提供标准化的接口来获取相机配置、机械臂信息以及通信路径等关键系统信息。

bas_sys_config_ros项目（数据提供端）
1. 在bas_sys_config_ros项目中，主要工作是：
2. 从配置文件加载标定数据
3. 将数据转换为参数服务器格式
4. 发布到ROS 2参数服务器

## 功能特性

### 核心功能
- ✅ 相机数量配置管理
- ✅ 机械臂数量配置管理
- ✅ 机械臂类型枚举定义
- ✅ ROS 通信信息数据结构定义
- ✅ 通信路径自动生成
- ✅ 配置文件解析
- ✅ 服务端接口提供
- ✅ 统一标定参数处理（相机和机械臂）

### 技术特性
- 🚀 基于 ROS 2 Humble 构建
- 🔧 模块化设计，易于扩展
- 📦 提供 C++ 库供其他项目集成
- 🎯 高效的配置信息访问
- 📚 完整的 Doxygen 注释文档
- ⚡ 使用ROS 2原生回调机制，确保参数一致性

## 项目架构

```
bas_sys_config_ros/
├── include/bas_sys_config_ros/
│   ├── ros_comm_info.h             # ROS通信信息数据结构定义
│   ├── sys_config_mgr.h            # 系统配置管理器类
│   ├── sys_config_struct.hpp       # 系统相机和机械臂配置信息类定义
│   ├── calib_param_handler.h       # 统一标定参数处理器头文件
│   └── sys_config_utils.h          # 系统配置工具函数
├── src/
│   ├── sys_config_ros_node.cpp     # 主节点实现
│   ├── sys_config_mgr.cpp          # 系统配置管理器实现
│   ├── calib_param_handler.cpp     # 统一标定参数处理器实现
│   └── sys_config_utils.cpp        # 系统配置工具函数实现
├── package.xml                     # 包配置文件
├── CMakeLists.txt                  # 构建配置文件
└── README.md                       # 本说明文档
```

## 数据结构定义

### CamConfigInfo 系统相机配置信息类

包含系统中每个相机的详细配置信息：

```cpp
class CamConfigInfo
{
public:
    bool is_enable;               // 是否启用
    std::string serial_number;    // 序列号
    std::string user_name;        // 用户名称
    unsigned int armNum;          // 机械臂数量
    std::vector<ArmConfigInfo> armInfoList;  // 当前相机配置的机械臂配置信息列表
};
typedef std::vector<CamConfigInfo> CamConfigInfo1D;
```

### ArmConfigInfo 系统机械臂配置信息类

包含系统中每个机械臂的详细配置信息：

```cpp
class ArmConfigInfo
{
public:
    bool is_enable;               // 是否启用
    std::string robot_arm_ip;     // 机械臂IP地址
    std::string user_name;        // 用户名称
};
typedef std::vector<ArmConfigInfo> ArmConfigInfoList;
```

### RosCommMethod 通信方式枚举

定义了ROS中可用的通信方式：

```cpp
enum class RosCommMethod : uint8_t
{
  TOPIC = 0,      ///< 话题通信
  SERVICE = 1,    ///< 服务通信
  PARAMETER = 2,  ///< 参数服务器
  ACTION = 3      ///< 动作通信
};
```

### RosCommMsgType 通信消息类型枚举

定义了系统中各种通信服务或话题的消息类型：

```cpp
enum class RosCommMsgType : uint16_t
{
  SERVICE_SYS_CAM_NUM = 0,      ///< 请求获取系统相机个数的服务
  SERVICE_CAM_INTRINSICS,       ///< 请求获取系统相机内参的服务
  SERVICE_ARUCO_RESULTS,        ///< 请求获取系统Aruco码识别的服务
  SERVICE_ARUCO_CALIB_RESULTS,  ///< 请求获取系统Aruco码识别标定转换后的结果的服务
  
  SERVICE_MAX,

  TOPIC_SYS_CAM_NUM = 10000,    ///< 订阅获取系统相机个数的话题
  TOPIC_CAM_INTRINSICS,         ///< 订阅获取系统相机内参的话题
  TOPIC_ARUCO_RESULTS,          ///< 订阅获取系统Aruco码识别的话题
  COMM_ARUCO_CALIB_RESULTS     ///< 订阅获取系统Aruco码识别标定转换后的结果的话题
  TOPIC_MAX
};
```

### RosCommInfo 通信信息结构体

包含进行ROS通信所需的所有信息：

```cpp
struct RosCommInfo
{
  RosCommMethod comm_method;  ///< 通信方式
  uint8_t cam_id;             ///< 相机ID
  uint8_t arm_id;             ///< 机械臂ID
  std::string name;           ///< 服务/话题名称
};
```

## API 接口

### SysConfigMgr 系统配置管理器类

提供系统配置信息的加载和访问接口：

```cpp
class SysConfigMgr
{
public:
  // 构造函数
  SysConfigMgr();
  
  // 加载系统配置
  bool loadSysConfigData(const std::string& sys_config_path);
  
  // 获取系统配置的相机数量
  uint8_t getSysCamNum() const;
  
  // 获取系统配置的机械臂数量
  uint8_t getSysArmNum() const;
  
  // 获取系统启用的相机数量
  uint8_t getSysEnableCamNum() const;
  
  // 获取系统启用的机械臂数量
  uint8_t getSysEnableArmNum() const;
  
  // 获取系统启用的相机ID列表
  std::vector<uint32_t> getSysEnableCamIds() const;
  
  // 获取系统启用的机械臂ID列表
  std::vector<uint32_t> getSysEnableArmIds() const;
  
  // 获取系统相机配置信息列表
  const SysConfig::CamConfigInfo1D& getSysCamInfoList() const;
  
  // 获取系统机械臂配置信息列表
  const SysConfig::ArmConfigInfoList& getSysArmInfoList() const;

private:
  // 加载系统相机配置文件
  bool loadSysCamConfig(const std::string& sys_config_path); // sys_config_path为相机配置文件完整路径
  
  // 加载系统机械臂配置文件
  bool loadSysArmConfig(const std::string& sys_config_path); // sys_config_path为机械臂配置文件完整路径
  
  // 检查配置文件是否存在并加载YAML节点
  bool loadConfigFile(const std::string& sys_config_path, YAML::Node& config);
};
```

### 统一标定参数处理器

#### 概述

统一标定参数处理器提供了一种使用ROS 2原生回调机制一次性获取所有标定参数完整、一致状态的解决方案，解决了原有实现中每个参数变化都会触发单独回调的问题。

#### 功能特点

1. **一次性回调**: 使用ROS 2原生`add_on_set_parameters_callback`机制，确保所有相关参数变化时只触发一次回调
2. **数据一致性**: 保证获取到的标定数据是完整且一致的
3. **易于集成**: 提供简单的API接口，方便在现有项目中集成使用
4. **统一接口**: 支持相机和机械臂两种类型的标定参数处理

#### 核心组件

##### CalibParamHandler类模板

这是核心的参数处理类模板，提供了以下主要功能：

- `CalibParamHandler(rclcpp::Node::SharedPtr node)`: 构造函数，传入ROS节点指针
- `void registerCalibParamCallback(std::function<void(const DataType&)>)`: 注册标定参数回调函数
- `bool getCalibDatFromServer(const std::string&, DataType&)`: 从参数服务器获取标定数据

##### 类型别名

为了方便使用，提供了以下类型别名：
```cpp
using CamCalibParamHandler = CalibParamHandler<handeyecalib::CamCalibInfoList>;
using ArmCalibParamHandler = CalibParamHandler<handeyecalib::ArmCalibInfo>;
```

#### 使用方法

##### 1. 创建参数处理器实例

```cpp
#include "bas_sys_config_ros/calib_param_handler.h"

// 在节点类中创建参数处理器成员变量
std::unique_ptr<RosComm::CamCalibParamHandler> cam_calib_param_handler_;
std::unique_ptr<RosComm::ArmCalibParamHandler> arm_calib_param_handler_;

// 在构造函数中初始化
cam_calib_param_handler_ = std::make_unique<RosComm::CamCalibParamHandler>(shared_from_this());
arm_calib_param_handler_ = std::make_unique<RosComm::ArmCalibParamHandler>(shared_from_this());
```

##### 2. 注册回调函数

```cpp
// 注册相机标定参数回调
cam_calib_param_handler_->registerCalibParamCallback(
    [this](const handeyecalib::CamCalibInfoList& calib_data) {
        // 处理相机标定数据更新
        this->onCamCalibDataUpdated(calib_data);
    });

// 注册机械臂标定参数回调
arm_calib_param_handler_->registerCalibParamCallback(
    [this](const handeyecalib::ArmCalibInfo& calib_data) {
        // 处理机械臂标定数据更新
        this->onArmCalibDataUpdated(calib_data);
    });
```

##### 3. 获取初始标定数据

```cpp
// 获取相机初始标定数据
handeyecalib::CamCalibInfoList initial_cam_calib_data;
if (cam_calib_param_handler_->getCalibDatFromServer("sys_cam_calib_list", initial_cam_calib_data)) {
    // 处理初始相机标定数据
    onCamCalibDataUpdated(initial_cam_calib_data);
}

// 获取机械臂初始标定数据
handeyecalib::ArmCalibInfo initial_arm_calib_data;
if (arm_calib_param_handler_->getCalibDatFromServer("sys_arm_calib_list.arm_0", initial_arm_calib_data)) {
    // 处理初始机械臂标定数据
    onArmCalibDataUpdated(initial_arm_calib_data);
}
```

### 解析通信信息函数

提供独立的函数供其他项目直接调用：

```
// 根据通信消息类型、相机ID和机械臂ID解析通信信息
RosCommInfo parseCommInfo(RosCommMsgType msg_type, uint8_t cam_id, uint8_t arm_id);

// 根据通信消息类型、相机ID和机械臂ID生成服务/话题名称
std::string generateCommName(RosCommMsgType msg_type, uint8_t cam_id, uint8_t arm_id);
```

### 使用示例

在其他ROS项目中使用bas_sys_config_ros提供的功能：

```cpp
#include "bas_sys_config_ros/sys_config_mgr.h"
#include "bas_sys_config_ros/sys_config_utils.h"

// 示例：创建系统配置管理器并获取配置信息
SysConfig::SysConfigMgr sys_config_mgr;
std::string sys_config_path = "../bas_sys_config/cam_config";

// 加载系统配置并检查是否成功
if (!sys_config_mgr.loadSysConfigData(sys_config_path)) {
    std::cerr << "Failed to load system configuration!" << std::endl;
    // 根据需要决定是否退出程序
    return -1;
}

uint8_t config_cam_count = sys_config_mgr.getSysCamNum();
uint8_t config_arm_count = sys_config_mgr.getSysArmNum();
uint8_t enable_cam_count = sys_config_mgr.getSysEnableCamNum();
uint8_t enable_arm_count = sys_config_mgr.getSysEnableArmNum();

// 获取启用的相机和机械臂ID列表
auto enable_cam_ids = sys_config_mgr.getSysEnableCamIds();
auto enable_arm_ids = sys_config_mgr.getSysEnableArmIds();

// 获取系统相机配置信息列表
auto cam_list = sys_config_mgr.getSysCamInfoList();
for (size_t i = 0; i < cam_list.size(); ++i) {
    std::cout << "Camera " << i << " enable: " << cam_list[i].is_enable << std::endl;
}

// 获取系统机械臂配置信息列表
auto arm_list = sys_config_mgr.getSysArmInfoList();
for (size_t i = 0; i < arm_list.size(); ++i) {
    std::cout << "Arm " << i << " enable: " << arm_list[i].is_enable << std::endl;
}

// 示例：获取相机0的Aruco标定结果服务名称
auto comm_info = SysConfig::parseCommInfo(
    SysConfig::RosCommMsgType::SERVICE_ARUCO_CALIB_RESULTS,
    0,  // cam_id
    SysConfig::RobotArmType::LEFT_ARM  // arm_id
);

// 返回的comm_info.name将是："cam_0/aruco_detection/calib_result"
```

## 配置文件

模块从 `../bas_sys_config/cam_config/` 目录下的配置文件中读取配置信息：

### sys_cam_config.yaml 系统相机配置文件

```yaml
sys_cam_config:
  cam_num: 3

cam_0:
  is_enable: false
  serial_number: 336222072291
  username: "head_camera"
  arm_num: 2
  
cam_1:
  is_enable: true
  serial_number: 336222072292
  username: "left_arm_camera"
  arm_num: 1
  
cam_2:
  is_enable: true
  serial_number: 336222072293
  username: "right_arm_camera"
  arm_num: 1
```

### sys_arm_config.yaml 系统机械臂配置文件

```yaml
sys_arm_config:
  arm_num: 2

arm_0:
  is_enable: false
  robot_arm_ip: "192.168.5.1"
  username: "left_robot_arm"
  
arm_1:
  is_enable: true
  robot_arm_ip: "192.168.5.2"
  username: "right_robot_arm"
```

## 编译和安装

### 依赖项
- ROS 2 Humble
- yaml-cpp

### 编译方法

使用 colcon 编译：

```bash
cd your_ros2_workspace
colcon build --packages-select bas_sys_config_ros
```

### 安装结果

编译成功后将生成以下文件：
- 可执行文件: `install/bas_sys_config_ros/lib/bas_sys_config_ros/sys_config_ros_node`
- 库文件: `install/bas_sys_config_ros/lib/libbas_sys_config_ros_core.so`
- 头文件: `install/bas_sys_config_ros/include/bas_sys_config_ros/`

## 运行方法

启动系统配置节点：

```bash
ros2 run bas_sys_config_ros sys_config_ros_node
```

节点将定期输出当前配置的相机数量信息。

## 集成到其他项目

### CMakeLists.txt 配置

```cmake
find_package(bas_sys_config_ros REQUIRED)

# 链接库文件
target_link_libraries(your_target
  bas_sys_config_ros::bas_sys_config_ros_core
)

# 或者使用接口库（仅需要头文件）
target_link_libraries(your_target
  bas_sys_config_ros::bas_sys_config_ros_utils
)
```

### package.xml 配置

```xml
<depend>bas_sys_config_ros</depend>
```

### 代码包含

```cpp
#include "bas_sys_config_ros/sys_config_mgr.h"
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_sys_config_ros/sys_config_utils.h"
#include "bas_sys_config_ros/calib_param_handler.h"
```

## 设计理念

### 1. 配置集中管理
所有系统级配置信息集中管理，避免配置分散导致的不一致性问题。

### 2. 接口标准化
提供标准化的API接口，便于其他项目集成和使用。

### 3. 扩展性设计
通过配置文件而非硬编码的方式支持新增机械臂类型，提高系统的可扩展性。

### 4. 模块化解耦
将配置管理、信息解析等功能模块化，降低各部分之间的耦合度。

### 5. 统一参数处理
通过统一的参数处理器，减少代码冗余，提高代码复用率，提供一致的API接口。

## 注意事项

1. 确保 `bas_sys_config` 目录存在于项目根目录下
2. 配置文件路径为相对路径：`../bas_sys_config/cam_config/sys_cam_config.yaml`
3. 项目专为 Jetson Orin NX Super 开发板（Ubuntu 22.04 + ROS Humble）环境设计
4. 不支持 Windows 环境下的编译和运行

## 故障排除

### 常见问题

1. **找不到配置文件**
   ```
   Error loading config file: ../bas_sys_config/cam_config/sys_cam_config.yaml
   ```
   解决方案：确保 `bas_sys_config` 目录存在于正确位置

2. **相机数量为0**
   ```
   Current camera count: 0
   ```
   解决方案：检查 `sys_cam_config.yaml` 文件中是否正确定义了相机数量配置

## 更新日志

### v0.0.2
- 新增机械臂配置管理功能
- 新增系统相机和机械臂配置信息类定义
- 新增SysConfigMgr类的扩展接口
- 新增统一标定参数处理器，支持相机和机械臂标定参数的一致性处理

### v0.0.1
- 初始版本
- 实现基本的配置管理功能
- 提供通信信息解析接口
- 支持相机数量配置读取