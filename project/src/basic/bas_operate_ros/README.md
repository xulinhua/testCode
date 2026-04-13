# bas_operate_ros

## 项目概述

bas_operate_ros 是一个ROS Humble项目，它依赖于data_handler项目，用于将data_handler项目中param_reflector中的参数类型转换为ROS Humble项目中的参数类型。

## 项目功能

### 核心功能

1. **参数类型转换**：提供函数将bas_operate中的ParamInfo类型转换为ROS中的Parameter类型
2. **批量参数设置**：提供一次性原子性设置所有参数的接口
3. **类型映射**：支持基本类型（bool, int, float, double, string）及其数组类型的转换
4. **参数获取**：从ROS参数服务器获取参数值并转换为bas_operate中的参数信息
5. **通用参数处理**：支持多种数据类型的参数转换和处理，包括基本类型和数组类型

### 支持的参数类型

- 基本类型：bool, int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t, float, double, std::string
- 数组类型：std::vector<bool>, std::vector<int8_t>, std::vector<uint8_t>, std::vector<int16_t>, std::vector<uint16_t>, std::vector<int32_t>, std::vector<uint32_t>, std::vector<int64_t>, std::vector<uint64_t>, std::vector<float>, std::vector<double>, std::vector<std::string>

## 项目架构

### 模块组成

bas_operate_ros项目主要由以下几个模块组成：

1. **param_to_server**：负责将bas_operate参数转换为ROS参数并发送到参数服务器
2. **param_from_server**：负责从ROS参数服务器获取参数并转换为bas_operate参数
3. **param_utils**：提供参数处理的通用工具函数

### 文件结构

```
bas_operate_ros/
├── include/bas_operate_ros/
│   ├── param_to_server.hpp     # 参数发送到服务器的接口
│   ├── param_from_server.hpp   # 从服务器获取参数的接口
│   └── param_utils.hpp         # 参数处理工具函数
├── src/
│   ├── param_to_server.cpp     # 参数发送到服务器的实现
│   ├── param_from_server.cpp   # 从服务器获取参数的实现
│   └── param_utils.cpp         # 参数处理工具函数实现
├── CMakeLists.txt             # 构建配置
└── package.xml                # ROS包描述
```

### 命名空间

- **basros**：所有ROS相关功能的函数和类都位于此命名空间下
- **basmodule**：引用bas_operate中的参数反射相关类型

## 实现逻辑

### 参数转换逻辑

1. **从bas_operate到ROS**：
   - 通过`paraInfoToRos`函数将bas_operate中的ParamInfo转换为rclcpp::Parameter
   - 根据ParamInfo中的参数类型（ParamType）调用相应的转换函数
   - 支持40+种不同类型的转换，包括基本类型和数组类型
   - 使用参数前缀来组织参数名称

2. **从ROS到bas_operate**：
   - 通过`paraInfoFromRosType`函数将rclcpp::Parameter转换为bas_operate中的ParamInfo
   - 根据ROS参数的类型匹配到对应的bas_operate参数类型
   - 支持从ROS节点或参数客户端获取参数
   - 支持批量参数转换

3. **参数设置逻辑**：
   - `setParamsAtomically`函数实现一次性原子性设置所有参数
   - 使用ROS参数服务器的set_parameters接口
   - 确保参数设置的原子性和一致性

4. **参数获取逻辑**：
   - 支持从ROS节点指针或参数客户端获取参数
   - 提供通用的参数提取函数，支持所有支持的数据类型

### 核心接口

所有接口均在`basros`命名空间下，以普通函数形式提供：

- `setParamsAtomically()` - 一次性原子性设置所有参数
- `paraInfoToRos()` - 通用参数转换函数，将bas_operate中的参数信息转换为ROS参数
- `paraInfoFromRosType()` - 通用参数转换函数，将ROS参数服务器中的参数值转换为bas_operate中的参数信息
- `boolParamToRos()`, `int8ParamToRos()`, `uint8ParamToRos()`, `int16ParamToRos()`, `uint16ParamToRos()`, `int32ParamToRos()`, `uint32ParamToRos()`, `int64ParamToRos()`, `uint64ParamToRos()`, `floatParamToRos()`, `doubleParamToRos()`, `stringParamToRos()` 及其数组类型的转换函数 - 各种类型的参数转换函数

## 使用示例

### 基本参数转换

```cpp
#include "bas_operate_ros/param_to_server.hpp"
#include "data_handler/param_reflector.hpp"

// 假设有一个ConfigReflector派生类实例
MyConfig config;
auto bas_params = config.getParamsSaved();

// 转换为ROS参数
std::vector<rclcpp::Parameter> ros_params;
basros::paraInfoToRos(bas_params, "", ros_params);

// 在ROS节点中设置参数
basros::setParamsAtomically(this, ros_params);
```

### 从ROS参数服务器获取参数

```cpp
#include "bas_operate_ros/param_from_server.hpp"

// 从ROS节点获取参数
datahandler::ParamInfo param_info;
bool success = basros::paraInfoFromRosType(this, "", param_info);


## 在其他ROS包中使用

要在其他ROS包中使用bas_operate_ros，需要在package.xml中添加依赖：

```xml
<depend>bas_operate_ros</depend>
```

并在CMakeLists.txt中添加：

```cmake
find_package(bas_operate_ros REQUIRED)

ament_target_dependencies(your_target
  bas_operate_ros
)
```

## 依赖关系

- rclcpp
- rcl_interfaces
- std_msgs
- sensor_msgs
- bas_operate

## 构建

本项目使用ament_cmake构建系统，支持ROS Humble环境下的编译。

```bash
colcon build --packages-select bas_operate_ros
```
