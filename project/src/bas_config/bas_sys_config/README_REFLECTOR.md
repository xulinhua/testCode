# ConfigReflector 使用说明

> **注意**: 从版本X.X.X开始，ConfigReflector已重构为继承自`basmodule::ConfigReflector`，共享bas_operate中的通用实现。

## 概述

ConfigReflector 是一个通用的数据结构反射器，用于自动化处理配置数据结构的参数读取和写入。它能够解析任意数据结构中的参数信息，包括参数名称、数据类型和参数值，并提供统一的接口来访问这些信息。

## 主要特性

1. **自动参数解析**：自动识别数据结构中的所有参数
2. **类型安全**：支持多种基础数据类型（bool, int, uint, float, double, string等）
3. **统一接口**：提供一致的API来访问不同数据结构的参数
4. **ROS集成**：与ROS参数服务器无缝集成，简化参数读取过程
5. **易于扩展**：可以轻松添加对新数据结构的支持

## 核心组件

### ParamType 枚举
定义了支持的参数类型：
- BOOL
- INT8, UINT8
- INT16, UINT16
- INT32, UINT32
- INT64, UINT64
- FLOAT, DOUBLE
- STRING
- UNKNOWN

### ParamInfo 结构体
存储单个参数的信息：
- `name`：参数名称
- `type`：参数类型
- `value`：参数值（使用std::any存储）
- `ptr`：指向参数内存地址的指针
- `size`：参数大小

### ConfigReflector 基类
提供反射器的基础功能：
- `getParams()`：获取所有参数信息
- `getParamByName()`：根据名称获取参数信息
- `getParamsMsg()`：获取参数打印信息
- `registerParam()`：注册参数（模板函数）

### 特定数据结构反射器
- `ArmConfigInfoReflector`：用于ArmConfigInfo结构体
- `CamConfigInfoReflector`：用于CamConfigInfo结构体

## 使用示例

### 基本用法

```cpp
#include "bas_sys_config/config_reflector.hpp"
#include "bas_sys_config/sys_config_struct.hpp"

// 创建数据结构实例
SysConfig::ArmConfigInfo arm_info;
arm_info.is_enable = true;
arm_info.arm_id = 1;
arm_info.robot_arm_ip = "192.168.1.100";
arm_info.user_name = "robot_arm_1";

// 创建反射器
SysConfig::ArmConfigInfoReflector reflector(arm_info);

// 获取所有参数
const auto& params = reflector.getParams();
for (const auto& param : params) {
    std::cout << "参数名: " << param.name 
              << ", 类型: " << param.getTypeString() << std::endl;
}

// 根据名称查找参数
const SysConfig::ParamInfo* param = reflector.getParamByName("robot_arm_ip");
if (param) {
    std::string* ip_ptr = static_cast<std::string*>(param->ptr);
    std::cout << "IP地址: " << *ip_ptr << std::endl;
}
```

### ROS集成使用

```cpp
#include "bas_sys_config_ros/calib_info_server.h"

// 创建ROS节点和参数客户端
auto node = rclcpp::Node::make_shared("test_node");
auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);

// 使用增强版函数读取机械臂配置
SysConfig::ArmConfigInfo arm_info_from_server;
bool result = RosComm::getArmConfigInfoFromParamServerEnhanced(
    parameters_client, 
    "sys_arm_list.arm_1", 
    arm_info_from_server);
```

## 扩展支持新的数据结构

要为新的数据结构创建反射器，需要：

1. 继承`ConfigReflector`基类
2. 在构造函数中调用`initReflection`函数
3. 在`initReflection`函数中使用`registerParam`注册所有参数

示例：
```cpp
class NewConfigInfoReflector : public ConfigReflector {
public:
    explicit NewConfigInfoReflector(NewConfigInfo& info) {
        initReflection(info);
    }
    
private:
    void initReflection(NewConfigInfo& info) {
        registerParam("param1", info.param1);
        registerParam("param2", info.param2);
        // 注册更多参数...
    }
};
```

## 编译和构建

确保在CMakeLists.txt中包含了相关源文件：
```cmake
add_library(bas_sys_config_core
  src/sys_config_struct.cpp
  src/config_reflector.cpp
)
```

## 优势

使用ConfigReflector相比传统方法的优势：

1. **减少重复代码**：不再需要手动逐个读取每个参数
2. **提高可维护性**：当数据结构发生变化时，只需更新反射器定义
3. **类型安全**：编译时检查参数类型
4. **易于调试**：可以方便地获取参数打印信息
5. **统一接口**：不同的数据结构使用相同的访问接口

## 注意事项

1. 复杂数据类型（如vector、map等）需要特殊处理
2. 枚举类型需要显式转换为整数类型进行处理
3. 内存管理需要注意，确保参数对象的生命周期长于反射器对象