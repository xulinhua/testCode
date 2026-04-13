# bas_operate 基础操作工具库

## 项目简介

bas_operate 是一个专为 ROS2 项目设计的基础操作工具库，提供常用的工具函数和基础功能实现。该项目旨在简化开发过程，提供跨平台的文件操作和通用工具函数支持。

## 子模块说明

### 1. 文件操作模块 (file_operate)
提供文件和目录操作功能，包括：
- 目录创建与验证
- 路径处理与解析
- 项目名称验证
- 可执行文件路径获取
- 安装目录路径解析

### 2. 基础工具模块 (bas_utils)
提供常用的工具函数集合，封装在 `basmodule` 命名空间中：
- **字符串工具**：trim、大小写转换、分割、替换、前缀后缀检查
- **数学工具**：随机数生成、浮点数比较、角度转换
- **时间工具**：时间戳获取、格式化、时间差计算
- **容器工具**：元素查找、过滤、映射

### 3. 测试模块
包含单元测试代码，验证各工具函数的正确性和稳定性。

## 构建环境

- **操作系统**：Ubuntu 22.04
- **ROS版本**：ROS 2 Humble
- **硬件平台**：Jetson Orin NX Super 开发板
- **编译器**：支持 C++17 标准的编译器

## 构建与安装

### 构建项目
```bash
cd /path/to/your/workspace
colcon build --packages-select bas_operate
```

### 安装项目
```bash
source install/setup.bash
```

## 功能测试

### 运行测试程序
```bash
# 运行 bas_utils 测试
ros2 run bas_operate bas_utils_test
```

### 测试内容
测试程序会验证以下功能：

1. **字符串工具函数**
   - `trim()` - 去除字符串首尾空白字符
   - `to_lower()` / `to_upper()` - 字符串大小写转换
   - `split()` - 字符串分割
   - `replace()` - 字符串替换
   - `starts_with()` / `ends_with()` - 前缀后缀检查

2. **数学工具函数**
   - `random_int()` / `random_double()` - 随机数生成
   - `float_equal()` - 浮点数比较
   - `degrees_to_radians()` / `radians_to_degrees()` - 角度转换

3. **时间工具函数**
   - `get_timestamp_ms()` - 获取当前时间戳
   - `format_timestamp()` - 时间格式化
   - `duration_ms()` - 时间差计算

4. **容器工具函数**
   - `contains()` - 元素查找
   - `filter()` - 容器过滤
   - `map()` - 容器映射

## 使用示例

### 在其他项目中使用 bas_operate

1. 在 CMakeLists.txt 中添加依赖：
```cmake
find_package(bas_operate REQUIRED)

# 链接库
ament_target_dependencies(your_target bas_operate)
target_link_libraries(your_target bas_operate::bas_operate)
```

2. 在代码中包含头文件：
```cpp
#include "bas_operate/file_operate.hpp"
#include "bas_operate/bas_utils.hpp"
#include "data_handler/param_reflector.hpp"

// 使用命名空间
using namespace basmodule;

// 示例：使用字符串工具
std::string trimmed = trim("  hello world  ");
std::string lower = to_lower("HELLO WORLD");

// 示例：使用参数反射器
struct MyConfig {
    bool enable;
    int value;
    std::string name;
};

// 创建自定义反射器
class MyConfigReflector : public ConfigReflector {
public:
    explicit MyConfigReflector(MyConfig& config) {
        registerParam("enable", config.enable);
        registerParam("value", config.value);
        registerParam("name", config.name);
    }
};

// 使用反射器
MyConfig config{true, 42, "test"};
MyConfigReflector reflector(config);
reflector.getParamsMsg();
```

## 注意事项

1. 所有工具函数均定义在 `basmodule` 命名空间中
2. 项目使用 C++17 标准，确保编译器支持相应特性
3. 在 Jetson Orin NX Super 开发板上经过充分测试
4. 构建时需要 ROS 2 Humble 环境支持

## 维护信息

- **作者**：chenwang
- **最后更新**：2025年12月19日