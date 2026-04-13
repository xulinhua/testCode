# Piper ROS 开发指南

## 项目结构

```
piper_ros/
├── src/                           # 项目源代码根目录
│   ├── CMakeLists.txt             # CMake构建配置文件
│   ├── package.xml                # ROS包描述文件
│   ├── setup.py                   # Python包安装配置
│   ├── requirements.txt           # Python依赖项
│   ├── README.md                  # 项目说明文档
│   ├── DEVELOPMENT_GUIDE.md       # 开发指南（本文档）
│   ├── config/                    # 配置文件目录
│   │   └── tts_config.yaml       # TTS配置文件
│   ├── launch/                    # 启动文件目录
│   │   └── tts_node.launch.py    # 节点启动文件
│   ├── piper_ros/                 # Python包目录
│   │   ├── __init__.py           # 包初始化文件
│   │   └── tts_node.py           # 主要TTS节点实现
│   ├── tests/                     # 测试脚本目录
│   │   ├── test_tts_node.py      # 节点测试脚本
│   │   ├── test_imports.py       # 导入测试脚本
│   │   └── verify_tts_functionality.py # TTS功能验证脚本
```

## 开发环境设置

### 依赖项安装

```bash
# 安装Python依赖
pip install -r requirements.txt

# 确保piper_py项目已正确安装
cd ../../piper_py
pip install -e .

# 确保audio_basic_py项目可用
cd ../../../audio_basic_py
pip install -e .
```

### ROS 2环境配置

确保已安装ROS 2 Humble并正确配置环境：

```bash
# 源化ROS 2环境
source /opt/ros/humble/setup.bash

# 进入项目目录
cd src/tts/pipertts/piper_ros

# 构建项目
colcon build

# 源化工作空间
source install/setup.bash
```

## 代码结构说明

### 主要组件

1. **TTS节点 (tts_node.py)**：
   - 实现ROS 2节点功能
   - 订阅文本消息并发布音频数据
   - 集成Piper TTS引擎
   - 支持音频文件保存功能

2. **配置管理**：
   - 使用YAML配置文件管理参数
   - 支持运行时参数调整

3. **启动文件**：
   - 提供灵活的节点启动方式
   - 支持参数化配置

### 消息类型

- **输入消息**：`std_msgs/String` (话题: `/voice reply`)
- **输出消息**：`audio_common_msgs/AudioData` (话题: `audio/speaker`)

## 开发流程

### 1. 修改代码

在 `src/piper_ros/tts_node.py` 中进行主要的逻辑修改。

### 2. 测试导入

```bash
cd src/tts/pipertts/piper_ros
python tests/test_imports.py
```

### 3. 功能验证

```bash
cd src/tts/pipertts/piper_ros
python tests/verify_tts_functionality.py
```

### 4. 构建和运行

```bash
# 进入项目目录
cd src/tts/pipertts/piper_ros

# 构建项目
colcon build

# 源化环境
source install/setup.bash

# 运行节点
ros2 run piper_ros tts_node

# 或使用launch文件
ros2 launch piper_ros tts_node.launch.py
```

## 测试方法

### 单元测试

目前项目中包含以下测试脚本：

1. **test_imports.py**：验证基本导入是否正常
2. **verify_tts_functionality.py**：验证TTS核心功能是否正常
3. **test_tts_node.py**：测试节点的基本功能

### 集成测试

1. 启动TTS节点
2. 发送测试文本消息
3. 验证音频数据是否正确发布
4. 检查音频文件是否按配置保存

```bash
# 发送测试消息
ros2 topic pub /voice reply std_msgs/msg/String "data: '你好，世界'"
```

## 常见问题和解决方案

### 1. 导入错误

**问题**：无法导入 `rclpy` 或其他ROS 2相关模块

**解决方案**：
- 确保已正确源化ROS 2环境
- 检查是否在ROS 2工作空间中构建项目

### 2. 模型加载失败

**问题**：无法加载Piper模型

**解决方案**：
- 检查模型文件路径是否正确
- 确保模型文件存在且可访问
- 验证模型文件完整性

### 3. 音频数据发布异常

**问题**：音频数据无法正确发布到话题

**解决方案**：
- 检查话题名称是否正确
- 验证消息类型是否匹配
- 确认节点是否正常运行

## 扩展开发

### 添加新功能

1. 在 `src/piper_ros/tts_node.py` 中添加新的参数配置
2. 更新 `src/config/tts_config.yaml` 文件
3. 修改 `src/launch/tts_node.launch.py` 添加新参数
4. 实现新功能的逻辑代码

### 性能优化

1. 考虑使用多线程处理文本消息队列
2. 优化音频数据的处理和发布流程
3. 添加缓存机制避免重复合成相同文本

## 版本管理

### 发布新版本

1. 更新 `src/package.xml` 中的版本号
2. 更新 `src/setup.py` 中的版本号
3. 更新 `src/README.md` 中的版本信息
4. 提交更改并打标签

### 版本兼容性

- 确保与piper_py项目的版本兼容
- 验证与audio_basic_py项目的接口兼容性
- 测试在不同ROS 2环境中的兼容性