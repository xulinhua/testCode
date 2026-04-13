# Piper ROS 封装器

## 项目概述

Piper ROS 是一个基于 ROS 2 Humble 的语音合成封装器项目，用于将纯 Python 实现的 Piper 语音合成引擎集成到 ROS 2 系统中。该项目允许机器人系统通过订阅文本消息来生成语音，并将合成的音频数据通过 ROS 话题发布。

## 项目架构

```
piper_ros/
├── src/                           # 项目源代码根目录
│   ├── CMakeLists.txt             # CMake构建配置文件
│   ├── package.xml                # ROS包描述文件
│   ├── setup.py                   # Python包安装配置
│   ├── requirements.txt           # Python依赖项
│   ├── README.md                  # 项目说明文档
│   ├── DEVELOPMENT_GUIDE.md       # 开发指南
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

## 功能特性

1. **文本到语音合成**：订阅 `/voice reply` 话题接收文本输入，使用 Piper 引擎进行语音合成
2. **音频数据发布**：将合成的音频数据通过 `audio/speaker` 话题发布
3. **音频文件保存**：可配置是否将合成的音频数据保存为 WAV 文件
4. **ROS 2 集成**：完全集成到 ROS 2 Humble 环境中，支持参数配置和动态重配置
5. **音频处理复用**：复用 `audio_basic_py` 项目中的音频处理接口，避免重复开发

## 实现原理

### 核心组件

1. **PiperTTSNode**：主要的 ROS 2 节点类，负责：
   - 订阅文本消息
   - 调用 Piper 语音合成引擎
   - 发布音频数据
   - 保存音频文件（可选）

2. **消息订阅与发布**：
   - 订阅：`/voice reply` (std_msgs/String)
   - 发布：`audio/speaker` (audio_common_msgs/AudioData)

### 工作流程

1. 节点启动时加载 Piper 模型
2. 订阅 `/voice reply` 话题等待文本输入
3. 接收到文本后调用 Piper 引擎进行语音合成
4. 将合成的音频数据封装为 AudioData 消息并发布到 `audio/speaker` 话题
5. 如果启用音频保存功能，将音频数据保存为 WAV 文件

## 配置参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| save_audio_to_file | bool | false | 是否保存合成的音频数据到文件 |
| audio_save_directory | string | ../../resources/audio_data_ros | 音频文件保存目录 |
| model_path | string | ../../resources/models/zh_CN/medium/zh_CN-huayan-medium.onnx | Piper模型路径 |
| config_path | string | ../../resources/models/zh_CN/medium/zh_CN-huayan-medium.onnx.json | Piper模型配置文件路径 |
| sample_rate | int | 22050 | 音频采样率 |
| speaker_id | int | 0 | 说话人ID |

## 安装与构建

### 依赖项

- ROS 2 Humble
- Python 3.10+
- Piper相关依赖（已包含在piper_py项目中）
- audio_common_msgs ROS包

### 构建步骤

```bash
# 进入项目根目录
cd src/tts/pipertts/piper_ros

# 构建项目
colcon build

# 源化环境
source install/setup.bash
```

## 使用方法

### 启动节点

```bash
# 使用默认参数启动
ros2 run piper_ros tts_node

# 使用launch文件启动
ros2 launch piper_ros tts_node.launch.py

# 带参数启动
ros2 launch piper_ros tts_node.launch.py save_audio_to_file:=true
```

### 发送文本消息测试

```bash
# 发送测试文本
ros2 topic pub /voice_reply std_msgs/msg/String "data: '你好，世界'"

# 发送一次测试文本
ros2 topic pub -1 /voice_reply std_msgs/msg/String "data: '你好，新益昌小浩'"

# 发送两次测试文本，间隔7秒
ros2 topic pub -1 /voice_reply std_msgs/msg/String "data: '你好，这是第一次测试'" && sleep 7 && ros2 topic pub -1 /voice_reply std_msgs/msg/String "data: '你好，这是第二次测试'"
```

### 订听音频数据

```bash
# 订听音频数据话题
ros2 topic echo audio/speaker
```

## 测试流程

1. **环境准备**：
   - 确保ROS 2 Humble环境已正确安装
   - 确保piper_py项目已正确构建
   - 确保audio_common_msgs包已安装

2. **构建项目**：
   ```bash
   cd src/tts/pipertts/piper_ros
   colcon build
   source install/setup.bash
   ```

3. **启动节点**：
   ```bash
   ros2 launch piper_ros tts_node.launch.py
   ```

4. **功能测试**：
   - 发送文本消息到 `/voice reply` 话题
   - 验证是否在 `audio/speaker` 话题收到音频数据
   - 检查是否按配置保存了音频文件

5. **性能测试**：
   - 测试不同长度文本的合成时间
   - 验证音频质量
   - 检查资源占用情况

## 开发指南

详细的开发指南请参考 [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) 文件。

## 版本信息

### v1.0.0 (2025-11-11)

- 初始版本发布
- 实现基本的文本到语音合成功能
- 支持ROS 2话题订阅与发布
- 支持音频文件保存功能
- 集成piper_py项目的核心功能

## 开发者信息

- **作者**：AI Assistant
- **版本**：1.0.0
- **许可证**：Apache License 2.0