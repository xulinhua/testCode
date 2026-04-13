# Audio I/O Module ROS Wrapper - NVIDIA Jetson Orin NX Super

ROS 2 Humble封装的音频I/O模块，提供实时音频采集和播放功能，专为NVIDIA Jetson Orin NX Super开发板设计。

## 概述

该ROS 2包封装了`audio_io_module`的核心功能：
- `audio_publisher`: 从麦克风实时采集音频数据并发布到ROS话题
- `audio_subscriber`: 订阅ROS话题的音频数据并通过扬声器播放

## 功能特性

- 麦克风设备的实时音频采集
- 音频数据发布到ROS话题，供ASR（自动语音识别）处理
- 订阅音频数据话题并通过扬声器播放
- 音频设备和参数的动态配置
- 专为NVIDIA Jetson Orin NX Super开发板优化
- 支持Ubuntu 22.04 + ROS 2 Humble环境

## 系统要求

- **硬件平台**: NVIDIA Jetson Orin NX Super开发板
- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS 2 Humble
- **Python版本**: Python 3.10+
- **依赖包**: PyAudio
- **音频模块**: audio_io_module（位于父目录）

## 项目结构

```
\\192.168.11.129\testCode\ai_speech\speech\audio_io_module_ros/
├── src/                        # 源代码目录
│   ├── audio_io_module_ros/     # Python包
│   │   ├── __init__.py
│   │   ├── audio_publisher.py   # 音频发布节点
│   │   ├── audio_subscriber.py  # 音频订阅节点
│   │   └── msg/                 # 自定义消息目录
│   ├── config/                  # 配置文件目录
│   ├── launch/                  # 启动文件目录
│   │   └── audio_io_launch.py   # 主启动文件
│   ├── resource/               # 资源文件目录
│   ├── test/                   # 测试文件目录
│   ├── package.xml             # ROS包定义文件
│   ├── setup.cfg               # Python包配置
│   └── setup.py                # Python安装脚本
├── build/                      # 构建输出目录
├── install/                    # 安装目录（构建后生成）
└── log/                        # 构建日志目录
```

## 安装和配置

### 1. 环境准备

确保ROS 2 Humble已正确安装和配置：
```bash
# 加载ROS 2环境
source /opt/ros/humble/setup.bash
```

### 2. 依赖安装

安装PyAudio依赖：
```bash
# 安装PyAudio
pip install pyaudio

# 或者使用系统包管理器
sudo apt-get update
sudo apt-get install python3-pyaudio
```

### 3. 构建包（关键步骤）

**重要提示**: 在audio_io_module_ros目录下直接构建，无需复制到ROS工作空间：
```bash
# 进入项目根目录
cd ~/testCode/ai_speech/speech/audio_io_module_ros

# 删除之前的构建结果（确保干净构建）
rm -rf build install log

# 使用colcon构建包
colcon build

# 加载构建后的环境（这步最重要！）
source install/setup.bash
```

### 4. 环境验证

验证包是否被正确识别：
```bash
# 检查ROS包列表
ros2 pkg list | grep audio_io_module_ros

# 检查包信息
ros2 pkg prefix audio_io_module_ros

# 检查可执行文件
ros2 pkg executables audio_io_module_ros

# 检查包的ROS路径（关键验证）
echo $ROS_PACKAGE_PATH | grep audio_io_module_ros
```

## 使用说明

### 启动节点（推荐方式）

```bash
# 在项目根目录下执行
cd ~/testCode/ai_speech/speech/audio_io_module_ros
source install/setup.bash
ros2 launch audio_io_module_ros audio_io_launch.py
```

### 手动启动节点

```bash
# 启动音频发布节点
ros2 run audio_io_module_ros audio_publisher

# 启动音频订阅节点
ros2 run audio_io_module_ros audio_subscriber
```

### 参数配置

#### 音频发布节点参数：
- `mic_device_index`: 麦克风设备索引（默认：24，对应USB麦克风）
- `sample_rate`: 音频采样率（默认：44100）
- `channels`: 音频声道数（默认：1，单声道）
- `chunk_size`: 音频块大小（默认：1024）

#### 音频订阅节点参数：
- `speaker_device_index`: 扬声器设备索引（默认：25，对应USB扬声器）
- `sample_rate`: 音频采样率（默认：44100）
- `channels`: 音频声道数（默认：1，单声道）

### 设备配置

包已针对Jetson Orin NX Super的特定USB音频设备进行配置：
- USB麦克风: hw:2,0（PyAudio索引24）
- USB扬声器: hw:3,0（PyAudio索引25）

## 常见问题解决

### 问题1: "Package 'audio_io_module_ros' not found"

**错误示例**:
```
Package 'audio_io_module_ros' not found: "package 'audio_io_module_ros' not found, searching: ['/home/user/OrbbecSDK/orbbec_ws/install/orbbec_description', ...]"
```

**原因**: 环境未正确加载，包未在ROS包路径中注册。**主要原因是在构建后没有执行 `source install/setup.bash`**。

**解决方案**:
```bash
# 确保在项目根目录下
cd ~/testCode/ai_speech/speech/audio_io_module_ros

# 检查是否已构建
ls -la install/

# 如果install目录存在，直接加载环境（这是最关键的一步！）
source install/setup.bash

# 验证包是否可用
ros2 pkg list | grep audio_io_module_ros

# 如果包不存在，重新构建
rm -rf build install log
colcon build
source install/setup.bash

# 检查包的安装路径
ros2 pkg prefix audio_io_module_ros

# 检查可执行文件是否生成
ros2 pkg executables audio_io_module_ros
```

**关键检查点**:
- 确保 `$ROS_PACKAGE_PATH` 包含您的包路径
- 确保执行了 `source install/setup.bash`

### 问题2: 音频设备无法访问

**解决方案**: 检查音频设备权限和连接状态：
```bash
# 查看可用的音频设备
arecord -l

# 检查PyAudio可检测的设备
python3 -c "import pyaudio; p = pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)['name']}') for i in range(p.get_device_count())]; p.terminate()"

# 如果权限不足，将用户添加到音频组
sudo usermod -a -G audio $USER

# 重新登录或重新加载组权限
newgrp audio
```

### 问题3: 依赖模块导入失败

**错误示例**:
```
无法导入audio_io_module: No module named 'audio_io_module'
请确保audio_io_module目录在正确位置
```

**原因**: audio_io_module模块未被正确安装到ROS环境中，导致ROS节点无法找到这个模块。

**解决方案**:
1. 确保audio_io_module目录与audio_io_module_ros目录位于同一父目录下
2. 重新构建项目，确保setup.py正确配置了依赖关系
3. 确保执行了 `source install/setup.bash` 加载环境

```bash
# 检查audio_io_module目录结构
ls -la ~/testCode/ai_speech/speech/audio_io_module/

# 检查Python路径设置
python3 -c "import sys; print(sys.path)"

# 重新构建项目
cd ~/testCode/ai_speech/speech
colcon build

# 加载环境
source install/setup.bash
```

### 问题4: "ModuleNotFoundError: No module named 'audio_io_module'"

**原因**: 这是由于audio_io_module模块没有被正确安装到ROS环境中。

**解决方案**: 
1. 修改setup.py文件，将audio_io_module作为数据文件包含在安装包中
2. 在节点文件中使用正确的路径导入audio_io_module模块

```bash
# 重新构建项目
cd ~/testCode/ai_speech/speech
colcon build

# 加载环境
source install/setup.bash
```

### 问题5: "'data_files' must be relative" 构建错误

**错误示例**:
```
'data_files' must be relative, '/home/user/testCode/ai_speech/speech/src/audio_io_module_ros/../audio_io_module/test_resample_playback.py' is absolute
```

**原因**: 在setup.py的data_files配置中使用了绝对路径，而colcon要求使用相对路径。

**解决方案**:
1. 移除setup.py中[data_files](file:///192.168.11.129/testCode\ai_speech\speech\src\audio_io_module_ros\setup.py#L17-L39)配置中对audio_io_module文件的引用
2. 修改节点文件中的导入逻辑，使用相对路径或安装路径查找audio_io_module

```bash
# 重新构建项目
cd ~/testCode/ai_speech/speech
colcon build

# 加载环境
source install/setup.bash
```

### 问题6: "无法导入audio_io_module: 无法找到audio_io_module目录"

**错误示例**:
```
无法导入audio_io_module: 无法找到audio_io_module目录
请确保audio_io_module目录在正确位置
```

**原因**: 节点运行时无法在预期路径找到已安装的audio_io_module模块。

**解决方案**:
1. 确保audio_io_module项目正确配置了setup.py，包含py_modules配置
2. 重新构建整个工作空间，确保audio_io_module被正确安装
3. 修改节点文件中的导入逻辑，使用更可靠的路径查找机制

```bash
# 重新构建项目
cd ~/testCode/ai_speech/speech
colcon build

# 加载环境
source install/setup.bash
```

### 问题7: "无法导入audio_io_module: No module named 'audio_io_module'"

**错误示例**:
```
无法导入audio_io_module: No module named 'audio_io_module'
请确保audio_io_module目录在正确位置
```

**原因**: audio_io_module被安装为独立的Python模块而不是包，导入方式不正确。

**解决方案**:
1. 修改节点文件中的导入方式，从`from audio_io_module.audio_io_manager import AudioIOManager`改为`from audio_io_manager import AudioIOManager`
2. 重新构建整个工作空间
3. 确保执行了 `source install/setup.bash` 加载环境

```bash
# 重新构建项目
cd ~/testCode/ai_speech/speech
colcon build

# 加载环境
source install/setup.bash
```

### 问题8: 节点启动正常但无音频反应

**错误示例**:
```
[INFO] [1762576268.700846487] [audio_subscriber]: 音频管理器初始化成功，使用扬声器设备索引: 25
[INFO] [1762576268.700850199] [audio_publisher]: 音频管理器初始化成功，使用麦克风设备索引: 24
[INFO] [1762576268.708609005] [audio_subscriber]: 音频订阅节点已启动
[INFO] [1762576268.723763792] [audio_publisher]: 音频发布节点已启动
```

**原因**: 节点虽然正常启动，但可能存在以下问题：
1. 音频设备索引配置不正确
2. 采样率设置与设备不兼容
3. 音频数据未正确发布或订阅
4. ALSA配置问题导致音频无法正常处理

**解决方案**:
1. 检查设备索引是否正确，可通过以下命令查看：
   ```bash
   python3 -c "import pyaudio; p = pyaudio.PyAudio(); [print(f'{i}: {p.get_device_info_by_index(i)['name']}') for i in range(p.get_device_count())]; p.terminate()"
   ```
2. 修改采样率参数，尝试使用设备支持的采样率
3. 检查日志输出，确认音频数据是否被正确发布和接收
4. 忽略ALSA警告信息，这些通常不影响基本功能

```bash
# 重新启动节点并观察详细日志
ros2 launch audio_io_module_ros audio_io_launch.py
```

## 故障排除流程

如果遇到问题，请按以下步骤排查：

1. **检查基础环境**:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 pkg list | grep audio_io_module_ros
   ```

2. **重新构建**:
   ```bash
   cd ~/testCode/ai_speech/speech/audio_io_module_ros
   colcon build
   source install/setup.bash
   ```

3. **验证包注册**:
   ```bash
   ros2 pkg list | grep audio_io_module_ros
   ros2 pkg prefix audio_io_module_ros
   ```

4. **测试启动**:
   ```bash
   ros2 launch audio_io_module_ros audio_io_launch.py
   ```

## 架构说明

该包采用标准的ROS 2 Python包架构：
- 使用`ament_python`构建系统
- 遵循ROS 2包规范
- 支持动态参数配置
- 提供完整的launch文件支持

## 多项目集成与构建流程

### 项目结构说明

本项目采用多项目集成架构：
- `audio_io_module`: 跨平台的纯Python音频处理模块（无ROS依赖）
- `audio_io_module_ros`: ROS 2包装器，调用audio_io_module的接口

两个项目位于同一工作空间：
```
~/testCode/ai_speech/speech/
├── audio_io_module/          # 跨平台音频模块
├── audio_io_module_ros/      # ROS 2包装器
├── install/                  # 构建输出
└── log/                      # 构建日志
```

### 多项目构建方案

#### 方案1：工作空间级构建（推荐）

在speech目录下同时构建两个项目：
```bash
# 进入工作空间根目录
cd ~/testCode/ai_speech/speech

# 清理之前的构建结果
rm -rf build install log

# 构建所有包（audio_io_module_ros会自动依赖audio_io_module）
colcon build

# 加载环境
source install/setup.bash

# 验证包是否可用
ros2 pkg list | grep audio_io_module_ros
```

### 集成技术实现

`audio_io_module_ros`项目通过以下方式集成`audio_io_module`：

1. **Python路径**: 在运行时自动添加`audio_io_module`到Python路径
2. **导入方式**: 在ROS节点中通过相对路径或安装路径导入`audio_io_module`的功能

示例导入代码：
```python
# 在audio_io_module_ros的Python文件中导入
from audio_io_manager import AudioIOManager
```

### 构建配置说明

当前的`setup.py`已配置为支持多项目集成：
- `audio_io_module`的`setup.py`中添加了`py_modules`配置，确保模块文件被正确安装为独立模块
- 在节点文件中使用正确的导入方式导入模块

### 优势特点

1. **保持跨平台性**: `audio_io_module`保持纯Python项目，无ROS依赖
2. **项目独立性**: 两个项目可以独立开发和测试
3. **构建灵活性**: 支持工作空间级构建和单独构建
4. **维护简便**: 修改任何项目代码都能自动反映到集成中

## 与audio_io_module的集成

该包作为`audio_io_module`项目的ROS包装器，通过上述集成方案实现以下核心功能：
- 音频采集和播放
- 设备管理
- 采样率转换
- 音量控制

ROS包装器将这些功能暴露为ROS节点，可以集成到更大的机器人系统中。

## 许可证

本项目采用Apache License 2.0许可证。