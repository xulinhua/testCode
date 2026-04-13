# ROS 2 音频处理工具包 (audio_common)
## 项目信息
- **公司**: hoson 
- **项目简介**: 这是一个基于PortAudio的ROS 2音频处理工具包，提供了完整的音频采集、播放和文本转语音(TTS)功能。该工具包支持在Jetson Orin NX Super开发板等嵌入式平台上运行，适用于机器人音频处理应用场景。

## 目录

1. [项目概述](#项目概述)
2. [系统架构](#系统架构)
3. [环境要求](#环境要求)
4. [安装说明](#安装说明)
   - [创建并激活Python虚拟环境](#创建并激活python虚拟环境)
   - [安装系统依赖](#安装系统依赖)
   - [克隆项目](#克隆项目)
   - [安装Python依赖](#安装python依赖)
   - [构建项目](#构建项目)
   - [源化工作空间](#源化工作空间)
5. [Docker支持](#docker支持)
6. [节点详解](#节点详解)
   - [audio_capturer_node](#audio_capturer_node)
   - [audio_player_node](#audio_player_node)
   - [music_node](#music_node)
   - [tts_node](#tts_node)
7. [配置说明](#配置说明)
8. [使用示例](#使用示例)
   - [音频采集与播放](#音频采集与播放)
   - [文本转语音](#文本转语音)
   - [音乐播放](#音乐播放)
9. [在Jetson平台部署](#在jetson平台部署)
   - [环境要求](#环境要求-1)
   - [问题诊断与解决方案](#问题诊断与解决方案)
   - [构建问题诊断与解决方案](#构建问题诊断与解决方案)
   - [运行验证](#运行验证)
   - [功能测试方法](#功能测试方法)
   - [Docker部署](#docker部署)
   - [TTS](#tts)
   - [Music Player](#music-player)
10. [唤醒词检测功能](#唤醒词检测功能)
    - [工作原理](#工作原理)
    - [功能特点](#功能特点)
    - [配置参数](#配置参数)
    - [使用示例](#使用示例-1)
    - [唤醒词检测测试](#唤醒词检测测试)
    - [Snowboy配置说明](#snowboy配置说明)

## 项目概述

audio_common是一个功能完整的ROS 2音频处理工具包，包含以下核心功能：

- **音频采集**：从麦克风设备采集音频数据并发布到ROS 2话题
- **音频播放**：订阅音频话题并播放音频数据
- **音乐播放**：播放本地WAV格式音频文件
- **文本转语音(TTS)**：将文本转换为音频并播放
- **唤醒词检测**：基于Snowboy的唤醒词检测功能，支持自定义唤醒词

该工具包专为机器人应用场景设计，支持灵活的设备配置和参数化设置。

## 系统架构

```
                    +-----------------+
                    |  audio_common   |
                    |     包          |
                    +--------+--------+
                             |
        +--------------------+--------------------+
        |                    |                    |
+-------v-------+    +-------v-------+    +-------v-------+
| 音频采集节点  |    | 音频播放节点  |    | 音频服务节点  |
| (Capturer)    |    |  (Player)     |    | (Music/TTS)   |
+-------+-------+    +-------+-------+    +-------+-------+
        |                    |                    |
        |                    |                    |
+-------v-------+    +-------v-------+    +-------v-------+
| PortAudio库   |    | PortAudio库   |    | PortAudio库   |
| 音频接口      |    | 音频接口      |    | 音频接口      |
+---------------+    +---------------+    +---------------+


                    +-----------------+
                    | ROS 2 系统      |
                    | 消息传递机制    |
                    +--------+--------+
                             |
                     +-------v-------+
                     |  audio_common_msgs 包  |
                     |  (自定义消息类型)      |
                     +-----------------------+
```

## 环境要求

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS 2 Humble
- **Python版本**: Python 3.10.12 (在speech_env虚拟环境中运行)
- **编译工具**: colcon, cmake, gcc
- **音频库**: PortAudio, ALSA
- **TTS引擎**: espeak-ng (可选，用于TTS节点的直接内存生成模式)

## 安装说明

### 创建并激活Python虚拟环境

```shell
# 创建虚拟环境
python3 -m venv speech_env

# 激活虚拟环境
source speech_env/bin/activate
```

### 安装系统依赖

```shell
# 安装PortAudio开发库
sudo apt-get update
sudo apt-get install portaudio19-dev python3-pyaudio python3-numpy

# 安装espeak-ng开发库（可选，用于TTS节点的直接内存生成模式）
sudo apt-get install espeak-ng espeak-ng-dev
```
# 安装ROS 2 Humble (如果尚未安装)
# 请参考官方文档: https://docs.ros.org/en/humble/Installation.html
```

### 克隆项目

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
```

### 安装Python依赖

在speech_env虚拟环境中手动安装以下依赖项：

```shell
# 激活虚拟环境
source speech_env/bin/activate

# 安装Python依赖
pip3 install pyaudio numpy
```

### 构建项目

```shell
cd ~/ros2_ws
# 安装ROS依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建项目
colcon build --packages-select audio_common
```

### 源化工作空间

```shell
source install/setup.bash
```

## Docker支持

您可以创建一个Docker镜像来测试audio_common。在audio_common目录下使用以下命令：

```shell
docker build -t audio_common .
```

镜像创建完成后，使用以下命令运行Docker容器：

```shell
docker run -it --rm --device /dev/snd audio_common
```

## 节点详解

### audio_capturer_node

音频采集节点，从麦克风设备采集音频数据并发布到`audio/microphone`话题。该节点支持唤醒词检测功能，可以识别特定的唤醒词并控制音频数据的发布。

<details>
<summary>点击展开详细信息</summary>

#### 参数说明

- **format**: 音频采集格式，支持PortAudio定义的格式如`paInt16` (16-bit)等。默认值: `paInt16`
- **channels**: 音频通道数，`1`表示单声道，`2`表示立体声。默认值: `1`
- **rate**: 采样率，每秒采集的样本数。默认值: `16000`
- **chunk**: 每帧音频数据的大小。默认值: `512`
- **device**: 音频输入设备ID，`-1`表示使用默认输入设备。默认值: `-1`
- **device_name**: 音频设备名称，如`hw:2,0`，优先级高于device参数。默认值: `""`
- **frame_id**: 音频帧标识符，用于与其他数据流同步。默认值: `""`
- **wake_word.enabled**: 是否启用唤醒词检测功能。默认值: `true`
- **wake_word.model_path**: 唤醒词模型文件路径。默认值: `"src/snowboy/src/resources/models/hoson.pmdl"`
- **wake_word.resource_path**: 唤醒词资源文件路径。默认值: `"src/snowboy/src/resources/common.res"`
- **wake_word.sensitivity**: 唤醒词检测灵敏度(0.0-1.0)。默认值: `0.5`

#### ROS 2 接口

- **发布话题**: `audio/microphone` - 发布从麦克风采集的音频数据
  - 消息类型: `audio_common_msgs/msg/AudioStamped`

</details>

### audio_player_node

音频播放节点，订阅音频话题并播放音频数据。

<details>
<summary>点击展开详细信息</summary>

#### 参数说明

- **channels**: 音频通道数，`1`表示单声道，`2`表示立体声。默认值: `2`
- **device**: 音频输出设备ID，`-1`表示使用默认输出设备。默认值: `-1`
- **device_name**: 音频设备名称，如`hw:3,0`，优先级高于device参数。默认值: `""`

#### ROS 2 接口

- **订阅话题**: `audio/speaker` - 订阅需要播放的音频数据
  - 消息类型: `audio_common_msgs/msg/AudioStamped`

- **发布话题**: `audio/speaker_status` - 发布播放器状态
  - 消息类型: `std_msgs/msg/Bool`

</details>

### music_node

音乐播放节点，播放本地WAV格式音频文件。

<details>
<summary>点击展开详细信息</summary>

#### 参数说明

- **chunk_time**: 每个音频块的持续时间(毫秒)。默认值: `50`
- **frame_id**: 音频帧标识符，用于与其他数据流同步。默认值: `""`

#### ROS 2 接口

- **发布话题**: `audio/music` - 发布从文件读取的音频数据
  - 消息类型: `audio_common_msgs/msg/AudioStamped`

- **服务接口**: `/music_play` - 播放指定音乐文件的服务
  - 服务类型: `audio_common_msgs/srv/MusicPlay`

</details>

### tts_node

文本转语音节点，将输入文本转换为音频并播放。

<details>
<summary>点击展开详细信息</summary>

#### 参数说明

- **chunk**: 每个音频帧的大小。默认值: `4096`
- **frame_id**: 音频帧标识符，用于与其他数据流同步。默认值: `""`
- **save_audio_files**: 是否保存生成的音频文件到当前目录。默认值: `false`

#### ROS 2 接口

- **订阅话题**: `/voice_reply` - 订阅需要转换为语音的文本消息
  - 消息类型: `std_msgs/msg/String`

- **发布话题**: `audio/speaker` - 发布TTS生成的音频数据
  - 消息类型: `audio_common_msgs/msg/AudioStamped`

- **动作接口**: `/say` - 将文本转换为语音的动作接口
  - 动作类型: `audio_common_msgs/action/TTS`

</details>

## 配置说明

项目支持通过YAML配置文件进行设备参数设置。配置文件位于`config/device.yaml`：

```
# USB音频设备配置
audio_capturer:
  device_name: "hw:2,0"  # USB麦克风设备号，默认为 hw:2,0
  rate: 16000           # 采样率(Hz)
  channels: 1           # 声道数(1=单声道, 2=立体声)
  chunk: 512            # 数据块大小

audio_player:
  device_name: "hw:3,0"  # USB扬声器设备号，默认为 hw:3,0
  channels: 2           # 声道数(1=单声道, 2=立体声)

# 唤醒词检测配置
wake_word:
  enabled: true                             # 是否启用唤醒词检测功能
  model_path: "src/snowboy/src/resources/models/hoson.pmdl"   # 唤醒词模型文件路径
  resource_path: "src/snowboy/src/resources/common.res"       # 资源文件路径
  sensitivity: 0.5                          # 检测灵敏度(0.0-1.0)
```

使用配置文件启动节点的示例：

```shell
# 启动音频采集节点
ros2 run audio_common audio_capturer_node --ros-args --params-file config/device.yaml

# 启动音频播放节点
ros2 run audio_common audio_player_node --ros-args --params-file config/device.yaml
```

## 使用示例

### 音频采集与播放

```shell
# 终端1: 启动音频采集节点
ros2 run audio_common audio_capturer_node --ros-args -p device_name:=hw:2,0

# 终端2: 启动音频播放节点
ros2 run audio_common audio_player_node --ros-args -p device_name:=hw:3,0
```

### 文本转语音

```shell
# 注意：此方案需要先安装espeak-ng和sox工具
# Ubuntu/Debian系统安装命令：
sudo apt-get install espeak-ng sox

# 终端1: 启动Piper TTS节点（推荐方案）
ros2 launch piper_ros tts_node.launch.py

# 终端2: 启动音频播放节点
ros2 run audio_common audio_player_node --ros-args -p device_name:=hw:3,0

# 终端3: 发送文本消息进行测试
ros2 topic pub /voice_reply std_msgs/msg/String "data: '我在'"

# 或者使用Python脚本自动发送测试消息
# 终端3: 运行测试脚本
python3 src/speech/audio_common/audio_common/test/tts_test.py
```

### 文本转语音（备用方案 - 需要安装espeak-ng）

```
# 注意：此方案需要先安装espeak-ng和sox工具
# Ubuntu/Debian系统安装命令：
# sudo apt-get install espeak-ng sox

# 终端1: 启动TTS节点（启用音频文件保存功能）
ros2 run audio_common tts_node --ros-args -p save_audio_files:=true

# 终端2: 启动音频播放节点
ros2 run audio_common audio_player_node --ros-args -p device_name:=hw:3,0

# 终端3: 发送TTS请求（通过动作接口）
ros2 action send_goal /say audio_common_msgs/action/TTS "{text: '你好，世界'}"

# 或者直接发布文本消息到/voice_reply话题进行测试
# 终端3: 发送文本消息进行测试
ros2 topic pub /voice_reply std_msgs/msg/String "data: '我在'"

# 或者使用Python脚本自动发送测试消息
# 终端3: 运行测试脚本
python3 src/speech/audio_common/audio_common/test/tts_test.py
```

### 音乐播放

```shell
# 终端1: 启动音乐播放节点
ros2 run audio_common music_node

# 终端2: 启动音频播放节点
ros2 run audio_common audio_player_node --ros-args -p device_name:=hw:3,0

# 终端3: 播放音乐
ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: 'elevator'}"
```

## 在Jetson平台部署

### 环境要求

- 系统: Ubuntu 22.04
- ROS版本: ROS 2 Humble
- 音频设备: USB麦克风(hw:2,0)和USB扬声器(hw:3,0)

### 问题诊断与解决方案

#### 问题现象
当使用 `ros2 run audio_common tts_node` 启动TTS节点并发送文本消息时，出现以下错误：
```
sh: 1: espeak-ng: not found
[ERROR] [1763608876.943848912] [tts_node]: TTS命令执行失败
```

#### 问题原因
[audio_common](file:///192.168.11.129/testCode/ai_speech/speech/src/audio_common) 包中的 [tts_node.cpp](file:///192.168.11.129/testCode/ai_speech/speech/src/audio_common/audio_common/src/audio_common/tts_node.cpp) 节点依赖于系统中的 `espeak-ng` 命令行工具和 `sox` 音频处理工具。如果系统中未安装这些工具，就会出现命令找不到的错误。

#### 解决方案
1. **推荐方案（使用Piper TTS）**：
   项目已集成基于 Piper 的 TTS 节点，该方案不依赖外部命令行工具，具有更好的性能和兼容性。
   ```shell
   # 启动Piper TTS节点
   ros2 launch piper_ros tts_node.launch.py
   ```

2. **备用方案（安装依赖工具）**：
   如果需要使用 [audio_common](file:///192.168.11.129/testCode/ai_speech/speech/src/audio_common) 包中的TTS节点，需要先安装依赖工具：
   ```shell
   # Ubuntu/Debian系统安装命令
   sudo apt-get update
   sudo apt-get install espeak-ng sox
   ```

### 音频播放无声问题诊断与解决方案

#### 问题现象
TTS节点成功生成音频数据，但音频播放节点出现以下错误：
```
[ERROR] [1763609695.414282967] [audio_player_node]: PortAudio write error: Invalid stream pointer
```

#### 问题原因
1. **PortAudio流指针无效**：音频播放节点在尝试写入音频数据时，使用的流指针无效或已损坏
2. **音频格式不匹配**：TTS节点发布的音频格式与音频播放节点期望的格式不兼容
3. **采样率处理问题**：音频数据采样率与设备采样率不匹配时的处理逻辑存在问题

#### 解决方案
1. **重启音频播放节点**：
   ```shell
   # 停止当前的音频播放节点
   # 然后重新启动
   ros2 run audio_common audio_player_node --ros-args -p device_name:=hw:3,0
   ```

2. **检查音频设备状态**：
   ```shell
   # 列出所有音频设备
   aplay -l
   
   # 测试扬声器是否正常工作
   aplay -D hw:3,0 /path/to/test.wav
   ```

3. **使用推荐的Piper TTS方案**：
   Piper TTS节点与音频播放节点有更好的兼容性
   ```shell
   # 启动Piper TTS节点
   ros2 launch piper_ros tts_node.launch.py
   
   # 启动音频播放节点
   ros2 run audio_common audio_player_node --ros-args -p device_name:=hw:3,0
   
   # 发送测试文本
   ros2 topic pub /voice_reply std_msgs/msg/String "data: '我在'"
   ```

### 构建问题诊断与解决方案

#### 问题现象
在构建audio_common包时出现以下错误：
```
CMake Error at /usr/share/cmake-3.22/Modules/FindPkgConfig.cmake:603 (message):
  A required package was not found
```

#### 问题原因
CMake无法正确找到espeak-ng库，这通常发生在以下情况：
1. 系统中未安装espeak-ng开发包
2. CMakeLists.txt文件配置不正确
3. pkg-config工具未正确配置

#### 解决方案
1. **安装espeak-ng开发包**：
   ```shell
   # 安装espeak-ng开发库
   sudo apt-get update
   sudo apt-get install espeak-ng espeak-ng-dev
   
   # 验证安装
   pkg-config --libs espeak-ng
   ```

2. **检查CMakeLists.txt配置**：
   项目已修复CMakeLists.txt文件，添加了更健壮的库查找机制：
   - 使用pkg_check_modules进行可选查找
   - 如果pkg-config找不到，则手动查找头文件和库文件
   - 添加了适当的警告信息，当找不到espeak-ng时会回退到命令行版本

3. **清理并重新构建**：
   ```shell
   # 清理构建目录
   rm -rf build install log
   
   # 重新构建
   colcon build --packages-select audio_common
   ```

### 运行验证

```shell
# 检查音频设备
ros2 run audio_common audio_capturer_node --ros-args -p device_name:=hw:2,0 --ros-args --log-level INFO

# 验证音频采集与播放
ros2 launch audio_common audio_test.launch.py
```

### 功能测试方法

#### 1. 验证音频采集功能

```shell
# 终端1: 启动音频采集节点
ros2 run audio_common audio_capturer_node --ros-args --params-file config/device_config.yaml

# 终端2: 查看采集到的音频数据
ros2 topic echo /audio/microphone

# 终端3: 检查话题发布频率
ros2 topic hz /audio/microphone
```

#### 2. 验证音频播放功能

```shell
# 终端1: 启动音频播放节点
ros2 run audio_common audio_player_node --ros-args --params-file config/device_config.yaml

# 终端2: 发送测试音频数据
ros2 topic pub /audio/speaker audio_common_msgs/msg/AudioStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, audio: {info: {format: 16, channels: 1, chunk: 512, rate: 16000}, audio_data: {float32_data: [], int32_data: [], int16_data: [1000, -1000, 1000, -1000], int8_data: [], uint8_data: []}}}}"
```

#### 3. 使用系统工具测试音频设备

```shell
# 列出所有音频设备
arecord -l
aplay -l

# 测试麦克风录制
arecord -D hw:2,0 -f cd test.wav

# 测试扬声器播放
aplay -D hw:3,0 test.wav

# 实时监听麦克风输入
arecord -D hw:2,0 -f cd - | aplay -D hw:3,0 -
```

#### 4. 配置参数说明

配置文件 `config/device_config.yaml` 包含以下参数：

```yaml
# USB音频设备配置
audio_capturer:
  device_name: "hw:2,0"  # USB麦克风设备号
  rate: 16000           # 采样率(Hz)
  channels: 1           # 声道数(1=单声道, 2=立体声)
  chunk: 512            # 数据块大小

audio_player:
  device_name: "hw:3,0"  # USB扬声器设备号
  channels: 2           # 声道数(1=单声道, 2=立体声)

# 唤醒词检测配置
wake_word:
  enabled: true                             # 是否启用唤醒词检测功能
  model_path: "src/snowboy/src/resources/models/hoson.pmdl"   # 唤醒词模型文件路径
  resource_path: "src/snowboy/src/resources/common.res"       # 资源文件路径
  sensitivity: 0.5                          # 检测灵敏度(0.0-1.0)
```

### Docker部署

项目提供Docker支持，便于在不同环境中部署：

```shell
# 构建Docker镜像
docker build -t audio_common .

# 运行容器(需挂载音频设备)
docker run -it --rm --device /dev/snd audio_common
```

### TTS

```shell
ros2 run audio_common tts_node
```

```shell
ros2 run audio_common audio_player_node
```

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World'}"
```

### Music Player

```shell
ros2 run audio_common music_node
```

```shell
ros2 run audio_common audio_player_node
```

```shell
ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: 'elevator'}"
```

## 唤醒词检测功能

audio_capturer_node节点支持基于Snowboy的唤醒词检测功能。该功能默认启用，当检测到预设的唤醒词时，会开启播报模式并开始发布音频数据。

### 工作原理

1. 默认情况下，唤醒词检测功能处于启用状态
2. 当检测到唤醒词时，系统进入播报模式
3. 在播报模式下，实时采集的音频数据会被发布到`audio/microphone`话题
4. 如果未检测到唤醒词，系统处于静默模式，不会发布音频数据
5. 用户可以通过配置参数控制是否启用唤醒词检测功能

### 功能特点

- **模块化设计**：唤醒词检测功能与音频采集功能解耦，便于维护和扩展
- **灵活配置**：支持通过参数配置唤醒词模型路径、资源文件路径和检测灵敏度
- **默认启用**：唤醒词检测功能默认启用，符合机器人语音助手的使用场景
- **播报模式控制**：只有在检测到唤醒词时才发布音频数据，减少系统资源消耗
- **跨平台支持**：在Jetson Orin NX Super开发板上经过验证，支持Ubuntu 22.04 + ROS 2 Humble环境

### 配置参数

唤醒词检测功能可通过以下参数进行配置：

- `wake_word.enabled`: 是否启用唤醒词检测功能（默认为true）
- `wake_word.model_path`: 唤醒词模型文件路径
- `wake_word.resource_path`: 唤醒词资源文件路径
- `wake_word.sensitivity`: 检测灵敏度，范围为0.0-1.0

### 使用示例

```shell
# 启动音频采集节点（启用唤醒词检测）
ros2 run audio_common audio_capturer_node --ros-args --params-file config/device_config.yaml

# 启动音频播放节点
ros2 run audio_common audio_player_node --ros-args --params-file config/device_config.yaml

# 说出唤醒词（如"小浩小浩"）以激活播报模式
# 此时音频数据将开始发布并播放
```

### 唤醒词检测测试

要测试唤醒词检测功能，可以使用以下步骤：

1. 确保snowboy_python包已正确安装和配置
2. 使用默认配置启动音频采集节点
3. 说出预设的唤醒词（如"小浩小浩"）
4. 观察节点日志，应显示"检测到唤醒词，开启播报模式"的信息
5. 此时音频数据将开始发布

### Snowboy配置说明

Snowboy唤醒词检测引擎的配置由snowboy_python项目负责管理，audio_common项目只需要调用snowboy_python项目的接口进行唤醒词检测。默认配置如下：

- **模型文件路径**: `src/snowboy/src/resources/models/hoson.pmdl`
- **资源文件路径**: `src/snowboy/src/resources/common.res`
- **检测灵敏度**: `0.5`

这些配置参数可以通过audio_common的参数系统进行覆盖。

## 版本信息

### v1.0.0 (2025-11-07)
- **修改人**: chenwang 
- **修改内容**: 
   1）项目创建和初始版本发布
   2）麦克风采集数据发布话题测试OK

### v1.1.0 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
  1）增强设备检测机制，通过系统命令和实际数据读取双重验证设备状态
  2）添加运行时设备状态监控，实现设备断开后的自动重新连接功能
  3）改进错误处理机制，增强各种异常情况的处理能力
  4）优化代码结构，添加定期设备检查定时器和流活性检查
  5）snowboy唤醒词引擎嵌入测试OK
              
### v1.1.1 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
  1）优化设备自动发现机制，解决设备号变化导致的问题
  2）改进设备连接检查逻辑，避免过度严格的物理连接检查
  3）增强错误恢复机制，支持默认设备回退
  4）完善设备重新初始化逻辑，提高系统稳定性

### v1.1.2 (2025-11-20)
- **修改人**: chenwang
- **修改内容**:
  1）更新README文档，添加Piper TTS推荐使用方案
  2）添加TTS节点问题诊断与解决方案说明
  3）明确espeak-ng依赖工具的安装方法
  4）优化文本转语音使用示例说明

### v1.1.3 (2025-11-20)
- **修改人**: chenwang
- **修改内容**:
  1）添加音频播放无声问题诊断与解决方案说明
  2）提供PortAudio流指针错误的处理方法
  3）推荐使用Piper TTS与音频播放节点的组合方案

### v1.1.4 (2025-11-20)
- **修改人**: chenwang
- **修改内容**:
  1）修复CMakeLists.txt构建配置问题，确保在Jetson Orin NX Super开发板上正确构建
  2）改进espeak-ng库查找逻辑，添加回退机制
  3）优化TTS节点实现，保持与piper_ros的音频格式兼容性
