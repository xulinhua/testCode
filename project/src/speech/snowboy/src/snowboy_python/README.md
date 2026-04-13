# Snowboy Python 唤醒词检测包

这是一个基于Snowboy的Python唤醒词检测包，为ROS 2音频处理工具包提供唤醒词检测功能。

## 目录

1. [项目概述](#项目概述)
2. [系统要求](#系统要求)
3. [安装说明](#安装说明)
   - [激活Python虚拟环境](#激活python虚拟环境)
   - [安装依赖](#安装依赖)
   - [构建ROS 2包](#构建ros-2包)
4. [使用方法](#使用方法)
   - [基本使用](#基本使用)
   - [测试音频文件](#测试音频文件)
   - [实时检测](#实时检测)
5. [API说明](#api说明)
   - [SnowboyDetectorWrapper类](#snowboydetectorwrapper类)
     - [构造函数](#构造函数)
     - [detect方法](#detect方法)
     - [test_audio_file方法](#test_audio_file方法)
     - [start_detection方法](#start_detection方法)
     - [get_sample_rate方法](#get_sample_rate方法)
     - [get_num_channels方法](#get_num_channels方法)

## 项目概述

snowboy_python是一个Python包，封装了Snowboy唤醒词检测引擎的功能。它提供了简单易用的接口来检测音频数据中的唤醒词，支持自定义唤醒词模型。

## 系统要求

- Python 3.10.12 (在speech_env虚拟环境中运行)
- PyAudio
- NumPy

## 安装说明

### 激活Python虚拟环境

```shell
source speech_env/bin/activate
```

### 安装依赖

```shell
pip3 install pyaudio numpy
```

### 构建ROS 2包

```shell
cd ~/ros2_ws
colcon build --packages-select snowboy_python
source install/setup.bash
```

## 使用方法

### 基本使用

```python
from snowboy_python.snowboy_detector_wrapper import SnowboyDetectorWrapper

# 创建检测器实例
detector = SnowboyDetectorWrapper(
    model_path="src/snowboy/src/resources/models/snowboy.pmdl",
    resource_path="src/snowboy/src/resources/common.res",
    sensitivity=0.5
)

# 检测音频数据
result = detector.detect(audio_data, sample_rate, channels)
if result:
    print("检测到唤醒词!")
```

### 测试音频文件

```python
# 测试特定音频文件
detector.test_audio_file("path/to/audio/file.wav")
```

### 实时检测

```python
# 开始实时语音检测
detector.start_detection()
```

## API说明

### SnowboyDetectorWrapper类

#### 构造函数

```python
SnowboyDetectorWrapper(model_path, resource_path, sensitivity=0.5)
```

- `model_path` (str): 唤醒词模型文件路径
- `resource_path` (str): 资源文件路径
- `sensitivity` (float): 检测灵敏度 (0.0 - 1.0)

#### detect方法

```python
detect(audio_data, sample_rate, channels)
```

检测音频数据中是否包含唤醒词。

- `audio_data` (bytes): 音频数据
- `sample_rate` (int): 采样率
- `channels` (int): 声道数
- 返回值: bool - True表示检测到唤醒词，False表示未检测到

#### test_audio_file方法

```python
test_audio_file(audio_file_path)
```

通过传入录音路径识别是否存在唤醒词。

- `audio_file_path` (str): 音频文件路径

#### start_detection方法

```python
start_detection()
```

开始实时语音检测是否存在唤醒词。

#### get_sample_rate方法

```python
get_sample_rate()
```

获取检测器要求的采样率。

- 返回值: int - 采样率

#### get_num_channels方法

```python
get_num_channels()
```

获取检测器要求的声道数。

- 返回值: int - 声道数