# Audio Basic Py

## 项目信息
- **公司**: hoson 
- **项目简介**: audio_basic_py 是一个纯Python包项目，提供通用的音频处理功能。该项目不依赖于ROS，可以被其他项目直接调用。 

## 功能特性

- 音频文件检查
- 音频质量分析
- 音频播放功能
- 不依赖ROS环境

## 项目结构

```
audio_basic_py/
├── audio_basic_py/           # 核心模块
│   ├── __init__.py           # 包初始化文件
│   └── audio_utils.py        # 音频处理工具
├── setup.py                  # 安装配置文件
├── package.xml               # ROS包配置文件
└── README.md                 # 项目说明文件
```

## 安装依赖

```bash
pip install numpy soundfile pyaudio
```

## 安装项目

```bash
cd src/audio_basic_py
python setup.py install
```

## 使用方法

```python
from audio_basic_py import check_audio_file, play_audio_file

# 检查音频文件
check_audio_file("test.wav")

# 播放音频文件
play_audio_file("test.wav")
```

## API说明

### analyze_audio_quality(data, fs, filename)
分析音频质量特征

参数:
- data: 音频数据(numpy数组)
- fs: 采样率
- filename: 文件名

### check_audio_file(filename)
检查音频文件的基本信息

参数:
- filename: 音频文件路径

返回值:
- bool: 检查是否成功

### play_audio_file(filename)
尝试播放音频文件

参数:
- filename: 音频文件路径

返回值:
- bool: 播放是否成功

## 版本信息

### v1.0.0 (2025-11-09)
- **修改人**: chenwang 
- **修改内容**: 
   1）初始版本发布
   2）独立通用接口封装