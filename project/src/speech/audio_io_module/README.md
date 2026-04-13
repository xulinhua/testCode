# Audio I/O Module - 跨平台音频输入输出模块

跨平台的音频输入输出处理模块，支持实时音频采集和播放功能，可在多种操作系统上运行。

## 功能特性

1. **麦克风音频数据采集**：支持指定设备端口的音频输入，实时获取麦克风设备的音频数据，可用于连接FunASR模块进行实时流的语音识别
2. **音频数据录制与保存**：获取麦克风设备的音频数据并保存到output文件夹中
3. **音频数据播放**：支持指定设备端口的音频输出，可播放WAV文件
4. **音量控制接口**：提供可调节的音量控制接口，支持音频数据增益调节
- 实时音频采集（麦克风录音）
- 音频数据播放（扬声器输出）
- 音频文件保存和读取
- 音频重采样处理
- 音量增益控制
- 设备索引动态检测
- 跨平台支持（Windows/Linux/macOS）

## 系统要求

- Python 3.6+
- PyAudio
- NumPy (可选，用于音量控制)

## 设备信息

- USB麦克风设备号：hw:2,0
- USB扬声器设备号：hw:3,0

## 安装依赖

```bash
pip install -r requirements.txt

# Ubuntu/Debian系统安装PyAudio依赖
sudo apt-get install portaudio19-dev python3-pyaudio

# 安装Python包
pip install pyaudio
```

## 项目架构

### 核心组件
- [audio_io_manager.py](file:///192.168.11.129/testCode/ai_speech/speech/audio_io_module/audio_io_manager.py) - 核心音频管理类，提供所有音频处理功能
- [requirements.txt](file:///192.168.11.129/testCode/ai_speech/speech/audio_io_module/requirements.txt) - 项目依赖列表

### 测试脚本
- [test_record_audio.py](file:///192.168.11.129/testCode/ai_speech/speech/audio_io_module/test_record_audio.py) - 音频录制功能测试
- [test_play_audio.py](file:///192.168.11.129/testCode/ai_speech/speech/audio_io_module/test_play_audio.py) - 音频播放功能测试
- [test_volume_control.py](file:///192.168.11.129/testCode/ai_speech/speech/audio_io_module/test_volume_control.py) - 音量控制功能测试

## 实现原理

### 音频采集原理
1. 使用PyAudio库打开指定设备的音频输入流
2. 通过回调函数或循环读取方式获取实时音频数据
3. 支持音量增益调节，通过修改音频样本数值实现

### 音频播放原理
1. 使用PyAudio库打开指定设备的音频输出流
2. 读取WAV文件或直接播放音频数据
3. 支持音量增益调节，播放前对音频数据进行处理

### 音量控制原理
1. 音量增益范围限制在0.0-2.0之间
2. 对16位有符号整数音频样本进行数值缩放
3. 通过数学运算实现音量调节，不依赖外部库

### 采样率适配原理
1. 自动检测设备支持的采样率
2. 根据音频文件的实际采样率进行播放
3. 提供采样率兼容性检查，避免Invalid sample rate错误

## 使用方法

### 基本使用

```python
from audio_io_manager import AudioIOManager

# 创建音频管理器实例
# 根据实际设备情况设置设备索引
audio_manager = AudioIOManager(input_device_index=24, output_device_index=25)

# 实时音频采集
# audio_manager.capture_audio_stream(callback=your_callback_function)

# 录制音频并保存
audio_manager.capture_and_save_audio("output.wav", record_seconds=5)

# 播放音频文件
audio_manager.play_audio_file("audio.wav")

# 设置音量增益
audio_manager.set_volume_gain(1.5)  # 1.5倍音量

# 关闭音频引擎
audio_manager.close()
```

### 2. 高级功能

```python
# 设置音量增益 (0.0-2.0, 1.0为原始音量)
audio_manager.set_volume_gain(1.5)

# 实时音频流处理
def audio_callback(data):
    # 处理实时音频数据
    print(f"接收到音频数据: {len(data)} 字节")

# 启动实时音频采集
audio_manager.capture_audio_stream(callback=audio_callback)
```

## 设备索引说明

设备索引因平台和连接设备而异，可通过以下方式获取：

```python
import pyaudio

p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    print(f"{i}: {info['name']} - 输入:{info['maxInputChannels']} 输出:{info['maxOutputChannels']}")
p.terminate()
```

## 常见问题解决

### 问题1: ALSA库警告信息

**错误示例**:
```
ALSA lib pcm_dmix.c:1032:(snd_pcm_dmix_open) unable to open slave
ALSA lib pcm.c:2664:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.rear
...
```

**原因**: 这些是ALSA库的警告信息，通常是系统音频配置问题，不影响基本功能。

**解决方案**:
1. 这些警告信息可以忽略，不影响音频功能正常使用
2. 如果需要消除警告，可以配置ALSA配置文件或使用PulseAudio作为中间层

### 问题2: "Invalid sample rate" 错误

**错误示例**:
```
Expression 'alsa_snd_pcm_mmap_begin' failed ...
[Errno -9999] Unanticipated host error
```

**原因**: 指定的采样率与设备不兼容。

**解决方案**:
1. 使用设备支持的采样率（如32000Hz, 44100Hz, 48000Hz等）
2. 修改代码中的采样率参数
3. 使用设备的默认采样率

### 问题3: 设备索引不正确

**错误示例**:
```
Error opening stream: [Errno -9996] Invalid input device
```

**原因**: 指定的设备索引不存在或不可用。

**解决方案**:
1. 使用上述方法列出所有可用设备
2. 根据设备名称选择正确的索引
3. 确保设备已正确连接

### 问题4: 权限不足

**错误示例**:
```
Error opening stream: [Errno -9997] Invalid sample rate
```

**原因**: 当前用户没有访问音频设备的权限。

**解决方案**:
```bash
# 将用户添加到audio组
sudo usermod -a -G audio $USER

# 重新登录或重新加载组权限
newgrp audio
```

## 测试脚本

项目包含多个测试脚本用于验证功能：

- `test_record_audio.py`: 手动录音测试
- `test_play_audio.py`: 音频播放测试
- `test_volume_control.py`: 音量控制测试
- `check_device_info.py`: 设备信息检查

运行测试脚本：
```bash
python test_record_audio.py
python test_play_audio.py
python test_volume_control.py
python check_device_info.py
```

## API参考

### AudioIOManager类

#### 构造函数
```python
AudioIOManager(input_device_index=24, output_device_index=25)
```

#### 主要方法

- `capture_and_save_audio(filename, record_seconds=5)`: 录制音频并保存到文件
- `play_audio_file(filepath)`: 播放音频文件
- `capture_audio_stream(callback)`: 实时音频流采集
- `play_audio_data(audio_data)`: 播放音频数据
- `set_volume_gain(gain)`: 设置音量增益
- `close()`: 关闭音频引擎

