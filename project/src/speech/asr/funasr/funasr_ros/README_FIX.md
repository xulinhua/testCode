# FunASR ROS节点修复说明

## 问题分析

在原始实现中，实时流语音识别无法输出正确的文本数据，一直显示"ASR Result dict: {'text': ''}"。经过详细分析，发现以下主要问题：

1. **采样率不匹配**：audio_capturer_node使用44100Hz采样率，而FunASR模型期望16000Hz
2. **音频数据格式转换问题**：在asr_node.py中，音频数据转换可能不正确
3. **流式识别参数设置**：chunk_size和is_final参数使用不当
4. **缓存管理问题**：streaming_cache在每次识别时应该重置

## 修复方案

### 1. 修改asr_node.py中的关键参数

```python
# 修改chunk_size参数为960，与流式识别chunk_size匹配
self.declare_parameter('chunk_size', 960)

# 在perform_asr方法中使用正确的chunk_size参数[0, 10, 5]和960采样点
chunk_stride = 960  # 960采样点，对应60ms，与chunk_size=[0, 10, 5]匹配
```

### 2. 添加音频重采样功能

```python
def resample_audio(self, audio_data, input_rate, output_rate):
    """
    重采样音频数据
    
    Args:
        audio_data (np.array): 输入音频数据
        input_rate (int): 输入采样率
        output_rate (int): 输出采样率
        
    Returns:
        np.array: 重采样后的音频数据
    """
    if input_rate == output_rate:
        return audio_data
        
    # 计算重采样比例
    ratio = output_rate / input_rate
    new_length = int(len(audio_data) * ratio)
    
    # 使用线性插值进行重采样
    old_indices = np.arange(len(audio_data))
    new_indices = np.linspace(0, len(audio_data) - 1, new_length)
    resampled_data = np.interp(new_indices, old_indices, audio_data)
    
    return resampled_data.astype(np.float32)
```

### 3. 改进流式识别逻辑

```python
# 使用流式识别方法处理实时音频数据
result = self.asr_processor.recognize_streaming_audio(
    speech_chunk, 
    input_sample_rate, 
    self.streaming_cache,  # 使用共享缓存
    chunk_size=[0, 10, 5],  # 使用正确的chunk_size参数
    encoder_chunk_look_back=4,
    decoder_chunk_look_back=1,
    enable_logging=False  # 关闭过程日志输出，避免显示进度条等信息
)
```

### 4. 优化音频缓冲区管理

```python
# 累积足够的音频数据再进行识别（至少0.6秒的数据，16000Hz * 0.6 = 9600个采样点）
if len(self.audio_buffer) >= 9600 or len(self.audio_buffer) >= 38400:  # 0.6-2.4秒数据
```

## 使用方法

1. 启动音频采集节点：
```bash
ros2 run audio_common audio_capturer_node
```

2. 启动ASR节点：
```bash
ros2 launch funasr_ros asr_node.launch.py
```

3. 对着麦克风说出唤醒词，然后说话进行语音识别测试

## 预期效果

修复后，系统应该能够正确识别语音并输出文本结果，而不是一直显示空文本。

## 注意事项

1. 确保FunASR模型已正确下载并配置
2. 确保音频设备正常工作
3. 在嘈杂环境中可能影响识别效果