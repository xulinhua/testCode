# 音频数据消息类型
# 存储音频样本数据及相关参数

# 音频样本数据 (int8数组)
int8[] data

# 采样率 (Hz)
uint32 sample_rate

# 声道数
uint8 channels

# 时间戳 (Unix时间戳)
float64 timestamp