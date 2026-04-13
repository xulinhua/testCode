# FunASR ROS Wrapper

## 项目信息
- **公司**: hoson 
- **项目简介**: 这是一个基于FunASR的ROS 2语音识别节点，用于在ROS Humble环境下实现语音交互功能。该节点订阅[auduo_common](../audio_common/README.md)项目发布的音频话题，调用FunASR进行语音识别，并将结果以ROS话题的形式发布。  

## 目录结构

```
funasr_ros/
├── CMakeLists.txt              # 构建配置文件
├── package.xml                 # 包描述文件
├── config/                     # 配置文件目录
│   └── asr_config.yaml         # ASR配置文件
├── launch/                     # 启动文件目录
│   └── asr_node.launch.py      # 节点启动文件
├── scripts/                    # 脚本目录
│   └── asr_node.py             # ASR节点主程序
└── README.md                   # 项目说明文档
```

## 功能特性

1. **ROS 2集成**：完全集成在ROS 2 Humble环境中
2. **音频订阅**：订阅audio_common项目发布的`/audio/microphone`话题
3. **语音识别**：调用FunASR进行语音识别处理
4. **结果发布**：将识别结果发布到`/asr/result`话题
5. **灵活配置**：支持通过YAML文件配置模型路径、类型等参数
6. **多模型支持**：支持PyTorch和ONNX格式的模型
7. **QoS兼容性**：与audio_common项目使用相同的QoS配置，确保消息传递的兼容性
8. **结果格式处理**：支持多种FunASR返回格式，确保识别结果正确发布
9. **结果验证**：只发布有效的语音识别结果，过滤空结果或异常结果
10. **字典格式支持**：正确处理FunASR返回的字典格式结果，提取有效文本
11. **减少终端输出**：只在有有效识别结果时才输出信息，避免无效信息干扰

## 依赖项目

- [audio_common](../audio_common/README.md)：提供音频采集和播放功能
- [funasr_py](../funasr_py/README.md)：提供FunASR语音识别核心功能

## 系统架构

```mermaid
graph TB
    A[麦克风设备] --> B[Audio Capturer Node]
    B --> C[/audio/microphone话题]
    C --> D[ASR Node]
    D --> E[/asr/result话题]
    E --> F[下游应用节点]
    D --> G[FunASR模型]
```

## 实现原理

1. **音频数据订阅**：ASR节点订阅audio_common项目发布的`/audio/microphone`话题，获取实时音频数据
2. **数据转换**：将ROS消息格式的音频数据转换为FunASR可处理的numpy数组格式
3. **语音识别**：调用FunASR的ASRProcessor进行语音识别处理
4. **结果验证**：对识别结果进行有效性检查，只发布有效结果
5. **结果发布**：将识别结果封装为ROS消息并发布到`/asr/result`话题

## 安装部署说明

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- 已安装并配置好的FunASR环境（参考[FunASR部署指南](../funasr_py/FunASR部署指南.md)）

### 安装依赖

确保已安装所有依赖项：

```bash
# 激活Python虚拟环境（如果使用）
source ~/speech_env/bin/activate

# 安装funasr_py项目依赖
pip install -r ../funasr_py/requirements.txt

# 安装ROS 2 Python依赖
sudo apt update
sudo apt install python3-numpy
```

### 构建项目

```bash
# 在ROS工作空间根目录下执行
colcon build --packages-select funasr_ros

# 构建完成后，source工作空间
source install/setup.bash
```

## 配置说明

配置文件位于`config/asr_config.yaml`：

```yaml
/**:
  ros__parameters:
    # 模型配置
    model_path: "../resources/models/paraformer-zh-streaming/onnx"  # 模型路径
    model_type: "onnx"  # 模型类型: pt (PyTorch) 或 onnx
    
    # 音频配置
    audio_format: 16  # 音频格式 (16表示16位)
    audio_channels: 1  # 声道数
    sample_rate: 16000  # 采样率
    chunk_size: 512  # 音频块大小
    
    # ASR配置
    enable_vad: true  # 是否启用语音活动检测
    enable_punctuation: true  # 是否启用标点符号
    
    # 话题配置
    microphone_topic: "/audio/microphone"  # 麦克风音频输入话题
    asr_result_topic: "/asr/result"  # ASR结果输出话题
```

## 使用说明

### 启动节点

#### 方法1：直接运行节点

```bash
# 激活Python虚拟环境（如果使用）
source ~/speech_env/bin/activate

# 启动ASR节点
ros2 run funasr_ros asr_node.py
```

#### 方法2：使用launch文件启动

```bash
# 激活Python虚拟环境（如果使用）
source ~/speech_env/bin/activate

# 使用launch文件启动节点
ros2 launch funasr_ros asr_node.launch.py
```

#### 方法3：使用自定义配置启动

```bash
# 激活Python虚拟环境（如果使用）
source ~/speech_env/bin/activate

# 使用自定义参数启动节点
ros2 launch funasr_ros asr_node.launch.py \
  model_path:="../resources/models/paraformer-zh/onnx" \
  model_type:="onnx" \
  microphone_topic:="/audio/microphone" \
  asr_result_topic:="/asr/result"
```

### 查看识别结果

```bash
# 查看ASR识别结果
ros2 topic echo /asr/result
```

### 完整测试流程

```bash
# 终端1: 启动音频采集节点
ros2 run audio_common audio_capturer_node --ros-args --params-file ../audio_common/config/device_config.yaml

# 终端2: 启动ASR节点
ros2 run funasr_ros asr_node.py

# 终端3: 查看识别结果
ros2 topic echo /asr/result
```

注意：确保audio_capturer_node和asr_node使用兼容的QoS设置，以避免"New publisher discovered on topic offering incompatible QoS"警告。

## API说明

### 订阅的话题

- `/audio/microphone` (audio_common_msgs/msg/AudioStamped)：从麦克风采集的音频数据

### 发布的话题

- `/asr/result` (std_msgs/msg/String)：语音识别结果

### 参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| model_path | string | "../resources/models/paraformer-zh-streaming/onnx" | 模型路径 |
| model_type | string | "onnx" | 模型类型 (pt 或 onnx) |
| microphone_topic | string | "/audio/microphone" | 麦克风音频输入话题 |
| asr_result_topic | string | "/asr/result" | ASR结果输出话题 |
| audio_format | int | 16 | 音频格式 |
| audio_channels | int | 1 | 声道数 |
| sample_rate | int | 16000 | 采样率 |
| chunk_size | int | 512 | 音频块大小 |
| enable_vad | bool | true | 是否启用语音活动检测 |
| enable_punctuation | bool | true | 是否启用标点符号 |

## 实现流程

1. **初始化阶段**：
   - 加载配置参数
   - 初始化ASR处理器
   - 加载指定的FunASR模型
   - 创建ROS订阅者和发布者

2. **运行阶段**：
   - 订阅音频话题，接收音频数据
   - 将音频数据转换为FunASR可处理的格式
   - 调用FunASR进行语音识别
   - 对识别结果进行有效性检查
   - 只发布有效的识别结果

3. **数据流**：
   - 音频数据：麦克风设备 → audio_capturer_node → /audio/microphone话题 → asr_node
   - 识别结果：asr_node → /asr/result话题 → 下游应用节点

## 测试流程

1. **环境准备**：
   - 确保已安装所有依赖项
   - 确保FunASR模型已下载并放置在正确位置
   - 确保audio_common节点正常运行

2. **功能测试**：
   - 启动audio_capturer_node
   - 启动asr_node
   - 对着麦克风说话
   - 观察`/asr/result`话题的输出

3. **性能测试**：
   - 测试不同长度语音的识别准确率
   - 测试实时性表现
   - 测试不同环境噪声下的识别效果

## 常见问题解决

### "'python3\r': No such file or directory" 错误

这个错误是由于在Windows环境下编辑脚本文件导致的换行符问题。在Linux系统中，脚本文件需要使用Unix风格的换行符(LF)，而Windows使用的是Windows风格的换行符(CRLF)。

**解决方案：**
1. 在Ubuntu环境中重新创建脚本文件
2. 使用`dos2unix`命令转换文件格式：
   ```bash
   dos2unix install/funasr_ros/lib/funasr_ros/asr_node.py
   ```
3. 或者手动编辑文件，确保使用LF换行符

### "未找到funasr库"错误

如果遇到"未找到funasr库，请先安装"的错误提示，请参考[FunASR部署指南](../funasr_py/FunASR部署指南.md)中的详细部署步骤，确保正确安装所有依赖项。

### "ImportError: No module named 'asr_core'"错误

请确保已正确构建funasr_py项目，并且Python路径设置正确。

### 识别结果为空

1. 检查模型是否正确加载
2. 检查音频数据是否正常接收
3. 检查音频格式和采样率是否匹配

### QoS不兼容问题

如果遇到"New publisher discovered on topic offering incompatible QoS"警告，这表示发布者和订阅者之间的QoS设置不兼容。当前版本已修复此问题，确保使用与audio_common项目兼容的QoS配置。

### 持续显示"正在进行语音识别"但无结果输出

这通常是因为FunASR模型返回的结果格式与代码期望的格式不一致。当前版本已增强对多种返回格式的支持：
1. 自动处理列表格式的返回结果
2. 支持多种文本键名（text, pred_text, prediction等）
3. 对未知格式进行字符串转换以确保结果发布

### 无效或异常结果处理

当前版本增加了结果验证机制，只发布有效的语音识别结果：
1. 过滤空结果或只包含空白字符的结果
2. 过滤无意义的字符组合（如只有标点符号）
3. 过滤过短的结果（少于2个字符）
4. 正确处理字典格式结果，提取有效文本内容

### 字典格式结果处理

FunASR模型可能返回字典格式的结果（如{'key': 'rand_key_xxx', 'text': '识别文本'}），当前版本能够：
1. 正确识别字典格式结果
2. 提取字典中的text字段作为识别结果
3. 对text字段进行有效性验证
4. 只发布包含有效文本的字典结果

### 减少终端输出

为了减少终端输出干扰，当前版本做了以下优化：
1. 只在节点初始化时输出基本信息
2. 只在有有效识别结果时才输出识别结果
3. 移除了"正在进行语音识别"等过程性信息输出
4. 节点启动时会显示"ASR node is now listening for audio input..."提示信息

## 注意事项

1. 本项目需要在ROS 2 Humble环境下运行
2. 需要预先安装并配置好FunASR环境
3. 需要确保模型文件已正确下载并放置在指定位置
4. 建议在Python虚拟环境中运行以避免依赖冲突
5. 实时语音识别对系统性能有一定要求
6. 在跨平台开发时，注意文件换行符格式问题
7. 模型文件现在统一存储在`../resources/models`目录下，便于管理和部署
8. 确保audio_common和funasr_ros使用兼容的QoS设置以避免消息传递问题
9. 只有有效的识别结果才会被发布，避免无效结果干扰下游应用
10. 支持FunASR返回的多种格式结果，包括字典格式
11. 节点启动后会显示监听状态，但不会持续输出过程信息


## 版本信息

### v1.0.0 (2025-11-09)
- **修改人**: chenwang 
- **修改内容**: 
   1）初始版本发布
   2）实现基本的语音识别功能
   3）支持PyTorch和ONNX模型
   4）提供完整的ROS 2集成
   5）更新模型配置路径
   6）语音识别后的结果数据话题发布正常

### v1.0.1 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
   1）修复QoS不兼容问题，确保与audio_common项目使用相同的QoS配置
   2）添加QoS兼容性说明文档
   3）优化README文档结构和内容

### v1.0.2 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
   1）增强对FunASR模型返回结果格式的处理能力
   2）支持多种返回格式，确保识别结果正确发布
   3）添加对结果格式处理的详细说明

### v1.0.3 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
   1）添加结果验证机制，只发布有效的语音识别结果
   2）过滤空结果、异常结果和无意义的结果
   3）增强系统的健壮性和可靠性

### v1.0.4 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
   1）增强对字典格式结果的处理能力
   2）正确提取字典中的text字段并进行验证
   3）只发布包含有效文本的识别结果

### v1.0.5 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
   1）减少终端输出，只在有有效识别结果时才输出信息
   2）移除过程性信息输出，避免无效信息干扰
   3）优化日志输出级别，提高用户体验