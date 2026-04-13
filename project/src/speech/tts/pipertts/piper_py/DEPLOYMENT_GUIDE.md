# Piper Speech Synthesis 部署指南

## 概述

本文档详细说明如何在 Jetson Orin NX Super 开发板（Ubuntu 22.04 + ROS Humble 环境）上部署和运行 Piper Speech Synthesis 项目。

## 硬件要求

- **硬件平台**: Jetson Orin NX Super 16GB
- **操作系统**: Ubuntu 22.04
- **Python版本**: 3.10.12
- **JetPack版本**: 6.2及以上
- **L4T版本**: 36.4.7
- **CUDA版本**: 12.6
- **cuDNN版本**: 9.3
- **TensorRT版本**: 10.3
- **ROS版本**: ROS2 Humble

## 软件依赖

### 系统级依赖

```bash
# 更新系统包
sudo apt update
sudo apt upgrade -y

# 安装基础依赖
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    libsndfile1 \
    portaudio19-dev
```

### Python 依赖

```bash
# 创建虚拟环境
python3 -m venv ~/piper_venv
source ~/piper_venv/bin/activate

# 升级pip
pip install --upgrade pip

# 安装核心依赖
pip install \
    numpy>=1.21.0 \
    pyyaml>=6.0 \
    onnxruntime-gpu>=1.17.0 \
    soundfile \
    pyaudio

# 可选：piper-phonemize
pip install piper-phonemize
```

## 快速部署步骤

### 方法一：使用自动安装脚本（推荐）

1. **克隆或下载项目代码**
   ```bash
   cd ~/
   git clone <repository-url> piper_speech
   cd piper_speech
   ```

2. **运行自动安装脚本**
   ```bash
   chmod +x scripts/jetson_setup.sh
   ./scripts/jetson_setup.sh
   ```

3. **激活环境**
   ```bash
   source ~/.bashrc
   piper_activate
   ```

### 方法二：手动部署

1. **创建项目目录结构**
   ```bash
   mkdir -p ~/piper_speech/{models,config,output,logs,examples}
   cd ~/piper_speech
   ```

2. **复制项目文件**
   ```bash
   # 复制所有Python模块文件
   cp -r piper_speech/* ./
   ```

3. **安装依赖**
   ```bash
   python3 scripts/install_dependencies.py
   ```

## 模型文件准备

### 下载 Piper 语音模型

Piper 项目提供多种语音模型，您可以从以下位置下载：

1. **官方仓库**：https://github.com/rhasspy/piper/releases
2. **社区模型**：https://huggingface.co/rhasspy/piper-voices

### 推荐模型

对于 Jetson Orin NX Super，推荐使用以下模型：

- `en_US-lessac-medium.onnx` - 英语，中等质量
- `en_US-lessac-high.onnx` - 英语，高质量
- `zh_CN-huayan-medium.onnx` - 中文，中等质量

### 模型文件放置

将下载的模型文件放置到 `models/` 目录：

```bash
models/
├── en_US-lessac-medium.onnx
├── en_US-lessac-medium.onnx.json
├── zh_CN-huayan-medium.onnx
└── zh_CN-huayan-medium.onnx.json
```

## 配置说明

### 默认配置 (config/default.yaml)

```yaml
audio:
  sample_rate: 22050
  channels: 1
  format: pcm_s16le

model:
  path: "models/en_US-lessac-medium.onnx"
  config_path: "models/en_US-lessac-medium.onnx.json"
  speaker_id: 0

synthesis:
  max_text_length: 500
  batch_size: 1
  enable_streaming: false

performance:
  use_gpu: true
  threads: 4
  optimization_level: 3
```

### Jetson 平台优化配置 (config/jetson.yaml)

```yaml
performance:
  use_gpu: true
  threads: 8
  optimization_level: 3
  
  # Jetson 特定优化
  cuda_device_id: 0
  gpu_mem_limit: 2147483648  # 2GB
  trt_fp16_enable: true
```

## 验证部署

### 1. 基础功能测试

```bash
# 运行演示脚本
python run_demo.py
```

### 2. 命令行工具测试

```bash
# 合成单个文本
python cli/tts_cli.py -t "Hello, Jetson!" -o output/test.wav

# 批量处理
python cli/tts_cli.py -f input.txt -o output/batch/
```

### 3. 性能测试

```bash
# 运行性能测试
python -c "
import sys
sys.path.append('.')
from run_demo import performance_test
performance_test()
"
```

## 集成到 ROS Humble

### 创建 ROS 包

1. **创建 ROS2 工作空间**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **创建 ROS 包**
   ```bash
   ros2 pkg create piper_tts --build-type ament_python --dependencies rclpy std_msgs
   ```

3. **集成 Piper TTS**
   ```python
   # piper_tts/piper_tts/node.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   
   # 导入 Piper TTS
   import sys
   sys.path.append('/home/nvidia/piper_speech')
   from piper_speech import TTSEngine, AudioGenerator
   
   class PiperTTSNode(Node):
       def __init__(self):
           super().__init__('piper_tts_node')
           
           # 初始化 Piper TTS
           self.tts_engine = TTSEngine()
           self.audio_generator = AudioGenerator()
           
           # 加载模型
           self.tts_engine.load_model('models/en_US-lessac-medium.onnx')
           
           # 创建订阅者
           self.subscription = self.create_subscription(
               String, 'tts_input', self.tts_callback, 10)
           
           # 创建发布者
           self.audio_pub = self.create_publisher(String, 'audio_output', 10)
           
       def tts_callback(self, msg):
           text = msg.data
           self.get_logger().info(f'合成语音: {text}')
           
           # 语音合成
           audio_data = self.tts_engine.synthesize(text)
           
           if audio_data is not None:
               # 保存或发布音频数据
               output_path = f'/tmp/tts_{int(time.time())}.wav'
               self.audio_generator.save_wav(audio_data, output_path)
               
               # 发布音频文件路径
               audio_msg = String()
               audio_msg.data = output_path
               self.audio_pub.publish(audio_msg)
   ```

### 编译和运行

```bash
# 编译 ROS 包
cd ~/ros2_ws
colcon build --packages-select piper_tts

# 运行节点
source install/setup.bash
ros2 run piper_tts piper_tts_node
```

## 性能优化

### GPU 加速配置

1. **启用 TensorRT**
   ```python
   # 在配置中启用 TensorRT
   providers = [
       ('TensorrtExecutionProvider', {
           'trt_max_workspace_size': 1 << 30,
           'trt_fp16_enable': True,
       }),
       'CUDAExecutionProvider',
       'CPUExecutionProvider'
   ]
   ```

2. **内存优化**
   ```yaml
   performance:
     gpu_mem_limit: 2147483648  # 限制 GPU 内存使用
     arena_extend_strategy: kNextPowerOfTwo
   ```

### 多线程处理

```python
# 启用多线程
session_options = ort.SessionOptions()
session_options.intra_op_num_threads = 4
session_options.inter_op_num_threads = 4
```

## 故障排除

### 常见问题

1. **模型加载失败**
   - 检查模型文件路径是否正确
   - 确认模型文件完整性
   - 验证文件权限

2. **GPU 加速未启用**
   - 检查 CUDA 安装
   - 验证 onnxruntime-gpu 版本
   - 检查 GPU 内存使用情况

3. **音频输出问题**
   - 检查音频设备权限
   - 验证采样率设置
   - 确认音频文件格式

### 日志调试

启用详细日志输出：

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 维护和更新

### 定期维护

1. **更新依赖**
   ```bash
   pip list --outdated
   pip install --upgrade <package-name>
   ```

2. **清理缓存**
   ```bash
   pip cache purge
   rm -rf ~/.cache/pip
   ```

3. **备份配置**
   ```bash
   tar -czf piper_backup_$(date +%Y%m%d).tar.gz config/ models/
   ```

### 版本升级

1. **备份当前版本**
2. **下载新版本代码**
3. **比较配置文件差异**
4. **测试新版本功能**
5. **逐步替换生产环境**

## 技术支持

### 获取帮助

- **项目文档**: 查看 `docs/` 目录
- **问题反馈**: 创建 GitHub Issue
- **社区支持**: 加入相关技术社区

### 资源链接

- [Piper 官方文档](https://github.com/rhasspy/piper)
- [Jetson 开发者中心](https://developer.nvidia.com/embedded/learn/get-started-jetson)
- [ROS2 文档](https://docs.ros.org/en/humble/)

---

**注意**: 本文档会根据项目更新而持续维护，请定期查看最新版本。