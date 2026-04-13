# FunASR在Jetson Orin NX Super 16GB + ROS Humble部署指南（Python虚拟环境版）

> 优化版本：确保模型从本地目录加载，支持在Jetson Orin NX Super开发板Ubuntu22.04 + ROS Humble环境下构建成功及运行正常

## 环境说明
- **硬件平台**: Jetson Orin NX Super 16GB
- **操作系统**: Ubuntu 22.04
- **Python版本**: 3.10.12
- **JetPack版本**: 6.2及以上
- **L4T版本**: 36.4.7
- **CUDA版本**: 12.6
- **cuDNN版本**: 9.3
- **TensorRT版本**: 10.3
- **ROS版本**: ROS2 Humble
- **虚拟环境路径**: ~/speech_env
- **FunASR源码路径**: ~/testCode/ai_speech/speech/src/asr/funasr_py
- **模型存储路径**: ~/testCode/ai_speech/speech/src/asr/funasr/resources/models
- **音频数据路径**: ~/testCode/ai_speech/speech/src/asr/funasr/resources/audio_data

## 1. 系统环境准备

### 1.1 系统环境检查
```bash
# 检查基础环境
nvcc --version  # 应显示CUDA 12.6
python3 --version  # 应显示Python 3.10.12
nvidia-smi  # 确认GPU驱动正常

# 检查ROS2环境
source /opt/ros/humble/setup.bash
ros2 --version  # 确认ROS2 Humble安装
```

### 1.2 系统更新与基础依赖
```bash
sudo apt update
sudo apt upgrade -y

# 安装编译工具和基础依赖
sudo apt install -y build-essential cmake git wget curl
sudo apt install -y python3-pip python3-dev python3-venv python3-virtualenv

# 安装音频处理相关系统依赖
sudo apt install -y libsndfile1-dev libasound2-dev portaudio19-dev
sudo apt install -y ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
sudo apt install -y libssl-dev libffi-dev libxml2-dev libxslt1-dev
sudo apt install -y libhdf5-dev libopenblas-dev liblapack-dev

# 配置CUDA环境变量
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
source ~/.bashrc
```

## 2. Python虚拟环境创建与配置

### 2.1 创建专用虚拟环境
```bash
# 创建Python虚拟环境
python3 -m venv ~/speech_env

# 激活虚拟环境
source ~/speech_env/bin/activate

# 验证虚拟环境
which python3  # 应显示 ~/speech_env/bin/python3
which pip3     # 应显示 ~/speech_env/bin/pip3

# 升级pip
pip3 install --upgrade pip
```

# 创建环境激活脚本（可选）
```bash
echo 'alias activate_funasr="source ~/speech_env/bin/activate"' >> ~/.bashrc
echo 'export SPEECH_ENV_PATH=~/speech_env' >> ~/.bashrc
source ~/.bashrc
```

### 2.2 虚拟环境管理脚本
创建虚拟环境管理脚本 `~/speech_env/manage_venv.sh`：
```bash
#!/bin/bash

VENV_PATH="$HOME/speech_env"

case "$1" in
    "activate")
        source "$VENV_PATH/bin/activate"
        echo "✅ FunASR虚拟环境已激活"
        ;;
    "deactivate")
        deactivate
        echo "✅ FunASR虚拟环境已退出"
        ;;
    "status")
        if [[ "$VIRTUAL_ENV" == "$VENV_PATH" ]]; then
            echo "✅ FunASR虚拟环境已激活"
            echo "虚拟环境路径: $VIRTUAL_ENV"
        else
            echo "❌ FunASR虚拟环境未激活"
        fi
        ;;
    "install")
        if [[ "$VIRTUAL_ENV" == "$VENV_PATH" ]]; then
            pip3 install "$2"
        else
            echo "请先激活虚拟环境: source ~/speech_env/bin/activate"
        fi
        ;;
    "list")
        if [[ "$VIRTUAL_ENV" == "$VENV_PATH" ]]; then
            pip3 list
        else
            echo "请先激活虚拟环境"
        fi
        ;;
    *)
        echo "使用方法: $0 {activate|deactivate|status|install|list}"
        echo "  activate   - 激活虚拟环境"
        echo "  deactivate - 退出虚拟环境"
        echo "  status     - 检查虚拟环境状态"
        echo "  install    - 安装包 (需要包名参数)"
        echo "  list       - 列出已安装包"
        ;;
esac
```

给脚本执行权限：
```bash
chmod +x ~/speech_env/manage_venv.sh
```

## 3. 虚拟环境中安装基础依赖

### 3.1 激活虚拟环境并安装PyTorch
```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 验证虚拟环境状态
python3 --version
pip3 --version

# 在python虚拟环境下安装适用于Jetson ARM架构的PyTorch
pip3 install torch-2.8.0-cp310-cp310-linux_aarch64.whl
pip3 install torchvision-0.23.0-cp310-cp310-linux_aarch64.whl
pip3 install torchaudio-2.8.0-cp310-cp310-linux_aarch64.whl

# ->>>>>>下面这个适用于x86主板下的PyTorch安装，不适用于Arm架构
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# 如果上述方法失败，尝试CPU版本
# pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
#--<<<<<<

# 验证Numpy版本
python3 -c "import numpy; print(numpy.__version__)"

# 验证PyTorch安装
python3 -c "
import torch
print(f'✅ PyTorch版本: {torch.__version__}')
print(f'✅ CUDA可用: {torch.cuda.is_available()}')
print(f'✅ CUDA版本: {torch.version.cuda}')
print(f'✅ GPU设备: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"无GPU\"}'
"
```

### 3.2 安装音频处理库
```bash
# 确保在虚拟环境中
source ~/speech_env/bin/activate

# 安装音频处理核心库
pip3 install soundfile pydub librosa pyaudio
pip3 install resampy webrtcvad pyannote.audio

# 安装音频编解码库
pip3 install av  # PyAV库，替代部分FFmpeg功能

# 验证音频库安装
python3 -c "
import soundfile
import librosa
import pyaudio
print('✅ 音频处理库安装成功')
"
```

### 3.3 安装深度学习与模型库
```bash
# 确保在虚拟环境中
source ~/speech_env/bin/activate

# 安装Transformer相关库（注意版本兼容性）
# 使用兼容版本避免NumPy 2.x冲突
pip3 install "transformers<5.0.0" "datasets<3.0.0" accelerate
pip3 install tokenizers sentencepiece protobuf

# 安装模型管理工具
pip3 install modelscope huggingface_hub addict simplejson

# 安装推理加速库
pip3 install onnxruntime

# TensorRT安装（Jetson专用方案）
# 注意：Jetson Tegra系统不支持pip安装tensorrt，使用系统预装版本
# pip3 install tensorrt  # 这行会失败，不要执行

# 安装科学计算库（注意NumPy版本兼容性）
# PyTorch需要NumPy 1.x版本，避免使用NumPy 2.x
pip3 install "numpy<2" scipy matplotlib jupyter

# 安装性能优化库
pip3 install numba cython

# 安装ROS2 Python接口
pip3 install rclpy std_msgs sensor_msgs

# 安装语音合成库（用于测试音频生成）
pip3 install pyttsx3

# 验证深度学习库安装
python3 -c "
import transformers
import onnxruntime
import modelscope
import numpy
try:
    import tensorrt
    print('✅ TensorRT导入成功（使用系统版本）')
except ImportError:
    print('⚠️ TensorRT导入失败，但不影响FunASR基本功能')
try:
    import pyttsx3
    print('✅ pyttsx3安装成功')
except ImportError:
    print('❌ pyttsx3安装失败')
print('✅ 深度学习库安装成功')
"
```

## 4. FunASR源码安装

### 4.1 项目准备
```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 进入FunASR项目目录
cd ~/testCode/ai_speech/speech/src/asr/funasr_py

# 检查项目结构
ls -la  # 确认存在setup.py或其他安装文件

# 检查当前Python环境
which python3
```

### 4.2 在虚拟环境中安装FunASR
```bash
# 确保在虚拟环境中
source ~/speech_env/bin/activate

# 方式1：使用setup.py安装所有依赖
pip3 install -e .[all]

# 方式2：如果方式1失败，尝试基础安装
pip3 install -e .

# 安装项目依赖
pip3 install -r requirements.txt

# 安装FunASR
pip3 install funasr

# 验证FunASR安装
python3 -c "
import funasr
print(f'✅ FunASR版本: {funasr.__version__}')
print('✅ FunASR安装成功')
"
```

### 4.3 编译C++扩展（如需要）
```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 检查是否有编译脚本
if [ -f "build.sh" ]; then
    chmod +x build.sh
    ./build.sh
fi

# 或者使用CMake编译
if [ -f "CMakeLists.txt" ]; then
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
fi
```

## 5. 模型下载与配置

### 5.1 使用项目脚本下载模型
项目提供了[model_config.py](file:///192.168.11.129/testCode/ai_speech/speech/src/asr/funasr_py/model_config.py)配置文件和测试脚本，支持交互式选择下载15种不同的FunASR模型。

```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 进入项目目录
cd ~/testCode/ai_speech/speech/src/asr/funasr_py

# 交互式下载模型
python tests/test_model_download.py

# 或者直接指定模型编号下载
python tests/download_model.py --model-id 3

# 显示支持的模型列表
python tests/test_model_download.py
```

支持的模型包括：
- 1. SenseVoiceSmall - 多语言语音理解模型
- 2. paraformer-zh - 非流式中文语音识别模型
- 3. paraformer-zh-streaming - 流式中文语音识别模型（默认）
- 4. paraformer-en - 英文语音识别模型
- 5. conformer-en - 英文语音识别模型（Conformer架构）
- 6. ct-punc - 中文标点恢复模型
- 7. fsmn-vad - 语音活动检测模型
- 8. fsmn-kws - 关键词唤醒模型
- 9. fa-zh - 中文语音活动检测模型
- 10. cam++ - 说话人验证模型
- 11. Whisper-large-v3 - 多语言语音识别模型
- 12. Whisper-large-v3-turbo - Whisper轻量化版本
- 13. Qwen-Audio - 基于Qwen的音频理解模型
- 14. Qwen-Audio-Chat - 基于Qwen的音频对话模型
- 15. emotion2vec+large - 情感识别模型

## 6. 模型导出为ONNX格式

### 6.1 安装导出依赖
```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 安装ONNX相关依赖
pip3 install onnx onnxruntime

# 验证安装
python3 -c "
try:
    import onnx
    import onnxruntime
    print('✅ ONNX依赖安装成功')
except ImportError as e:
    print(f'❌ ONNX依赖安装失败: {e}'
"
```

### 6.2 使用脚本导出模型
项目中提供了专门的模型导出测试脚本，支持交互式选择模型并导出为ONNX格式。

```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 进入项目目录
cd ~/testCode/ai_speech/speech/src/asr/funasr_py

# 交互式导出模型
python tests/test_model_export.py
```

导出的模型将保存在以下路径：
- 对于模型3 (paraformer-zh-streaming)：`./resources/models/paraformer-zh-streaming/onnx/`
- 其他模型：`./resources/models/{模型名}/onnx/`

导出完成后会生成以下文件：
- `model.onnx` - 编码器模型
- `decoder.onnx` - 解码器模型
- `config.yaml` - 模型配置文件（从原始模型复制并适配）
- `tokens.json` - 词汇表文件（从原始模型复制）
- `configuration.json` - 模型配置信息（从原始模型复制）

## 7. 环境验证与测试

### 7.1 完整环境验证
```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 完整环境验证脚本
python3 -c "
import sys

def check_import(module_name):
    try:
        __import__(module_name)
        return True, f'✅ {module_name} 导入成功'
    except ImportError as e:
        return False, f'❌ {module_name} 导入失败: {e}'

# 检查关键依赖
modules_to_check = [
    'torch', 'numpy', 'soundfile', 'librosa',
    'modelscope', 'funasr', 'transformers',
    'onnxruntime', 'rclpy'
]

print('=== FunASR环境完整性检查 ===')
all_passed = True

for module in modules_to_check:
    success, message = check_import(module)
    print(message)
    if not success:
        all_passed = False

print(f'=== 检查结果: {\"通过✅\" if all_passed else \"失败❌\"} ===')

# 检查CUDA可用性
if all_passed:
    import torch
    if torch.cuda.is_available():
        print(f'✅ CUDA可用，GPU设备: {torch.cuda.get_device_name(0)}')
    else:
        print('⚠️ CUDA不可用，将使用CPU模式'
"
```

## 8. 常见问题解决

### 8.1 常见问题解决

#### 问题1：TensorRT安装失败（Jetson专用）
```bash
# 错误信息：TensorRT does not currently build wheels for Tegra systems
# 原因：Jetson Tegra系统不支持pip安装tensorrt

# 解决方案1：使用系统预装TensorRT
python3 -c "
import sys
sys.path.append('/usr/lib/python3.10/dist-packages')
try:
    import tensorrt as trt
    print(f'✅ 系统TensorRT版本: {trt.__version__}')
except ImportError as e:
    print(f'❌ 系统TensorRT未安装: {e}'
"

# 解决方案2：检查系统TensorRT安装
sudo apt update
sudo apt install tensorrt -y

# 解决方案3：跳过TensorRT安装（FunASR非必需）
# TensorRT对FunASR是可选依赖，可以跳过安装
```

#### 问题2：虚拟环境无法激活
```bash
# 解决方案：重新创建虚拟环境
rm -rf ~/speech_env
python3 -m venv ~/speech_env
```

#### 问题3：依赖冲突
```bash
# 解决方案：清理并重新安装
source ~/speech_env/bin/activate
pip3 freeze > requirements.txt
pip3 uninstall -y -r requirements.txt
pip3 install -r requirements.txt
```

#### 问题4：CUDA不可用
```bash
# 解决方案：检查CUDA安装
nvcc --version
nvidia-smi
# 如果CUDA不可用，使用CPU版本的PyTorch
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

#### 问题5：NumPy版本兼容性错误
```bash
# 错误信息：A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6
# 解决方案：降级NumPy版本
pip3 uninstall numpy -y
pip3 install "numpy<2"

# 同时修复datasets库版本冲突
pip3 uninstall datasets -y
pip3 install "datasets<3.0.0"

# 验证修复
python3 -c "import numpy; print(f'NumPy版本: {numpy.__version__}')"
```

#### 问题6：simplejson模块缺失
```bash
# 错误信息：No module named 'simplejson'
# 解决方案：安装simplejson模块
pip3 install simplejson

# 验证安装
python3 -c "import simplejson; print('✅ simplejson安装成功')"
```

#### 问题7：模型下载失败
```bash
# 解决方案：检查网络连接，或手动下载
# 访问 https://modelscope.cn 搜索相应模型
```

#### 问题8：网络代理导致依赖安装失败
```bash
# 错误信息：Cannot connect to proxy. Connection refused
# 原因：系统配置了代理服务器，但代理服务器无法连接

# 解决方案1：临时取消代理设置
unset http_proxy
unset https_proxy

# 解决方案2：使用国内镜像源安装
pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple

# 解决方案3：检查网络连接
ping pypi.org
```

### 8.2 性能优化建议
```bash
# 1. 使用GPU加速
# 确保PyTorch正确识别CUDA

# 2. 模型量化优化
# 在虚拟环境中安装优化工具
source ~/speech_env/bin/activate
pip3 install onnxruntime-gpu

# 3. 内存优化
# 对于Jetson设备，合理设置batch size
# 使用流式处理减少内存占用
```

## 9. 虚拟环境使用指南

### 9.1 日常使用命令
```bash
# 激活虚拟环境
source ~/speech_env/bin/activate

# 检查虚拟环境状态
which python3
which pip3

# 退出虚拟环境
deactivate
```

## 总结

本指南提供了基于Python虚拟环境的FunASR完整部署流程，主要优势：

1. **环境隔离**：避免与系统Python环境冲突
2. **易于管理**：提供完整的管理脚本
3. **可重复性**：requirements.txt可重现环境
4. **ROS2集成**：支持与ROS2 Humble无缝集成
5. **故障恢复**：快速重建环境的能力

按照本指南步骤操作，您将获得一个稳定、可维护的FunASR部署环境。

---
*文档版本: 1.1*  
*更新日期: 2025-11-10*  
*作者: chenwang*  
*适用环境: Jetson Orin NX Super 16GB + Ubuntu 22.04 + ROS2 Humble*