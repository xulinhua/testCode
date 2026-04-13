#!/bin/bash

# Piper Speech Synthesis - Jetson Orin NX Super 设置脚本
# 适用于 Ubuntu 22.04 + ROS Humble 环境

set -e

echo "=================================================="
echo "Piper Speech Synthesis - Jetson 设置脚本"
echo "=================================================="

# 检查系统信息
echo "检查系统信息..."
lsb_release -a
uname -a

# 检查Python版本
echo "检查Python版本..."
python3 --version

# 检查CUDA版本
echo "检查CUDA版本..."
nvcc --version || echo "CUDA未安装或未在PATH中"

# 检查TensorRT
echo "检查TensorRT..."
dpkg -l | grep tensorrt || echo "TensorRT未安装"

# 更新系统包
echo "更新系统包..."
sudo apt update
sudo apt upgrade -y

# 安装系统依赖
echo "安装系统依赖..."
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

# 创建Python虚拟环境
echo "创建Python虚拟环境..."
python3 -m venv ~/piper_venv
source ~/piper_venv/bin/activate

# 升级pip
echo "升级pip..."
pip install --upgrade pip

# 安装Python依赖
echo "安装Python依赖..."

# 基础依赖
pip install \
    numpy>=1.21.0 \
    pyyaml>=6.0 \
    wave \
    logging

# 安装ONNX Runtime GPU版本（针对Jetson优化）
echo "安装ONNX Runtime GPU..."

# 方法1：从NVIDIA PyIndex安装
pip install nvidia-pyindex
pip install onnxruntime-gpu==1.17.0

# 方法2：如果上面的方法失败，尝试从源码编译
# pip install onnxruntime-gpu --extra-index-url https://pypi.ngc.nvidia.com

# 安装音频处理相关库
pip install \
    soundfile \
    pyaudio

# 安装piper-phonemize（可选）
echo "安装piper-phonemize..."
pip install piper-phonemize || echo "piper-phonemize安装失败，将使用简化音素转换"

# 验证安装
echo "验证安装..."
python3 -c "
import numpy as np
print('✓ numpy:', np.__version__)

import yaml
print('✓ pyyaml: 导入成功')

import onnxruntime as ort
print('✓ onnxruntime:', ort.__version__)

# 检查GPU支持
try:
    providers = ort.get_available_providers()
    print('✓ 可用推理提供者:', providers)
    if 'CUDAExecutionProvider' in providers:
        print('✓ CUDA支持: 已启用')
    else:
        print('⚠ CUDA支持: 未启用')
except Exception as e:
    print('✗ 检查推理提供者失败:', e)
"

# 创建项目目录结构
echo "创建项目目录结构..."
mkdir -p models config output logs

# 下载示例模型（如果需要）
echo "下载示例模型..."
if [ ! -f "models/test_voice.onnx" ]; then
    echo "请从以下链接下载Piper语音模型:"
    echo "https://github.com/rhasspy/piper/releases"
    echo "或使用现有的模型文件"
fi

# 设置环境变量
echo "设置环境变量..."
cat >> ~/.bashrc << EOF

# Piper Speech Synthesis 环境变量
export PIPER_SPEECH_HOME=\$(pwd)
export PYTHONPATH=\$PYTHONPATH:\$(pwd)

# 激活虚拟环境
alias piper_activate="source ~/piper_venv/bin/activate"

# 快速启动命令
alias piper_demo="cd \$(pwd) && source ~/piper_venv/bin/activate && python run_demo.py"
alias piper_cli="cd \$(pwd) && source ~/piper_venv/bin/activate && python cli/tts_cli.py"
EOF

# 创建启动脚本
echo "创建启动脚本..."
cat > start_piper.sh << 'EOF'
#!/bin/bash

# Piper Speech Synthesis 启动脚本

# 激活虚拟环境
source ~/piper_venv/bin/activate

# 设置Python路径
export PYTHONPATH=$(pwd):$PYTHONPATH

# 运行演示
python run_demo.py "$@"
EOF

chmod +x start_piper.sh

# 创建性能测试脚本
echo "创建性能测试脚本..."
cat > benchmark_piper.sh << 'EOF'
#!/bin/bash

# Piper Speech Synthesis 性能测试脚本

# 激活虚拟环境
source ~/piper_venv/bin/activate

# 设置Python路径
export PYTHONPATH=$(pwd):$PYTHONPATH

# 运行性能测试
python -c "
import sys
sys.path.append('.')
from run_demo import performance_test
performance_test()
"
EOF

chmod +x benchmark_piper.sh

# 安装完成
echo ""
echo "=================================================="
echo "安装完成！"
echo "=================================================="
echo ""
echo "下一步操作:"
echo "1. 重新加载bash配置: source ~/.bashrc"
echo "2. 下载Piper语音模型文件到models/目录"
echo "3. 运行演示: ./start_piper.sh"
echo "4. 或使用快速命令: piper_demo"
echo ""
echo "可用命令:"
echo "  piper_demo    - 运行语音合成演示"
echo "  piper_cli     - 使用命令行工具"
echo "  ./benchmark_piper.sh - 运行性能测试"
echo ""

# 显示系统资源信息
echo "系统资源信息:"
free -h
echo ""
nvidia-smi || echo "NVIDIA驱动未安装或未在PATH中"

# 重新加载bash配置
source ~/.bashrc

echo "设置脚本执行完成！"