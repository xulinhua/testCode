# FunASR Python Package

## 项目信息
- **公司**: hoson 
- **项目简介**: 这是一个基于FunASR的语音识别Python包，可在多种平台上运行。该项目是一个独立的纯Python包，不依赖ROS Humble环境。 

## 依赖项目

本项目依赖于同级目录下的`audio_basic_py`项目，该项目提供了通用的音频处理功能。

## 功能特性

1. **模型管理**：
   - 支持多种FunASR模型的下载和管理
   - 支持将PyTorch模型导出为ONNX格式
   - 支持加载PyTorch和ONNX格式的模型

2. **音频处理**：
   - 音频文件加载和质量分析
   - 实时音频流处理

3. **语音识别**：
   - 支持对音频文件进行语音识别
   - 支持实时语音识别

## 项目结构

```
funasr_py/
├── asr_core/              # ASR核心模块
│   ├── model_manager.py   # 模型管理器
│   ├── audio_processor.py # 音频处理器
│   └── asr_processor.py   # ASR处理器
├── model_config.py       # 模型配置
├── tests/                 # 测试模块
│   ├── test_model_download.py
│   ├── test_model_export.py
│   ├── test_model_loading.py
│   ├── test_asr_recognition.py
│   └── test_audio_processing.py     # 音频处理功能测试
├── resources/
│   ├── audio_data/        # 音频数据目录
│   └── models/            # 模型目录
└── requirements.txt       # 依赖配置文件
```

## 安装部署说明

本项目需要在特定的环境中进行部署，详细的部署步骤请参考 [FunASR部署指南.md](FunASR部署指南.md) 文件。该指南提供了在Jetson Orin NX Super开发板上完整部署FunASR的详细步骤，包括系统环境准备、Python虚拟环境创建与配置、依赖库安装、FunASR源码安装、模型下载与配置等。

请按照部署指南中的步骤操作，确保所有依赖库正确安装，特别是PyTorch和FunASR库。

## 安装依赖

项目推荐在Python虚拟环境speech_env下运行，但不是必须的。如果您选择使用虚拟环境，请确保已激活该环境：

```bash
# 激活Python虚拟环境（可选）
source ~/speech_env/bin/activate  # Linux/macOS
# 或
~/speech_env\Scripts\activate     # Windows

# 安装项目依赖
pip install -r requirements.txt
```

## 构建和使用

### 安装项目

```bash
# 激活Python虚拟环境（可选）
source ~/speech_env/bin/activate  # Linux/macOS
# 或
~/speech_env\Scripts\activate     # Windows

# 进入项目目录
cd src/asr/funasr_py

# 安装项目
pip install -e .
```

### 在ROS环境下构建（可选）

该项目虽然是独立的Python包，但也可以在ROS环境下构建和使用：

```bash
# 激活Python虚拟环境（可选）
source ~/speech_env/bin/activate  # Linux/macOS
# 或
~/speech_env\Scripts\activate     # Windows

# 在ROS工作空间根目录下执行
colcon build --packages-select funasr_py

# 构建完成后，source工作空间
source install/setup.bash
```

### 运行测试

#### 方法1：直接运行测试脚本（推荐在虚拟环境中）

```bash
# 激活Python虚拟环境（可选）
source ~/speech_env/bin/activate  # Linux/macOS
# 或
~/speech_env\Scripts\activate     # Windows

# 进入项目目录
cd src/asr/funasr_py

# 确保已安装所有依赖
pip install -r requirements.txt

# 测试模型下载
python tests/test_model_download.py

# 测试模型导出
python tests/test_model_export.py

# 测试模型加载
python tests/test_model_loading.py

# 测试ASR识别
python tests/test_asr_recognition.py

# 测试音频处理功能
python tests/test_audio_processing.py
```

#### 方法2：通过ROS 2运行测试脚本

```bash
# 在ROS工作空间根目录下执行
colcon build --packages-select funasr_py

# 构建完成后，source工作空间
source install/setup.bash

# 通过ROS 2运行测试脚本
ros2 run funasr_py test_model_download
ros2 run funasr_py test_model_export
ros2 run funasr_py test_model_loading
ros2 run funasr_py test_asr_recognition
ros2 run funasr_py test_audio_processing
```

## 使用说明

1. **准备音频文件**：将需要识别的音频文件放入`resources/audio_data`目录中

2. **下载模型**：运行模型下载测试脚本选择并下载需要的模型

3. **导出模型**（可选）：如果需要使用ONNX模型，可以导出PyTorch模型为ONNX格式

4. **进行语音识别**：运行ASR识别测试脚本，选择模型和音频文件进行识别

## 支持的模型

- SenseVoiceSmall：多语言语音理解模型
- paraformer-zh：非流式中文语音识别模型
- paraformer-zh-streaming：流式中文语音识别模型
- paraformer-en：英文语音识别模型
- conformer-en：基于Conformer的英文语音识别模型
- 以及其他多种模型

## 常见问题解决

### "未找到funasr库"错误

如果您遇到"未找到funasr库，请先安装"的错误提示，请参考[FunASR部署指南.md](FunASR部署指南.md)中的详细部署步骤，确保正确安装所有依赖项。

### "Exec format error"错误

如果在Jetson Orin NX Super开发板上运行时遇到"Exec format error"错误，请确保在CMakeLists.txt中正确设置了Python脚本的执行权限。

### 网络代理导致依赖安装失败

如果在安装依赖时遇到"Cannot connect to proxy"或"Connection refused"错误，请检查系统代理设置。可以尝试以下解决方案：

1. 临时取消代理设置：
   ```bash
   unset http_proxy
   unset https_proxy
   ```

2. 使用国内镜像源安装：
   ```bash
   pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple
   ```

## 注意事项

1. 本项目为纯Python包，不依赖ROS环境，可以独立使用
2. 需要安装相应的依赖库才能正常运行
3. 模型文件较大，请确保有足够的存储空间
4. 实时语音识别需要麦克风设备支持
5. 本项目可以被其他项目（包括ROS项目）导入和使用
6. 虽然项目不依赖ROS，但可以在ROS环境下构建和使用
7. 项目推荐在Python虚拟环境~/speech_env下运行，但不是必须的
8. 模型和音频数据现在统一存储在`resources`目录下，便于管理和部署

## 版本信息

### v1.0.0 (2025-11-06)
- **修改人**: chenwang 
- **修改内容**: 
   1）项目创建和初始版本发布
   2）完善功能模块化封装

### v1.0.1 (2025-11-10)
- **修改人**: chenwang 
- **修改内容**: 
   1）更新模型下载、加载和音频数据路径，指向相对路径asr\funasr\resources目录
   2）确保能在Jetson Orin NX Super开发板Ubuntu22.04 + ROS Humble环境下构建成功及运行正常