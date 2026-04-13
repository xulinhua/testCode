"""
Piper Speech Synthesis Module
============================

基于Piper项目的语音合成系统，支持文本转语音并保存为WAV文件。

模块功能：
- 文本到音素的转换
- 语音合成推理
- WAV文件生成
- 配置参数管理
- 多语音模型支持

作者: AI Assistant
版本: 1.0.1
"""

__version__ = "1.0.1"
__author__ = "AI Assistant"

from .tts_engine import TTSEngine
from .config_manager import ConfigManager
from .audio_generator import AudioGenerator
from .voice_model import VoiceModelManager

__all__ = ["TTSEngine", "ConfigManager", "AudioGenerator", "VoiceModelManager"]