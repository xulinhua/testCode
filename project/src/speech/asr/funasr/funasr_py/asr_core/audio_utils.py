#!/usr/bin/env python3
"""
音频工具模块
包含音频数据转换、结果验证、重采样等工具函数
"""

import sys
import os

# 添加audio_basic_py路径
audio_basic_path = os.path.join(os.path.dirname(__file__), "..", "..", "audio_basic_py")
if audio_basic_path not in sys.path:
    sys.path.append(audio_basic_path)

# 从audio_basic_py导入函数，修复导入路径问题
try:
    # 首先尝试直接从audio_basic_py.audio_utils导入
    from audio_basic_py.audio_utils import convert_audio_data, resample_audio
    from audio_basic_py.asr_utils import is_valid_result, _is_text_valid
except ImportError:
    # 如果失败，尝试从audio_basic_py.audio_basic_py.audio_utils导入（开发环境）
    try:
        from audio_basic_py.audio_basic_py.audio_utils import convert_audio_data, resample_audio
        from audio_basic_py.audio_basic_py.asr_utils import is_valid_result, _is_text_valid
    except ImportError:
        # 如果都失败，尝试从已安装的包导入
        try:
            from audio_basic_py import audio_utils, asr_utils
            convert_audio_data = audio_utils.convert_audio_data
            resample_audio = audio_utils.resample_audio
            is_valid_result = asr_utils.is_valid_result
            _is_text_valid = asr_utils._is_text_valid
        except ImportError:
            # 最后的备选方案
            from audio_basic_py.audio_basic_py.audio_utils import convert_audio_data, resample_audio
            from audio_basic_py.audio_basic_py.asr_utils import is_valid_result, _is_text_valid

__all__ = [
    'convert_audio_data',
    'is_valid_result',
    'resample_audio',
    '_is_text_valid'
]