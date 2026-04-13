"""
音频处理基础工具包
提供通用的音频处理功能，不依赖于ROS
"""

# 从子包导入函数
try:
    # 首先尝试从audio_basic_py.audio_utils导入
    from audio_basic_py.audio_utils import (
        analyze_audio_quality,
        check_audio_file,
        play_audio_file,
        analyze_audio_data,
        save_audio_data,
        print_audio_stats,
        convert_audio_data,
        resample_audio
    )
    from audio_basic_py.asr_utils import (
        is_valid_result,
        _is_text_valid
    )
except ImportError:
    # 如果失败，尝试从audio_basic_py.audio_basic_py.audio_utils导入（开发环境）
    from audio_basic_py.audio_basic_py.audio_utils import (
        analyze_audio_quality,
        check_audio_file,
        play_audio_file,
        analyze_audio_data,
        save_audio_data,
        print_audio_stats,
        convert_audio_data,
        resample_audio
    )
    from audio_basic_py.audio_basic_py.asr_utils import (
        is_valid_result,
        _is_text_valid
    )

__all__ = [
    'analyze_audio_quality',
    'check_audio_file',
    'play_audio_file',
    'analyze_audio_data',
    'save_audio_data',
    'print_audio_stats',
    'convert_audio_data',
    'resample_audio',
    'is_valid_result',
    '_is_text_valid'
]