"""
高级功能示例
============

展示piper_speech库的高级功能，如音频处理和参数调整。
"""

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from piper_speech import TTSEngine, ConfigManager, AudioGenerator
import numpy as np


def audio_processing():
    """音频处理示例"""
    print("=== 音频处理示例 ===")
    
    # 创建音频生成器
    audio_generator = AudioGenerator(sample_rate=22050)
    
    # 生成静音
    silence_duration = 2.0  # 2秒
    silence_audio = audio_generator.generate_silence(silence_duration)
    print(f"生成长度为 {silence_duration} 秒的静音")
    
    # 保存静音文件
    audio_generator.save_wav(silence_audio, "output/silence.wav")
    
    # 音频连接示例
    audio_segments = [
        silence_audio,  # 静音
        silence_audio,  # 再静音
    ]
    
    concatenated_audio = audio_generator.concatenate_audio(audio_segments)
    audio_generator.save_wav(concatenated_audio, "output/concatenated.wav")
    print("音频连接完成")


def config_management():
    """配置管理示例"""
    print("\n=== 配置管理示例 ===")
    
    # 创建配置管理器
    config_manager = ConfigManager("config")
    
    # 加载默认配置
    config_manager.load_config("default.yaml")
    
    # 获取配置值
    sample_rate = config_manager.get("audio.sample_rate")
    max_text_length = config_manager.get("synthesis.max_text_length")
    
    print(f"当前采样率: {sample_rate}")
    print(f"最大文本长度: {max_text_length}")
    
    # 修改配置
    config_manager.set("audio.sample_rate", 16000)
    config_manager.set("synthesis.batch_size", 4)
    
    # 验证配置
    if config_manager.validate_config():
        print("配置验证通过")
    
    # 保存当前配置
    config_manager.save_config("current_config.yaml")
    
    # 获取所有配置
    all_config = config_manager.get_all_config()
    print(f"完整配置: {all_config}")


def performance_optimization():
    """性能优化示例"""
    print("\n=== 性能优化示例 ===")
    
    # 创建配置管理器
    config_manager = ConfigManager("config")
    
    # 加载Jetson平台特定配置
    config_manager.load_platform_config("jetson")
    
    # 获取性能相关配置
    use_gpu = config_manager.get("performance.use_gpu")
    threads = config_manager.get("performance.threads")
    optimization_level = config_manager.get("performance.optimization_level")
    
    print(f"使用GPU: {use_gpu}")
    print(f"线程数: {threads}")
    print(f"优化级别: {optimization_level}")
    
    # 模拟不同配置下的性能比较
    configs = [
        {"name": "CPU模式", "use_gpu": False, "threads": 4},
        {"name": "GPU模式", "use_gpu": True, "threads": 8},
        {"name": "高性能模式", "use_gpu": True, "threads": 16, "optimization_level": 3}
    ]
    
    for config in configs:
        print(f"\n配置: {config['name']}")
        print(f"  使用GPU: {config.get('use_gpu', use_gpu)}")
        print(f"  线程数: {config.get('threads', threads)}")
        print(f"  优化级别: {config.get('optimization_level', optimization_level)}")


def text_processing():
    """文本处理示例"""
    print("\n=== 文本处理示例 ===")
    
    # 测试文本
    test_texts = [
        "Hello, world!",
        "这是一个中文测试。",
        "1234567890",
        "特殊字符: !@#$%^&*()",
        "这是一段比较长的文本，用于测试文本长度限制和分段处理功能。"
    ]
    
    # 创建语音合成引擎
    tts_engine = TTSEngine()
    
    for text in test_texts:
        print(f"\n原文: {text}")
        
        # 音素转换
        phonemes = tts_engine.text_to_phonemes(text)
        print(f"音素序列: {phonemes}")
        
        # 检查文本长度
        if len(text) > 100:
            print("警告: 文本过长，建议分段处理")
        
    tts_engine.close()


if __name__ == "__main__":
    audio_processing()
    config_management()
    performance_optimization()
    text_processing()