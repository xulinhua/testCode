"""
基础使用示例
============

展示如何基本使用piper_speech库进行语音合成。
"""

import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from piper_speech import TTSEngine, ConfigManager, AudioGenerator


def basic_synthesis():
    """基础语音合成示例"""
    print("=== 基础语音合成示例 ===")
    
    # 初始化配置管理器
    config_manager = ConfigManager("config")
    config_manager.load_config("default.yaml")
    config_manager.load_platform_config("jetson")
    
    # 创建语音合成引擎
    tts_engine = TTSEngine()
    
    # 加载模型（这里需要实际模型文件）
    model_path = config_manager.get("model.path")
    config_path = config_manager.get("model.config_path")
    
    print(f"模型路径: {model_path}")
    print(f"配置路径: {config_path}")
    
    # 检查模型文件是否存在
    if os.path.exists(model_path) and os.path.exists(config_path):
        if tts_engine.load_model(model_path, config_path):
            
            # 获取模型信息
            model_info = tts_engine.get_model_info()
            print(f"模型信息: {model_info}")
            
            # 创建音频生成器
            audio_generator = AudioGenerator(sample_rate=tts_engine.sample_rate)
            
            # 合成语音
            text = "你好，这是一个语音合成测试。"
            audio_data = tts_engine.synthesize(text)
            
            if audio_data is not None:
                # 保存音频文件
                output_path = "output/basic_synthesis.wav"
                if audio_generator.save_wav(audio_data, output_path):
                    print(f"音频已保存到: {output_path}")
                    
                    # 获取音频信息
                    audio_info = audio_generator.get_audio_info(audio_data)
                    print(f"音频信息: {audio_info}")
                else:
                    print("音频保存失败")
            else:
                print("语音合成失败")
        else:
            print("模型加载失败")
    else:
        print("模型文件不存在，请确保模型文件已正确放置")
    
    # 关闭引擎
    tts_engine.close()
    print("示例完成")


def batch_synthesis():
    """批量语音合成示例"""
    print("\n=== 批量语音合成示例 ===")
    
    # 初始化配置管理器
    config_manager = ConfigManager("config")
    config_manager.load_config("default.yaml")
    
    # 创建语音合成引擎
    tts_engine = TTSEngine()
    
    # 加载模型（这里需要实际模型文件）
    model_path = config_manager.get("model.path")
    
    if os.path.exists(model_path):
        if tts_engine.load_model(model_path):
            
            # 创建音频生成器
            audio_generator = AudioGenerator(sample_rate=tts_engine.sample_rate)
            
            # 批量文本
            texts = [
                "这是第一个测试句子。",
                "这是第二个测试句子。",
                "这是第三个测试句子。"
            ]
            
            audio_data_list = []
            for i, text in enumerate(texts):
                print(f"合成第 {i+1} 个句子: {text}")
                audio_data = tts_engine.synthesize(text)
                if audio_data is not None:
                    audio_data_list.append(audio_data)
                else:
                    print(f"第 {i+1} 个句子合成失败")
            
            # 批量保存
            if audio_data_list:
                saved_files = audio_generator.batch_save(
                    audio_data_list, 
                    "output/batch_synthesis_{index}.wav"
                )
                print(f"批量保存完成: {len(saved_files)} 个文件")
            
        else:
            print("模型加载失败")
    else:
        print("模型文件不存在")
    
    # 关闭引擎
    tts_engine.close()
    print("批量示例完成")


if __name__ == "__main__":
    basic_synthesis()
    batch_synthesis()