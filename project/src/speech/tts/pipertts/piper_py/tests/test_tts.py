#!/usr/bin/env python3
"""
测试TTS引擎
"""

import os
import sys
import time
import wave
import numpy as np

# 添加模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from piper_py.tts_engine import TTSEngine
from piper_py.config_manager import ConfigManager

def test_fixed_engine():
    """测试TTS引擎"""
    print("=" * 60)
    print("测试TTS引擎")
    print("=" * 60)
    
    # 创建引擎实例
    engine = TTSEngine()
    
    # 模型路径
    model_path = "../../resources/models/zh_CN/medium/zh_CN-huayan-medium.onnx"
    config_path = "../../resources/models/zh_CN/medium/zh_CN-huayan-medium.onnx.json"
    
    # 检查文件是否存在
    if not os.path.exists(model_path):
        print(f"❌ 模型文件不存在: {model_path}")
        return False
    
    if not os.path.exists(config_path):
        print(f"❌ 配置文件不存在: {config_path}")
        return False
    
    print("✓ 模型文件检查通过")
    
    # 加载模型
    print("正在加载模型...")
    start_time = time.time()
    
    if not engine.load_model(model_path, config_path):
        print("❌ 模型加载失败")
        return False
    
    load_time = time.time() - start_time
    print(f"✓ 模型加载成功，耗时: {load_time:.2f}秒")
    
    # 测试文本
    test_texts = [
        "什么是语音合成呢？语音合成，是通过机械的、电子的方法产生人造语音的技术。",
        "它是将计算机自己产生的、或外部输入的文字信息转变为可以听得懂的、流利的汉语口语输出的技术。"
    ]
    
    # 初始化配置管理器
    config_manager = ConfigManager("../config")
    config_manager.load_config("default.yaml")
    config_manager.load_platform_config("jetson")
    
    # 确保输出目录存在
    output_dir = config_manager.get("audio.output_dir", "../../resources/audio_data")
    os.makedirs(output_dir, exist_ok=True)
    
    # 测试每个文本
    for i, text in enumerate(test_texts):
        print(f"\n测试文本 {i+1}: {text}")
        
        # 合成语音
        start_time = time.time()
        audio_data = engine.synthesize(text)
        synth_time = time.time() - start_time
        
        if audio_data is None:
            print(f"❌ 语音合成失败")
            continue
        
        # 计算音频时长
        audio_duration = len(audio_data) / engine.sample_rate
        
        print(f"✓ 合成成功")
        print(f"  合成耗时: {synth_time:.2f}秒")
        print(f"  音频时长: {audio_duration:.2f}秒")
        print(f"  采样点数: {len(audio_data)}")
        
        # 保存音频文件
        output_path = os.path.join(output_dir, f"test_fixed_{i+1}.wav")
        if engine.save_wav(audio_data, output_path):
            print(f"✓ 音频已保存: {output_path}")
        
        # 显示音频波形信息
        print(f"  音频波形范围: [{np.min(audio_data)}, {np.max(audio_data)}]")
        print(f"  音频均值: {np.mean(audio_data):.2f}")
    
    # 关闭引擎
    engine.close()
    
    print("\n" + "=" * 60)
    print("测试完成！")
    print("=" * 60)
    
    return True

if __name__ == "__main__":
    test_fixed_engine()