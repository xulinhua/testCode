#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试音频播放功能，特别针对采样率转换
"""

import time
import os
from audio_io_manager import AudioIOManager

def test_resample_playback():
    """测试带采样率转换的音频播放功能"""
    print("=== 带采样率转换的音频播放测试 ===")
    
    # 创建音频管理器实例
    print("1. 创建音频管理器实例...")
    # 根据项目规范，USB麦克风设备号为hw:2,0（索引24），USB扬声器设备号为hw:3,0（索引25）
    audio_manager = AudioIOManager(input_device_index=24, output_device_index=25)
    if audio_manager.output_device_info:
        print("   扬声器设备:", audio_manager.output_device_info['name'])
    else:
        print("   无法获取扬声器设备信息")
    print("   设备支持的默认采样率:", audio_manager.default_rate)
    
    try:
        # 测试播放音频文件
        print("\n2. 播放音频文件...")
        # 获取项目目录下的1.wav文件路径
        wav_file_path = os.path.join(os.path.dirname(__file__), "1.wav")
        print(f"   正在播放 {wav_file_path}...")
        
        if os.path.exists(wav_file_path):
            # 检查文件信息
            import wave
            wf = wave.open(wav_file_path, 'rb')
            file_rate = wf.getframerate()
            file_channels = wf.getnchannels()
            file_sampwidth = wf.getsampwidth()
            print(f"   音频文件信息: 采样率={file_rate}Hz, 声道数={file_channels}, 采样宽度={file_sampwidth}字节")
            wf.close()
            
            # 播放音频文件
            audio_manager.play_audio_file(wav_file_path)
        else:
            print(f"   音频文件 {wav_file_path} 不存在")
            
        print("\n=== 播放测试完成 ===")
        
    except Exception as e:
        print(f"发生错误: {e}")
        print("可能的原因:")
        print("1. USB扬声器设备不支持当前采样率")
        print("2. 音频文件采样率与设备不兼容")
        print("3. 设备索引配置错误")
        import traceback
        traceback.print_exc()
    finally:
        audio_manager.close()

if __name__ == "__main__":
    test_resample_playback()