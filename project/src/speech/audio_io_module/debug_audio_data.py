#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调试音频数据处理
"""
import wave
import struct
import os
from audio_io_manager import AudioIOManager

def debug_audio_data():
    """调试音频数据处理"""
    print("=== 调试音频数据处理 ===")
    
    # 音频文件路径
    wav_file_path = os.path.join(os.path.dirname(__file__), "1.wav")
    
    if not os.path.exists(wav_file_path):
        print(f"音频文件 {wav_file_path} 不存在")
        return
        
    # 打开音频文件
    wf = wave.open(wav_file_path, 'rb')
    file_rate = wf.getframerate()
    file_channels = wf.getnchannels()
    file_format = wf.getsampwidth()
    
    print(f"原始音频文件信息:")
    print(f"  采样率: {file_rate} Hz")
    print(f"  声道数: {file_channels}")
    print(f"  采样宽度: {file_format} 字节")
    
    # 读取一些音频数据进行分析
    data = wf.readframes(1024)
    print(f"  读取数据长度: {len(data)} 字节")
    
    # 根据采样宽度处理数据
    if file_format == 1:  # 8位音频
        print(f"  8位音频样本数量: {len(data)}")
        print(f"  前10个样本值: {list(data[:10])}")
    elif file_format == 2:  # 16位音频
        samples = struct.unpack('<' + 'h' * (len(data) // 2), data)
        print(f"  16位音频样本数量: {len(samples)}")
        print(f"  前10个样本值: {samples[:10]}")
    
    wf.close()
    
    print("\n=== 调试完成 ===")

if __name__ == "__main__":
    debug_audio_data()