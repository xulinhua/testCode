#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查音频设备信息
"""
import pyaudio

def check_device_info():
    """检查音频设备信息"""
    print("=== 音频设备信息检查 ===")
    
    engine = pyaudio.PyAudio()
    
    try:
        # 获取所有音频设备信息
        device_count = engine.get_device_count()
        print(f"找到 {device_count} 个音频设备:\n")
        
        # 查找USB设备
        usb_input_device_index = None
        usb_output_device_index = None
        
        for i in range(device_count):
            device_info = engine.get_device_info_by_index(i)
            device_name = str(device_info['name'])  # 确保是字符串类型
            max_input_channels = int(device_info['maxInputChannels'])
            max_output_channels = int(device_info['maxOutputChannels'])
            
            print(f"设备 {i}: {device_name}")
            print(f"  最大输入声道数: {max_input_channels}")
            print(f"  最大输出声道数: {max_output_channels}")
            print(f"  默认采样率: {device_info['defaultSampleRate']}")
            print()
            
            # 查找USB设备（包含USB关键词的设备）
            if 'usb' in device_name.lower() or 'USB' in device_name:
                if max_input_channels > 0 and usb_input_device_index is None:
                    usb_input_device_index = i
                    print(f"  -> 检测到USB输入设备，索引: {i}")
                if max_output_channels > 0 and usb_output_device_index is None:
                    usb_output_device_index = i
                    print(f"  -> 检测到USB输出设备，索引: {i}")
        
        # 如果找到了USB设备，测试它们
        if usb_input_device_index is not None:
            print(f"\n测试USB输入设备 (索引 {usb_input_device_index}):")
            input_device_info = engine.get_device_info_by_index(usb_input_device_index)
            max_input_channels = int(input_device_info['maxInputChannels'])
            
            if max_input_channels > 0:
                for channels in range(1, min(max_input_channels + 1, 3)):  # 最多测试2声道
                    try:
                        stream = engine.open(
                            format=pyaudio.paInt16,
                            channels=channels,
                            rate=int(input_device_info['defaultSampleRate']),  # 使用设备默认采样率
                            input=True,
                            input_device_index=usb_input_device_index,
                            frames_per_buffer=1024
                        )
                        print(f"  {channels} 声道: 支持")
                        stream.close()
                    except Exception as e:
                        print(f"  {channels} 声道: 不支持 ({e})")
            else:
                print("  设备不支持输入")
                
        if usb_output_device_index is not None:
            print(f"\n测试USB输出设备 (索引 {usb_output_device_index}):")
            output_device_info = engine.get_device_info_by_index(usb_output_device_index)
            max_output_channels = int(output_device_info['maxOutputChannels'])
            
            if max_output_channels > 0:
                for channels in range(1, min(max_output_channels + 1, 3)):  # 最多测试2声道
                    try:
                        stream = engine.open(
                            format=pyaudio.paInt16,
                            channels=channels,
                            rate=int(output_device_info['defaultSampleRate']),  # 使用设备默认采样率
                            output=True,
                            output_device_index=usb_output_device_index,
                            frames_per_buffer=1024
                        )
                        print(f"  {channels} 声道: 支持")
                        stream.close()
                    except Exception as e:
                        print(f"  {channels} 声道: 不支持 ({e})")
            else:
                print("  设备不支持输出")
                
        # 如果没找到USB设备，列出所有设备供手动选择
        if usb_input_device_index is None and usb_output_device_index is None:
            print("\n未自动检测到USB设备，请手动检查以下设备列表:")
            for i in range(device_count):
                device_info = engine.get_device_info_by_index(i)
                device_name = str(device_info['name'])
                max_input_channels = int(device_info['maxInputChannels'])
                max_output_channels = int(device_info['maxOutputChannels'])
                
                if max_input_channels > 0 or max_output_channels > 0:
                    print(f"设备 {i}: {device_name}")
                    print(f"  最大输入声道数: {max_input_channels}")
                    print(f"  最大输出声道数: {max_output_channels}")
                    print(f"  默认采样率: {device_info['defaultSampleRate']}")
                    print()
                
    finally:
        engine.terminate()
        
    print("\n=== 检查完成 ===")

if __name__ == "__main__":
    check_device_info()