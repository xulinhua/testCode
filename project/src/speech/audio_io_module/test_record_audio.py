#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版录音测试脚本
仅实现手动开始录音和手动停止录音功能
"""

import time
import os
import threading
import pyaudio
import wave
from audio_io_manager import AudioIOManager

class SimpleRecordTest:
    def __init__(self):
        """初始化录音测试"""
        print("=== 录音测试 ===")
        # 根据内存中的设备索引映射关系，USB麦克风索引为24
        self.audio_manager = AudioIOManager(input_device_index=24, output_device_index=25)
        self.is_recording = False
        self.stream = None
        self.frames = []
        self.recording_thread = None
        self.pyaudio_instance = None
        
        # 显示设备信息
        self._show_device_info()
        
    def _show_device_info(self):
        """显示设备信息"""
        print("\n--- 设备信息 ---")
        if self.audio_manager.input_device_info:
            print(f"麦克风设备: {self.audio_manager.input_device_info['name']}")
            print(f"最大输入声道数: {int(self.audio_manager.input_device_info['maxInputChannels'])}")
            print(f"设备默认采样率: {int(self.audio_manager.input_device_info['defaultSampleRate'])}")
        else:
            print("无法获取麦克风设备信息")
            
        print(f"使用的默认采样率: {self.audio_manager.default_rate}")
        print(f"默认声道数: {self.audio_manager.default_channels}")
        
    def start_recording(self):
        """开始录音"""
        if self.is_recording:
            print("已经在录音中...")
            return
            
        print("\n--- 开始录音 ---")
        self.frames = []  # 清空之前的录音数据
        
        # 初始化PyAudio
        self.pyaudio_instance = pyaudio.PyAudio()
        
        # 获取设备参数
        chunk = self.audio_manager.default_chunk
        channels = self.audio_manager.default_channels
        rate = self.audio_manager.default_rate
        format = self.audio_manager.default_format
        
        # 确保声道数不超过设备支持的最大声道数
        if self.audio_manager.input_device_info:
            max_input_channels = int(self.audio_manager.input_device_info['maxInputChannels'])
            channels = min(channels, max_input_channels)
        
        # 打开音频流
        try:
            self.stream = self.pyaudio_instance.open(
                format=format,
                channels=channels,
                rate=rate,
                input=True,
                input_device_index=self.audio_manager.input_device_index,
                frames_per_buffer=chunk
            )
        except Exception as e:
            print(f"打开音频流失败: {e}")
            if self.pyaudio_instance:
                self.pyaudio_instance.terminate()
            return
            
        # 在后台线程中开始录音
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._record_audio)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        
        print("录音已开始... 按回车键停止录音")
        
    def _record_audio(self):
        """在后台线程中录音"""
        chunk = self.audio_manager.default_chunk
        
        try:
            while self.is_recording and self.stream:
                # 读取音频数据，添加exception_on_overflow=False参数来避免溢出错误
                data = self.stream.read(chunk, exception_on_overflow=False)
                self.frames.append(data)
        except Exception as e:
            print(f"\n录音过程中发生错误: {e}")
            
    def stop_recording(self):
        """停止录音"""
        if not self.is_recording:
            print("当前没有在录音...")
            return
            
        print("停止录音...")
        self.is_recording = False
        
        # 等待录音线程结束
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=2)
            
        # 停止并关闭音频流
        if self.stream:
            try:
                self.stream.stop_stream()
                self.stream.close()
            except Exception as e:
                print(f"关闭音频流时出错: {e}")
            finally:
                self.stream = None
                
        # 终止PyAudio实例
        if self.pyaudio_instance:
            try:
                self.pyaudio_instance.terminate()
            except Exception as e:
                print(f"终止PyAudio时出错: {e}")
            finally:
                self.pyaudio_instance = None
            
        # 保存录音文件
        self._save_audio()
        
    def _save_audio(self):
        """保存录音文件"""
        if not self.frames:
            print("没有录音数据可保存")
            return
            
        timestamp = int(time.time())
        filename = f"recorded_audio_{timestamp}.wav"
        
        # 确保output目录存在
        output_dir = os.path.join(os.path.dirname(__file__), 'output')
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        filepath = os.path.join(output_dir, filename)
        
        # 获取设备参数
        channels = self.audio_manager.default_channels
        rate = self.audio_manager.default_rate
        format = self.audio_manager.default_format
        
        # 确保声道数不超过设备支持的最大声道数
        if self.audio_manager.input_device_info:
            max_input_channels = int(self.audio_manager.input_device_info['maxInputChannels'])
            channels = min(channels, max_input_channels)
        
        # 保存为WAV文件
        try:
            p = pyaudio.PyAudio()
            wf = wave.open(filepath, 'wb')
            wf.setnchannels(channels)
            wf.setsampwidth(p.get_sample_size(format))
            wf.setframerate(rate)
            wf.writeframes(b''.join(self.frames))
            wf.close()
            p.terminate()
            
            print(f"音频已保存到: {filepath}")
        except Exception as e:
            print(f"保存音频文件时出错: {e}")

def main():
    """主函数"""
    test = SimpleRecordTest()
    
    try:
        while True:
            print("\n=== 录音控制 ===")
            print("r - 开始录音")
            print("s - 停止录音")
            print("q - 退出")
            print("请选择操作: ", end='')
            
            choice = input().strip().lower()
            
            if choice == 'r':
                test.start_recording()
                # 如果开始录音，等待用户按回车键停止
                if test.is_recording:
                    input()
                    test.stop_recording()
            elif choice == 's':
                test.stop_recording()
            elif choice == 'q':
                print("退出程序...")
                break
            else:
                print("无效选择，请重新输入")
                
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行过程中发生错误: {e}")
    finally:
        # 确保资源被正确释放
        if hasattr(test.audio_manager, 'engine'):
            test.audio_manager.close()
        print("程序已退出")

if __name__ == "__main__":
    main()