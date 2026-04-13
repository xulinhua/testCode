#!/usr/bin/env python3
import os
import sys
import time
import pyaudio
import wave
import numpy as np
from ctypes import *
import enum
import datetime

# 添加当前包路径以确保可以导入 snowboydetect
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# 尝试导入snowboydetect模块
try:
    import snowboydetect
    print("✅ 成功导入 snowboydetect")
    _SNOWBOY_AVAILABLE = True
except ImportError as e:
    print(f"❌ 无法导入 snowboydetect: {e}")
    print(f"当前目录: {current_dir}")
    print(f"Python路径: {sys.path}")
    print("警告: Snowboy语音唤醒功能将不可用")
    _SNOWBOY_AVAILABLE = False

# 唤醒状态

class SimpleDetector:
    def __init__(self, model_path, resource_file, sensitivity=0.5):
        # 检查snowboy是否可用
        if not _SNOWBOY_AVAILABLE:
            print("Snowboy不可用，无法创建检测器")
            raise RuntimeError("Snowboy不可用，无法创建检测器")
        self.model_path = model_path
        self.resource_file = resource_file
        self.sensitivity = sensitivity
        print(f"开始加载资源文件")
        print(f"加载资源: {self.resource_file}")
        print(f"加载模型: {self.model_path}")
        
        # 检查文件是否存在
        if not os.path.exists(self.resource_file):
            print(f"❌ 资源文件不存在: {self.resource_file}")
            
        if not os.path.exists(self.model_path):
            print(f"❌ 模型文件不存在: {self.model_path}")
        
        # 创建检测器
        try:
            self.detector = snowboydetect.SnowboyDetect(
                resource_filename=self.resource_file.encode(),
                model_str=self.model_path.encode()
            )
            self.detector.SetSensitivity(str(sensitivity).encode())
            self.detector.SetAudioGain(1.0)
            
            print(f"音频参数: {self.detector.SampleRate()}Hz, {self.detector.NumChannels()}声道, {self.detector.BitsPerSample()}位")
            
        except Exception as e:
            print(f"❌ 创建检测器失败: {e}")
            raise
        
        # 音频设置
        self.chunk_size = 2048
        self.audio = None
        self.stream = None
        self.running = False
        self.last_wakeup_time = 0  # 上次唤醒时间戳
        
    # 获取唤醒状态
    def get_wakeup_status(self):
        return self.wakeup_status
    
    # 设置唤醒状态
    def set_wakeup_status(self, wakeup_status):
        self.wakeup_status = wakeup_status

    def resample_audio(self, data, original_rate, target_rate, channels):
        """音频重采样"""
        try:
            audio_data = np.frombuffer(data, dtype=np.int16)
            
            if channels == 2:
                audio_data = audio_data.reshape(-1, 2)
                audio_data = audio_data.mean(axis=1).astype(np.int16)
            
            ratio = target_rate / original_rate
            new_length = int(len(audio_data) * ratio)
            
            x_old = np.linspace(0, 1, len(audio_data))
            x_new = np.linspace(0, 1, new_length)
            resampled = np.interp(x_new, x_old, audio_data).astype(np.int16)
            
            return resampled.tobytes()
            
        except Exception as e:
            print(f"重采样错误: {e}")
            return data
    
    # 识别是否存在唤醒词并且打印时间节点
    # 是返回 1
    # 否返回 0
    def Run(self, data, file_sample_rate, file_channels, file_sample_width):
        if len(data) == 0:
            return 0
        result = 0
        detections = 0
        total_frames = 0

        total_frames += len(data) // (file_sample_width * file_channels)
        timestamp = total_frames / file_sample_rate
            
        result = self.RunDetection(data, file_sample_rate, file_channels, self.sensitivity)
        
        if result > 0:
            detections += 1
            print(f"✅ 检测到关键词! (ID: {result}) - 时间: {timestamp:.2f}秒")

        return result
    
    # 识别是否存在唤醒词 是返回 1/否返回 0
    # data              : 音频数据
    # file_sample_rate  : 音频频率
    # file_channels     : 音频通道数
    # sensitivity       : 灵敏度 范围0.0~1.0，该值越大，越容易识别也越容易误识别，该值越小越容易漏识别
    def RunDetection(self, data, file_sample_rate, file_channels, sensitivity=0.5):
        # 如果snowboy不可用，直接返回0表示未检测到唤醒词
        if not _SNOWBOY_AVAILABLE:
            return 0
            
        if len(data) == 0:
            return 0
        result = 0
        target_sample_rate = self.detector.SampleRate()
        target_channels = self.detector.NumChannels()
        self.detector.SetSensitivity(str(sensitivity).encode())
        
        needs_resample = file_sample_rate != target_sample_rate or file_channels != target_channels
        if needs_resample:
            # print("🔄 检测到格式不匹配，启用自动重采样...")
            resample_data = self.resample_audio(data, file_sample_rate, target_sample_rate, file_channels)   
            result = self.detector.RunDetection(resample_data)
        else:
            result = self.detector.RunDetection(data) 

        if result > 0:
            self.last_wakeup_time = time.time()  # 上次唤醒时间戳
            wakeup_time_str = datetime.datetime.fromtimestamp(self.last_wakeup_time).strftime('%Y-%m-%d %H:%M:%S')
            print(f"🎯 唤醒状态设置为 ON， 灵敏度：{sensitivity}，唤醒时间: {wakeup_time_str}")
            
        return result

    # 通过传入录音路径识别是否存在唤醒词
    def test_audio_file(self, audio_file_path):
        """测试音频文件中的语音唤醒"""
        if not os.path.exists(audio_file_path):
            print(f"❌ 音频文件不存在: {audio_file_path}")
            return False
            
        print(f"🎵 测试音频文件: {audio_file_path}")
        
        try:
            wf = wave.open(audio_file_path, 'rb')
            
            file_channels = wf.getnchannels()
            file_sample_width = wf.getsampwidth()
            file_sample_rate = wf.getframerate()
            target_sample_rate = self.detector.SampleRate()
            target_channels = self.detector.NumChannels()
            print(f"音频文件参数: {file_sample_rate}Hz, {file_channels}声道, {file_sample_width}字节/样本")
            print(f"模型要求参数: {target_sample_rate}Hz, {target_channels}声道")
            detections = 0
            print(f"\n📊 开始识别:")
            while True:
                data = wf.readframes(self.chunk_size)
                if len(data) == 0:
                    break
                result = self.Run(data, file_sample_rate, file_channels, file_sample_width)
                if result > 0:
                    detections += 1

            wf.close()

            print(f"\n📊 测试完成:")
            # print(f"   总音频时长: {total_frames/target_sample_rate:.2f}秒")
            print(f"   检测到唤醒次数: {detections}")
            
            if detections > 0:
                print("🎉 语音唤醒功能正常！")
            else:
                print("❌ 未检测到唤醒词")
                
            return detections > 0
            
        except Exception as e:
            print(f"❌ 处理音频文件时出错: {e}")
            return False
    
    # 开始实时语音检测是否存在唤醒词
    def start_detection(self):
        """开始实时语音检测"""
        self.running = True
        
        try:
            self.audio = pyaudio.PyAudio()
            
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.detector.NumChannels(),
                rate=self.detector.SampleRate(),
                input=True,
                frames_per_buffer=self.chunk_size,
                stream_callback=None
            )
            
            print("🎧 开始实时监听... 请说唤醒词")
            print("按 Ctrl+C 停止")
            
            detection_count = 0
            start_time = time.time()
            
            while self.running:
                try:
                    data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                    result = self.detector.RunDetection(data)
                    
                    if result > 0:
                        detection_count += 1
                        current_time = time.time() - start_time
                        print(f"✅ 检测到关键词! (ID: {result}) - 第{detection_count}次 - 时间: {current_time:.2f}秒")
                        # 发布唤醒事件（如果需要）
                        # self.publish_wakeup_event(result, current_time)
                        
                except Exception as e:
                    print(f"音频处理错误: {e}")
                    break
                    
        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"错误: {e}")
        finally:
            self.stop()
            
    def stop(self):
        """停止检测"""
        self.running = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio:
            self.audio.terminate()
        print("检测已停止")