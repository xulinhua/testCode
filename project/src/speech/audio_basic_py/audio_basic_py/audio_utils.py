#!/usr/bin/env python3
"""
音频处理工具模块
包含音频文件检查、质量分析、播放等功能
"""

import os
import numpy as np
import wave
import datetime


def analyze_audio_quality(data, fs, filename):
    """分析音频质量特征"""
    print("📊 音频质量分析:")
    
    # 计算基本统计信息
    max_amplitude = np.max(np.abs(data))
    mean_amplitude = np.mean(np.abs(data))
    rms = np.sqrt(np.mean(data**2))
    
    print(f"   📈 最大幅度: {max_amplitude:.4f}")
    print(f"   📈 平均幅度: {mean_amplitude:.4f}")
    print(f"   📈 RMS值: {rms:.4f}")
    
    # 检查动态范围
    dynamic_range = 20 * np.log10(max_amplitude / (np.std(data) + 1e-10))
    print(f"   🔊 动态范围: {dynamic_range:.2f} dB")
    
    # 检查是否为语音（通过频谱分析）
    if len(data) > 1024:
        # 计算频谱
        spectrum = np.abs(np.fft.fft(data[:1024]))
        freq = np.fft.fftfreq(1024, 1/fs)
        
        # 语音通常在80-400Hz有基频能量
        speech_band = spectrum[(freq > 80) & (freq < 400)]
        if len(speech_band) > 0:
            speech_energy = np.max(speech_band)
            total_energy = np.max(spectrum)
            speech_ratio = speech_energy / total_energy if total_energy > 0 else 0
            
            print(f"   🎤 语音能量占比: {speech_ratio:.2%}")
            if speech_ratio > 0.1:
                print("   ✅ 检测到语音特征")
            else:
                print("   ⚠️ 语音特征较弱")

def check_audio_file(filename):
    """检查音频文件的基本信息"""
    print(f"\n=== 检查音频文件: {filename} ===")
    
    # 检查文件是否存在
    if not os.path.exists(filename):
        print(f"❌ 文件不存在: {filename}")
        return False
    
    # 检查文件大小
    file_size = os.path.getsize(filename)
    print(f"📁 文件大小: {file_size} 字节")
    
    if file_size == 0:
        print("❌ 文件为空")
        return False
    
    # 尝试读取音频文件
    try:
        import soundfile as sf
        data, fs = sf.read(filename)
        
        print(f"✅ 音频文件读取成功")
        print(f"🎵 采样率: {fs} Hz")
        print(f"🎵 采样点数: {len(data)}")
        print(f"🎵 时长: {len(data)/fs:.2f} 秒")
        print(f"🎵 声道数: {data.ndim}")
        
        if len(data) > 0:
            print(f"🎵 音频范围: {np.min(data):.3f} 到 {np.max(data):.3f}")
            print(f"🎵 音频均值: {np.mean(data):.3f}")
            
            # 检查是否为静音
            if np.max(np.abs(data)) < 0.001:
                print("⚠️ 警告: 音频可能为静音")
            else:
                print("✅ 音频包含有效信号")
            
            # 分析音频质量
            analyze_audio_quality(data, fs, filename)
        
        return True
        
    except ImportError:
        print("❌ soundfile库未安装，无法读取音频文件")
        return False
    except Exception as e:
        print(f"❌ 读取音频文件失败: {e}")
        return False

def play_audio_file(filename):
    """尝试播放音频文件"""
    print(f"\n=== 尝试播放音频: {filename} ===")
    
    if not os.path.exists(filename):
        print("❌ 文件不存在，无法播放")
        return False
    
    # 方法1: 使用pyaudio播放
    try:
        import pyaudio
        import wave
        
        wf = wave.open(filename, 'rb')
        p = pyaudio.PyAudio()
        
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                       channels=wf.getnchannels(),
                       rate=wf.getframerate(),
                       output=True)
        
        print("🔊 正在播放音频...")
        data = wf.readframes(1024)
        while data:
            stream.write(data)
            data = wf.readframes(1024)
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        print("✅ 音频播放完成")
        return True
        
    except Exception as e:
        print(f"❌ pyaudio播放失败: {e}")
    
    # 方法2: 使用系统命令播放
    try:
        import subprocess
        import platform
        
        system = platform.system()
        if system == "Linux":
            # Linux系统使用aplay或paplay
            result = subprocess.run(['which', 'aplay'], capture_output=True)
            if result.returncode == 0:
                subprocess.run(['aplay', filename])
                print("✅ 使用aplay播放完成")
                return True
            else:
                result = subprocess.run(['which', 'paplay'], capture_output=True)
                if result.returncode == 0:
                    subprocess.run(['paplay', filename])
                    print("✅ 使用paplay播放完成")
                    return True
        elif system == "Darwin":  # macOS
            subprocess.run(['afplay', filename])
            print("✅ 使用afplay播放完成")
            return True
        elif system == "win32":
            subprocess.run(['powershell', '-c', f'(New-Object Media.SoundPlayer "{filename}").PlaySync()'])
            print("✅ 使用Windows Media播放完成")
            return True
            
    except Exception as e:
        print(f"❌ 系统命令播放失败: {e}")
    
    print("⚠️ 所有播放方法都失败，请手动检查音频文件")
    return False


def analyze_audio_data(audio_data, sample_rate=16000):
    """
    分析音频数据并返回统计信息
    
    Args:
        audio_data (list or np.array): 音频数据
        sample_rate (int): 采样率
        
    Returns:
        dict: 包含统计信息的字典
    """
    # 将音频数据转换为numpy数组
    if not isinstance(audio_data, np.ndarray):
        np_audio_data = np.array(audio_data, dtype=np.float32)
    else:
        np_audio_data = audio_data.astype(np.float32)
    
    # 计算统计数据
    stats = {
        'length': len(audio_data),
        'min': float(np.min(np_audio_data)),
        'max': float(np.max(np_audio_data)),
        'mean': float(np.mean(np_audio_data)),
        'std': float(np.std(np_audio_data)),
        'is_silent': bool(np.allclose(np_audio_data, 0.0, atol=1e-6)),
        'sample_rate': sample_rate
    }
    
    return stats


def save_audio_data(audio_data, sample_rate=16000, filename=None, debug_dir="debug_audio"):
    """
    保存音频数据为WAV文件用于调试
    
    Args:
        audio_data (list or np.array): 音频数据
        sample_rate (int): 采样率
        filename (str): 文件名，如果为None则自动生成
        debug_dir (str): 调试文件保存目录
        
    Returns:
        str: 保存的文件路径
    """
    # 创建调试目录
    if not os.path.exists(debug_dir):
        os.makedirs(debug_dir)
    
    # 生成文件名
    if filename is None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{debug_dir}/audio_{timestamp}.wav"
    
    # 将音频数据转换为numpy数组
    if not isinstance(audio_data, np.ndarray):
        np_audio_data = np.array(audio_data, dtype=np.float32)
    else:
        np_audio_data = audio_data.astype(np.float32)
    
    # 保存为WAV文件
    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)  # 单声道
        wf.setsampwidth(2)  # 16位
        wf.setframerate(sample_rate)
        
        # 将float32转换为int16
        audio_int16 = (np_audio_data * 32767).astype(np.int16)
        wf.writeframes(audio_int16.tobytes())
    
    return filename


def print_audio_stats(stats):
    """
    打印音频数据统计信息
    
    Args:
        stats (dict): analyze_audio_data返回的统计信息
    """
    print(f"音频数据统计 - 长度: {stats['length']}, "
          f"最小值: {stats['min']:.6f}, "
          f"最大值: {stats['max']:.6f}, "
          f"平均值: {stats['mean']:.6f}, "
          f"标准差: {stats['std']:.6f}")
    
    if stats['is_silent']:
        print("警告: 音频数据全为零或接近零!")

def generate_test_audio(duration=1.0, sample_rate=16000, frequency=440):
    """
    生成测试音频数据
    
    Args:
        duration (float): 持续时间（秒）
        sample_rate (int): 采样率
        frequency (int): 频率（Hz）
        
    Returns:
        list: 音频数据列表
    """
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    test_signal = 0.5 * np.sin(2 * np.pi * frequency * t)
    
    return test_signal.tolist()


def convert_audio_data(msg, audio_format):
    """
    将音频消息数据转换为浮点数组
    
    Args:
        msg: 音频消息对象
        audio_format (int): 音频格式
        
    Returns:
        list: 浮点数组
    """
    audio_data = []
    
    # 根据音频格式转换数据
    if audio_format == 16:
        # 16位整数格式
        for sample in msg.audio.audio_data.int16_data:
            audio_data.append(float(sample) / 32768.0)
    elif audio_format == 32:
        # 32位整数格式
        for sample in msg.audio.audio_data.int32_data:
            audio_data.append(float(sample) / 2147483648.0)
    else:
        # 默认为浮点格式
        audio_data = msg.audio.audio_data.float32_data
        
    return audio_data


def resample_audio(audio_data, input_rate, output_rate):
    """
    重采样音频数据
    
    Args:
        audio_data (np.array): 输入音频数据
        input_rate (int): 输入采样率
        output_rate (int): 输出采样率
        
    Returns:
        np.array: 重采样后的音频数据
    """
    if input_rate == output_rate:
        return audio_data
        
    # 计算重采样比例
    ratio = output_rate / input_rate
    new_length = int(len(audio_data) * ratio)
    
    # 使用线性插值进行重采样
    old_indices = np.arange(len(audio_data))
    new_indices = np.linspace(0, len(audio_data) - 1, new_length)
    resampled_data = np.interp(new_indices, old_indices, audio_data)
    
    return resampled_data.astype(np.float32)


def list_audio_files(audio_data_path):
    """
    列出音频数据目录中的所有音频文件
    
    Args:
        audio_data_path (str): 音频数据目录路径
        
    Returns:
        list: 音频文件列表
    """
    # 添加调试信息，打印audio_data_path值
    print(f"🔍 音频数据目录路径: {audio_data_path}")
    print(f"🔍 路径是否存在: {os.path.exists(audio_data_path)}")
    
    audio_extensions = ['.wav', '.mp3', '.flac', '.aac', '.m4a']
    audio_files = []
    
    if os.path.exists(audio_data_path):
        print("🔍 查找音频文件...")
        for file in os.listdir(audio_data_path):
            if any(file.lower().endswith(ext) for ext in audio_extensions):
                audio_files.append(file)
        print(f"✅ 找到 {len(audio_files)} 个音频文件: {audio_files}")
    else:
        print("❌ 音频数据目录不存在")
    
    return audio_files