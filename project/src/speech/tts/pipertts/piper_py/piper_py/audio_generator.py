"""
音频生成和保存模块
================

负责将合成的音频数据保存为WAV文件，并提供音频格式转换功能。

主要功能：
1. WAV文件生成
2. 音频格式转换
3. 音频质量设置
4. 批量音频处理
"""

import os
import wave
import numpy as np
from typing import Optional, List, Union
import logging

logger = logging.getLogger(__name__)


class AudioGenerator:
    """
    音频生成器类
    
    负责音频数据的保存和格式转换。
    """
    
    def __init__(self, sample_rate: int = 22050):
        """
        初始化音频生成器
        
        参数:
            sample_rate (int): 音频采样率
        """
        self.sample_rate = sample_rate
        self.audio_data = None
    
    def save_wav(self, audio_data: np.ndarray, file_path: str, 
                 sample_rate: Optional[int] = None) -> bool:
        """
        保存音频数据为WAV文件
        
        参数:
            audio_data (np.ndarray): 音频数据数组
            file_path (str): 保存文件路径
            sample_rate (int, optional): 采样率，如果为None则使用实例采样率
            
        返回:
            bool: 保存是否成功
        """
        try:
            if sample_rate is None:
                sample_rate = self.sample_rate
            
            # 确保目录存在
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            
            # 确保音频数据是16位PCM格式
            if audio_data.dtype != np.int16:
                audio_data = self._convert_to_pcm16(audio_data)
            
            # 打开WAV文件
            with wave.open(file_path, 'wb') as wav_file:
                # 设置WAV文件参数
                wav_file.setnchannels(1)  # 单声道
                wav_file.setsampwidth(2)  # 16位 = 2字节
                wav_file.setframerate(sample_rate)
                
                # 写入音频数据
                wav_file.writeframes(audio_data.tobytes())
            
            logger.info(f"音频已保存到: {file_path}")
            logger.info(f"文件大小: {os.path.getsize(file_path)} 字节")
            
            return True
            
        except Exception as e:
            logger.error(f"保存WAV文件失败: {e}")
            return False
    
    def _convert_to_pcm16(self, audio_data: np.ndarray) -> np.ndarray:
        """
        将音频数据转换为16位PCM格式
        
        参数:
            audio_data (np.ndarray): 原始音频数据
            
        返回:
            np.ndarray: 16位PCM音频数据
        """
        # 归一化到[-1, 1]范围
        if np.max(np.abs(audio_data)) > 0:
            normalized = audio_data / np.max(np.abs(audio_data))
        else:
            normalized = audio_data
        
        # 转换为16位整数
        pcm_16 = (normalized * 32767).astype(np.int16)
        
        return pcm_16
    
    def generate_silence(self, duration: float, sample_rate: Optional[int] = None) -> np.ndarray:
        """
        生成指定时长的静音
        
        参数:
            duration (float): 静音时长（秒）
            sample_rate (int, optional): 采样率
            
        返回:
            np.ndarray: 静音音频数据
        """
        if sample_rate is None:
            sample_rate = self.sample_rate
        
        num_samples = int(duration * sample_rate)
        silence = np.zeros(num_samples, dtype=np.int16)
        
        return silence
    
    def concatenate_audio(self, audio_segments: List[np.ndarray]) -> np.ndarray:
        """
        连接多个音频片段
        
        参数:
            audio_segments (List[np.ndarray]): 音频片段列表
            
        返回:
            np.ndarray: 连接后的音频数据
        """
        if not audio_segments:
            return np.array([], dtype=np.int16)
        
        # 确保所有音频都是16位格式
        processed_segments = []
        for segment in audio_segments:
            if segment.dtype != np.int16:
                processed_segments.append(self._convert_to_pcm16(segment))
            else:
                processed_segments.append(segment)
        
        # 连接所有片段
        concatenated = np.concatenate(processed_segments)
        
        return concatenated
    
    def normalize_audio(self, audio_data: np.ndarray, target_level: float = 0.8) -> np.ndarray:
        """
        归一化音频电平
        
        参数:
            audio_data (np.ndarray): 原始音频数据
            target_level (float): 目标电平（0.0到1.0）
            
        返回:
            np.ndarray: 归一化后的音频数据
        """
        # 转换为浮点数进行归一化
        float_data = audio_data.astype(np.float32) / 32768.0
        
        # 计算当前峰值
        current_peak = np.max(np.abs(float_data))
        
        if current_peak > 0:
            # 应用归一化
            normalized = float_data * (target_level / current_peak)
            
            # 确保不超过目标电平
            normalized = np.clip(normalized, -target_level, target_level)
            
            # 转换回16位整数
            normalized_int16 = (normalized * 32767).astype(np.int16)
            
            return normalized_int16
        else:
            return audio_data
    
    def trim_silence(self, audio_data: np.ndarray, threshold: float = 0.01, 
                    silence_duration: float = 0.1) -> np.ndarray:
        """
        去除音频开头和结尾的静音
        
        参数:
            audio_data (np.ndarray): 原始音频数据
            threshold (float): 静音检测阈值
            silence_duration (float): 静音持续时长（秒）
            
        返回:
            np.ndarray: 去除静音后的音频数据
        """
        # 转换为浮点数
        float_data = audio_data.astype(np.float32) / 32768.0
        
        # 计算RMS能量
        frame_size = int(0.01 * self.sample_rate)  # 10ms帧
        frames = []
        
        for i in range(0, len(float_data), frame_size):
            frame = float_data[i:i+frame_size]
            if len(frame) > 0:
                rms = np.sqrt(np.mean(frame**2))
                frames.append(rms)
        
        # 找到非静音的起始和结束位置
        silence_threshold = threshold
        min_silence_frames = int(silence_duration * self.sample_rate / frame_size)
        
        # 寻找起始位置
        start_frame = 0
        for i, rms in enumerate(frames):
            if rms > silence_threshold:
                start_frame = max(0, i - 1)
                break
        
        # 寻找结束位置
        end_frame = len(frames)
        for i in range(len(frames)-1, -1, -1):
            if frames[i] > silence_threshold:
                end_frame = min(len(frames), i + 2)
                break
        
        # 转换为采样点位置
        start_sample = start_frame * frame_size
        end_sample = min(len(audio_data), end_frame * frame_size)
        
        # 截取音频
        trimmed_audio = audio_data[start_sample:end_sample]
        
        logger.info(f"音频修剪: {len(audio_data)} -> {len(trimmed_audio)} 采样点")
        
        return trimmed_audio
    
    def get_audio_info(self, audio_data: np.ndarray) -> dict:
        """
        获取音频信息
        
        参数:
            audio_data (np.ndarray): 音频数据
            
        返回:
            dict: 音频信息字典
        """
        duration = len(audio_data) / self.sample_rate
        
        # 转换为浮点数计算统计信息
        float_data = audio_data.astype(np.float32) / 32768.0
        
        info = {
            "duration_seconds": duration,
            "sample_count": len(audio_data),
            "sample_rate": self.sample_rate,
            "max_amplitude": float(np.max(np.abs(float_data))),
            "rms_amplitude": float(np.sqrt(np.mean(float_data**2))),
            "data_type": str(audio_data.dtype)
        }
        
        return info
    
    def batch_save(self, audio_data_list: List[np.ndarray], 
                  file_pattern: str, start_index: int = 1) -> List[str]:
        """
        批量保存音频文件
        
        参数:
            audio_data_list (List[np.ndarray]): 音频数据列表
            file_pattern (str): 文件名模式，支持{index}占位符
            start_index (int): 起始索引
            
        返回:
            List[str]: 保存的文件路径列表
        """
        saved_files = []
        
        for i, audio_data in enumerate(audio_data_list):
            index = start_index + i
            file_path = file_pattern.format(index=index)
            
            if self.save_wav(audio_data, file_path):
                saved_files.append(file_path)
            else:
                logger.warning(f"保存文件失败: {file_path}")
        
        logger.info(f"批量保存完成: {len(saved_files)}/{len(audio_data_list)} 文件")
        return saved_files