#!/usr/bin/env python3
"""
音频处理器模块
包含音频文件加载、质量分析等功能
"""

import os
import sys
import numpy as np

# 导入音频工具
from audio_basic_py import check_audio_file, analyze_audio_quality

class AudioProcessor:
    """音频处理器类"""
    
    def __init__(self, audio_data_path="../resources/audio_data"):
        """
        初始化音频处理器
        
        Args:
            audio_data_path (str): 音频数据根目录路径
        """
        # 获取当前文件的目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 尝试多种可能的音频数据路径，按照优先级排序
        possible_paths = [
            # 安装环境路径 - share目录（实际安装路径）
            os.path.abspath(os.path.join(current_dir, "..", "..", "..", "share", "funasr_py", "resources", "audio_data")),
            # 安装环境路径 - lib/share目录（某些系统路径）
            os.path.abspath(os.path.join(current_dir, "..", "..", "lib", "share", "funasr_py", "resources", "audio_data")),
            # 开发环境路径 - 外部resources目录
            os.path.abspath(os.path.join(current_dir, "..", "..", "..", "resources", "audio_data")),
            # 安装环境路径 - share目录（旧版本路径）
            os.path.abspath(os.path.join(current_dir, "..", "share", "funasr_py", "resources", "audio_data")),
            # 开发环境路径 - 内部resources目录（旧版本）
            os.path.abspath(os.path.join(current_dir, "..", "..", "resources", "audio_data")),
            # 当前目录下的resources
            os.path.abspath(os.path.join(os.getcwd(), "resources", "audio_data")),
            # 环境变量指定的路径
            os.environ.get("FUNASR_AUDIO_DATA_PATH", "")
        ]
        
        # 查找第一个存在的路径
        self.audio_data_path = ""
        for path in possible_paths:
            if path and os.path.exists(path):
                self.audio_data_path = path
                break
        
        # 如果都没找到，使用第一个路径作为默认路径
        if not self.audio_data_path:
            self.audio_data_path = possible_paths[0]
        
        print(f"📁 音频数据目录: {self.audio_data_path}")
    
    def load_audio_file(self, filename):
        """
        加载音频文件
        
        Args:
            filename (str): 音频文件名
            
        Returns:
            tuple: (data, fs) 音频数据和采样率，如果加载失败返回(None, None)
        """
        # 构建完整路径
        file_path = os.path.join(self.audio_data_path, filename)
        
        print(f"\n=== 加载音频文件: {filename} ===")
        
        # 检查文件是否存在
        if not os.path.exists(file_path):
            print(f"❌ 文件不存在: {file_path}")
            return None, None
        
        # 检查文件大小
        file_size = os.path.getsize(file_path)
        print(f"📁 文件大小: {file_size} 字节")
        
        if file_size == 0:
            print("❌ 文件为空")
            return None, None
        
        # 尝试读取音频文件
        try:
            import soundfile as sf
            data, fs = sf.read(file_path)
            
            print(f"✅ 音频文件加载成功")
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
            
            return data, fs
            
        except ImportError:
            print("❌ soundfile库未安装，无法读取音频文件")
            return None, None
        except Exception as e:
            print(f"❌ 读取音频文件失败: {e}")
            return None, None
    
    def check_audio_file(self, filename):
        """
        检查音频文件
        
        Args:
            filename (str): 音频文件名
        """
        file_path = os.path.join(self.audio_data_path, filename)
        return check_audio_file(file_path)
    
    def list_audio_files(self):
        """
        列出音频数据目录中的所有音频文件
        
        Returns:
            list: 音频文件列表
        """
        try:
            # 动态导入 audio_basic_py.audio_basic_py.audio_utils
            import sys
            import os
            # 添加audio_basic_py路径
            audio_basic_path = os.path.join(os.path.dirname(__file__), "..", "..", "audio_basic_py")
            if audio_basic_path not in sys.path:
                sys.path.append(audio_basic_path)
            
            from audio_basic_py.audio_basic_py.audio_utils import list_audio_files
            return list_audio_files(self.audio_data_path)
        except ImportError as e:
            print(f"无法导入必要的模块: {e}")
            return []