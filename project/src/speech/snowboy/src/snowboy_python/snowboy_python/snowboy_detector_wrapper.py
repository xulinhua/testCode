#!/usr/bin/env python3
import os
import sys

# 添加当前包路径以确保可以导入 simpledetector
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# 初始化全局变量
_SIMPLE_DETECTOR_AVAILABLE = False
SimpleDetector = None

try:
    from simpledetector import SimpleDetector
    print("✅ 成功导入 SimpleDetector")
    _SIMPLE_DETECTOR_AVAILABLE = True
except ImportError as e:
    print(f"❌ 无法导入 SimpleDetector: {e}")
    print(f"当前目录: {current_dir}")
    print(f"Python路径: {sys.path}")
    print("警告: Snowboy语音唤醒功能将不可用")


class SnowboyDetectorWrapper:
    """
    Snowboy唤醒词检测器包装类
    为C++调用提供简单的接口
    """
    
    def __init__(self, model_path, resource_path, sensitivity=0.5):
        """
        初始化唤醒词检测器
        
        Args:
            model_path (str): 唤醒词模型文件路径
            resource_path (str): 资源文件路径
            sensitivity (float): 检测灵敏度 (0.0 - 1.0)
        """
        self.detector = None
        
        # 检查文件是否存在
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"模型文件不存在: {model_path}")
            
        if not os.path.exists(resource_path):
            raise FileNotFoundError(f"资源文件不存在: {resource_path}")
            
        if not _SIMPLE_DETECTOR_AVAILABLE or SimpleDetector is None:
            raise RuntimeError("SimpleDetector不可用，无法创建检测器")
            
        try:
            self.detector = SimpleDetector(
                model_path=model_path,
                resource_file=resource_path,
                sensitivity=sensitivity
            )
            print(f"✅ SnowboyDetectorWrapper 初始化成功")
            print(f"   模型路径: {model_path}")
            print(f"   资源路径: {resource_path}")
            print(f"   灵敏度: {sensitivity}")
        except Exception as e:
            raise RuntimeError(f"创建检测器失败: {e}")
    
    def detect(self, audio_data, sample_rate, channels, detect_sensitivity = 0.5):
        """
        检测音频数据中是否包含唤醒词
        
        Args:
            audio_data (bytes): 音频数据
            sample_rate (int): 采样率
            channels (int): 声道数
            
        Returns:
            bool: True表示检测到唤醒词，False表示未检测到
        """
        if self.detector is None:
            return False
            
        try:
            # 调用检测方法
            result = self.detector.RunDetection(
                data=audio_data,
                file_sample_rate=sample_rate,
                file_channels=channels,
                sensitivity=detect_sensitivity
            )
            return result > 0
        except Exception as e:
            print(f"检测过程中出错: {e}")
            return False
    
    def get_sample_rate(self):
        """
        获取检测器要求的采样率
        
        Returns:
            int: 采样率
        """
        if self.detector is None:
            return 0
        return self.detector.detector.SampleRate()
    
    def get_num_channels(self):
        """
        获取检测器要求的声道数
        
        Returns:
            int: 声道数
        """
        if self.detector is None:
            return 0
        return self.detector.detector.NumChannels()


# 测试代码
if __name__ == "__main__":
    try:
        # 使用正确的相对路径
        wrapper = SnowboyDetectorWrapper(
            model_path="../../../src/snowboy/src/resources/models/hoson.pmdl",
            resource_path="../../../src/snowboy/src/resources/common.res",
            sensitivity=0.5
        )
        print("✅ SnowboyDetectorWrapper 测试初始化成功")
        
        # 测试获取参数
        sample_rate = wrapper.get_sample_rate()
        channels = wrapper.get_num_channels()
        print(f"检测器参数: 采样率={sample_rate}, 声道数={channels}")
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")