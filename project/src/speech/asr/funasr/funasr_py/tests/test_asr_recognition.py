#!/usr/bin/env python3
"""
ASR识别测试模块
"""

import sys
import os
import numpy as np

# 添加项目根目录到Python路径
project_root = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
sys.path.insert(0, os.path.abspath(project_root))

# 添加audio_basic_py包到Python路径
audio_basic_py_path = os.path.join(project_root, 'src', 'audio_basic_py')
if audio_basic_py_path not in sys.path:
    sys.path.insert(0, audio_basic_py_path)

# 确保能正确导入必要的库
try:
    import funasr
    print("✅ 成功导入必要的库")
except ImportError as e:
    print(f"❌ 无法导入必要的库: {e}")
    print("请确保您已在当前Python环境中安装所需库:")
    print("   pip install funasr")
    sys.exit(1)

from asr_core.model_manager import ModelManager
from asr_core.audio_processor import AudioProcessor
from asr_core.asr_processor import ASRProcessor
from model_config import show_model_list, get_model_info

def test_asr_recognition():
    """测试ASR识别功能"""
    print("=== ASR识别测试 ===")
    
    # 创建处理器，使用新的资源路径
    model_manager = ModelManager("../resources/models")
    audio_processor = AudioProcessor("../resources/audio_data")
    asr_processor = ASRProcessor()
    
    # 显示模型列表
    show_model_list()
    
    # 获取用户选择
    model_info = get_model_info()
    
    # 选择模型
    while True:
        choice = input(f"\n请选择要使用的模型编号 (默认为3 - {model_info['3']['name']}): ").strip()
        
        # 如果用户直接按回车，使用默认值3
        if choice == "":
            choice = "3"
            print(f"3")  # 输出选择的编号
        
        # 选择模型类型
        model_type = input("请选择模型类型 (pt/onnx, 默认为pt): ").strip().lower()
        if model_type == "":
            model_type = "pt"
        
        # 加载模型
        selected_model = model_manager.load_model(choice, model_type)
        if selected_model:
            # 加载ASR模型
            if asr_processor.load_asr_model(selected_model):
                print(f"✅ ASR模型加载成功!")
                break
            else:
                print("❌ ASR模型加载失败!")
                return
        else:
            retry = input("\n是否重新选择模型? (y/n, 默认为n): ").strip().lower()
            if retry not in ["y", "yes"]:
                print("取消加载")
                return
    
    # 列出可用的音频文件
    audio_files = audio_processor.list_audio_files()
    if not audio_files:
        print("❌ 未找到任何音频文件，请将音频文件放置在 resources/audio_data 目录中")
        return
    
    print(f"\n📂 找到 {len(audio_files)} 个音频文件:")
    for i, file in enumerate(audio_files):
        print(f"  {i+1}. {file}")
    
    # 选择音频文件
    while True:
        try:
            file_choice = input(f"\n请选择要识别的音频文件编号 (默认为1 - {audio_files[0]}): ").strip()
            
            # 如果用户直接按回车，使用默认值1
            if file_choice == "":
                file_choice = "1"
                print(f"1")  # 输出选择的编号
            
            file_index = int(file_choice) - 1
            if 0 <= file_index < len(audio_files):
                selected_file = audio_files[file_index]
                break
            else:
                print("❌ 无效的选择，请重新输入")
        except ValueError:
            print("❌ 请输入有效的数字")
    
    # 加载音频文件
    audio_data, fs = audio_processor.load_audio_file(selected_file)
    if audio_data is None:
        print("❌ 音频文件加载失败!")
        return
    
    # 进行语音识别
    result = asr_processor.recognize_audio_file(audio_data, fs)
    if result:
        print(f"\n🎉 语音识别结果:")
        print(result)
    else:
        print("❌ 语音识别失败!")

if __name__ == "__main__":
    test_asr_recognition()