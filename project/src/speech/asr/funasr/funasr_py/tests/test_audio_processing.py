#!/usr/bin/env python3
"""
音频处理功能测试脚本
测试audio_basic_py包在funasr_py项目中的使用
"""

import sys
import os

# 尝试导入必要的库，但允许失败
try:
    import funasr
    print("✅ 成功导入必要的库")
except ImportError as e:
    print(f"⚠️ 无法导入funasr库: {e}")
    print("注意：某些功能可能无法正常工作")

def test_audio_basic_py_import():
    """测试audio_basic_py包的导入"""
    try:
        from audio_basic_py import check_audio_file, analyze_audio_quality, play_audio_file
        print("✅ 成功导入audio_basic_py包")
        print("   可用函数: check_audio_file, analyze_audio_quality, play_audio_file")
        return True
    except ImportError as e:
        print(f"❌ 导入audio_basic_py包失败: {e}")
        return False

def test_model_config_import():
    """测试model_config的导入"""
    try:
        from funasr_py.model_config import MODEL_INFO, get_model_info, get_model_by_id, show_model_list
        print("✅ 成功导入funasr_py.model_config")
        print(f"   支持的模型数量: {len(MODEL_INFO)}")
        return True
    except ImportError as e:
        print(f"❌ 导入funasr_py.model_config失败: {e}")
        return False

def test_core_modules_import():
    """测试核心模块的导入"""
    try:
        from funasr_py.asr_core.model_manager import ModelManager
        print("✅ 成功导入funasr_py.asr_core.model_manager")
    except ImportError as e:
        print(f"❌ 导入funasr_py.asr_core.model_manager失败: {e}")
        return False

    try:
        from funasr_py.asr_core.audio_processor import AudioProcessor
        print("✅ 成功导入funasr_py.asr_core.audio_processor")
    except ImportError as e:
        print(f"❌ 导入funasr_py.asr_core.audio_processor失败: {e}")
        return False

    try:
        from funasr_py.asr_core.asr_processor import ASRProcessor
        print("✅ 成功导入funasr_py.asr_core.asr_processor")
    except ImportError as e:
        print(f"❌ 导入funasr_py.asr_core.asr_processor失败: {e}")
        return False

    return True

def test_audio_file_processing():
    """测试音频文件处理功能"""
    try:
        from funasr_py.asr_core.audio_processor import AudioProcessor
        from audio_basic_py import check_audio_file
        
        # 创建音频处理器，使用新的资源路径
        audio_processor = AudioProcessor(audio_data_path="../resources/audio_data")
        
        # 检查音频文件
        audio_files = audio_processor.list_audio_files()
        print(f"📋 音频数据目录中的文件: {audio_files}")
        
        # 如果存在1.wav文件，进行测试
        if "1.wav" in audio_files:
            print("🔊 测试音频文件处理...")
            # 检查音频文件
            result = audio_processor.check_audio_file("1.wav")
            print(f"   音频文件检查结果: {result}")
        else:
            print("⚠️ 未找到测试音频文件 resources/audio_data/1.wav")
            
        return True
    except Exception as e:
        print(f"❌ 音频文件处理测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("=== FunASR Py 项目功能测试 ===\n")
    
    # 测试导入
    print("1. 测试模块导入...")
    import_tests = [
        test_audio_basic_py_import(),
        test_model_config_import(),
        test_core_modules_import()
    ]
    
    if all(import_tests):
        print("✅ 所有模块导入测试通过\n")
    else:
        print("❌ 部分模块导入测试失败\n")
        return False
    
    # 测试音频处理功能
    print("2. 测试音频处理功能...")
    if test_audio_file_processing():
        print("✅ 音频处理功能测试通过\n")
    else:
        print("❌ 音频处理功能测试失败\n")
        return False
    
    print("=== 所有测试完成 ===")
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)