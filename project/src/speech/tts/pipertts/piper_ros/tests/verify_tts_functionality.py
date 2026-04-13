#!/usr/bin/env python3
"""
验证TTS功能的脚本
"""

import sys
import os

# 添加项目路径
# 由于项目结构调整，需要更新路径引用
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
piper_py_path = os.path.join(project_root, '..', '..', 'piper_py')
audio_basic_path = os.path.join(project_root, '..', '..', '..', 'audio_basic_py', 'audio_basic_py')

sys.path.insert(0, project_root)
sys.path.insert(0, piper_py_path)
sys.path.insert(0, audio_basic_path)

def verify_tts_functionality():
    """验证TTS功能"""
    print("验证TTS功能...")
    
    try:
        # 导入TTSEngine
        from piper_py.tts_engine import TTSEngine
        print("✓ 成功导入TTSEngine")
        
        # 创建TTS引擎实例
        tts_engine = TTSEngine()
        print("✓ 成功创建TTS引擎实例")
        
        # 检查引擎初始化状态
        print(f"✓ TTS引擎初始化状态: {tts_engine.is_loaded}")
        
        # 尝试文本到音素转换
        test_text = "你好世界"
        phonemes = tts_engine.text_to_phonemes(test_text)
        print(f"✓ 文本到音素转换: '{test_text}' -> {phonemes}")
        
        # 尝试音素到ID转换
        if phonemes:
            phoneme_ids = tts_engine.phonemes_to_ids(phonemes)
            print(f"✓ 音素到ID转换: {phonemes} -> {phoneme_ids[:10]}...")  # 只显示前10个ID
        
        print("TTS功能验证完成")
        return True
        
    except Exception as e:
        print(f"✗ TTS功能验证失败: {e}")
        return False

if __name__ == '__main__':
    success = verify_tts_functionality()
    if success:
        print("\n✓ TTS功能验证通过")
    else:
        print("\n✗ TTS功能验证失败")
        sys.exit(1)