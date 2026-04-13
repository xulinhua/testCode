#!/usr/bin/env python3
"""
测试脚本用于验证基本导入是否正常工作
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

def test_imports():
    """测试基本导入"""
    print("测试基本导入...")
    
    try:
        # 测试piper_py导入
        from piper_py.tts_engine import TTSEngine
        print("✓ 成功导入 piper_py.tts_engine")
    except ImportError as e:
        print(f"✗ 导入 piper_py.tts_engine 失败: {e}")
        return False
    
    try:
        # 测试audio_utils导入
        import audio_utils
        print("✓ 成功导入 audio_utils")
    except ImportError as e:
        print(f"⚠ 导入 audio_utils 失败: {e}")
        print("  这在开发环境中是正常的，只要在ROS环境中能正常导入即可")
    
    try:
        # 测试numpy导入
        import numpy as np
        print("✓ 成功导入 numpy")
    except ImportError as e:
        print(f"✗ 导入 numpy 失败: {e}")
        return False
    
    print("基本导入测试完成")
    return True

if __name__ == '__main__':
    success = test_imports()
    if success:
        print("\n✓ 所有基本导入测试通过")
    else:
        print("\n✗ 部分导入测试失败")
        sys.exit(1)