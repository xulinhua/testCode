#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速测试脚本
===========

验证Piper语音合成系统的基本功能
"""

import os
import sys
import logging

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_model_loading():
    """测试模型加载功能"""
    print("=" * 50)
    print("测试模型加载功能")
    print("=" * 50)
    
    try:
        from piper_py.voice_model import VoiceModelManager
        
        # 创建模型管理器
        model_manager = VoiceModelManager("../../resources/models")
        
        # 检查模型数量
        models = model_manager.get_model_list()
        print(f"发现 {len(models)} 个模型")
        
        # 显示模型信息
        for model in models:
            print(f"  - {model['name']}")
            print(f"    语言: {model.get('language', '未知')}")
            print(f"    质量: {model.get('quality', '未知')}")
            print(f"    采样率: {model['sample_rate']}Hz")
        
        # 测试中文模型设置
        if model_manager.set_zh_cn_medium_model():
            print("✓ 成功设置中文中等质量模型")
        else:
            print("✗ 设置中文中等质量模型失败")
        
        return True
        
    except Exception as e:
        print(f"✗ 模型加载测试失败: {e}")
        return False


def test_text_file_loading():
    """测试文本文件加载功能"""
    print("\n" + "=" * 50)
    print("测试文本文件加载功能")
    print("=" * 50)
    
    try:
        from test_interactive import InteractiveTester
        
        # 创建测试器
        tester = InteractiveTester("../../resources/models")
        
        # 测试文本文件
        text_file = os.path.join("../../resources/test_text", "1.txt")
        if os.path.exists(text_file):
            content = tester.load_text_from_file(text_file)
            if content:
                print(f"✓ 成功加载文本文件，内容长度: {len(content)} 字符")
                print(f"  前100字符预览: {content[:100]}...")
                return True
            else:
                print("✗ 文本文件内容为空")
                return False
        else:
            print(f"✗ 文本文件不存在: {text_file}")
            return False
            
    except Exception as e:
        print(f"✗ 文本文件加载测试失败: {e}")
        return False


def test_directory_structure():
    """测试目录结构"""
    print("\n" + "=" * 50)
    print("测试目录结构")
    print("=" * 50)
    
    required_dirs = [
        "../../resources/models",
        "../../resources/models/zh_CN",
        "../../resources/models/zh_CN/medium",
        "../../resources/test_text",
        "../config",
        "../../resources/audio_data"
    ]
    
    all_good = True
    
    for dir_path in required_dirs:
        if os.path.exists(dir_path):
            print(f"✓ 目录存在: {dir_path}")
        else:
            print(f"✗ 目录不存在: {dir_path}")
            all_good = False
    
    # 检查模型文件
    model_files = [
        "../../resources/models/zh_CN/medium/zh_CN-huayan-medium.onnx",
        "../../resources/models/zh_CN/medium/zh_CN-huayan-medium.onnx.json"
    ]
    
    for file_path in model_files:
        if os.path.exists(file_path):
            print(f"✓ 模型文件存在: {file_path}")
        else:
            print(f"✗ 模型文件不存在: {file_path}")
            all_good = False
    
    return all_good


def main():
    """主函数"""
    print("Piper Speech 快速测试")
    print("=" * 50)
    
    tests = [
        ("目录结构", test_directory_structure),
        ("模型加载", test_model_loading),
        ("文本文件加载", test_text_file_loading)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"✗ {test_name} 测试异常: {e}")
            results.append((test_name, False))
    
    print("\n" + "=" * 50)
    print("测试结果汇总")
    print("=" * 50)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\n总测试: {total} 个")
    print(f"通过: {passed} 个")
    print(f"失败: {total - passed} 个")
    
    if passed == total:
        print("\n🎉 所有测试通过！系统可以正常使用。")
        print("\n使用方法:")
        print("1. 交互式测试: python run_test.py")
        print("2. 文件测试: python run_test.py file")
    else:
        print("\n❌ 部分测试失败，请检查上述错误信息。")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)