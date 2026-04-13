#!/usr/bin/env python3
"""
测试模型加载功能
"""

import sys
import os

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'asr_core'))

from asr_core.model_manager import ModelManager

def test_model_loading():
    """测试模型加载"""
    print("测试模型加载...")
    
    # 创建模型管理器实例
    model_manager = ModelManager()
    
    print(f"模型根目录: {model_manager.models_root_path}")
    
    # 获取默认模型信息
    model_info = model_manager.get_model_info()
    
    if model_info:
        print(f"模型名称: {model_info['name']}")
        print(f"模型路径: {model_info['local_path']}")
        print(f"模型类型: {model_info['type']}")
        
        # 验证模型路径是否存在
        if os.path.exists(model_info['local_path']):
            print("✅ 模型路径存在")
            
            # 列出模型目录中的文件
            print("模型目录中的文件:")
            for item in os.listdir(model_info['local_path']):
                print(f"  - {item}")
                
            return True
        else:
            print("❌ 模型路径不存在")
            return False
    else:
        print("❌ 无法获取模型信息")
        return False

if __name__ == "__main__":
    success = test_model_loading()
    if success:
        print("\n✅ 模型加载测试通过!")
        sys.exit(0)
    else:
        print("\n❌ 模型加载测试失败!")
        sys.exit(1)