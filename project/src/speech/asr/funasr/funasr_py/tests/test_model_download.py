#!/usr/bin/env python3
"""
模型下载测试模块
"""

import sys
import os

# 添加项目根目录到Python路径
project_root = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.abspath(project_root))

# 确保能正确导入必要的库
try:
    import funasr
    import modelscope
    print("✅ 成功导入必要的库")
except ImportError as e:
    print(f"❌ 无法导入必要的库: {e}")
    print("请确保您已在当前Python环境中安装所需库:")
    print("   pip install funasr modelscope")
    sys.exit(1)

from asr_core.model_manager import ModelManager
from model_config import show_model_list, get_model_info

def test_model_download():
    """测试模型下载功能"""
    print("=== 模型下载测试 ===")
    
    # 创建模型管理器，使用新的资源路径
    model_manager = ModelManager("../resources/models")
    
    # 显示模型列表
    show_model_list()
    
    # 获取用户选择
    model_info = get_model_info()
    
    while True:
        choice = input(f"\n请选择要下载的模型编号 (默认为3 - {model_info['3']['name']}): ").strip()
        
        # 如果用户直接按回车，使用默认值3
        if choice == "":
            choice = "3"
            print(f"3")  # 输出选择的编号
        
        if choice in model_info:
            selected_model = model_info[choice]
            model_name = selected_model['name']
            modelscope_name = selected_model['modelscope_name']
            
            print(f"\n您选择的模型是: {model_name}")
            print(f"描述: {selected_model['description']}")
            print(f"大小: {selected_model['size']}， 参数量：{selected_model['parameters']}")
            print(f"ModelScope名称: {modelscope_name}")
            
            # 下载模型
            model_dir = model_manager.download_model(model_name, modelscope_name)
            if model_dir:
                print(f"✅ 模型 {model_name} 下载成功!")
                print(f"📁 模型保存在: {model_dir}")
                break
            else:
                retry = input("\n是否重新选择模型? (y/n, 默认为n): ").strip().lower()
                if retry not in ["y", "yes"]:
                    print("取消下载")
                    break
        else:
            print("❌ 无效的模型编号，请重新选择")
            retry = input("\n是否重新选择模型? (y/n, 默认为n): ").strip().lower()
            if retry not in ["y", "yes"]:
                print("取消下载")
                break

if __name__ == "__main__":
    test_model_download()