#!/usr/bin/env python3
"""
模型导出测试模块
"""

import sys
import os

# 添加项目根目录到Python路径
project_root = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.abspath(project_root))

# 确保能正确导入funasr
try:
    import funasr
    print("✅ 成功导入funasr库")
except ImportError as e:
    print(f"❌ 无法导入funasr库: {e}")
    print("请确保您已在当前Python环境中安装funasr:")
    print("   pip install funasr")
    sys.exit(1)

from asr_core.model_manager import ModelManager
from model_config import show_model_list, get_model_info

def test_model_export():
    """测试模型导出功能"""
    print("=== 模型导出测试 ===")
    
    # 创建模型管理器，使用新的资源路径
    model_manager = ModelManager("../resources/models")
    
    # 显示模型列表
    show_model_list()
    
    # 获取用户选择
    model_info = get_model_info()
    
    while True:
        choice = input(f"\n请选择要导出的模型编号 (默认为3 - {model_info['3']['name']}): ").strip()
        
        # 如果用户直接按回车，使用默认值3
        if choice == "":
            choice = "3"
            print(f"3")  # 输出选择的编号
        
        # 加载模型
        selected_model = model_manager.load_model(choice, "pt")
        if selected_model:
            # 导出模型为ONNX格式
            model_manager.export_model_to_onnx(selected_model)
            break
        else:
            retry = input("\n是否重新选择模型? (y/n, 默认为n): ").strip().lower()
            if retry not in ["y", "yes"]:
                print("取消导出")
                break

if __name__ == "__main__":
    test_model_export()