"""
测试文本读取模块
================

从resources/test_text目录读取测试文本文件。
"""

import os

def read_test_texts():
    """
    从resources/test_text目录读取所有测试文本文件
    
    返回:
        list: 测试文本列表
    """
    test_texts = []
    test_text_dir = os.path.join(os.path.dirname(__file__), "..", "..", "resources", "test_text")
    
    if not os.path.exists(test_text_dir):
        print(f"测试文本目录不存在: {test_text_dir}")
        return test_texts
    
    # 读取所有.txt文件
    for filename in sorted(os.listdir(test_text_dir)):
        if filename.endswith(".txt"):
            file_path = os.path.join(test_text_dir, filename)
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    content = f.read().strip()
                    if content:
                        test_texts.append(content)
            except Exception as e:
                print(f"读取测试文本文件失败 {file_path}: {e}")
    
    return test_texts

if __name__ == "__main__":
    texts = read_test_texts()
    for i, text in enumerate(texts):
        print(f"测试文本 {i+1}: {text[:50]}...")