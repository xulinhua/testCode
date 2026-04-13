#!/usr/bin/env python3
"""
音频工具模块
包含音频数据转换、结果验证、重采样等工具函数
"""

import numpy as np


def is_valid_result(result):
    """
    检查ASR结果是否有效
    
    Args:
        result (str): ASR识别结果
        
    Returns:
        bool: 结果是否有效
    """
    # 检查结果是否为空
    if not result:
        return False
        
    # 如果结果是字典格式，提取text字段进行验证
    if isinstance(result, dict):
        if 'text' in result:
            text = result['text']
            # 检查text字段是否为空或只包含空白字符
            if not text or not text.strip():
                return False
            # 检查文本内容是否有效
            return _is_text_valid(text)
        else:
            # 如果字典中没有text字段，检查整个字典的字符串表示
            result_str = str(result)
            return _is_text_valid(result_str)
    
    # 检查结果是否为字符串
    if not isinstance(result, str):
        return False
        
    # 检查结果是否只包含空白字符
    if not result.strip():
        return False
        
    # 检查文本内容是否有效
    return _is_text_valid(result)


def _is_text_valid(text):
    """
    检查文本内容是否有效
    
    Args:
        text (str): 待检查的文本
        
    Returns:
        bool: 文本是否有效
    """
    # 检查文本是否只包含空白字符
    if not text.strip():
        return False
        
    # 检查文本是否包含有效字符（字母、数字、中文字符等）
    valid_chars = ''.join(c for c in text if c.isalnum() or c.isspace() or '\u4e00' <= c <= '\u9fff')
    if not valid_chars.strip():
        return False
        
    # 检查结果长度是否合理（避免过短的结果）
    if len(text.strip()) < 1:
        return False
        
    return True