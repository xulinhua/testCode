#!/usr/bin/env python3
"""
简化 URDF 文件中的 collision 模型
将所有 mesh collision 替换为简单的 box 几何体
"""

import re
import sys

def simplify_collision(urdf_content):
    """将 URDF 中的 mesh collision 替换为 box"""
    
    # 匹配 collision 块，包含 mesh 的
    collision_pattern = r'(<collision>\s*<origin[^>]*>\s*<geometry>\s*)<mesh[^>]*>(\s*</geometry>\s*</collision>)'
    
    # 替换为使用 box 的 collision
    # 使用一个较小的默认尺寸，避免碰撞检测过于复杂
    replacement = r'\1<box size="0.1 0.1 0.1"/>\2'
    
    simplified = re.sub(collision_pattern, replacement, urdf_content, flags=re.DOTALL)
    
    return simplified

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 simplify_collision.py <urdf_file>")
        sys.exit(1)
    
    urdf_file = sys.argv[1]
    
    # 读取文件
    with open(urdf_file, 'r', encoding='utf-8') as f:
        original_content = f.read()
    
    # 简化 collision
    simplified_content = simplify_collision(original_content)
    
    # 写回文件
    with open(urdf_file, 'w', encoding='utf-8') as f:
        f.write(simplified_content)
    
    # 统计替换数量
    original_count = len(re.findall(r'<mesh', original_content))
    simplified_count = len(re.findall(r'<mesh', simplified_content))
    replaced_count = original_count - simplified_count
    
    print(f"✓ 成功简化 {replaced_count} 个 collision 模型")
    print(f"  原始 mesh 数量：{original_count}")
    print(f"  简化后 mesh 数量：{simplified_count} (仅保留 visual)")

if __name__ == '__main__':
    main()
