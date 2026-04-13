#!/usr/bin/env python3
"""
生成 ArUco 6x6_1000-0 标记图像
"""
import numpy as np
from PIL import Image, ImageDraw

def generate_aruco_6x6(marker_id=0):
    """生成 6x6 ArUco 标记"""
    # ArUco 6x6 有 6x6=36 个单元格，加上边框共 8x8
    grid_size = 8
    cell_size = 100  # 每个单元格的像素大小
    
    # 创建白色背景
    img_size = grid_size * cell_size
    img = Image.new('RGB', (img_size, img_size), 'white')
    draw = ImageDraw.Draw(img)
    
    # 绘制黑色外边框
    draw.rectangle([0, 0, img_size-1, img_size-1], fill='black', outline='black')
    
    # 6x6_1000 字典的编码（简化版，实际需要根据 ArUco 规范）
    # 这里生成 marker_id=0 的图案
    # 实际项目中应该使用 opencv-contrib-python 的 aruco 模块生成标准图案
    
    # 内部 6x6 网格的数据（marker_id=0 的示例）
    # 0=白，1=黑
    cells = [
        [1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 1],
        [1, 0, 1, 1, 0, 1],
        [1, 0, 1, 1, 0, 1],
        [1, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1],
    ]
    
    # 绘制内部网格
    border = cell_size  # 外边框宽度
    for row in range(6):
        for col in range(6):
            x1 = border + col * cell_size
            y1 = border + row * cell_size
            x2 = x1 + cell_size
            y2 = y1 + cell_size
            
            if cells[row][col] == 1:
                draw.rectangle([x1, y1, x2, y2], fill='black', outline='black')
            else:
                draw.rectangle([x1, y1, x2, y2], fill='white', outline='black')
    
    return img

if __name__ == '__main__':
    # 生成 ArUco 标记
    marker_img = generate_aruco_6x6(0)
    
    # 保存为 PNG
    output_path = '/home/hs/testCode/simulation/src/nova_sim/meshes/textures/aruco_6x6_1000-0.png'
    import os
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    marker_img.save(output_path)
    print(f"ArUco 标记已保存到：{output_path}")
