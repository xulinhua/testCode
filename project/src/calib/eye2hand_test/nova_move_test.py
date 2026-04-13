#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
越疆 Nova2 机械臂数据采集脚本
基于 Nova2 机械臂控制接口实现
支持固件版本: 3.5.8.1 及以上
"""

import sys
import os
import argparse
import logging
import traceback

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入数据采集类
from nova_data_collector import NovaDataCollector, generate_data_points

# ========== 配置日志 ==========
logging.basicConfig(
    level=logging.DEBUG,  # 使用DEBUG级别显示详细信息
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="越疆 Nova2 机械臂 数据采集脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  1. 执行数据采集（禁用可视化）：
     python3 nova_move_test.py --ip 192.168.5.1 --no-visualize
     
  2. 执行数据采集（不保存图像）：
     python3 nova_move_test.py --ip 192.168.5.1 --no-save-images
        """
    )
    
    parser.add_argument("--ip", default="192.168.5.1", help="机械臂IP地址")
    parser.add_argument("--no-visualize", action="store_true", help="禁用可视化窗口")
    parser.add_argument("--no-save-images", action="store_true", help="不保存源图像")
    
    args = parser.parse_args()
    
    # 确定是否保存图像
    save_images = not args.no_save_images
    
    # 默认启用可视化，除非指定了--no-visualize参数
    visualize = not args.no_visualize
    
    collector = NovaDataCollector(
        robot_ip=args.ip, 
        visualize=visualize, 
        save_source_images=save_images
    )
    
    try:
        collector.collect_data()
    except KeyboardInterrupt:
        print("\n\n⚠️  程序被用户中断")
    except Exception as e:
        print(f"\n\n❌ 错误: {e}")
        traceback.print_exc()
    finally:
        # 释放资源
        try:
            if collector.pipeline is not None:
                collector.pipeline.stop()
        except:
            pass
        
        try:
            import cv2
            cv2.destroyAllWindows()
        except:
            pass
        
        collector.robot.disconnect()
        print("已释放所有资源。")


if __name__ == "__main__":
    main()