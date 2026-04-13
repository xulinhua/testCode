"""
命令行界面工具
=============

提供命令行接口，方便用户快速使用语音合成功能。
"""

import argparse
import os
import sys
import json
from typing import List, Optional

# 添加模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from piper_speech import TTSEngine, ConfigManager, AudioGenerator


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description="Piper语音合成命令行工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 基础使用
  python tts_cli.py -t "你好，世界" -o output.wav
  
  # 批量处理
  python tts_cli.py -f input.txt -o output_dir/
  
  # 自定义配置
  python tts_cli.py -t "测试文本" -c config/jetson.yaml -m models/voice.onnx
        """
    )
    
    # 输入参数
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('-t', '--text', help='输入文本')
    input_group.add_argument('-f', '--file', help='包含文本的文件路径')
    
    # 输出参数
    parser.add_argument('-o', '--output', required=True, help='输出文件或目录路径')
    
    # 配置参数
    parser.add_argument('-c', '--config', default='config/default.yaml', 
                       help='配置文件路径')
    parser.add_argument('-m', '--model', help='模型文件路径')
    parser.add_argument('-p', '--platform', default='jetson', 
                       help='平台配置 (jetson, default)')
    
    # 音频参数
    parser.add_argument('--sample-rate', type=int, help='采样率')
    parser.add_argument('--speaker-id', type=int, help='说话人ID')
    
    # 其他参数
    parser.add_argument('-v', '--verbose', action='store_true', 
                       help='详细输出')
    parser.add_argument('--batch-size', type=int, default=1, 
                       help='批处理大小')
    
    args = parser.parse_args()
    
    # 创建输出目录
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    
    # 运行命令行工具
    try:
        if args.text:
            synthesize_single_text(args)
        elif args.file:
            synthesize_batch_texts(args)
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)


def synthesize_single_text(args):
    """合成单个文本"""
    print(f"开始合成文本: {args.text}")
    
    # 初始化组件
    config_manager, tts_engine, audio_generator = initialize_components(args)
    
    # 合成语音
    audio_data = tts_engine.synthesize(args.text, args.speaker_id)
    
    if audio_data is not None:
        # 保存音频
        if audio_generator.save_wav(audio_data, args.output):
            print(f"音频已保存到: {args.output}")
            
            # 显示音频信息
            if args.verbose:
                audio_info = audio_generator.get_audio_info(audio_data)
                print(f"音频信息: {json.dumps(audio_info, indent=2, ensure_ascii=False)}")
        else:
            print("音频保存失败")
    else:
        print("语音合成失败")
    
    # 清理资源
    tts_engine.close()


def synthesize_batch_texts(args):
    """批量合成文本"""
    print(f"开始批量处理文件: {args.file}")
    
    # 读取文本文件
    texts = read_text_file(args.file)
    if not texts:
        print("文件为空或读取失败")
        return
    
    print(f"找到 {len(texts)} 个文本")
    
    # 初始化组件
    config_manager, tts_engine, audio_generator = initialize_components(args)
    
    # 批量处理
    audio_data_list = []
    for i, text in enumerate(texts):
        if args.verbose:
            print(f"处理第 {i+1}/{len(texts)} 个文本: {text[:50]}...")
        
        audio_data = tts_engine.synthesize(text, args.speaker_id)
        if audio_data is not None:
            audio_data_list.append(audio_data)
        else:
            print(f"第 {i+1} 个文本合成失败")
    
    # 批量保存
    if audio_data_list:
        # 创建输出目录
        if not os.path.exists(args.output):
            os.makedirs(args.output, exist_ok=True)
        
        # 生成文件名模式
        file_pattern = os.path.join(args.output, "batch_{index:03d}.wav")
        
        saved_files = audio_generator.batch_save(audio_data_list, file_pattern)
        print(f"批量保存完成: {len(saved_files)}/{len(texts)} 个文件")
        
        # 显示文件列表
        if args.verbose:
            print("生成的文件:")
            for file_path in saved_files:
                print(f"  {file_path}")
    else:
        print("没有成功合成的音频")
    
    # 清理资源
    tts_engine.close()


def initialize_components(args):
    """初始化组件"""
    # 配置管理器
    config_manager = ConfigManager("config")
    
    # 加载配置
    if os.path.exists(args.config):
        config_manager.load_config(args.config)
    else:
        print(f"警告: 配置文件不存在: {args.config}")
    
    # 加载平台配置
    config_manager.load_platform_config(args.platform)
    
    # 覆盖配置
    if args.model:
        config_manager.set("model.path", args.model)
    if args.sample_rate:
        config_manager.set("audio.sample_rate", args.sample_rate)
    
    # 验证配置
    if not config_manager.validate_config():
        raise ValueError("配置验证失败")
    
    # 语音合成引擎
    tts_engine = TTSEngine()
    
    # 加载模型
    model_path = config_manager.get("model.path")
    config_path = config_manager.get("model.config_path")
    
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"模型文件不存在: {model_path}")
    
    if not tts_engine.load_model(model_path, config_path):
        raise RuntimeError("模型加载失败")
    
    # 音频生成器
    sample_rate = config_manager.get("audio.sample_rate")
    audio_generator = AudioGenerator(sample_rate)
    
    return config_manager, tts_engine, audio_generator


def read_text_file(file_path: str) -> List[str]:
    """读取文本文件"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # 过滤空行和空白行
        texts = [line.strip() for line in lines if line.strip()]
        return texts
    except Exception as e:
        print(f"读取文件失败: {e}")
        return []


if __name__ == "__main__":
    main()