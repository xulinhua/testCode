#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式语音合成测试模块
====================

提供两种测试方式：
1. 终端交互模式：用户可输入文本进行实时合成
2. 文件测试模式：从指定文本文件加载内容进行合成

作者: Piper Speech Team
版本: 1.0.0
"""

import os
import sys
import argparse
import logging
from pathlib import Path

# 添加模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from piper_py import TTSEngine, ConfigManager, AudioGenerator
from piper_py.voice_model import VoiceModelManager


# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class InteractiveTester:
    """
    交互式语音合成测试器
    
    提供多种测试方式，支持实时交互和批量处理
    """
    
    def __init__(self, models_dir: str = "../../resources/models"):
        """
        初始化测试器
        
        参数:
            models_dir (str): 模型文件目录
        """
        self.models_dir = models_dir
        
        # 初始化组件
        self.config_manager = ConfigManager()
        self.model_manager = VoiceModelManager(models_dir)
        self.audio_generator = AudioGenerator()
        self.tts_engine = TTSEngine()
        
        # 设置默认配置
        self._setup_default_config()
        
        # 设置默认模型
        self._setup_default_model()
    
    def _setup_default_config(self):
        """设置默认配置"""
        # 重置为默认配置
        self.config_manager.reset_to_default()
        
        # 设置Jetson优化配置
        if self._is_jetson_platform():
            logger.info("检测到Jetson平台，启用优化配置")
            self.config_manager.load_platform_config("jetson")
    
    def _setup_default_model(self):
        """设置默认模型"""
        # 优先设置中文中等质量模型
        if not self.model_manager.set_zh_cn_medium_model():
            # 如果设置失败，尝试设置首选模型
            preferred_model = self.model_manager.get_preferred_model()
            if preferred_model:
                self.model_manager.set_current_model(preferred_model['name'])
            else:
                logger.warning("未找到可用模型，请检查模型目录")
    
    def _is_jetson_platform(self) -> bool:
        """检查是否为Jetson平台"""
        try:
            import platform
            # 检查机器架构
            machine = platform.machine()
            return 'aarch64' in machine or 'arm64' in machine
        except:
            return False
    
    def load_text_from_file(self, file_path: str) -> str:
        """
        从文件加载文本内容
        
        参数:
            file_path (str): 文本文件路径
            
        返回:
            str: 加载的文本内容
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            logger.info(f"从文件 {file_path} 加载文本成功，长度: {len(content)} 字符")
            return content
        except Exception as e:
            logger.error(f"加载文本文件失败: {e}")
            return ""
    
    def test_from_file(self, text_file: str, output_dir: str = "../../resources/audio_data"):
        """
        从文本文件进行语音合成测试
        
        参数:
            text_file (str): 文本文件路径
            output_dir (str): 输出音频目录
        """
        # 检查文件是否存在
        if not os.path.exists(text_file):
            logger.error(f"文本文件不存在: {text_file}")
            return
        
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        
        # 加载文本内容
        text_content = self.load_text_from_file(text_file)
        if not text_content:
            logger.error("文本内容为空")
            return
        
        # 按段落分割文本
        paragraphs = [p.strip() for p in text_content.split('\n\n') if p.strip()]
        
        logger.info(f"开始处理 {len(paragraphs)} 个段落")
        
        # 初始化TTS引擎
        if not self._initialize_tts_engine():
            return
        
        # 处理每个段落
        for i, paragraph in enumerate(paragraphs, 1):
            if not paragraph.strip():
                continue
                
            logger.info(f"处理第 {i} 个段落: {paragraph[:50]}...")
            
            # 合成语音
            audio_data = self.tts_engine.synthesize(paragraph)
            if audio_data is None:
                logger.error(f"第 {i} 个段落合成失败")
                continue
            
            # 保存音频文件
            output_file = os.path.join(output_dir, f"paragraph_{i}.wav")
            success = self.audio_generator.save_wav(audio_data, output_file)
            
            if success:
                logger.info(f"段落 {i} 合成成功，保存到: {output_file}")
            else:
                logger.error(f"段落 {i} 保存失败")
        
        logger.info("文件测试完成")
    
    def interactive_mode(self):
        """
        交互式测试模式
        
        允许用户输入文本进行实时合成
        """
        logger.info("进入交互式测试模式")
        logger.info("输入 'quit' 或 'exit' 退出")
        logger.info("输入 'models' 查看可用模型")
        logger.info("输入 'config' 查看当前配置")
        
        # 初始化TTS引擎
        if not self._initialize_tts_engine():
            logger.error("TTS引擎初始化失败，无法进入交互模式")
            return
        
        while True:
            try:
                # 获取用户输入
                user_input = input("\n请输入要合成的文本: ").strip()
                
                if user_input.lower() in ['quit', 'exit', '退出']:
                    logger.info("退出交互模式")
                    break
                
                elif user_input.lower() == 'models':
                    self._show_available_models()
                    continue
                
                elif user_input.lower() == 'config':
                    self._show_current_config()
                    continue
                
                elif not user_input:
                    logger.warning("输入为空，请重新输入")
                    continue
                
                # 合成语音
                logger.info("开始合成语音...")
                audio_data = self.tts_engine.synthesize(user_input)
                
                if audio_data is None:
                    logger.error("语音合成失败")
                    continue
                
                # 保存音频文件
                output_dir = "../../resources/audio_data"
                os.makedirs(output_dir, exist_ok=True)
                
                import time
                timestamp = int(time.time())
                output_file = os.path.join(output_dir, f"interactive_{timestamp}.wav")
                
                success = self.audio_generator.save_wav(audio_data, output_file)
                
                if success:
                    logger.info(f"合成成功！音频已保存到: {output_file}")
                    
                    # 显示音频信息
                    audio_info = self.audio_generator.get_audio_info(audio_data)
                    print(f"音频信息: {audio_info['duration_seconds']:.2f}秒, {audio_info['sample_count']}个采样点")
                else:
                    logger.error("音频保存失败")
                
            except KeyboardInterrupt:
                logger.info("\n用户中断，退出交互模式")
                break
            except Exception as e:
                logger.error(f"处理输入时发生错误: {e}")
    
    def _initialize_tts_engine(self) -> bool:
        """
        初始化TTS引擎
        
        返回:
            bool: 初始化是否成功
        """
        try:
            # 获取当前模型信息
            model_info = self.model_manager.get_current_model_info()
            if not model_info:
                logger.error("未设置当前模型")
                return False
            
            # 加载模型
            logger.info(f"加载模型: {model_info['name']}")
            success = self.tts_engine.load_model(
                model_path=model_info['path'],
                config_path=model_info['config_path']
            )
            
            if success:
                logger.info("TTS引擎初始化成功")
            else:
                logger.error("TTS引擎初始化失败")
            
            return success
            
        except Exception as e:
            logger.error(f"初始化TTS引擎失败: {e}")
            return False
    
    def _show_available_models(self):
        """显示可用模型列表"""
        models = self.model_manager.get_model_list()
        
        if not models:
            print("未找到可用模型")
            return
        
        print("\n可用模型列表:")
        print("-" * 60)
        
        for i, model in enumerate(models, 1):
            current_marker = " [当前]" if model['name'] == self.model_manager.current_model else ""
            print(f"{i}. {model['name']}{current_marker}")
            print(f"   语言: {model.get('language', '未知')}, "
                  f"质量: {model.get('quality', '未知')}, "
                  f"采样率: {model['sample_rate']}Hz")
        
        print("-" * 60)
        
        # 提供切换模型选项
        try:
            choice = input("输入模型编号切换模型 (按Enter继续): ").strip()
            if choice:
                index = int(choice) - 1
                if 0 <= index < len(models):
                    selected_model = models[index]
                    if self.model_manager.set_current_model(selected_model['name']):
                        print(f"已切换到模型: {selected_model['name']}")
                        # 重新初始化TTS引擎
                        self._initialize_tts_engine()
                    else:
                        print("切换模型失败")
                else:
                    print("无效的模型编号")
        except ValueError:
            print("请输入有效的数字")
    
    def _show_current_config(self):
        """显示当前配置"""
        print("\n当前配置:")
        print("-" * 40)
        
        # 显示基本配置
        sample_rate = self.config_manager.get("audio.sample_rate")
        max_text_length = self.config_manager.get("synthesis.max_text_length")
        
        print(f"采样率: {sample_rate}Hz")
        print(f"最大文本长度: {max_text_length}字符")
        
        # 显示当前模型信息
        model_info = self.model_manager.get_current_model_info()
        if model_info:
            print(f"当前模型: {model_info['name']}")
            print(f"模型语言: {model_info.get('language', '未知')}")
            print(f"模型质量: {model_info.get('quality', '未知')}")
        
        print("-" * 40)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Piper语音合成交互测试工具')
    parser.add_argument('--mode', choices=['file', 'interactive'], 
                       default='interactive', help='测试模式 (默认: interactive)')
    parser.add_argument('--text-file', type=str, help='文本文件路径 (文件模式使用)')
    parser.add_argument('--output-dir', type=str, default='../../resources/audio_data', 
                       help='输出音频目录 (默认: ../../resources/audio_data)')
    parser.add_argument('--models-dir', type=str, default='../../resources/models',
                       help='模型文件目录 (默认: ../../resources/models)')
    
    args = parser.parse_args()
    
    # 创建测试器
    tester = InteractiveTester(models_dir=args.models_dir)
    
    if args.mode == 'file':
        if not args.text_file:
            logger.error("文件模式需要指定 --text-file 参数")
            return
        
        tester.test_from_file(args.text_file, args.output_dir)
    
    else:  # interactive mode
        tester.interactive_mode()


if __name__ == "__main__":
    main()