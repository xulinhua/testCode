"""
演示脚本
========

演示如何使用piper_speech库进行语音合成。
"""

import os
import sys
import time
from typing import List

# 添加模块路径
sys.path.append(os.path.dirname(__file__))

from piper_py import TTSEngine, ConfigManager, AudioGenerator, VoiceModelManager
from test_text_reader import read_test_texts


def main():
    """主演示函数"""
    print("=" * 60)
    print("Piper语音合成演示")
    print("=" * 60)
    
    # 初始化配置管理器
    print("\n1. 初始化配置管理器...")
    config_manager = ConfigManager("config")
    
    # 加载默认配置
    config_manager.load_config("default.yaml")
    print("   ✓ 默认配置加载完成")
    
    # 加载Jetson平台配置
    config_manager.load_platform_config("jetson")
    print("   ✓ Jetson平台配置加载完成")
    
    # 验证配置
    if config_manager.validate_config():
        print("   ✓ 配置验证通过")
    else:
        print("   ✗ 配置验证失败")
        return
    
    # 创建语音合成引擎
    print("\n2. 初始化语音合成引擎...")
    tts_engine = TTSEngine()
    
    # 加载模型
    model_path = config_manager.get("model.default_model")
    config_path = config_manager.get("model.config_path")
    
    print(f"   模型路径: {model_path}")
    print(f"   配置路径: {config_path}")
    
    # 检查模型文件是否存在
    if not os.path.exists(model_path):
        print("   ⚠ 模型文件不存在，请确保已下载Piper语音模型")
        print("   您可以从以下链接下载模型:")
        print("   https://github.com/rhasspy/piper/releases")
        print("   或使用我们提供的测试模型")
        
        # 创建测试音频数据
        print("\n3. 创建测试音频数据...")
        audio_generator = AudioGenerator(sample_rate=22050)
        
        # 生成测试音频
        duration = 2.0
        test_audio = audio_generator.generate_silence(duration)
        
        # 保存测试音频
        output_dir = config_manager.get("audio.output_dir", "../resources/audio_data")
        output_path = os.path.join(output_dir, "test_demo.wav")
        if audio_generator.save_wav(test_audio, output_path):
            print(f"   ✓ 测试音频已保存到: {output_path}")
        
        return
    
    # 加载模型
    print("\n3. 加载语音模型...")
    start_time = time.time()
    
    if tts_engine.load_model(model_path, config_path):
        load_time = time.time() - start_time
        print(f"   ✓ 模型加载完成 (耗时: {load_time:.2f}秒)")
        
        # 显示模型信息
        print(f"   采样率: {tts_engine.sample_rate}Hz")
    else:
        print("   ✗ 模型加载失败")
        return
    
    # 创建音频生成器
    print("\n4. 初始化音频生成器...")
    audio_generator = AudioGenerator(sample_rate=tts_engine.sample_rate)
    print("   ✓ 音频生成器初始化完成")
    
    # 从resources/test_text目录读取测试文本
    test_texts = read_test_texts()
    
    # 如果没有读取到测试文本，使用默认文本
    if not test_texts:
        test_texts = [
            "你好，欢迎使用Piper语音合成系统！",
            "这是一个语音合成演示。",
            "系统运行正常，语音合成功能已准备就绪。",
            "感谢您的使用！"
        ]
    
    print("\n5. 开始语音合成演示...")
    
    audio_data_list = []
    
    for i, text in enumerate(test_texts):
        print(f"\n   第 {i+1}/{len(test_texts)} 句: {text}")
        
        # 合成语音
        start_time = time.time()
        audio_data = tts_engine.synthesize(text)
        synthesis_time = time.time() - start_time
        
        if audio_data is not None:
            print(f"   ✓ 合成完成 (耗时: {synthesis_time:.2f}秒)")
            
            # 获取音频信息
            audio_info = audio_generator.get_audio_info(audio_data)
            print(f"   音频时长: {audio_info['duration_seconds']:.2f}秒")
            print(f"   采样点数: {audio_info['sample_count']}")
            
            audio_data_list.append(audio_data)
        else:
            print("   ✗ 合成失败")
    
    # 保存音频文件
    print("\n6. 保存音频文件...")
    
    if audio_data_list:
        # 从配置文件获取输出目录
        output_dir = config_manager.get("audio.output_dir", "../resources/audio_data")
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # 保存单个文件
        for i, audio_data in enumerate(audio_data_list):
            output_path = os.path.join(output_dir, f"demo_{i+1}.wav")
            if audio_generator.save_wav(audio_data, output_path):
                print(f"   ✓ 已保存: {output_path}")
        
        # 合并所有音频
        print("\n7. 合并音频文件...")
        combined_audio = audio_generator.concatenate_audio(audio_data_list)
        
        combined_path = os.path.join(output_dir, "demo_combined.wav")
        if audio_generator.save_wav(combined_audio, combined_path):
            print(f"   ✓ 合并音频已保存: {combined_path}")
            
            # 获取合并音频信息
            combined_info = audio_generator.get_audio_info(combined_audio)
            print(f"   总时长: {combined_info['duration_seconds']:.2f}秒")
    
    # 关闭引擎
    print("\n8. 清理资源...")
    tts_engine.close()
    print("   ✓ 引擎已关闭")
    
    print("\n" + "=" * 60)
    print("演示完成！")
    print("所有音频文件已保存到 resources/audio_data/ 目录")
    print("=" * 60)


def performance_test():
    """性能测试函数"""
    print("\n" + "=" * 60)
    print("性能测试")
    print("=" * 60)
    
    # 初始化组件
    config_manager = ConfigManager("config")
    config_manager.load_config("default.yaml")
    config_manager.load_platform_config("jetson")
    
    tts_engine = TTSEngine()
    
    model_path = config_manager.get("model.default_model")
    if not os.path.exists(model_path):
        print("模型文件不存在，跳过性能测试")
        return
    
    # 加载模型
    if not tts_engine.load_model(model_path):
        print("模型加载失败，跳过性能测试")
        return
    
    # 测试文本
    test_text = "这是一个性能测试句子。"
    
    # 预热
    print("\n预热...")
    for _ in range(3):
        tts_engine.synthesize(test_text)
    
    # 性能测试
    print("\n开始性能测试...")
    num_tests = 10
    times = []
    
    for i in range(num_tests):
        start_time = time.time()
        audio_data = tts_engine.synthesize(test_text)
        end_time = time.time()
        
        if audio_data is not None:
            times.append(end_time - start_time)
            print(f"测试 {i+1}/{num_tests}: {times[-1]:.3f}秒")
        else:
            print(f"测试 {i+1}/{num_tests}: 失败")
    
    if times:
        avg_time = sum(times) / len(times)
        min_time = min(times)
        max_time = max(times)
        
        print(f"\n性能统计:")
        print(f"  平均耗时: {avg_time:.3f}秒")
        print(f"  最短耗时: {min_time:.3f}秒")
        print(f"  最长耗时: {max_time:.3f}秒")
        print(f"  测试次数: {len(times)}")
    
    # 清理
    tts_engine.close()


if __name__ == "__main__":
    # 运行主演示
    main()
    
    # 询问是否运行性能测试
    try:
        response = input("\n是否运行性能测试？(y/n): ").lower().strip()
        if response == 'y' or response == 'yes':
            performance_test()
    except:
        pass  # 忽略输入错误
    
    print("\n演示结束！")