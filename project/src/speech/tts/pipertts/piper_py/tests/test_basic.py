"""
基础测试模块
===========

测试piper_speech库的基本功能。
"""

import os
import sys
import unittest
import tempfile
import numpy as np

# 添加模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from piper_py import TTSEngine, ConfigManager, AudioGenerator


class TestConfigManager(unittest.TestCase):
    """配置管理器测试类"""
    
    def setUp(self):
        """测试前准备"""
        self.config_manager = ConfigManager()
        
        # 创建临时目录
        self.temp_dir = tempfile.mkdtemp()
        self.config_dir = os.path.join(self.temp_dir, "config")
        os.makedirs(self.config_dir, exist_ok=True)
        
        # 创建测试配置文件
        self.test_config = {
            "audio": {
                "sample_rate": 22050,
                "channels": 1
            },
            "model": {
                "path": "test_model.onnx",
                "config_path": "test_model.json"
            }
        }
        
        config_path = os.path.join(self.config_dir, "test.yaml")
        import yaml
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.test_config, f)
    
    def test_load_config(self):
        """测试配置加载"""
        config_manager = ConfigManager(self.config_dir)
        success = config_manager.load_config("test.yaml")
        
        self.assertTrue(success)
        self.assertEqual(config_manager.get("audio.sample_rate"), 22050)
    
    def test_get_set_config(self):
        """测试配置获取和设置"""
        config_manager = ConfigManager()
        
        # 测试设置配置
        success = config_manager.set("audio.sample_rate", 16000)
        self.assertTrue(success)
        
        # 测试获取配置
        value = config_manager.get("audio.sample_rate")
        self.assertEqual(value, 16000)
    
    def test_validate_config(self):
        """测试配置验证"""
        config_manager = ConfigManager()
        
        # 设置必要配置
        config_manager.set("tts.sample_rate", 22050)
        config_manager.set("model.default_model", "test.onnx")
        config_manager.set("tts.noise_scale", 0.667)
        
        # 验证配置
        valid = config_manager.validate_config()
        self.assertTrue(valid)
    
    def tearDown(self):
        """测试后清理"""
        import shutil
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)


class TestAudioGenerator(unittest.TestCase):
    """音频生成器测试类"""
    
    def setUp(self):
        """测试前准备"""
        self.audio_generator = AudioGenerator(sample_rate=22050)
        self.temp_dir = tempfile.mkdtemp()
    
    def test_save_wav(self):
        """测试WAV文件保存"""
        # 创建测试音频数据
        duration = 1.0  # 1秒
        num_samples = int(duration * 22050)
        test_audio = np.sin(2 * np.pi * 440 * np.arange(num_samples) / 22050)  # 440Hz正弦波
        test_audio = (test_audio * 32767).astype(np.int16)
        
        # 保存音频
        output_path = os.path.join(self.temp_dir, "test.wav")
        success = self.audio_generator.save_wav(test_audio, output_path)
        
        self.assertTrue(success)
        self.assertTrue(os.path.exists(output_path))
    
    def test_generate_silence(self):
        """测试静音生成"""
        duration = 2.0  # 2秒
        silence = self.audio_generator.generate_silence(duration)
        
        self.assertEqual(len(silence), int(duration * 22050))
        self.assertTrue(np.all(silence == 0))
    
    def test_concatenate_audio(self):
        """测试音频连接"""
        # 创建两个音频片段
        audio1 = np.array([1, 2, 3], dtype=np.int16)
        audio2 = np.array([4, 5, 6], dtype=np.int16)
        
        # 连接音频
        concatenated = self.audio_generator.concatenate_audio([audio1, audio2])
        
        expected = np.array([1, 2, 3, 4, 5, 6], dtype=np.int16)
        np.testing.assert_array_equal(concatenated, expected)
    
    def test_get_audio_info(self):
        """测试音频信息获取"""
        # 创建测试音频
        test_audio = np.array([1000, -1000, 2000, -2000], dtype=np.int16)
        
        # 获取音频信息
        info = self.audio_generator.get_audio_info(test_audio)
        
        self.assertIn("duration_seconds", info)
        self.assertIn("sample_count", info)
        self.assertEqual(info["sample_count"], 4)
    
    def tearDown(self):
        """测试后清理"""
        import shutil
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)


class TestTTSEngine(unittest.TestCase):
    """语音合成引擎测试类"""
    
    def setUp(self):
        """测试前准备"""
        self.tts_engine = TTSEngine()
    
    def test_text_to_phonemes(self):
        """测试文本转音素"""
        text = "hello world"
        phonemes = self.tts_engine.text_to_phonemes(text)
        
        self.assertIsInstance(phonemes, list)
        self.assertTrue(len(phonemes) > 0)
    
    def test_simple_text_to_phonemes(self):
        """测试简化的文本转音素"""
        text = "hello"
        phonemes = self.tts_engine.text_to_phonemes(text)
        
        self.assertIsInstance(phonemes, list)
        self.assertTrue(all(isinstance(p, str) for p in phonemes))
    
    def test_engine_initialization(self):
        """测试引擎初始化"""
        self.assertIsNotNone(self.tts_engine.session_options)
        self.assertIsNotNone(self.tts_engine.providers)
        self.assertFalse(self.tts_engine.is_loaded)
    
    def test_close_engine(self):
        """测试引擎关闭"""
        self.tts_engine.close()
        self.assertFalse(self.tts_engine.is_loaded)
    
    def tearDown(self):
        """测试后清理"""
        self.tts_engine.close()


def run_tests():
    """运行所有测试"""
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # 添加测试类
    suite.addTests(loader.loadTestsFromTestCase(TestConfigManager))
    suite.addTests(loader.loadTestsFromTestCase(TestAudioGenerator))
    suite.addTests(loader.loadTestsFromTestCase(TestTTSEngine))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)