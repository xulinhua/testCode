"""
语音合成引擎核心模块
==================

实现基于Piper的语音合成功能，包括文本处理、音素转换和音频生成。

主要功能：
1. 加载Piper语音模型
2. 文本转音素处理
3. ONNX推理执行
4. 音频数据生成
"""

import json
import logging
import wave
from typing import Dict, List, Optional, Union

import numpy as np
import onnxruntime as ort

# 尝试导入piper_phonemize，如果失败则使用替代方案
try:
    from piper_phonemize import phonemize_espeak
    PIPER_PHONEMIZE_AVAILABLE = True
except ImportError:
    PIPER_PHONEMIZE_AVAILABLE = False
    logging.warning("piper_phonemize模块未安装，将使用简化版本的音素转换")

# 配置日志
logger = logging.getLogger(__name__)

# 定义BOS/PAD/EOS标记（来自Piper官方实现）
BOS = "^"
EOS = "$"
PAD = "_"


def audio_float_to_int16(audio: np.ndarray) -> np.ndarray:
    """Convert audio from float32 to int16."""
    return (audio * 32767).astype(np.int16)


class TTSEngine:
    """
    基于官方Piper实现的语音合成引擎
    """
    
    def __init__(self):
        self.session = None
        self.model_config = {}
        self.sample_rate = 22050
        self.is_loaded = False
        
        # 初始化ONNX会话
        self._init_session()
    
    def _init_session(self) -> None:
        """
        初始化ONNX推理会话
        """
        # 获取可用的执行提供者
        available_providers = ort.get_available_providers()
        
        # 根据可用提供者动态设置优先级
        providers = []
        
        if 'TensorrtExecutionProvider' in available_providers:
            providers.append(('TensorrtExecutionProvider', {
                'trt_max_workspace_size': 1 << 30,
                'trt_fp16_enable': True,
            }))
        
        if 'CUDAExecutionProvider' in available_providers:
            providers.append(('CUDAExecutionProvider', {
                'device_id': 0,
                'arena_extend_strategy': 'kNextPowerOfTwo',
                'gpu_mem_limit': 2 * 1024 * 1024 * 1024,
                'cudnn_conv_algo_search': 'EXHAUSTIVE',
                'do_copy_in_default_stream': True,
            }))
        
        # 确保CPUExecutionProvider始终可用
        providers.append('CPUExecutionProvider')
        
        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        
        self.session_options = session_options
        self.providers = providers
        
        logger.info(f"可用的ONNX执行提供者: {available_providers}")
    
    def load_model(self, model_path: str, config_path: Optional[str] = None) -> bool:
        """
        加载Piper语音模型
        """
        try:
            if config_path is None:
                config_path = model_path.replace('.onnx', '.onnx.json')
            
            # 加载模型配置
            with open(config_path, 'r', encoding='utf-8') as f:
                self.model_config = json.load(f)
            
            # 设置采样率
            self.sample_rate = self.model_config.get('audio', {}).get('sample_rate', 22050)
            
            # 加载ONNX模型
            self.session = ort.InferenceSession(
                model_path, 
                sess_options=self.session_options,
                providers=self.providers
            )
            
            self.is_loaded = True
            logger.info(f"成功加载模型: {model_path}")
            logger.info(f"采样率: {self.sample_rate}Hz")
            
            return True
            
        except Exception as e:
            logger.error(f"加载模型失败: {e}")
            self.is_loaded = False
            return False
    
    def text_to_phonemes(self, text: str) -> List[str]:
        """
        将文本转换为音素序列
        
        基于官方Piper的eSpeak音素转换逻辑
        """
        try:
            # 使用piper-phonemize库进行音素转换
            # 对于中文，使用cmn（普通话）作为语音
            phonemes_list = phonemize_espeak(text, "cmn")
            
            # 将嵌套列表展平为单个列表
            phonemes = [phoneme for sentence_phonemes in phonemes_list for phoneme in sentence_phonemes]
            
            return phonemes
            
        except Exception as e:
            logger.error(f"音素转换失败: {e}")
            return []

    def phonemes_to_ids(self, phonemes: List[str]) -> List[int]:
        """
        音素序列转换为ID序列（基于官方Piper实现）
        """
        id_map = self.model_config.get('phoneme_id_map', {})
        
        # 开始标记 - BOS
        ids = list(id_map.get(BOS, [1]))
        
        # 音素序列
        for phoneme in phonemes:
            if phoneme in id_map:
                ids.extend(id_map[phoneme])
                # 每个音素后添加分隔符 - PAD
                ids.extend(id_map.get(PAD, [0]))
            else:
                logger.warning(f"未知音素: {phoneme}")
                # 使用空格作为默认音素
                ids.extend(id_map.get(' ', [3]))
        
        # 结束标记 - EOS
        ids.extend(id_map.get(EOS, [2]))
        
        return ids
    
    def synthesize(self, text: str, speaker_id: Optional[int] = None) -> Optional[np.ndarray]:
        """
        语音合成主函数（基于官方Piper实现）
        """
        if not self.is_loaded:
            logger.error("模型未加载")
            return None
        
        try:
            # 文本转音素
            phonemes = self.text_to_phonemes(text)
            if not phonemes:
                logger.error("音素转换失败")
                return None
            
            logger.debug(f"转换后的音素序列: {phonemes}")
            
            # 音素转ID序列
            phoneme_ids = self.phonemes_to_ids(phonemes)
            
            logger.debug(f"音素ID序列: {phoneme_ids}")
            
            # 执行合成
            audio_data = self.synthesize_ids(phoneme_ids, speaker_id)
            
            if audio_data is not None and len(audio_data) > 0:
                logger.info(f"成功合成音频，长度: {len(audio_data)} 采样点")
                return audio_data
            else:
                logger.error("音频合成失败，输出为空")
                return None
            
        except Exception as e:
            logger.error(f"语音合成失败: {e}")
            return None
    
    def synthesize_ids(self, phoneme_ids: List[int], speaker_id: Optional[int] = None) -> np.ndarray:
        """
        从音素ID序列合成音频（基于官方Piper实现）
        """
        if len(phoneme_ids) == 0:
            logger.error("音素ID序列为空")
            return np.array([], dtype=np.int16)
        
        # 准备输入数据 - 严格按照官方格式
        phoneme_ids_array = np.expand_dims(np.array(phoneme_ids, dtype=np.int64), 0)
        phoneme_ids_lengths = np.array([phoneme_ids_array.shape[1]], dtype=np.int64)
        
        # 获取推理参数
        noise_scale = self.model_config.get('inference', {}).get('noise_scale', 0.667)
        length_scale = self.model_config.get('inference', {}).get('length_scale', 1.0)
        noise_w = self.model_config.get('inference', {}).get('noise_w', 0.8)
        
        scales = np.array([noise_scale, length_scale, noise_w], dtype=np.float32)
        
        # 构建输入字典 - 严格按照官方格式
        inputs = {
            "input": phoneme_ids_array,
            "input_lengths": phoneme_ids_lengths,
            "scales": scales
        }
        
        # 添加说话人ID（如果适用且模型支持多说话人）
        # 只有当模型支持多个说话人时才添加sid输入
        num_speakers = self.model_config.get('num_speakers', 1)
        if speaker_id is not None and num_speakers > 1:
            sid = np.array([speaker_id], dtype=np.int64)
            inputs["sid"] = sid
        
        try:
            # 执行推理 - 使用官方相同的run方法
            output = self.session.run(None, inputs)
            
            if len(output) == 0:
                logger.error("ONNX推理输出为空")
                return np.array([], dtype=np.int16)
            
            # 处理输出 - 按照官方格式
            audio = output[0].squeeze((0, 1))
            
            # 转换为16位PCM格式
            audio = audio_float_to_int16(audio.squeeze())
            
            return audio
            
        except Exception as e:
            logger.error(f"ONNX推理失败: {e}")
            return np.array([], dtype=np.int16)
    
    def save_wav(self, audio_data: np.ndarray, file_path: str) -> bool:
        """
        保存音频为WAV文件
        """
        try:
            with wave.open(file_path, 'wb') as wav_file:
                wav_file.setnchannels(1)  # 单声道
                wav_file.setsampwidth(2)  # 16位
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(audio_data.tobytes())
            
            logger.info(f"音频已保存到: {file_path}")
            return True
            
        except Exception as e:
            logger.error(f"保存音频失败: {e}")
            return False
    
    def close(self) -> None:
        """
        关闭引擎，释放资源
        """
        if self.session:
            del self.session
            self.session = None
        
        self.is_loaded = False
        logger.info("语音合成引擎已关闭")