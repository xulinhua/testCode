"""
配置管理模块
============

管理语音合成系统的配置参数，支持多种配置文件和平台特定配置。

主要功能：
1. 配置文件的加载和保存
2. 多平台配置适配
3. 配置参数验证
4. 动态配置更新
"""

import os
import yaml
import json
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)


class ConfigManager:
    """
    配置管理类
    
    负责管理语音合成系统的各种配置参数。
    """
    
    def __init__(self, config_dir: str = "config"):
        """
        初始化配置管理器
        
        参数:
            config_dir (str): 配置文件目录路径
        """
        self.config_dir = config_dir
        self.config = {}
        self.platform_config = {}
        
        # 默认配置
        self._default_config = {
            "audio": {
                "sample_rate": 22050,
                "channels": 1,
                "format": "pcm_s16le"
            },
            "model": {
                "path": "../resources/models/default.onnx",
                "config_path": "../resources/models/default.onnx.json",
                "speaker_id": 0
            },
            "synthesis": {
                "max_text_length": 500,
                "batch_size": 1,
                "enable_streaming": False
            },
            "performance": {
                "use_gpu": True,
                "threads": 4,
                "optimization_level": 3
            }
        }
    
    def load_config(self, config_file: str = "default.yaml") -> bool:
        """
        加载配置文件
        
        参数:
            config_file (str): 配置文件名
            
        返回:
            bool: 加载是否成功
        """
        try:
            config_path = os.path.join(self.config_dir, config_file)
            
            with open(config_path, 'r', encoding='utf-8') as f:
                if config_file.endswith('.yaml') or config_file.endswith('.yml'):
                    loaded_config = yaml.safe_load(f)
                elif config_file.endswith('.json'):
                    loaded_config = json.load(f)
                else:
                    logger.error(f"不支持的配置文件格式: {config_file}")
                    return False
            
            # 深度合并配置
            self._deep_merge(self.config, loaded_config)
            
            logger.info(f"成功加载配置文件: {config_path}")
            return True
            
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            return False
    
    def load_platform_config(self, platform: str = "jetson") -> bool:
        """
        加载平台特定配置
        
        参数:
            platform (str): 平台名称
            
        返回:
            bool: 加载是否成功
        """
        platform_file = f"{platform}.yaml"
        platform_path = os.path.join(self.config_dir, platform_file)
        
        if not os.path.exists(platform_path):
            logger.warning(f"平台配置文件不存在: {platform_path}")
            return False
        
        try:
            with open(platform_path, 'r', encoding='utf-8') as f:
                platform_config = yaml.safe_load(f)
            
            # 应用平台特定配置
            self._deep_merge(self.config, platform_config)
            self.platform_config = platform_config
            
            logger.info(f"成功加载平台配置: {platform_path}")
            return True
            
        except Exception as e:
            logger.error(f"加载平台配置失败: {e}")
            return False
    
    def _deep_merge(self, base: Dict[Any, Any], update: Dict[Any, Any]) -> None:
        """
        深度合并两个字典
        
        参数:
            base (Dict): 基础字典
            update (Dict): 更新字典
        """
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._deep_merge(base[key], value)
            else:
                base[key] = value
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        获取配置值
        
        参数:
            key (str): 配置键，支持点分隔（如 "audio.sample_rate"）
            default (Any): 默认值
            
        返回:
            Any: 配置值
        """
        keys = key.split('.')
        value = self.config
        
        try:
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default
    
    def set(self, key: str, value: Any) -> bool:
        """
        设置配置值
        
        参数:
            key (str): 配置键，支持点分隔
            value (Any): 配置值
            
        返回:
            bool: 设置是否成功
        """
        keys = key.split('.')
        config = self.config
        
        try:
            # 遍历到最后一个键的父级
            for k in keys[:-1]:
                if k not in config:
                    config[k] = {}
                config = config[k]
            
            # 设置最终值
            config[keys[-1]] = value
            return True
            
        except Exception as e:
            logger.error(f"设置配置失败: {e}")
            return False
    
    def save_config(self, config_file: str = "current.yaml") -> bool:
        """
        保存当前配置到文件
        
        参数:
            config_file (str): 配置文件名
            
        返回:
            bool: 保存是否成功
        """
        try:
            config_path = os.path.join(self.config_dir, config_file)
            
            with open(config_path, 'w', encoding='utf-8') as f:
                yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True)
            
            logger.info(f"配置已保存到: {config_path}")
            return True
            
        except Exception as e:
            logger.error(f"保存配置失败: {e}")
            return False
    
    def validate_config(self) -> bool:
        """
        验证配置有效性
        
        返回:
            bool: 配置是否有效
        """
        try:
            # 验证必要配置项
            required_keys = [
                "tts.sample_rate",
                "model.default_model",
                "tts.noise_scale"
            ]
            
            for key in required_keys:
                if self.get(key) is None:
                    logger.error(f"缺少必要配置项: {key}")
                    return False
            
            # 验证数值范围
            sample_rate = self.get("tts.sample_rate")
            if sample_rate <= 0:
                logger.error("采样率必须大于0")
                return False
            
            max_text_length = self.get("synthesis.max_text_length")
            if max_text_length is not None and max_text_length <= 0:
                logger.error("最大文本长度必须大于0")
                return False
            
            logger.info("配置验证通过")
            return True
            
        except Exception as e:
            logger.error(f"配置验证失败: {e}")
            return False
    
    def get_all_config(self) -> Dict[str, Any]:
        """
        获取所有配置
        
        返回:
            Dict[str, Any]: 完整配置字典
        """
        return self.config.copy()
    
    def reset_to_default(self) -> None:
        """
        重置为默认配置
        """
        self.config = self._default_config.copy()
        logger.info("配置已重置为默认值")