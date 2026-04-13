"""
语音模型管理模块
==============

管理多个语音模型，支持模型切换和配置。
"""

import os
import json
from typing import Dict, List, Optional, Any, Any
import logging

logger = logging.getLogger(__name__)


class VoiceModelManager:
    """
    语音模型管理器类
    
    负责管理多个语音模型，支持模型发现、加载和切换。
    """
    
    def __init__(self, models_dir: str = "../resources/models"):
        """
        初始化模型管理器
        
        参数:
            models_dir (str): 模型文件目录
        """
        self.models_dir = models_dir
        self.models = {}  # 模型信息字典
        self.current_model = None
        
        # 扫描模型目录
        self.scan_models()
        
        # 如果未发现模型，尝试从中文模型目录加载
        if not self.models:
            logger.info("未发现模型，尝试从中文模型目录加载...")
            self.scan_models_recursive()
    
    def scan_models(self) -> List[str]:
        """
        扫描模型目录，发现可用的语音模型
        
        返回:
            List[str]: 发现的模型文件列表
        """
        if not os.path.exists(self.models_dir):
            logger.warning(f"模型目录不存在: {self.models_dir}")
            return []
        
        model_files = []
        
        # 优先扫描中文模型目录
        zh_cn_medium_dir = os.path.join(self.models_dir, "zh_CN", "medium")
        if os.path.exists(zh_cn_medium_dir):
            logger.info(f"发现中文模型目录: {zh_cn_medium_dir}")
            model_files.extend(self._scan_single_directory(zh_cn_medium_dir))
        
        # 如果中文模型目录未找到模型，继续扫描根目录
        if not model_files:
            model_files.extend(self._scan_single_directory(self.models_dir))
        
        logger.info(f"共发现 {len(model_files)} 个模型")
        return model_files
    
    def scan_models_recursive(self) -> List[str]:
        """
        递归扫描模型目录，发现所有可用的语音模型
        
        返回:
            List[str]: 发现的模型文件列表
        """
        if not os.path.exists(self.models_dir):
            logger.warning(f"模型目录不存在: {self.models_dir}")
            return []
        
        model_files = []
        
        for root, dirs, files in os.walk(self.models_dir):
            for file_name in files:
                if file_name.endswith('.onnx'):
                    model_path = os.path.join(root, file_name)
                    config_path = model_path.replace('.onnx', '.onnx.json')
                    
                    # 检查配置文件是否存在
                    if os.path.exists(config_path):
                        try:
                            # 读取模型配置
                            with open(config_path, 'r', encoding='utf-8') as f:
                                model_config = json.load(f)
                            
                            # 提取相对路径作为模型名称，便于识别
                            rel_path = os.path.relpath(model_path, self.models_dir)
                            model_name = rel_path.replace('.onnx', '').replace(os.sep, '_')
                            
                            # 提取模型信息
                            model_info = {
                                'path': model_path,
                                'config_path': config_path,
                                'name': model_name,
                                'config': model_config,
                                'sample_rate': model_config.get('audio', {}).get('sample_rate', 22050),
                                'language': model_config.get('language', {}).get('code', 'unknown'),
                                'quality': model_config.get('audio', {}).get('quality', 'unknown')
                            }
                            
                            # 添加到模型列表
                            self.models[model_info['name']] = model_info
                            model_files.append(model_path)
                            
                            logger.info(f"发现模型: {model_info['name']} (语言: {model_info['language']}, 质量: {model_info['quality']})")
                            
                        except Exception as e:
                            logger.error(f"读取模型配置失败 {config_path}: {e}")
        
        logger.info(f"递归扫描共发现 {len(model_files)} 个模型")
        return model_files
    
    def _scan_single_directory(self, directory: str) -> List[str]:
        """
        扫描单个目录中的模型文件
        
        参数:
            directory (str): 要扫描的目录
            
        返回:
            List[str]: 发现的模型文件列表
        """
        model_files = []
        
        for file_name in os.listdir(directory):
            if file_name.endswith('.onnx'):
                model_path = os.path.join(directory, file_name)
                config_path = model_path.replace('.onnx', '.onnx.json')
                
                # 检查配置文件是否存在
                if os.path.exists(config_path):
                    try:
                        # 读取模型配置
                        with open(config_path, 'r', encoding='utf-8') as f:
                            model_config = json.load(f)
                        
                        # 提取相对路径作为模型名称
                        rel_path = os.path.relpath(model_path, self.models_dir)
                        model_name = rel_path.replace('.onnx', '').replace(os.sep, '_')
                        
                        # 提取模型信息
                        model_info = {
                            'path': model_path,
                            'config_path': config_path,
                            'name': model_name,
                            'config': model_config,
                            'sample_rate': model_config.get('audio', {}).get('sample_rate', 22050),
                            'language': model_config.get('language', {}).get('code', 'unknown'),
                            'quality': model_config.get('audio', {}).get('quality', 'unknown')
                        }
                        
                        # 添加到模型列表
                        self.models[model_info['name']] = model_info
                        model_files.append(model_path)
                        
                        logger.info(f"发现模型: {model_info['name']} (语言: {model_info['language']}, 质量: {model_info['quality']})")
                        
                    except Exception as e:
                        logger.error(f"读取模型配置失败 {config_path}: {e}")
        
        return model_files
    
    def get_model_list(self) -> List[Dict]:
        """
        获取模型列表
        
        返回:
            List[Dict]: 模型信息列表
        """
        return list(self.models.values())
    
    def get_model_info(self, model_name: str) -> Optional[Dict]:
        """
        获取指定模型的信息
        
        参数:
            model_name (str): 模型名称
            
        返回:
            Optional[Dict]: 模型信息，如果不存在返回None
        """
        return self.models.get(model_name)
    
    def set_current_model(self, model_name: str) -> bool:
        """
        设置当前使用的模型
        
        参数:
            model_name (str): 模型名称
            
        返回:
            bool: 设置是否成功
        """
        if model_name in self.models:
            self.current_model = model_name
            logger.info(f"当前模型设置为: {model_name}")
            return True
        else:
            logger.error(f"模型不存在: {model_name}")
            return False
    
    def get_current_model_info(self) -> Optional[Dict]:
        """
        获取当前模型的信息
        
        返回:
            Optional[Dict]: 当前模型信息
        """
        if self.current_model:
            return self.models.get(self.current_model)
        return None
    
    def add_model(self, model_path: str, config_path: Optional[str] = None, 
                 model_name: Optional[str] = None) -> bool:
        """
        添加新模型
        
        参数:
            model_path (str): 模型文件路径
            config_path (str, optional): 配置文件路径
            model_name (str, optional): 模型名称
            
        返回:
            bool: 添加是否成功
        """
        try:
            if config_path is None:
                config_path = model_path.replace('.onnx', '.onnx.json')
            
            if model_name is None:
                model_name = os.path.basename(model_path).replace('.onnx', '')
            
            # 检查文件是否存在
            if not os.path.exists(model_path):
                logger.error(f"模型文件不存在: {model_path}")
                return False
            
            if not os.path.exists(config_path):
                logger.error(f"配置文件不存在: {config_path}")
                return False
            
            # 读取配置
            with open(config_path, 'r', encoding='utf-8') as f:
                model_config = json.load(f)
            
            # 创建模型信息
            model_info = {
                'path': model_path,
                'config_path': config_path,
                'name': model_name,
                'config': model_config,
                'sample_rate': model_config.get('audio', {}).get('sample_rate', 22050)
            }
            
            # 添加到模型列表
            self.models[model_name] = model_info
            
            logger.info(f"成功添加模型: {model_name}")
            return True
            
        except Exception as e:
            logger.error(f"添加模型失败: {e}")
            return False
    
    def remove_model(self, model_name: str) -> bool:
        """
        移除模型
        
        参数:
            model_name (str): 模型名称
            
        返回:
            bool: 移除是否成功
        """
        if model_name in self.models:
            # 如果移除的是当前模型，清除当前模型设置
            if self.current_model == model_name:
                self.current_model = None
            
            del self.models[model_name]
            logger.info(f"已移除模型: {model_name}")
            return True
        else:
            logger.warning(f"模型不存在: {model_name}")
            return False
    
    def get_model_statistics(self) -> Dict:
        """
        获取模型统计信息
        
        返回:
            Dict: 统计信息
        """
        total_models = len(self.models)
        
        # 统计不同采样率的模型数量
        sample_rate_stats = {}
        for model_info in self.models.values():
            sr = model_info['sample_rate']
            sample_rate_stats[sr] = sample_rate_stats.get(sr, 0) + 1
        
        return {
            'total_models': total_models,
            'current_model': self.current_model,
            'sample_rate_distribution': sample_rate_stats
        }
    
    def find_model_by_sample_rate(self, sample_rate: int) -> List[Dict]:
        """
        根据采样率查找模型
        
        参数:
            sample_rate (int): 目标采样率
            
        返回:
            List[Dict]: 匹配的模型列表
        """
        matching_models = []
        
        for model_info in self.models.values():
            if model_info['sample_rate'] == sample_rate:
                matching_models.append(model_info)
        
        return matching_models
    
    def set_zh_cn_medium_model(self) -> bool:
        """
        设置中文中等质量模型为当前模型
        
        返回:
            bool: 设置是否成功
        """
        # 查找中文模型
        zh_models = []
        for model_info in self.models.values():
            if model_info.get('language') == 'zh_CN':
                zh_models.append(model_info)
        
        if not zh_models:
            logger.warning("未找到中文模型")
            return False
        
        # 优先选择中等质量模型
        medium_models = [m for m in zh_models if m.get('quality') == 'medium']
        
        if medium_models:
            # 如果有中等质量模型，选择第一个
            selected_model = medium_models[0]
        else:
            # 否则选择第一个中文模型
            selected_model = zh_models[0]
        
        return self.set_current_model(selected_model['name'])
    
    def find_models_by_language(self, language_code: str) -> List[Dict]:
        """
        根据语言代码查找模型
        
        参数:
            language_code (str): 语言代码 (如 'zh_CN', 'en_US')
            
        返回:
            List[Dict]: 匹配的模型列表
        """
        matching_models = []
        
        for model_info in self.models.values():
            if model_info.get('language') == language_code:
                matching_models.append(model_info)
        
        return matching_models
    
    def get_preferred_model(self) -> Optional[Dict]:
        """
        获取首选的模型（优先中文模型）
        
        返回:
            Optional[Dict]: 首选模型信息
        """
        # 优先查找中文模型
        zh_models = self.find_models_by_language('zh_CN')
        
        if zh_models:
            # 在中文模型中优先选择中等质量
            medium_zh_models = [m for m in zh_models if m.get('quality') == 'medium']
            if medium_zh_models:
                return medium_zh_models[0]
            return zh_models[0]
        
        # 如果没有中文模型，返回第一个可用模型
        if self.models:
            return list(self.models.values())[0]
        
        return None
    
    def export_model_list(self, output_file: str) -> bool:
        """
        导出模型列表到文件
        
        参数:
            output_file (str): 输出文件路径
            
        返回:
            bool: 导出是否成功
        """
        try:
            # 准备导出数据
            export_data = {
                'models': self.models,
                'current_model': self.current_model,
                'statistics': self.get_model_statistics()
            }
            
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"模型列表已导出到: {output_file}")
            return True
            
        except Exception as e:
            logger.error(f"导出模型列表失败: {e}")
            return False