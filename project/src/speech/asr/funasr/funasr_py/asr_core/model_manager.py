#!/usr/bin/env python3
"""
模型管理器模块
包含模型配置、下载、导出、加载等功能
"""

import os
import sys
from pathlib import Path

# 导入模型配置 - 直接导入model_config模块
from model_config import MODEL_INFO, get_model_info, get_model_by_id, show_model_list

class ModelManager:
    """模型管理器类"""
    
    def __init__(self, models_root_path=None):
        """
        初始化模型管理器
        
        Args:
            models_root_path (str): 模型根目录路径
        """
        # 如果没有指定模型根路径，则尝试自动检测
        if models_root_path is None:
            # 获取当前文件的目录
            current_dir = os.path.dirname(os.path.abspath(__file__))
            
            # 尝试多种可能的模型路径，按照优先级排序
            possible_paths = [
                # 安装环境路径 - 实际安装路径（最高优先级）
                os.path.abspath(os.path.join(current_dir, "..", "..", "..", "share", "funasr_py", "resources", "models")),
                # 安装环境路径 - lib/share目录（某些系统路径）
                os.path.abspath(os.path.join(current_dir, "..", "..", "lib", "share", "funasr_py", "resources", "models")),
                # 开发环境路径 - 外部resources目录
                os.path.abspath(os.path.join(current_dir, "..", "..", "..", "resources", "models")),
                # 安装环境路径 - share目录（旧版本路径）
                os.path.abspath(os.path.join(current_dir, "..", "share", "funasr_py", "resources", "models")),
                # 开发环境路径 - 内部resources目录（旧版本）
                os.path.abspath(os.path.join(current_dir, "..", "..", "resources", "models")),
                # 当前目录下的resources
                os.path.abspath(os.path.join(os.getcwd(), "resources", "models")),
                # 环境变量指定的路径
                os.environ.get("FUNASR_MODELS_PATH", "")
            ]
            
            # 查找第一个存在的路径，并且包含模型文件
            self.models_root_path = ""
            for path in possible_paths:
                if path and os.path.exists(path) and os.listdir(path):
                    self.models_root_path = path
                    break
            
            # 如果都没找到，使用第一个存在的路径作为默认路径
            if not self.models_root_path:
                for path in possible_paths:
                    if path and os.path.exists(path):
                        self.models_root_path = path
                        break
            
            # 如果还是没找到，使用实际安装路径作为默认路径
            if not self.models_root_path:
                self.models_root_path = possible_paths[0]  # 使用实际安装路径
        else:
            self.models_root_path = os.path.abspath(models_root_path)
        
        # 确保模型根目录存在
        Path(self.models_root_path).mkdir(parents=True, exist_ok=True)
        print(f"📁 模型根目录: {self.models_root_path}")
    
    def check_model_exists(self, model_path):
        """检查模型是否已存在"""
        return os.path.exists(model_path) and os.listdir(model_path)
    
    def download_model(self, model_name, modelscope_name, download_path=None):
        """
        下载指定的FunASR模型到指定路径
        
        Args:
            model_name (str): 模型名称
            modelscope_name (str): ModelScope上的模型名称
            download_path (str): 下载路径，如果为None则使用默认路径
        
        Returns:
            str: 模型下载后的路径
        """
        try:
            # 如果未指定下载路径，则使用默认路径
            if download_path is None:
                download_path = os.path.join(self.models_root_path, model_name)
            
            # 确保下载路径存在
            Path(download_path).mkdir(parents=True, exist_ok=True)
            
            # 导入modelscope
            try:
                from modelscope import snapshot_download
            except ImportError:
                print("❌ 未找到modelscope库，请先安装:")
                print("   pip install modelscope")
                return None
            
            print(f"📥 开始下载模型: {model_name}")
            print(f"📂 下载路径: {download_path}")
            
            # 下载模型
            model_dir = snapshot_download(
                modelscope_name,
                cache_dir=download_path
            )
            
            print(f"✅ 模型下载成功!")
            print(f"📁 模型保存在: {model_dir}")
            return model_dir
            
        except Exception as e:
            print(f"❌ 模型下载失败: {e}")
            return None
    
    def export_model_to_onnx(self, model_info, output_path=None):
        """
        使用FunASR的正确方式将模型导出为ONNX格式
        
        Args:
            model_info (dict): 模型信息
            output_path (str): ONNX输出路径，如果为None则自动创建
        """
        try:
            # 导入必要的库
            try:
                from funasr import AutoModel
            except ImportError:
                print("❌ 未找到funasr库，请先安装:")
                print("   pip install funasr")
                return False
            
            model_name = model_info['name']
            model_path = model_info['local_path']
            model_base_path = model_info['base_path']
            
            print(f"📥 准备导出模型: {model_name}")
            print(f"📂 模型路径: {model_path}")
            
            # 如果未指定输出路径，则根据模型编号使用特定路径
            if output_path is None:
                # 如果选择的是模型3 (paraformer-zh-streaming)，使用特定路径
                if model_name == "paraformer-zh-streaming":
                    output_path = os.path.join(model_base_path, "onnx")
                else:
                    output_path = os.path.join(model_base_path, f"{model_name}")
            
            print(f"💾 导出路径: {output_path}")
            
            # 检查输出目录是否已存在
            if os.path.exists(output_path):
                print(f"⚠️  目录 {output_path} 已存在")
            else:
                # 加载模型
                print("🔄 加载模型中...")
                model = AutoModel(model=model_path)
                
                print("✅ 模型加载成功")
                print("🔄 正在导出为ONNX格式...")
                
                # 导出为ONNX
                res = model.export(quantize=False, output_dir=output_path)
                
                print(f"✅ 模型已成功导出为ONNX格式!")
                print(f"📁 导出目录: {output_path}")
                
                # 复制必要的配置文件以支持funasr-onnx加载
                print("🔄 正在复制配置文件...")
                try:
                    import shutil
                    import yaml
                    
                    # 需要复制的文件列表
                    required_files = [
                        "config.yaml",
                        "tokens.json", 
                        "configuration.json",
                        "am.mvn",
                        "seg_dict"
                    ]
                    
                    # 复制所有必需的文件
                    for file_name in required_files:
                        original_file_path = os.path.join(model_path, file_name)
                        target_file_path = os.path.join(output_path, file_name)
                        
                        if os.path.exists(original_file_path):
                            shutil.copy2(original_file_path, target_file_path)
                            print(f"✅ 复制文件: {target_file_path}")
                        else:
                            print(f"⚠️ 文件不存在，跳过: {original_file_path}")
                    
                    # 修改config.yaml以适应ONNX模型
                    target_config_path = os.path.join(output_path, "config.yaml")
                    if os.path.exists(target_config_path):
                        # 添加ONNX特定的配置
                        onnx_config_addition = """
# ONNX模型配置
onnx_model: true
onnx_model_dir: ./
onnx_model_file: model.onnx
onnx_decoder_file: decoder.onnx
"""
                        
                        with open(target_config_path, 'a', encoding='utf-8') as f:
                            f.write(onnx_config_addition)
                        print(f"✅ 更新配置文件: {target_config_path}")
                        
                except Exception as e:
                    print(f"⚠️ 配置文件复制失败: {e}")
                    # 创建基本的config.yaml文件
                    config_content = """model_dir: ./
model_file: model.onnx
decoder_file: decoder.onnx
model_type: onnx
model_name: paraformer
sample_rate: 16000
"""
                    config_path = os.path.join(output_path, "config.yaml")
                    with open(config_path, "w", encoding="utf-8") as f:
                        f.write(config_content)
                    print(f"✅ 创建基本配置文件: {config_path}")
                
                # 显示导出的详细文件路径
                if os.path.exists(output_path):
                    print("📋 导出的文件:")
                    for file in os.listdir(output_path):
                        full_path = os.path.join(output_path, file)
                        print(f"   📄 {full_path}")
                
                return True
            
        except ImportError as e:
            error_msg = str(e).lower()
            if "onnx" in error_msg:
                print("❌ 缺少ONNX依赖库，模型导出需要以下库:")
                print("   pip install onnx")
                print("   pip install onnxruntime  # 可选，用于测试ONNX模型")
                print("\n💡 安装建议:")
                print("   # 在虚拟环境中安装")
                print("   source ~/funasr_venv/bin/activate")
                print("   pip install onnx onnxruntime")
            elif "funasr" in error_msg:
                print("❌ 未找到funasr库，请先安装:")
                print("   pip install funasr")
                print("\n💡 安装建议:")
                print("   # 在虚拟环境中安装")
                print("   source ~/funasr_venv/bin/activate")
                print("   pip install funasr")
            else:
                print("❌ 缺少必要的依赖库，请安装:")
                print("   pip install funasr onnx onnxruntime")
                print("\n💡 安装建议:")
                print("   # 在虚拟环境中安装")
                print("   source ~/funasr_venv/bin/activate")
                print("   pip install funasr onnx onnxruntime")
            return False
        except Exception as e:
            print(f"❌ 模型导出失败: {e}")
            return False
    
    def load_model(self, model_id, model_type="pt"):
        """
        从本地加载模型
        
        Args:
            model_id (str): 模型编号
            model_type (str): 模型类型 ("pt" for PyTorch, "onnx" for ONNX)
        
        Returns:
            dict: 选定的模型信息，如果加载失败返回None
        """
        if model_id not in MODEL_INFO:
            print(f"❌ 无效的模型编号: {model_id}")
            return None
        
        selected_model = MODEL_INFO[model_id]
        model_name = selected_model['name']
        modelscope_name = selected_model['modelscope_name']
        
        # 显示模型信息
        print(f"\n您选择的模型是: {model_name}")
        print(f"描述: {selected_model['description']}")
        print(f"大小: {selected_model['size']}， 参数量：{selected_model['parameters']}")
        print(f"ModelScope名称: {modelscope_name}")
        
        # 构建模型路径 - 使用实际的嵌套路径结构
        model_base_path = os.path.abspath(os.path.join(self.models_root_path, model_name))
        
        # 根据模型类型确定模型路径
        if model_type == "onnx":
            model_path = os.path.join(model_base_path, "onnx")
        else:  # 默认为PyTorch模型
            model_path = os.path.join(model_base_path, "iic", modelscope_name.split("/")[-1])
        
        # 检查模型是否存在
        if not self.check_model_exists(model_path):
            print(f"❌ 模型 {model_name} ({model_type}格式) 未找到，请先下载模型")
            print(f"   下载指令: modelscope download --model {modelscope_name}")
            return None
        
        print(f"✅ 模型 {model_name} ({model_type}格式) 已成功加载!")
        print(f"📁 模型路径: {model_path}")
        
        # 返回模型信息
        selected_model['local_path'] = model_path
        selected_model['base_path'] = model_base_path
        selected_model['type'] = model_type
        return selected_model
    
    def load_model_by_path(self, model_path, model_type="onnx"):
        """
        通过指定路径加载模型（为ROS节点提供简化接口）
        
        Args:
            model_path (str): 模型路径
            model_type (str): 模型类型 ("pt" for PyTorch, "onnx" for ONNX)
        
        Returns:
            dict: 模型信息，如果加载失败返回None
        """
        # 检查模型路径是否存在
        if not os.path.exists(model_path):
            print(f"❌ 模型路径不存在: {model_path}")
            return None
            
        # 如果路径是目录且为空
        if os.path.isdir(model_path) and not os.listdir(model_path):
            print(f"❌ 模型目录为空: {model_path}")
            return None
            
        print(f"✅ 从路径加载模型成功!")
        print(f"📁 模型路径: {model_path}")
        
        # 构造模型信息字典
        model_info = {
            'name': 'custom_model',
            'local_path': model_path,
            'type': model_type
        }
        
        return model_info
    
    def get_model_info(self):
        """
        获取预配置的模型信息（为ROS节点提供默认模型信息）
        
        Returns:
            dict: 预配置的模型信息
        """
        # 预配置的模型路径
        model_base_path = os.path.abspath(os.path.join(self.models_root_path, "paraformer-zh-streaming"))
        # 默认使用pt模型类型
        model_path = os.path.join(model_base_path, "iic", "speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online")
        
        print(f"🔍 检查模型路径: {model_path}")
        print(f"🔍 模型目录是否存在: {os.path.exists(model_path)}")
        if os.path.exists(model_path):
            print(f"🔍 模型目录内容: {os.listdir(model_path)}")
        
        # 检查模型是否存在
        if not self.check_model_exists(model_path):
            print(f"❌ 默认模型路径不存在: {model_path}")
            # 提供更详细的错误信息
            print(f"🔍 请检查以下路径是否存在模型文件:")
            print(f"   安装环境: {os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'share', 'funasr_py', 'resources', 'models'))}")
            print(f"   开发环境: {os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'resources', 'models'))}")
            print(f"   环境变量: {os.environ.get('FUNASR_MODELS_PATH', '未设置')}")
            print(f"💡 解决方案:")
            print(f"   1. 下载模型到上述路径之一")
            print(f"   2. 或者设置环境变量 FUNASR_MODELS_PATH 指向模型目录")
            print(f"   3. 或者手动创建符号链接")
            return None
            
        print(f"✅ 使用默认模型配置!")
        print(f"📁 模型路径: {model_path}")
        
        # 构造模型信息字典，默认模型类型为pt
        model_info = {
            'name': 'paraformer-zh-streaming',
            'local_path': model_path,
            'type': 'pt'
        }
        
        return model_info
    
    def list_models(self):
        """显示模型列表"""
        show_model_list()