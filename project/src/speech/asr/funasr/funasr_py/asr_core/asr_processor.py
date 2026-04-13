#!/usr/bin/env python3
"""
ASR处理器模块
包含ASR模型管理、语音识别处理等功能
"""

import os
import sys
import numpy as np

# 添加项目根目录到Python路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

class ASRProcessor:
    """ASR处理器类"""
    
    def __init__(self, model_dir=None):
        """
        初始化ASR处理器
        
        Args:
            model_dir (str): 模型目录路径
        """
        self.model_dir = model_dir
        self.model = None
        self.is_real_time = False
        
        # 初始化模型
        if model_dir:
            self.init_model(model_dir)
    
    def load_asr_model(self, model_info):
        """
        加载ASR模型
        
        Args:
            model_info (dict): 模型信息
            
        Returns:
            bool: 是否成功加载模型
        """
        try:
            # 延迟导入，避免在没有安装funasr时出错
            import funasr
            AutoModel = getattr(funasr, 'AutoModel')
            
            model_path = model_info['local_path']
            model_type = model_info.get('type', 'pt')
            
            print(f"🔄 正在加载ASR模型: {model_info['name']} ({model_type})")
            print(f"📂 模型路径: {model_path}")
            
            # 加载模型，禁用更新检查以避免网络请求阻塞
            self.model = AutoModel(model=model_path, disable_update=True)
            self.model_type = model_type
            
            print("✅ ASR模型加载成功!")
            return True
            
        except ImportError:
            print("❌ 未找到funasr库，请先安装:")
            print("   pip install funasr")
            return False
        except Exception as e:
            print(f"❌ ASR模型加载失败: {e}")
            return False
    
    def init_model(self, model_dir):
        """
        初始化ASR模型
        
        Args:
            model_dir (str): 模型目录路径
        """
        try:
            # 直接加载模型，而不是创建新的ModelManager
            if self.load_asr_model({'local_path': model_dir}):
                print(f"✅ ASR模型初始化成功: {model_dir}")
            else:
                print(f"❌ ASR模型初始化失败: {model_dir}")
                self.model = None
        except Exception as e:
            print(f"❌ ASR模型初始化失败: {e}")
            self.model = None
    
    def recognize_streaming_audio(self, audio_data, sample_rate=16000, cache=None, 
                                chunk_size=None, encoder_chunk_look_back=0, 
                                decoder_chunk_look_back=0, enable_logging=True, is_final=False):
        """
        流式音频识别
        
        Args:
            audio_data (np.array): 音频数据
            sample_rate (int): 采样率
            cache (dict): 缓存
            chunk_size (list): 块大小
            encoder_chunk_look_back (int): 编码器回看
            decoder_chunk_look_back (int): 解码器回看
            enable_logging (bool): 是否启用日志
            is_final (bool): 是否为最后一个块
            
        Returns:
            dict: 识别结果
        """
        if self.model is None:
            print("❌ ASR模型未初始化")
            return None
            
        try:
            if enable_logging:
                print("🔄 正在进行流式语音识别...")
            
            # 使用模型进行流式识别
            result = self.model.generate(
                input=audio_data,
                cache=cache,
                is_final=is_final,
                chunk_size=chunk_size,
                encoder_chunk_look_back=encoder_chunk_look_back,
                decoder_chunk_look_back=decoder_chunk_look_back,
                disable_pbar=True  # 禁用进度条输出
            )
            
            if enable_logging:
                print("✅ 流式语音识别完成!")
            return result
        except Exception as e:
            print(f"❌ 流式音频识别失败: {e}")
            return None
    
    def recognize_audio_file(self, audio_data, fs=16000, enable_logging=None):
        """
        对音频文件进行语音识别
        
        Args:
            audio_data (np.array): 音频数据
            fs (int): 采样率
            enable_logging (bool): 是否启用日志输出，如果为None则使用默认值True
            
        Returns:
            dict: 识别结果
        """
        # 调用perform_asr_streaming方法进行识别，不输出调试信息
        # return self.perform_asr_streaming(audio_data, sample_rate, debug_audio=False)
        # 处理enable_logging参数，如果为None则使用默认值True
        if enable_logging is None:
            enable_logging = True
            
        if self.model is None:
            if enable_logging:
                print("❌ 请先加载ASR模型")
            return None
        
        try:
            if enable_logging:
                print("🔄 正在进行语音识别...")
            
            # 确保音频数据是float32类型并且范围在[-1, 1]
            if audio_data.dtype != np.float32:
                if audio_data.dtype == np.int16:
                    audio_data = audio_data.astype(np.float32) / 32768.0
                elif audio_data.dtype == np.int32:
                    audio_data = audio_data.astype(np.float32) / 2147483648.0
                else:
                    audio_data = audio_data.astype(np.float32)
            
            # 如果是多声道，转换为单声道
            if audio_data.ndim > 1:
                audio_data = np.mean(audio_data, axis=1)
            
            # 进行语音识别
            result = self.model.generate(input=audio_data, disable_pbar=True)
            
            if enable_logging:
                print("✅ 语音识别完成!")
            return result
            
        except Exception as e:
            if enable_logging:
                print(f"❌ 语音识别失败: {e}")
            return None
    
    def stop_real_time_asr(self):
        """停止实时语音识别"""
        self.is_real_time = False
        print("⏹️ 正在停止实时语音识别...")

    def perform_asr_streaming(self, audio_data, input_sample_rate, debug_audio=False):
        """
        执行流式语音识别
        
        Args:
            audio_data (list or np.array): 音频数据
            input_sample_rate (int): 输入音频的采样率
            debug_audio (bool): 是否输出调试信息
            
        Returns:
            str: 识别结果
        """
        # 导入必要的工具函数
        try:
            # 注意：audio_basic_py.asr_utils 需要在使用时动态导入，因为它不在当前包中
            # 动态导入 audio_basic_py.asr_utils
            import sys
            import os
            # 添加audio_basic_py路径
            audio_basic_path = os.path.join(os.path.dirname(__file__), "..", "..", "audio_basic_py")
            if audio_basic_path not in sys.path:
                sys.path.append(audio_basic_path)
            
            # 修复导入路径问题，使用更健壮的导入方式
            try:
                from audio_basic_py.asr_utils import is_valid_result
            except ImportError:
                from audio_basic_py.audio_basic_py.asr_utils import is_valid_result
        except ImportError as e:
            print(f"无法导入必要的模块: {e}")
            return ""
        
        # 添加测试代码：输出音频数据统计信息用于调试（可配置）
        if debug_audio:
            try:
                # 动态导入 audio_basic_py.audio_basic_py.audio_utils
                import sys
                import os
                # 添加audio_basic_py路径
                audio_basic_path = os.path.join(os.path.dirname(__file__), "..", "..", "audio_basic_py")
                if audio_basic_path not in sys.path:
                    sys.path.append(audio_basic_path)
                
                # 修复导入路径问题，使用更健壮的导入方式
                try:
                    from audio_basic_py.audio_utils import analyze_audio_data, save_audio_data, print_audio_stats, resample_audio
                except ImportError:
                    from audio_basic_py.audio_basic_py.audio_utils import analyze_audio_data, save_audio_data, print_audio_stats, resample_audio
                
                # 使用音频调试工具模块分析音频数据
                stats = analyze_audio_data(audio_data, input_sample_rate)
                
                # 输出统计信息
                print_audio_stats(stats)
                
                # 保存音频数据到文件用于进一步分析
                try:
                    filename = save_audio_data(audio_data, input_sample_rate)
                    print(f"已保存音频数据到: {filename}")
                except Exception as e:
                    print(f"保存音频数据失败: {e}")
                    
                # 检查音频数据是否有效
                if stats['is_silent']:
                    print("音频数据为静音，可能影响识别效果")
                elif stats['std'] < 0.01:  # 标准差过小
                    print(f"音频数据标准差过小 ({stats['std']:.6f})，可能影响识别效果")
                    
            except Exception as e:
                print(f"计算音频数据统计信息失败: {e}")
        
        # 添加局部临时变量控制是否打印语音识别结果输出，默认为True
        print_result = True
        print_proc_info = False
        try:
            # 转换为numpy数组
            import numpy as np
            if not isinstance(audio_data, np.ndarray):
                if isinstance(audio_data, bytes):
                    np_audio_data = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
                else:
                    # 否则转换为numpy数组
                    np_audio_data = np.array(audio_data, dtype=np.float32)
            else:
                # 如果已经是numpy数组，确保数据类型正确
                if audio_data.dtype == np.int16:
                    np_audio_data = audio_data.astype(np.float32) / 32768.0
                else:
                    np_audio_data = audio_data.astype(np.float32)
            
            # 如果采样率不匹配，需要重采样到16000Hz
            if input_sample_rate != 16000:
                if debug_audio:
                    print(f"重采样音频数据从 {input_sample_rate}Hz 到 16000Hz")
                # 动态导入 resample_audio
                try:
                    import sys
                    import os
                    # 添加audio_basic_py路径
                    audio_basic_path = os.path.join(os.path.dirname(__file__), "..", "..", "audio_basic_py")
                    if audio_basic_path not in sys.path:
                        sys.path.append(audio_basic_path)
                    
                    # 修复导入路径问题，使用更健壮的导入方式
                    try:
                        from audio_basic_py.audio_utils import resample_audio
                    except ImportError:
                        from audio_basic_py.audio_basic_py.audio_utils import resample_audio
                    np_audio_data = resample_audio(np_audio_data, input_sample_rate, 16000)
                    input_sample_rate = 16000
                    if debug_audio:
                        # 动态导入 save_audio_data
                        try:
                            import sys
                            import os
                            # 添加audio_basic_py路径
                            audio_basic_path = os.path.join(os.path.dirname(__file__), "..", "..", "audio_basic_py")
                            if audio_basic_path not in sys.path:
                                sys.path.append(audio_basic_path)
                            
                            # 修复导入路径问题，使用更健壮的导入方式
                            try:
                                from audio_basic_py.audio_utils import save_audio_data
                            except ImportError:
                                from audio_basic_py.audio_basic_py.audio_utils import save_audio_data
                            filename = save_audio_data(np_audio_data, input_sample_rate)
                            print(f"已保存转换采样率以后的音频数据到: {filename}")
                        except Exception as e:
                            print(f"保存重采样音频数据失败: {e}")
                except Exception as e:
                    print(f"重采样音频数据失败: {e}")
                    return ""
            
            # 实现流式识别逻辑
            # 配置流式识别参数
            chunk_size = [0, 10, 5]  # 流式识别的块大小配置
            encoder_chunk_look_back = 4  # 编码器回看块数
            decoder_chunk_look_back = 1  # 解码器回看块数
            chunk_stride = chunk_size[1] * 960  # 计算块步长
            
            # 初始化缓存
            cache = {}  # 缓存对象，用于存储中间结果
            result = ""  # 存储识别结果
            speech = np_audio_data  # 输入音频数据
            total_chunk_num = int((len(speech) - 1) / chunk_stride + 1)  # 计算总的块数
            
            # 分块处理音频数据
            for i in range(total_chunk_num):
                speech_chunk = speech[i*chunk_stride:(i+1)*chunk_stride]
                is_final = i == total_chunk_num - 1  # 是否为最后一个块
                
                # 调用流式识别函数
                res = self.recognize_streaming_audio(
                    audio_data=speech_chunk,
                    sample_rate=16000,
                    cache=cache,
                    chunk_size=chunk_size,
                    encoder_chunk_look_back=encoder_chunk_look_back,
                    decoder_chunk_look_back=decoder_chunk_look_back,
                    enable_logging=False,  # 在流式处理中关闭日志以避免过多输出
                    is_final=is_final
                )
                
                if res and len(res) > 0 and "text" in res[0]:
                    result += res[0]["text"]
            
            if debug_audio:
                print(f"流式语音识别完成，结果: {result}")
            
            # 返回结果
            return result
            
        except Exception as e:
            print(f"流式语音识别过程中出现异常: {e}")
            return ""