#!/usr/bin/env python3
"""
FunASR模型配置文件
包含所有支持的模型信息
"""

MODEL_INFO = {
    "1": {
        "name": "SenseVoiceSmall",
        "modelscope_name": "iic/SenseVoiceSmall",
        "description": "多语言语音理解模型，支持中文、粤语、英文、日语、韩语等多种语言的语音识别和理解",
        "size": "约90MB",
        "parameters": "330M"
    },
    "2": {
        "name": "paraformer-zh",
        "modelscope_name": "iic/speech_paraformer-large-vad-punc_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
        "description": "非流式中文语音识别模型，具有高精度的中文语音识别能力，支持标点符号预测和语音活动检测",
        "size": "约2.3GB",
        "parameters": "220M"
    },
    "3": {
        "name": "paraformer-zh-streaming",
        "modelscope_name": "iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online",
        "description": "流式中文语音识别模型，支持实时语音识别，适用于在线语音识别场景",
        "size": "约840M",
        "parameters": "220M"
    },
    "4": {
        "name": "paraformer-en",
        "modelscope_name": "iic/speech_paraformer-large_asr_nat-en-16k-common-vocab10027-pytorch",
        "description": "英文语音识别模型，针对英文语音识别进行了优化",
        "size": "约2.3GB",
        "parameters": "220M"
    },
    "5": {
        "name": "conformer-en",
        "modelscope_name": "iic/speech_conformer_asr_nat-en-16k-common-vocab10027-pytorch",
        "description": "英文语音识别模型，基于Conformer架构",
        "size": "约1.2GB",
        "parameters": "220M"
    },
    "6": {
        "name": "ct-punc",
        "modelscope_name": "iic/punc_ctb9_vit_base_char_zh",
        "description": "中文标点恢复模型，用于为识别结果添加标点符号",
        "size": "约300MB",
        "parameters": "290M"
    },
    "7": {
        "name": "fsmn-vad",
        "modelscope_name": "iic/speech_fsmn_vad_zh-cn-16k-common-pytorch",
        "description": "语音活动检测模型，用于检测音频中的语音活动",
        "size": "约200MB",
        "parameters": "0.4M"
    },
    "8": {
        "name": "fsmn-kws",
        "modelscope_name": "iic/speech_kws_fsmn_char_ctc_zh-cn-16k-common",
        "description": "关键词唤醒模型，用于检测特定关键词",
        "size": "约200MB",
        "parameters": "0.7M"
    },
    "9": {
        "name": "fa-zh",
        "modelscope_name": "iic/speech_fa-zh-cn-16k-common",
        "description": "中文语音活动检测模型",
        "size": "约100MB",
        "parameters": "38M"
    },
    "10": {
        "name": "cam++",
        "modelscope_name": "iic/speech_campplus_sv_zh-cn_16k-common",
        "description": "说话人验证模型，用于识别说话人身份",
        "size": "约150MB",
        "parameters": "7.2M"
    },
    "11": {
        "name": "Whisper-large-v3",
        "modelscope_name": "iic/Whisper-large-v3",
        "description": "多语言语音识别模型，基于Whisper架构，支持多种语言",
        "size": "约3GB",
        "parameters": "1.5B"
    },
    "12": {
        "name": "Whisper-large-v3-turbo",
        "modelscope_name": "iic/Whisper-large-v3-turbo",
        "description": "Whisper的轻量化版本，速度更快但精度略有下降",
        "size": "约1.5GB",
        "parameters": "809M"
    },
    "13": {
        "name": "Qwen-Audio",
        "modelscope_name": "iic/Qwen-Audio",
        "description": "基于Qwen的音频理解模型，支持多种音频任务",
        "size": "约1.8GB",
        "parameters": "8B"
    },
    "14": {
        "name": "Qwen-Audio-Chat",
        "modelscope_name": "iic/Qwen-Audio-Chat",
        "description": "基于Qwen的音频对话模型，支持语音交互",
        "size": "约1.8GB",
        "parameters": "8B"
    },
    "15": {
        "name": "emotion2vec+large",
        "modelscope_name": "iic/emotion2vec+large",
        "description": "情感识别模型，用于识别语音中的情感",
        "size": "约1.2GB",
        "parameters": "300M"
    }
}

# 创建一个函数来获取模型信息，便于在其他脚本中使用
def get_model_info():
    """获取模型信息字典"""
    return MODEL_INFO

def get_model_by_id(model_id):
    """根据模型ID获取模型信息"""
    return MODEL_INFO.get(model_id)

def show_model_list():
    """显示模型列表"""
    print("=" * 80)
    print("📚 FunASR支持的模型列表:")
    print("=" * 80)
    for key, model in MODEL_INFO.items():
        print(f"{key}. {model['name']}")
        print(f"   描述: {model['description']}")
        print(f"   大小: {model['size']}， 参数量：{model['parameters']}")
        print("-" * 80)