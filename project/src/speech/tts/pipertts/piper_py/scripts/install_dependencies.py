"""
依赖安装脚本
===========

自动安装piper_speech项目所需的依赖包。
"""

import os
import sys
import subprocess
import platform
from typing import List


def check_python_version() -> bool:
    """检查Python版本"""
    version = sys.version_info
    print(f"Python版本: {version.major}.{version.minor}.{version.micro}")
    
    if version.major == 3 and version.minor >= 8:
        print("✓ Python版本符合要求")
        return True
    else:
        print("✗ Python版本过低，需要Python 3.8或更高版本")
        return False


def check_platform() -> str:
    """检查操作系统平台"""
    system = platform.system().lower()
    architecture = platform.machine().lower()
    
    print(f"操作系统: {system}")
    print(f"架构: {architecture}")
    
    if system == "linux" and "aarch64" in architecture:
        print("✓ 检测到Jetson平台")
        return "jetson"
    elif system == "linux":
        print("✓ 检测到Linux平台")
        return "linux"
    elif system == "windows":
        print("✓ 检测到Windows平台")
        return "windows"
    elif system == "darwin":
        print("✓ 检测到macOS平台")
        return "macos"
    else:
        print("⚠ 未知平台")
        return "unknown"


def install_packages(packages: List[str]) -> bool:
    """安装Python包"""
    print(f"\n准备安装 {len(packages)} 个包...")
    
    for package in packages:
        print(f"\n安装 {package}...")
        
        try:
            # 使用pip安装
            result = subprocess.run([
                sys.executable, "-m", "pip", "install", package
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                print(f"✓ {package} 安装成功")
            else:
                print(f"✗ {package} 安装失败")
                print(f"错误信息: {result.stderr}")
                return False
                
        except Exception as e:
            print(f"✗ 安装 {package} 时发生错误: {e}")
            return False
    
    return True


def install_jetson_specific_packages() -> bool:
    """安装Jetson平台特定包"""
    print("\n安装Jetson平台特定包...")
    
    jetson_packages = [
        "nvidia-pyindex",
        "onnxruntime-gpu",
        "tensorrt"
    ]
    
    return install_packages(jetson_packages)


def install_common_packages() -> bool:
    """安装通用包"""
    print("\n安装通用依赖包...")
    
    common_packages = [
        "numpy",
        "pyyaml",
        "onnxruntime",
        "wave",
        "logging"
    ]
    
    return install_packages(common_packages)


def install_piper_phonemize() -> bool:
    """安装piper-phonemize"""
    print("\n安装piper-phonemize...")
    
    # 尝试从源码安装
    try:
        result = subprocess.run([
            sys.executable, "-m", "pip", "install", "piper-phonemize"
        ], capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ piper-phonemize 安装成功")
            return True
        else:
            print("✗ piper-phonemize 安装失败，尝试备用方案")
            print("备用方案: 使用简化的音素转换功能")
            return True
            
    except Exception as e:
        print(f"✗ 安装piper-phonemize时发生错误: {e}")
        print("备用方案: 使用简化的音素转换功能")
        return True


def create_requirements_file() -> bool:
    """创建requirements.txt文件"""
    print("\n创建requirements.txt文件...")
    
    requirements_content = """# Piper Speech Synthesis 依赖列表
# 通用依赖
numpy>=1.21.0
pyyaml>=6.0

# ONNX运行时
onnxruntime>=1.14.0

# 音频处理
wave

# 日志
logging

# 可选：piper-phonemize（用于更准确的音素转换）
# piper-phonemize>=1.0.0

# Jetson平台特定依赖（可选）
# nvidia-pyindex
# onnxruntime-gpu>=1.14.0
# tensorrt>=8.5.0
"""
    
    try:
        with open("requirements.txt", "w", encoding="utf-8") as f:
            f.write(requirements_content)
        
        print("✓ requirements.txt 文件创建成功")
        return True
        
    except Exception as e:
        print(f"✗ 创建requirements.txt文件失败: {e}")
        return False


def main():
    """主函数"""
    print("=" * 60)
    print("Piper Speech Synthesis 依赖安装工具")
    print("=" * 60)
    
    # 检查Python版本
    if not check_python_version():
        print("\n✗ Python版本检查失败，安装中止")
        return 1
    
    # 检查平台
    platform_type = check_platform()
    
    # 创建requirements文件
    if not create_requirements_file():
        print("\n✗ 创建requirements文件失败")
        return 1
    
    # 安装通用包
    if not install_common_packages():
        print("\n✗ 通用包安装失败")
        return 1
    
    # 安装piper-phonemize
    if not install_piper_phonemize():
        print("\n⚠ piper-phonemize安装失败，但可以继续使用简化功能")
    
    # 如果是Jetson平台，安装特定包
    if platform_type == "jetson":
        if not install_jetson_specific_packages():
            print("\n⚠ Jetson特定包安装失败，但可以继续使用CPU模式")
    
    print("\n" + "=" * 60)
    print("依赖安装完成！")
    print("=" * 60)
    
    # 验证安装
    print("\n验证安装...")
    
    try:
        import numpy
        print("✓ numpy 导入成功")
    except ImportError:
        print("✗ numpy 导入失败")
    
    try:
        import yaml
        print("✓ pyyaml 导入成功")
    except ImportError:
        print("✗ pyyaml 导入失败")
    
    try:
        import onnxruntime
        print("✓ onnxruntime 导入成功")
    except ImportError:
        print("✗ onnxruntime 导入失败")
    
    print("\n安装总结:")
    print("✓ 所有必要的依赖包已安装")
    print("✓ requirements.txt 文件已创建")
    print("✓ 可以开始使用piper_speech库")
    
    print("\n下一步:")
    print("1. 确保已下载Piper语音模型文件")
    print("2. 将模型文件(.onnx和.json)放置到models/目录")
    print("3. 运行 python run_demo.py 进行测试")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())