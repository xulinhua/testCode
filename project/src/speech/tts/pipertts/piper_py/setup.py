#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Piper Speech - 语音合成系统
安装配置脚本
"""

from setuptools import setup, find_packages
import os

# 读取项目描述
with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

# 读取依赖要求
with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

# 包信息
setup(
    name="piper_py",  # 修改包名为piper_py以匹配项目结构
    version="1.0.1",
    author="Piper Speech Team",
    author_email="piper-speech@example.com",
    description="基于Piper TTS引擎的高性能语音合成系统，专为Jetson Orin NX Super优化",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/example/piper-speech",
    packages=find_packages(),  # 修复包查找路径
    package_dir={"": "."},  # 保持正确的包目录结构
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Operating System :: POSIX :: Linux",
        "Topic :: Multimedia :: Sound/Audio :: Speech",
    ],
    python_requires=">=3.10",
    install_requires=requirements,
    entry_points={
        "console_scripts": [
            "piper-speech=piper_py.tts_engine:main",
            "piper-tts=piper_py.tts_engine:main",
        ],
    },
    include_package_data=True,
    package_data={
        "piper_py": [
            "config/*.yaml",
        ],
    },
    # 针对Jetson平台的额外依赖
    extras_require={
        "jetson": [
            "onnxruntime-gpu>=1.14.0",
            "jetson-stats>=3.1.0",
        ],
        "gui": [
            "pyqt5>=5.15.0",
            "pyqt5-tools>=5.15.0",
        ],
        # 添加piper_phonemize作为可选依赖
        "phonemize": [
            "piper-phonemize>=1.1.0,<2.0.0",
        ],
    },
)