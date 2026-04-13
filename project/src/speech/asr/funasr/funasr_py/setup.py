from setuptools import setup, find_packages
import os

package_name = 'funasr_py'

# 构建data_files配置
data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# 添加整个resources目录
resources_dir = os.path.join('..', 'resources')
if os.path.exists(resources_dir):
    # 使用递归方式添加整个resources目录
    for root, dirs, files in os.walk(resources_dir):
        # 过滤掉隐藏目录和临时目录
        dirs[:] = [d for d in dirs if not d.startswith('.') and not d.startswith('__')]
        
        # 构建目标目录路径 - share目录
        rel_root = os.path.relpath(root, '..')
        target_dir = os.path.join('share', package_name, rel_root)
        
        # 构建目标目录路径 - lib/share目录
        lib_target_dir = os.path.join('lib', 'share', package_name, rel_root)
        
        # 收集该目录下的所有文件
        file_paths = []
        for file in files:
            # 过滤掉隐藏文件
            if not file.startswith('.'):
                file_paths.append(os.path.join(root, file))
        
        # 如果有文件，添加到data_files
        if file_paths:
            data_files.append((target_dir, file_paths))
            data_files.append((lib_target_dir, file_paths))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    py_modules=['model_config'],  # 添加这一行以包含model_config.py模块
    data_files=data_files,
    install_requires=[
        'setuptools',
        'torch>=1.10.0',
        'torchvision>=0.11.0',
        'torchaudio>=0.10.0',
        'modelscope>=1.8.0',
        'funasr>=0.5.0',
        'soundfile>=0.12.0',
        'numpy>=1.21.0,<2.0.0',
        'scipy>=1.7.0',
        'pyaudio>=0.2.11',
        'PyYAML>=6.0',
        'onnx>=1.12.0',
        'onnxruntime>=1.12.0',
        'transformers>=4.0.0,<5.0.0',
        'datasets>=2.0.0,<3.0.0',
        'accelerate>=0.10.0',
        'tokenizers>=0.10.0',
        'sentencepiece>=0.1.96',
        'protobuf>=3.0.0',
        'huggingface_hub>=0.10.0',
        'addict>=2.4.0',
        'simplejson>=3.17.0',
        'resampy>=0.2.2',
        'webrtcvad>=2.0.10',
        'pyannote.audio>=2.0.0',
        'av>=9.0.0',
        'matplotlib>=3.5.0',
        'jupyter>=1.0.0',
        'numba>=0.55.0',
        'cython>=0.29.0',
        'rclpy>=3.0.0',
        'std_msgs>=1.0.0',
        'sensor_msgs>=1.0.0',
        'pyttsx3>=2.90',
        'audio_basic_py'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='FunASR Python Package for ROS Humble Environment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_model_download = funasr_py.tests.test_model_download:main',
            'test_model_export = funasr_py.tests.test_model_export:main',
            'test_model_loading = funasr_py.tests.test_model_loading:main',
            'test_asr_recognition = funasr_py.tests.test_asr_recognition:main',
            'test_audio_processing = funasr_py.tests.test_audio_processing:main',
        ],
    },
)