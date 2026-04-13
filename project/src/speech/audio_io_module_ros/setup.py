from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'audio_io_module_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.', include=['audio_io_module_ros', 'audio_io_module_ros.*']),
    package_data={'audio_io_module_ros': ['msg/*']},
    data_files=[
        # 标准ROS 2包索引文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # 包描述文件
        ('share/' + package_name, ['package.xml']),
        
        # 启动文件
        ('share/' + package_name + '/launch', 
            glob('launch/*.py')),
        
        # 配置文件
        ('share/' + package_name + '/config', 
            glob('config/*.yaml') + glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2 wrapper for audio I/O module',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_publisher = audio_io_module_ros.audio_publisher:main',
            'audio_subscriber = audio_io_module_ros.audio_subscriber:main',
        ],
    },
)