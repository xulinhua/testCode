from setuptools import setup
import os
from glob import glob

package_name = 'udp_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含launch文件和配置文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hs',
    maintainer_email='hs@example.com',
    description='UDP client for sending data from Jetson Orin NX',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_client_node = udp_client.udp_client_node:main',
        ],
    },
)
