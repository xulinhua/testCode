from setuptools import find_packages, setup

package_name = 'snowboy_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/snowboy_python.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 wrapper for Snowboy voice wakeup detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'snowboy_ros = snowboy_ros.fixed_demo:main',
        ],
    },
)

from setuptools import setup
import os
from glob import glob

