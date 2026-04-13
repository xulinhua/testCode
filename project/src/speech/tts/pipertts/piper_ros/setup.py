from setuptools import setup, find_packages

package_name = 'piper_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            []),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tts_node.launch.py']),
        ('share/' + package_name + '/config', ['config/tts_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS wrapper for Piper text-to-speech engine',
    license='Apache License 2.0',
    tests_require=['pytest'],
    # 移除entry_points，因为通过CMakeLists.txt安装可执行文件
)