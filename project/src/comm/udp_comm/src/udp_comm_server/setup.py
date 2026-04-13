from setuptools import find_packages, setup

package_name = 'udp_comm_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_server.launch.py']),
        ('share/' + package_name + '/config', ['config/udp_server_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='UDP Server for ROS 2 communication',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_server_node = udp_comm_server.udp_server_node:main',
        ],
    },
)