from setuptools import find_packages, setup

package_name = 'udp_comm_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_client.launch.py']),
        ('share/' + package_name + '/config', ['config/udp_client_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='UDP Client for ROS 2 communication',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_client_node = udp_comm_client.udp_client_node:main',
        ],
    },
)