from setuptools import setup
import glob
import os

package_name = 'snowboy_python'

# 查找所有需要包含的文件
# data_files = []

data_files = [
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
    packages=[package_name],
    data_files=data_files,
    # 明确包含所有文件
    package_data={
        package_name: ['*.so', '*.py', '*.pmdl', '*.umdl', '*.res'],
    },
    include_package_data=True,
    install_requires=['setuptools', 'pyaudio', 'numpy'],
    zip_safe=False,  # 重要：必须为 False
    maintainer='user',
    maintainer_email='user@example.com',
    description='Snowboy voice wakeup detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'snowboy_python = snowboy_python.fixed_demo:main',
        ],
    },
)