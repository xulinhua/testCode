from setuptools import setup, find_packages

setup(
    name="audio_io_module",
    version="0.0.1",
    packages=find_packages(),
    py_modules=['audio_io_manager', 'check_device_info', 'debug_audio_data'],
    install_requires=[
        "pyaudio>=0.2.11",
        "numpy>=1.21.0",
    ],
    author="user",
    author_email="user@example.com",
    description="Cross-platform audio I/O module for recording and playback",
    license="Apache License 2.0",
    python_requires=">=3.6",
)