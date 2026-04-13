"""
Audio I/O Module
================

A cross-platform audio input/output module for recording and playback.

This module provides:
- Audio recording from microphone devices
- Audio playback to speaker devices  
- Device information and configuration
- Real-time audio processing capabilities

Examples:
    >>> from audio_io_module import AudioIOManager
    >>> manager = AudioIOManager()
    >>> # Start recording or playback
"""

from .audio_io_manager import AudioIOManager

__all__ = ['AudioIOManager']
__version__ = '0.0.1'