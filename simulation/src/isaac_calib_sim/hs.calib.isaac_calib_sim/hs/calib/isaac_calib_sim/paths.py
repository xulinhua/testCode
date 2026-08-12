# -*- coding: utf-8 -*-
"""扩展路径解析。"""

from __future__ import annotations

import os
from typing import Tuple


def get_extension_paths(ext_path: str) -> Tuple[str, str, str]:
    """返回 (ext_root, data_dir, texture_dir)。"""
    ext_root = os.path.abspath(ext_path)
    data_dir = os.path.join(ext_root, "data")
    texture_dir = os.path.join(data_dir, "textures")
    os.makedirs(texture_dir, exist_ok=True)
    return ext_root, data_dir, texture_dir
