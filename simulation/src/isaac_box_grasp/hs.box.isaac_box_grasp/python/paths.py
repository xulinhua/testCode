# Copyright (c) 2024
"""扩展资源路径。"""

import os
from typing import Optional


def get_extension_paths(ext_path: str):
    """
    返回 (ext_root, data_dir, scenes_dir, robots_dir, calib_output_dir)。
    """
    ext_root = os.path.abspath(ext_path)
    data_dir = os.path.join(ext_root, "data")
    scenes_dir = os.path.join(data_dir, "scenes")
    robots_dir = os.path.join(data_dir, "robots")
    calib_output_dir = os.path.join(ext_root, "calib_output")
    return ext_root, data_dir, scenes_dir, robots_dir, calib_output_dir


def resolve_asset_file(name: str, base_dir: str) -> Optional[str]:
    """将 UI 文件名或绝对路径解析为存在的文件路径；空串返回 None。"""
    if not name or not str(name).strip():
        return None
    name = str(name).strip()
    if os.path.isabs(name):
        return name if os.path.isfile(name) else None
    path = os.path.join(base_dir, name)
    return path if os.path.isfile(path) else None
