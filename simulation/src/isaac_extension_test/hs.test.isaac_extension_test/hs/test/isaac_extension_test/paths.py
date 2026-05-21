# Copyright (c) 2024
"""扩展资源路径：仅使用本扩展包内的 data/。"""

import os


def get_extension_paths(ext_path: str):
    """
    返回 (ext_root, data_dir, raw_data_dir)。

    所有 USD/OBJ/贴图必须位于 <扩展根>/data/raw_data/，不依赖其他扩展目录。
    """
    ext_root = os.path.abspath(ext_path)
    data_dir = os.path.join(ext_root, "data")
    raw_data = os.path.join(data_dir, "raw_data")
    return ext_root, data_dir, raw_data
