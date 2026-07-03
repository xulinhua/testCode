# Copyright (c) 2024
"""扩展资源路径。"""

import os
from typing import Optional


def get_extension_paths(ext_path: str):
    ext_root = os.path.abspath(ext_path)
    data_dir = os.path.join(ext_root, "data")
    raw_data_dir = os.path.join(data_dir, "raw_data")
    return ext_root, data_dir, raw_data_dir


def resolve_mz_dir(raw_data_dir: str) -> Optional[str]:
    """解析 mz 桌子资源目录：本扩展 data/raw_data/mz，或回退到参考项目。"""
    candidates = [
        os.path.join(raw_data_dir, "mz"),
        os.path.abspath(
            os.path.join(
                raw_data_dir,
                "..",
                "..",
                "..",
                "isaac_extension_test",
                "hs.test.isaac_extension_test",
                "data",
                "raw_data",
                "mz",
            )
        ),
        os.path.abspath(
            os.path.join(
                raw_data_dir,
                "..",
                "..",
                "..",
                "data_collect_isaac",
                "IsaacsimExtGrasping",
                "data",
                "raw_data",
                "mz",
            )
        ),
    ]
    for path in candidates:
        usd = os.path.join(path, "mz.usd")
        if os.path.isfile(usd):
            return path
    return None
