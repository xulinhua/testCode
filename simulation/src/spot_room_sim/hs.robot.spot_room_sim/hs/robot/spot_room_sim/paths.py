# -*- coding: utf-8 -*-
"""扩展路径与本地资产解析。"""

from __future__ import annotations

import os
from typing import Optional, Tuple


def get_extension_root() -> str:
    """hs/.../spot_room_sim -> 扩展根（含 config/data）。"""
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def get_extension_paths(ext_path: str) -> Tuple[str, str]:
    """返回 (extension_root, data_dir)。"""
    root = os.path.abspath(ext_path)
    data = os.path.join(root, "data")
    os.makedirs(data, exist_ok=True)
    return root, data


def get_data_dir(ext_root: Optional[str] = None) -> str:
    root = ext_root or get_extension_root()
    data = os.path.join(root, "data")
    os.makedirs(data, exist_ok=True)
    return data


def local_simple_room_usd(ext_root: Optional[str] = None) -> Optional[str]:
    path = os.path.join(get_data_dir(ext_root), "scenes", "simple_room", "simple_room.usd")
    return path if os.path.isfile(path) and os.path.getsize(path) > 1000 else None


def simple_room_textures_are_lite(ext_root: Optional[str] = None) -> bool:
    marker = os.path.join(get_data_dir(ext_root), "scenes", "simple_room", ".lite_textures")
    return os.path.isfile(marker)


def local_spot_usd(ext_root: Optional[str] = None) -> Optional[str]:
    path = os.path.join(get_data_dir(ext_root), "robots", "spot", "spot.usd")
    return path if os.path.isfile(path) and os.path.getsize(path) > 1000 else None


def local_spot_policy_files(ext_root: Optional[str] = None) -> Tuple[Optional[str], Optional[str]]:
    base = os.path.join(get_data_dir(ext_root), "policies", "spot")
    pt = os.path.join(base, "spot_policy.pt")
    yaml = os.path.join(base, "spot_env.yaml")
    pt_ok = pt if os.path.isfile(pt) and os.path.getsize(pt) > 1000 else None
    yaml_ok = yaml if os.path.isfile(yaml) and os.path.getsize(yaml) > 10 else None
    return pt_ok, yaml_ok


def assets_ready(ext_root: Optional[str] = None) -> bool:
    room = local_simple_room_usd(ext_root)
    spot = local_spot_usd(ext_root)
    pt, yaml = local_spot_policy_files(ext_root)
    return bool(room and spot and pt and yaml)
