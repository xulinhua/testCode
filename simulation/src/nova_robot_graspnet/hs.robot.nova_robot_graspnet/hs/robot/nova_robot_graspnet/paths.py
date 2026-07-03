# -*- coding: utf-8 -*-
"""扩展资源路径解析：机器人 USD、抓取盒 USD、包围盒 meta。"""

from __future__ import annotations

import json
import os
from typing import Optional, Tuple


def get_extension_paths(ext_path: str) -> Tuple[str, str, str, str]:
    """根据扩展根目录推导 data 子路径。

    Args:
        ext_path: ``hs.robot.nova_robot_graspnet`` 扩展安装目录。

    Returns:
        ``(ext_root, data_dir, robot_dir, box_dir)`` 绝对路径元组。
    """
    ext_root = os.path.abspath(ext_path)
    data_dir = os.path.join(ext_root, "data")
    robot_dir = os.path.join(data_dir, "robot")
    box_dir = os.path.join(data_dir, "box")
    return ext_root, data_dir, robot_dir, box_dir


def resolve_box_usd(box_dir: str) -> Optional[str]:
    """选择优先加载的抓取盒 USD（Isaac 转换版优先于 OBJ 包装）。

    Args:
        box_dir: ``data/box`` 目录。

    Returns:
        存在的 ``grasp_box_prepared.usda`` 或 ``grasp_box.usda`` 绝对路径；否则 ``None``。
    """
    for name in ("grasp_box_prepared.usda", "grasp_box.usda"):
        path = os.path.join(box_dir, name)
        if os.path.isfile(path):
            return path
    return None


def resolve_box_texture_path(box_dir: str) -> Optional[str]:
    """从 ``model/*.mtl`` 的 ``map_Kd`` 解析贴图 PNG 绝对路径。

    Args:
        box_dir: ``data/box`` 目录。

    Returns:
        存在的贴图文件绝对路径；缺失时 ``None``。
    """
    model_dir = os.path.join(box_dir, "model")
    if not os.path.isdir(model_dir):
        return None

    for mtl_name in sorted(os.listdir(model_dir)):
        if not mtl_name.endswith(".mtl"):
            continue
        mtl_path = os.path.join(model_dir, mtl_name)
        try:
            with open(mtl_path, encoding="utf-8", errors="ignore") as handle:
                for line in handle:
                    stripped = line.strip()
                    if not stripped.startswith("map_Kd"):
                        continue
                    parts = stripped.split()
                    if len(parts) < 2:
                        continue
                    tex_name = parts[-1]
                    tex_path = os.path.join(model_dir, os.path.basename(tex_name))
                    if os.path.isfile(tex_path):
                        return os.path.abspath(tex_path)
                    print(
                        f"resolve_box_texture: missing {tex_path} "
                        f"(referenced in {mtl_name})"
                    )
                    return None
        except OSError:
            continue
    return None


def load_box_meta(box_dir: str) -> Optional[dict]:
    """读取 ``prepare_box_usd.py`` 生成的包围盒 meta（碰撞尺寸、缩放等）。

    Args:
        box_dir: ``data/box`` 目录。

    Returns:
        解析后的 JSON 字典；文件缺失或损坏时返回 ``None``。
    """
    path = os.path.join(box_dir, "grasp_box_meta.json")
    if not os.path.isfile(path):
        return None
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError):
        return None


def resolve_robot_usd(robot_dir: str) -> Optional[str]:
    """选择 Nova 机器人 USD（预处理版优先）。

    Args:
        robot_dir: ``data/robot`` 目录。

    Returns:
        ``nova_robot_prepared.usda`` 或 ``nova_robot.usda`` 绝对路径；否则 ``None``。
    """
    prepared = os.path.join(robot_dir, "nova_robot_prepared.usda")
    if os.path.isfile(prepared):
        return prepared
    path = os.path.join(robot_dir, "nova_robot.usda")
    return path if os.path.isfile(path) else None
