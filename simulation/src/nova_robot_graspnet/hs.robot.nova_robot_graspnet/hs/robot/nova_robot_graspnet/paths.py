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
    """解析抓取盒漫反射贴图 PNG 绝对路径。

    优先读 ``model/*.mtl`` 的 ``map_Kd``；若 MTL 无贴图行（常见于 trimesh 导出），
    则回退到 ``model/*.png`` / ``textures/*.png``。
    """
    model_dir = os.path.join(box_dir, "model")
    tex_dir = os.path.join(box_dir, "textures")

    if os.path.isdir(model_dir):
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
                        tex_name = os.path.basename(parts[-1])
                        for folder in (model_dir, tex_dir):
                            tex_path = os.path.join(folder, tex_name)
                            if os.path.isfile(tex_path):
                                return os.path.abspath(tex_path)
                        print(
                            f"resolve_box_texture: missing {tex_name} "
                            f"(referenced in {mtl_name})"
                        )
                        break
            except OSError:
                continue

        pngs = sorted(
            n for n in os.listdir(model_dir) if n.lower().endswith(".png")
        )
        if pngs:
            return os.path.abspath(os.path.join(model_dir, pngs[0]))

    if os.path.isdir(tex_dir):
        pngs = sorted(n for n in os.listdir(tex_dir) if n.lower().endswith(".png"))
        if pngs:
            return os.path.abspath(os.path.join(tex_dir, pngs[0]))
    return None


def ensure_box_texture_sidecar(box_dir: str) -> Optional[str]:
    """确保 ``data/box/textures/<png>`` 存在，供烘焙 USD 用相对路径引用。

    Returns:
        相对 ``box_dir`` 的资产路径（如 ``./textures/foo.png``）；无贴图时 ``None``。
    """
    src = resolve_box_texture_path(box_dir)
    if not src:
        return None
    name = os.path.basename(src)
    tex_dir = os.path.join(box_dir, "textures")
    os.makedirs(tex_dir, exist_ok=True)
    dst = os.path.join(tex_dir, name)
    if not os.path.isfile(dst) and not os.path.islink(dst):
        try:
            os.symlink(os.path.relpath(src, tex_dir), dst)
        except OSError:
            import shutil

            shutil.copy2(src, dst)
    return f"./textures/{name}"


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


def resolve_env_usd(data_dir: str) -> Optional[str]:
    """Isaac Grid 地面环境（``download_environment.py`` 缓存到 ``data/env/``）。

    Args:
        data_dir: ``data`` 目录。

    Returns:
        ``default_environment.usd`` 绝对路径；未下载时 ``None``。
    """
    path = os.path.join(data_dir, "env", "default_environment.usd")
    return path if os.path.isfile(path) else None


def resolve_scene_usd(data_dir: str) -> Optional[str]:
    """采集对齐的总场景（``scripts/bake_graspnet_scene.py`` 生成）。

    Args:
        data_dir: ``data`` 目录。

    Returns:
        ``scenes/nova_graspnet_scene.usda`` 绝对路径；未烘焙时 ``None``。
    """
    for name in ("nova_graspnet_scene.usda", "nova_graspnet_scene.usd"):
        path = os.path.join(data_dir, "scenes", name)
        if os.path.isfile(path):
            return path
    return None


def default_robot_dir() -> str:
    """扩展内 ``data/robot`` 目录。

    ``paths.py`` 位于 ``<ext>/hs/robot/nova_robot_graspnet/``，需上溯 4 级到扩展根。
    """
    cur = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        candidate = os.path.join(cur, "data", "robot")
        if os.path.isdir(candidate):
            return candidate
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    # 回退：固定 4 级上溯（与 get_extension_paths(ext_path) 布局一致）
    ext_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    return os.path.join(ext_root, "data", "robot")


def resolve_robot_urdf(robot_dir: Optional[str] = None) -> Optional[str]:
    """Nova 机器人 URDF（优先 position 版）。"""
    robot_dir = robot_dir or default_robot_dir()
    for name in ("nova_robot_position.urdf", "nova_robot.urdf"):
        path = os.path.join(robot_dir, name)
        if os.path.isfile(path):
            return path
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
