# -*- coding: utf-8 -*-
"""扩展资源路径解析：机器人 USD、抓取盒 USD、包围盒 meta。"""

from __future__ import annotations

import json
import os
from typing import Optional, Tuple

# 场景物体类型：纸盒（现有 Texture OBJ）/ 料盒（cassette 扫描 mesh）
OBJECT_KIND_PAPER_BOX = "paper_box"
OBJECT_KIND_CASSETTE = "cassette"
OBJECT_KIND_LABELS = (
    (OBJECT_KIND_PAPER_BOX, "Paper box"),
    (OBJECT_KIND_CASSETTE, "Cassette"),
)


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


def normalize_object_kind(kind: Optional[str]) -> str:
    """归一化物体类型；非法值回退纸盒。"""
    key = (kind or OBJECT_KIND_PAPER_BOX).strip().lower()
    if key in ("cassette", "料盒", "liaohe", "bin"):
        return OBJECT_KIND_CASSETTE
    return OBJECT_KIND_PAPER_BOX


def resolve_object_asset_dir(box_dir: str, kind: Optional[str] = None) -> str:
    """按物体类型返回资源目录（含 OBJ / meta / 贴图）。

    - 纸盒：``data/box``（``model/`` + ``grasp_box_meta.json``）
    - 料盒：``data/box/cassette``（``mesh/`` + ``grasp_box_meta.json``）
    """
    root = os.path.abspath(box_dir)
    if normalize_object_kind(kind) == OBJECT_KIND_CASSETTE:
        cassette = os.path.join(root, "cassette")
        if os.path.isdir(cassette):
            return cassette
    return root


def find_box_obj_path(asset_dir: str) -> Optional[str]:
    """在 ``model/``、``mesh/`` 或资源根下查找第一个 ``.obj``。"""
    for sub in ("model", "mesh", ""):
        folder = os.path.join(asset_dir, sub) if sub else asset_dir
        if not os.path.isdir(folder):
            continue
        for name in sorted(os.listdir(folder)):
            if name.lower().endswith(".obj"):
                return os.path.join(folder, name)
    return None


def resolve_box_usd(box_dir: str) -> Optional[str]:
    """选择优先加载的抓取盒 USD（Isaac 转换版优先于 OBJ 包装）。

    Args:
        box_dir: 物体资源目录（纸盒为 ``data/box``）。

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

    优先读 ``model/*.mtl`` / ``mesh/*.mtl`` 的 ``map_Kd``；若 MTL 无贴图行，
    则回退到同目录 ``*.png`` / ``textures/*.png``。
    """
    search_dirs = []
    for sub in ("model", "mesh", "textures"):
        d = os.path.join(box_dir, sub)
        if os.path.isdir(d):
            search_dirs.append(d)
    if os.path.isdir(box_dir):
        search_dirs.append(box_dir)

    for folder in search_dirs:
        for mtl_name in sorted(os.listdir(folder)):
            if not mtl_name.endswith(".mtl"):
                continue
            mtl_path = os.path.join(folder, mtl_name)
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
                        for cand_dir in search_dirs:
                            tex_path = os.path.join(cand_dir, tex_name)
                            if os.path.isfile(tex_path):
                                return os.path.abspath(tex_path)
                        print(
                            f"resolve_box_texture: missing {tex_name} "
                            f"(referenced in {mtl_name})"
                        )
                        break
            except OSError:
                continue

    for folder in search_dirs:
        pngs = sorted(
            n for n in os.listdir(folder) if n.lower().endswith(".png")
        )
        if pngs:
            return os.path.abspath(os.path.join(folder, pngs[0]))
    return None


def ensure_box_texture_sidecar(box_dir: str) -> Optional[str]:
    """确保 ``<asset_dir>/textures/<png>`` 存在，供烘焙 USD 用相对路径引用。

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
    """读取包围盒 meta（碰撞尺寸、缩放等）。

    Args:
        box_dir: 物体资源目录（``data/box`` 或 ``data/box/cassette``）。

    Returns:
        解析后的 JSON 字典；文件缺失或损坏时返回 ``None``。
    """
    for name in ("grasp_box_meta.json", "cassette_meta.json"):
        path = os.path.join(box_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8") as handle:
                return json.load(handle)
        except (OSError, json.JSONDecodeError):
            continue
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



def object_kind_pref_path(box_dir: str) -> str:
    """UI 选择持久化文件（``data/box/ui_object_kind.json``）。"""
    return os.path.join(os.path.abspath(box_dir), "ui_object_kind.json")


def read_object_kind_pref(box_dir: str) -> str:
    """读取上次选择的物体类型；缺省纸盒。"""
    path = object_kind_pref_path(box_dir)
    if not os.path.isfile(path):
        return OBJECT_KIND_PAPER_BOX
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        return normalize_object_kind(data.get("kind") if isinstance(data, dict) else data)
    except (OSError, json.JSONDecodeError, TypeError):
        return OBJECT_KIND_PAPER_BOX


def write_object_kind_pref(box_dir: str, kind: Optional[str]) -> str:
    """写入物体类型选择；返回归一化后的 kind。"""
    key = normalize_object_kind(kind)
    path = object_kind_pref_path(box_dir)
    try:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump({"kind": key}, handle, indent=2, ensure_ascii=False)
            handle.write("\n")
    except OSError as exc:
        print(f"write_object_kind_pref: {exc}")
    return key

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
