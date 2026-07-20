# -*- coding: utf-8 -*-
"""从 URDF 引用的 STL 解析包围盒（与 base_link 落物面高度对齐）。"""

from __future__ import annotations

import os
import re
import struct
import xml.etree.ElementTree as ET
from functools import lru_cache
from typing import Optional, Tuple

from .paths import default_robot_dir, resolve_robot_urdf


def _stl_axis_bounds(stl_path: str) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    with open(stl_path, "rb") as handle:
        header = handle.read(80)
        data = handle.read()

    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []

    if b"facet" in data[:400].lower() or b"solid" in header.lower():
        text = (header + data).decode("latin-1", errors="ignore")
        for x, y, z in re.findall(
            r"vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", text
        ):
            xs.append(float(x))
            ys.append(float(y))
            zs.append(float(z))
    else:
        if len(data) < 4:
            raise ValueError(f"STL too small: {stl_path}")
        tri_count = struct.unpack("<I", data[:4])[0]
        offset = 4
        for _ in range(tri_count):
            offset += 12
            for __ in range(3):
                x, y, z = struct.unpack("<fff", data[offset : offset + 12])
                xs.append(float(x))
                ys.append(float(y))
                zs.append(float(z))
                offset += 12
            offset += 2

    if not xs:
        raise ValueError(f"no vertices in STL: {stl_path}")

    return (
        (min(xs), max(xs)),
        (min(ys), max(ys)),
        (min(zs), max(zs)),
    )


def _resolve_mesh_path(robot_dir: str, mesh_ref: str) -> str:
    ref = mesh_ref.strip()
    for prefix in ("file://", "package://nova_sim/", "package://nova_sim_mujoco/"):
        if ref.startswith(prefix):
            ref = ref[len(prefix) :]
    if os.path.isabs(ref) and os.path.isfile(ref):
        return ref
    candidate = os.path.join(robot_dir, ref)
    if os.path.isfile(candidate):
        return candidate
    candidate = os.path.join(robot_dir, "meshes", os.path.basename(ref))
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(f"mesh not found for {mesh_ref!r} under {robot_dir}")


def _parse_urdf_mesh_path(urdf_path: str, link_name: str) -> str:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for link in root.findall("link"):
        if link.get("name") != link_name:
            continue
        for tag in ("visual", "collision"):
            node = link.find(tag)
            if node is None:
                continue
            mesh = node.find("./geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename")
            if filename:
                robot_dir = os.path.dirname(os.path.abspath(urdf_path))
                return _resolve_mesh_path(robot_dir, filename)
    raise ValueError(f"link {link_name!r} has no mesh in {urdf_path}")


@lru_cache(maxsize=4)
def base_link_stl_z_bounds(
    robot_dir: Optional[str] = None,
) -> Tuple[float, float]:
    """``base_link`` STL 在 link 坐标系下的 (min_z, max_z)。"""
    robot_dir = robot_dir or default_robot_dir()
    urdf_path = resolve_robot_urdf(robot_dir)
    if not urdf_path:
        raise FileNotFoundError(f"robot URDF missing under {robot_dir}")
    stl_path = _parse_urdf_mesh_path(urdf_path, "base_link")
    _, _, z_rng = _stl_axis_bounds(stl_path)
    return float(z_rng[0]), float(z_rng[1])
