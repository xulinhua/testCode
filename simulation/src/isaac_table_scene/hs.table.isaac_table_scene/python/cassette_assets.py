# -*- coding: utf-8 -*-
"""cassette 3D 扫描 mesh 路径、包围盒与 OBJ 解析。"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional, Tuple


# 20260518.obj 顶点包围盒（米，Blender 导出坐标系）
CASSETTE_BBOX_MIN = (-0.126950562, -0.047496078, -0.097422432)
CASSETTE_BBOX_MAX = (0.139123199, 0.041768234, 0.083815857)
CASSETTE_BBOX_CENTER = (
    (CASSETTE_BBOX_MIN[0] + CASSETTE_BBOX_MAX[0]) * 0.5,
    (CASSETTE_BBOX_MIN[1] + CASSETTE_BBOX_MAX[1]) * 0.5,
    (CASSETTE_BBOX_MIN[2] + CASSETTE_BBOX_MAX[2]) * 0.5,
)
CASSETTE_BBOX_SIZE = (
    CASSETTE_BBOX_MAX[0] - CASSETTE_BBOX_MIN[0],
    CASSETTE_BBOX_MAX[1] - CASSETTE_BBOX_MIN[1],
    CASSETTE_BBOX_MAX[2] - CASSETTE_BBOX_MIN[2],
)

CASSETTE_OBJ_REL = os.path.join("cassette", "mesh", "20260518.obj")
CASSETTE_USD_REL = os.path.join("data", "cassette.usd")
CASSETTE_MTL_REL = os.path.join("cassette", "mesh", "20260518.mtl")
CASSETTE_TEX_REL = os.path.join("cassette", "mesh", "20260518.png")


@dataclass(frozen=True)
class CassettePaths:
    obj_path: str
    usd_path: str
    mtl_path: str
    texture_path: str
    mesh_dir: str


@dataclass(frozen=True)
class CassetteMeshData:
    points: List[Tuple[float, float, float]]
    face_vertex_counts: List[int]
    face_vertex_indices: List[int]
    uv_points: List[Tuple[float, float]]
    face_uv_indices: List[int]


def resolve_cassette_paths(ext_root: str) -> CassettePaths:
    root = os.path.abspath(ext_root)
    mesh_dir = os.path.join(root, "cassette", "mesh")
    return CassettePaths(
        obj_path=os.path.join(root, CASSETTE_OBJ_REL),
        usd_path=os.path.join(root, CASSETTE_USD_REL),
        mtl_path=os.path.join(root, CASSETTE_MTL_REL),
        texture_path=os.path.join(root, CASSETTE_TEX_REL),
        mesh_dir=mesh_dir,
    )


def geometry_centering_offset() -> Tuple[float, float, float]:
    """link 下 geometry 平移：底面中心落在 link 原点（1:1，无缩放）。"""
    cx, cy, _ = CASSETTE_BBOX_CENTER
    z_min = CASSETTE_BBOX_MIN[2]
    return (-cx, -cy, -z_min)


def center_height_in_link() -> float:
    """mesh 几何中心相对 link 原点的 +Z 高度。"""
    return CASSETTE_BBOX_CENTER[2] - CASSETTE_BBOX_MIN[2]


def parse_obj_mesh(obj_path: str) -> CassetteMeshData:
    """解析 OBJ 顶点、三角面与 UV（faceVarying）。"""
    points: List[Tuple[float, float, float]] = []
    uv_points: List[Tuple[float, float]] = []
    face_vertex_counts: List[int] = []
    face_vertex_indices: List[int] = []
    face_uv_indices: List[int] = []

    if not os.path.isfile(obj_path):
        raise FileNotFoundError(f"cassette OBJ not found: {obj_path}")

    with open(obj_path, "r", encoding="utf-8", errors="ignore") as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                parts = line.split()
                points.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("vt "):
                parts = line.split()
                uv_points.append((float(parts[1]), float(parts[2])))
            elif line.startswith("f "):
                vidxs: List[int] = []
                vtidxs: List[int] = []
                for token in line.split()[1:]:
                    chunks = token.split("/")
                    vidxs.append(int(chunks[0]) - 1)
                    if len(chunks) > 1 and chunks[1]:
                        vtidxs.append(int(chunks[1]) - 1)
                    else:
                        vtidxs.append(0)
                if len(vidxs) < 3:
                    continue
                for i in range(1, len(vidxs) - 1):
                    face_vertex_counts.append(3)
                    face_vertex_indices.extend([vidxs[0], vidxs[i], vidxs[i + 1]])
                    face_uv_indices.extend([vtidxs[0], vtidxs[i], vtidxs[i + 1]])

    if not points or not face_vertex_indices:
        raise ValueError(f"no mesh data in {obj_path}")
    if not uv_points:
        uv_points = [(0.0, 0.0)]
        face_uv_indices = [0] * len(face_vertex_indices)
    return CassetteMeshData(
        points=points,
        face_vertex_counts=face_vertex_counts,
        face_vertex_indices=face_vertex_indices,
        uv_points=uv_points,
        face_uv_indices=face_uv_indices,
    )


def ensure_cassette_usd(ext_root: str) -> Optional[str]:
    """若已有 cassette.usd 则返回路径，否则 None。"""
    paths = resolve_cassette_paths(ext_root)
    if os.path.isfile(paths.usd_path):
        return paths.usd_path
    return None
