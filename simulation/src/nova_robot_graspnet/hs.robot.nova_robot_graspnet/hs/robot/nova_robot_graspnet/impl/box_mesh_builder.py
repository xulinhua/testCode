# -*- coding: utf-8 -*-
"""从 OBJ 在 Stage 上直接构建带 UV 的抓取盒 mesh（不依赖 Isaac OBJ 引用）。"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from pxr import Gf, Sdf, UsdGeom, UsdShade

from ..global_variables import BOX_LINK_PATH, BOX_VISUAL_PATH
from ..paths import load_box_meta, resolve_box_texture_path


@dataclass
class BoxMeshData:
    points: List[Tuple[float, float, float]]
    face_vertex_counts: List[int]
    face_vertex_indices: List[int]
    uv_points: List[Tuple[float, float]]
    face_uv_indices: List[int]


def parse_box_obj(obj_path: str, box_dir: str) -> BoxMeshData:
    """解析 OBJ：mm→m、几何居中（与 prepare_box_usd meta 一致）。"""
    meta = load_box_meta(box_dir) or {}
    scale = float(meta.get("scale", 0.001))
    center = meta.get("center_m", {})
    cx = float(center.get("x", 0.0))
    cy = float(center.get("y", 0.0))
    cz = float(center.get("z", 0.0))

    points_mm: List[Tuple[float, float, float]] = []
    uv_points: List[Tuple[float, float]] = []
    face_vertex_counts: List[int] = []
    face_vertex_indices: List[int] = []
    face_uv_indices: List[int] = []

    if not os.path.isfile(obj_path):
        raise FileNotFoundError(f"box OBJ not found: {obj_path}")

    t0 = time.time()
    with open(obj_path, "r", encoding="utf-8", errors="ignore") as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("v "):
                parts = line.split()
                points_mm.append((float(parts[1]), float(parts[2]), float(parts[3])))
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

    if not points_mm or not face_vertex_indices:
        raise ValueError(f"no mesh data in {obj_path}")

    points = [
        (
            p[0] * scale - cx,
            p[1] * scale - cy,
            p[2] * scale - cz,
        )
        for p in points_mm
    ]

    if not uv_points:
        uv_points = [(0.0, 0.0)]
        face_uv_indices = [0] * len(face_vertex_indices)

    elapsed = time.time() - t0
    print(
        f"box_mesh_builder: parsed OBJ {len(points)} verts, "
        f"{len(face_vertex_counts)} tris in {elapsed:.1f}s"
    )
    return BoxMeshData(
        points=points,
        face_vertex_counts=face_vertex_counts,
        face_vertex_indices=face_vertex_indices,
        uv_points=uv_points,
        face_uv_indices=face_uv_indices,
    )


def _compute_vertex_normals(
    points: List[Tuple[float, float, float]],
    face_counts: List[int],
    face_indices: List[int],
) -> List[Gf.Vec3f]:
    """按三角面累加顶点法线（RTX 无 normals 时会渲染成发黑）。"""
    accum = [[0.0, 0.0, 0.0] for _ in points]
    cursor = 0
    for count in face_counts:
        if count < 3:
            cursor += count
            continue
        corner = face_indices[cursor : cursor + count]
        cursor += count
        for i in range(1, count - 1):
            i0, i1, i2 = corner[0], corner[i], corner[i + 1]
            p0, p1, p2 = points[i0], points[i1], points[i2]
            e1 = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
            e2 = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])
            nx = e1[1] * e2[2] - e1[2] * e2[1]
            ny = e1[2] * e2[0] - e1[0] * e2[2]
            nz = e1[0] * e2[1] - e1[1] * e2[0]
            for vi in (i0, i1, i2):
                accum[vi][0] += nx
                accum[vi][1] += ny
                accum[vi][2] += nz

    normals: List[Gf.Vec3f] = []
    for n in accum:
        length = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5
        if length < 1e-8:
            normals.append(Gf.Vec3f(0.0, 0.0, 1.0))
        else:
            normals.append(Gf.Vec3f(n[0] / length, n[1] / length, n[2] / length))
    return normals


def _apply_material(
    stage,
    mesh_path: str,
    texture_path: Optional[str],
    looks_root: str,
) -> None:
    mat_path = f"{looks_root}/box_material"
    material = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)

    if texture_path and os.path.isfile(texture_path):
        tex_abs = os.path.abspath(texture_path).replace("\\", "/")
        st_reader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/stReader"))
        st_reader.CreateIdAttr("UsdPrimvarReader_float2")
        st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        st_reader.CreateInput("fallback", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.0, 0.0))

        tex_shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/DiffuseTexture"))
        tex_shader.CreateIdAttr("UsdUVTexture")
        tex_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_abs))
        tex_shader.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex_shader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        tex_shader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        tex_shader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            st_reader.ConnectableAPI(), "result"
        )
        tex_shader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex_shader.ConnectableAPI(), "rgb"
        )
    else:
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.72, 0.72, 0.72)
        )
        if texture_path:
            print(f"box_mesh_builder: texture missing {texture_path}, using gray fallback")

    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    prim = stage.GetPrimAtPath(mesh_path)
    if prim and prim.IsValid():
        UsdShade.MaterialBindingAPI(prim).Bind(material)


def build_box_mesh_on_stage(
    stage,
    box_dir: str,
    obj_path: str,
    *,
    mesh_path: Optional[str] = None,
    looks_root: Optional[str] = None,
) -> bool:
    """在 Stage 上创建 mesh 并绑定贴图。

    Args:
        mesh_path: mesh prim 路径，默认 ``BOX_VISUAL_PATH/mesh``。
        looks_root: 材质 Looks 根路径，默认 ``BOX_LINK_PATH/Looks``。
    """
    mesh_path = mesh_path or f"{BOX_VISUAL_PATH}/mesh"
    looks_root = looks_root or f"{BOX_LINK_PATH}/Looks"
    try:
        mesh_data = parse_box_obj(obj_path, box_dir)
    except (OSError, ValueError) as exc:
        print(f"box_mesh_builder: parse failed: {exc}")
        return False

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(mesh_path))
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in mesh_data.points])
    mesh.CreateFaceVertexCountsAttr(mesh_data.face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(mesh_data.face_vertex_indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)

    normals = _compute_vertex_normals(
        mesh_data.points,
        mesh_data.face_vertex_counts,
        mesh_data.face_vertex_indices,
    )
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    if mesh_data.uv_points and mesh_data.face_uv_indices:
        # OBJ/OpenGL 的 vt 与 UsdUVTexture 一致，勿做 1-v 翻转（翻转会采到贴图底部纯黑区）
        st = [
            Gf.Vec2f(
                float(mesh_data.uv_points[i][0]),
                float(mesh_data.uv_points[i][1]),
            )
            for i in mesh_data.face_uv_indices
        ]
        primvars = UsdGeom.PrimvarsAPI(mesh)
        st_attr = primvars.CreatePrimvar(
            "st",
            Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying,
        )
        st_attr.Set(st)

    texture_path = resolve_box_texture_path(box_dir)
    _apply_material(stage, mesh_path, texture_path, looks_root)
    if texture_path and os.path.isfile(texture_path):
        print(f"box_mesh_builder: mesh + texture at {mesh_path}")
    else:
        print(
            f"box_mesh_builder: mesh at {mesh_path} "
            "(put Texture_*.png next to OBJ under data/box/model/)"
        )

    return True
