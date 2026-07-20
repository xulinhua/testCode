# -*- coding: utf-8 -*-
"""从 OBJ 在 Stage 上直接构建带 UV 的抓取盒 mesh（不依赖 Isaac OBJ 引用）。"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from ..global_variables import (
    BOX_COLLISION_GEO_MESH,
    BOX_COLLISION_GEO_ROOT,
    BOX_COLLISION_PATH,
    BOX_LINK_PATH,
    BOX_SIZE_X,
    BOX_SIZE_Y,
    BOX_SIZE_Z,
    BOX_VISUAL_PATH,
)
from ..paths import ensure_box_texture_sidecar, load_box_meta, resolve_box_texture_path


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


def remove_legacy_box_collision_cube(stage) -> bool:
    """删除旧的轴对齐立方体碰撞体（与视觉 mesh 尺寸/中心不一致）。"""
    prim = stage.GetPrimAtPath(BOX_COLLISION_PATH)
    if not prim or not prim.IsValid():
        return False
    stage.RemovePrim(BOX_COLLISION_PATH)
    print(f"box_mesh_builder: removed legacy AABB collision @ {BOX_COLLISION_PATH}")
    return True


def _strip_collision_from_prim(prim) -> int:
    """从 prim 剥离碰撞相关 API（视觉 mesh 仅渲染）。"""
    from pxr import UsdPhysics

    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None

    count = 0
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
        count += 1
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        count += 1
    if PhysxSchema is not None:
        for api in (
            PhysxSchema.PhysxCollisionAPI,
            PhysxSchema.PhysxConvexHullCollisionAPI,
            PhysxSchema.PhysxConvexDecompositionCollisionAPI,
            PhysxSchema.PhysxTriangleMeshCollisionAPI,
        ):
            if prim.HasAPI(api):
                prim.RemoveAPI(api)
                count += 1
    attr = prim.GetAttribute("physics:approximation")
    if attr and attr.IsValid():
        attr.Clear()
    return count


def strip_visual_collision(stage, visual_root: str = BOX_VISUAL_PATH) -> int:
    """移除视觉子树上的碰撞 schema，避免与独立碰撞 mesh 重复。"""
    root = stage.GetPrimAtPath(visual_root)
    if not root or not root.IsValid():
        return 0
    count = 0
    for prim in Usd.PrimRange(root):
        count += _strip_collision_from_prim(prim)
    if count:
        print(f"box_mesh_builder: stripped collision API from {count} visual prim(s)")
    return count


def _extract_visual_mesh_data(stage, visual_root: str = BOX_VISUAL_PATH) -> Optional[BoxMeshData]:
    """从已加载的视觉 mesh 读取三角面（烘焙 USD 路径）。"""
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(visual_root)
    if not root or not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetPath().pathString.endswith("/placeholder"):
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not points or not counts or not indices:
            continue
        pts = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
        return BoxMeshData(
            points=pts,
            face_vertex_counts=[int(c) for c in counts],
            face_vertex_indices=[int(i) for i in indices],
            uv_points=[],
            face_uv_indices=[],
        )
    return None


def has_box_collision(stage) -> bool:
    """抓取盒刚体下是否已有可用碰撞 prim。"""
    root = stage.GetPrimAtPath(BOX_COLLISION_GEO_ROOT)
    if not root or not root.IsValid():
        return False
    from pxr import UsdPhysics

    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            return True
    return False


def _collision_size_from_meta(box_dir: Optional[str]) -> Tuple[float, float, float]:
    meta = load_box_meta(box_dir) if box_dir else None
    if meta and "size_m" in meta:
        size = meta["size_m"]
        return (
            float(size.get("x", BOX_SIZE_X)),
            float(size.get("y", BOX_SIZE_Y)),
            float(size.get("z", BOX_SIZE_Z)),
        )
    return (float(BOX_SIZE_X), float(BOX_SIZE_Y), float(BOX_SIZE_Z))


def _write_fallback_box_collision(stage, size_xyz: Tuple[float, float, float]) -> bool:
    """meta 尺寸轴对齐盒碰撞（OBJ/烘焙未就绪时防止无碰撞穿桌）。"""
    from pxr import UsdGeom, UsdPhysics

    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None

    sx, sy, sz = size_xyz
    path = f"{BOX_COLLISION_GEO_ROOT}/fallback_box"
    old = stage.GetPrimAtPath(path)
    if old and old.IsValid():
        stage.RemovePrim(path)

    if not stage.GetPrimAtPath(BOX_COLLISION_GEO_ROOT).IsValid():
        UsdGeom.Xform.Define(stage, Sdf.Path(BOX_COLLISION_GEO_ROOT))

    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)
    UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
    cube.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdGeom.Imageable(cube).CreateVisibilityAttr(UsdGeom.Tokens.invisible)

    coll = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    coll.CreateCollisionEnabledAttr(True)
    if PhysxSchema is not None:
        PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
    print(
        f"box_mesh_builder: fallback box collision @ {path} "
        f"({sx:.3f}×{sy:.3f}×{sz:.3f} m)"
    )
    return True


def _write_collision_mesh(stage, mesh_data: BoxMeshData) -> bool:
    """创建与视觉同拓扑的碰撞 mesh（动态刚体用 convexHull，源几何为真实三角面）。"""
    from pxr import UsdGeom, UsdPhysics

    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None

    old = stage.GetPrimAtPath(BOX_COLLISION_GEO_ROOT)
    if old and old.IsValid():
        stage.RemovePrim(BOX_COLLISION_GEO_ROOT)

    UsdGeom.Xform.Define(stage, Sdf.Path(BOX_COLLISION_GEO_ROOT))
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(BOX_COLLISION_GEO_MESH))
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in mesh_data.points])
    mesh.CreateFaceVertexCountsAttr(mesh_data.face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(mesh_data.face_vertex_indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)
    imageable = UsdGeom.Imageable(mesh)
    imageable.CreateVisibilityAttr(UsdGeom.Tokens.invisible)

    coll = UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    coll.CreateCollisionEnabledAttr(True)
    mesh_col = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    # convexHull 比 convexDecomposition 更快就绪，动态刚体更不易穿模
    mesh_col.CreateApproximationAttr("convexHull")

    if PhysxSchema is not None:
        PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(mesh.GetPrim())

    print(
        f"box_mesh_builder: mesh collision (convexHull) @ {BOX_COLLISION_GEO_MESH} "
        f"({len(mesh_data.points)} verts, {len(mesh_data.face_vertex_counts)} tris)"
    )
    return True


def setup_box_collision(
    stage,
    box_dir: Optional[str] = None,
    visual_root: str = BOX_VISUAL_PATH,
    mesh_data: Optional[BoxMeshData] = None,
    *,
    allow_fallback: bool = True,
    prefer_aabb: bool = True,
) -> int:
    """视觉仅渲染；碰撞在独立 collision_geo 上。

    默认用 meta AABB（纸箱扫描毛刺会撑大 convexHull，导致绿框不贴合）。
    ``prefer_aabb=False`` 时才写 mesh convexHull。
    """
    remove_legacy_box_collision_cube(stage)
    strip_visual_collision(stage, visual_root)

    if prefer_aabb:
        size = _collision_size_from_meta(box_dir)
        if _write_fallback_box_collision(stage, size):
            print(
                f"box_mesh_builder: AABB collision from meta "
                f"({size[0]:.3f}×{size[1]:.3f}×{size[2]:.3f} m)"
            )
            return 1

    if mesh_data is None:
        if box_dir:
            model_dir = os.path.join(box_dir, "model")
            if os.path.isdir(model_dir):
                for name in sorted(os.listdir(model_dir)):
                    if name.endswith(".obj"):
                        try:
                            mesh_data = parse_box_obj(os.path.join(model_dir, name), box_dir)
                        except (OSError, ValueError) as exc:
                            print(f"box_mesh_builder: OBJ parse for collision failed: {exc}")
                        break
        if mesh_data is None:
            mesh_data = _extract_visual_mesh_data(stage, visual_root)
    if mesh_data is not None and _write_collision_mesh(stage, mesh_data):
        return 1

    if allow_fallback:
        size = _collision_size_from_meta(box_dir)
        if _write_fallback_box_collision(stage, size):
            print("box_mesh_builder: using meta-sized fallback until mesh collision is ready")
            return 1

    print("box_mesh_builder: WARN no collision_geo created")
    return 0


def ensure_box_collision(stage, box_dir: Optional[str] = None) -> bool:
    """Play 前确保盒子有碰撞体；默认重写为 meta AABB（避免毛刺 convexHull）。"""
    return setup_box_collision(
        stage, box_dir=box_dir, allow_fallback=True, prefer_aabb=True
    ) > 0


def apply_box_mesh_collision(stage, visual_root: str = BOX_VISUAL_PATH) -> int:
    """兼容旧调用：委托 ``setup_box_collision``（无 box_dir 时从 visual 提取拓扑）。"""
    return setup_box_collision(stage, box_dir=None, visual_root=visual_root)


def _apply_material(
    stage,
    mesh_path: str,
    texture_path: Optional[str],
    looks_root: str,
    *,
    texture_asset: Optional[str] = None,
) -> None:
    """绑定 OmniPBR 材质（与采集 paper_box 一致；UsdPreviewSurface 在 RTX 上常不显示贴图）。"""
    mat_path = f"{looks_root}/box_material"
    material = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/Shader"))
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 1.0, 1.0)
    )

    asset = texture_asset
    if not asset and texture_path and os.path.isfile(texture_path):
        asset = os.path.abspath(texture_path).replace("\\", "/")
    if asset:
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(asset)
        )
    elif texture_path:
        print(f"box_mesh_builder: texture missing {texture_path}, using white OmniPBR")

    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateDisplacementOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateVolumeOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")

    prim = stage.GetPrimAtPath(mesh_path)
    if prim and prim.IsValid():
        if not prim.HasAPI(UsdShade.MaterialBindingAPI):
            UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(
            material, UsdShade.Tokens.strongerThanDescendants
        )


def build_box_mesh_on_stage(
    stage,
    box_dir: str,
    obj_path: str,
    *,
    mesh_path: Optional[str] = None,
    looks_root: Optional[str] = None,
    write_collision: bool = True,
) -> bool:
    """在 Stage 上创建 mesh 并绑定贴图。

    Args:
        mesh_path: mesh prim 路径，默认 ``BOX_VISUAL_PATH/mesh``。
        looks_root: 材质 Looks 根路径，默认 ``BOX_LINK_PATH/Looks``。
        write_collision: 为 False 时不写 ``/World/grasp_box/collision_geo``
            （离线烘焙 ``grasp_box_baked.usdc`` 必须关掉，否则会污染 defaultPrim）。
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
    # 烘焙到 grasp_box_baked.usdc 时用相对 ./textures/，与采集 paper_box 一致
    tex_rel = ensure_box_texture_sidecar(box_dir)
    _apply_material(
        stage,
        mesh_path,
        texture_path,
        looks_root,
        texture_asset=tex_rel,
    )
    if write_collision:
        try:
            setup_box_collision(stage, box_dir=box_dir, mesh_data=mesh_data)
        except Exception as exc:
            print(f"box_mesh_builder: setup_box_collision failed (visual mesh kept): {exc}")
    if texture_path and os.path.isfile(texture_path):
        print(f"box_mesh_builder: mesh + texture at {mesh_path}")
    else:
        print(
            f"box_mesh_builder: mesh at {mesh_path} "
            "(put Texture_*.png next to OBJ under data/box/model/)"
        )

    return True
