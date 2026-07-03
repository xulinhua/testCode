# -*- coding: utf-8 -*-
"""在 Stage 上从 cassette OBJ 创建可见 mesh + 贴图材质。"""

from __future__ import annotations

import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from ..cassette_assets import CassetteMeshData, CassettePaths, parse_obj_mesh


def build_cassette_mesh_prim(stage, mesh_path: str, mesh_data: CassetteMeshData):
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(mesh_path))
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in mesh_data.points])
    mesh.CreateFaceVertexCountsAttr(mesh_data.face_vertex_counts)
    mesh.CreateFaceVertexIndicesAttr(mesh_data.face_vertex_indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)

    if mesh_data.uv_points and mesh_data.face_uv_indices:
        st = [
            Gf.Vec2f(
                float(mesh_data.uv_points[i][0]),
                float(1.0 - mesh_data.uv_points[i][1]),
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
    return mesh


def apply_cassette_material(stage, mesh_prim_path: str, texture_path: str) -> None:
    if not os.path.isfile(texture_path):
        print(f"CassetteMeshBuilder: texture missing {texture_path}, using gray material")
        return

    tex_abs = os.path.abspath(texture_path).replace("\\", "/")
    mat_path = f"{mesh_prim_path}/Looks/cassette_mat"
    material = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)

    tex_shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/DiffuseTexture"))
    tex_shader.CreateIdAttr("UsdUVTexture")
    tex_shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex_abs))
    tex_shader.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex_shader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex_shader.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    prim = stage.GetPrimAtPath(mesh_prim_path)
    if prim and prim.IsValid():
        UsdShade.MaterialBindingAPI(prim).Bind(material)


def load_cassette_mesh_from_obj(stage, mesh_path: str, paths: CassettePaths):
    mesh_data = parse_obj_mesh(paths.obj_path)
    build_cassette_mesh_prim(stage, mesh_path, mesh_data)
    apply_cassette_material(stage, mesh_path, paths.texture_path)
    print(
        f"CassetteMeshBuilder: mesh {len(mesh_data.points)} verts at {mesh_path} "
        f"texture={paths.texture_path}"
    )
    return mesh_data
