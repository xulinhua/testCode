# -*- coding: utf-8 -*-
"""抓取盒视觉加载：烘焙 USD > 直接构建 mesh > 转换 USD 回退。"""

from __future__ import annotations

import os
from typing import Optional

from pxr import Sdf, Usd, UsdGeom

from ..global_variables import BOX_VISUAL_PATH
from ..paths import resolve_box_texture_path
from .box_mesh_builder import build_box_mesh_on_stage


def _find_obj_path(box_dir: str) -> Optional[str]:
    model_dir = os.path.join(box_dir, "model")
    if not os.path.isdir(model_dir):
        return None
    for name in sorted(os.listdir(model_dir)):
        if name.endswith(".obj"):
            return os.path.join(model_dir, name)
    return None


def _existing_baked(box_dir: str) -> Optional[str]:
    obj_path = _find_obj_path(box_dir)
    if not obj_path:
        return None
    baked = os.path.join(box_dir, "grasp_box_baked.usdc")
    if not os.path.isfile(baked):
        return None
    stamp = os.path.getmtime(obj_path)
    model_dir = os.path.join(box_dir, "model")
    for name in os.listdir(model_dir):
        if name.endswith(".png"):
            png = os.path.join(model_dir, name)
            stamp = max(stamp, os.path.getmtime(png))
    meta = os.path.join(box_dir, "grasp_box_meta.json")
    if os.path.isfile(meta):
        stamp = max(stamp, os.path.getmtime(meta))
    if os.path.getmtime(baked) >= stamp:
        return baked
    return None


def apply_box_textures(stage, box_dir: str) -> None:
    """延迟重试：对已加载 mesh 再次 patch 贴图路径（converted USD 路径用）。"""
    from ..global_variables import BOX_COLLISION_PATH, BOX_LINK_PATH

    texture_path = resolve_box_texture_path(box_dir)
    if texture_path and not os.path.isfile(texture_path):
        print(f"box_visual_loader: texture file missing: {texture_path}")

    try:
        stage.Load(Sdf.Path(BOX_LINK_PATH), Usd.LoadWithDescendants)
    except Exception:
        pass

    from pxr import Gf, UsdGeom, UsdShade

    mat_path = f"{BOX_LINK_PATH}/Looks/box_material"
    material = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)

    if os.path.isfile(texture_path) if texture_path else False:
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
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    bound = 0
    root = stage.GetPrimAtPath(BOX_LINK_PATH)
    if root and root.IsValid():
        for prim in Usd.PrimRange(root):
            if prim.GetPath().pathString.startswith(BOX_COLLISION_PATH):
                continue
            if prim.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(prim).Bind(material)
                bound += 1
    if bound:
        print(f"box_visual_loader: rebound texture on {bound} mesh(es)")
        _notify_assets_changed()


def load_box_visual(stage, box_dir: str) -> bool:
    """Load 抓取盒视觉：baked USD 或现场从 OBJ 构建带贴图 mesh。"""
    from isaacsim.core.utils.stage import add_reference_to_stage

    UsdGeom.Xform.Define(stage, Sdf.Path(BOX_VISUAL_PATH))

    baked = _existing_baked(box_dir)
    if baked:
        mesh_ref = f"{BOX_VISUAL_PATH}/mesh"
        add_reference_to_stage(usd_path=baked, prim_path=mesh_ref)
        prim = stage.GetPrimAtPath(mesh_ref)
        if prim and prim.IsValid():
            try:
                prim.Load()
            except Exception:
                pass
        print(f"box_visual_loader: visual from baked {baked}")
        return True

    obj_path = _find_obj_path(box_dir)
    if obj_path:
        print("box_visual_loader: building textured mesh from OBJ (~20-40s, one-time) ...")
        if build_box_mesh_on_stage(stage, box_dir, obj_path):
            _notify_assets_changed()
            return True

    print(
        "box_visual_loader: failed — check data/box/model/*.obj + *.png, "
        "or run: isaac_env/bin/python3.11 scripts/bake_box_mesh.py"
    )
    return False


def _notify_assets_changed() -> None:
    try:
        import omni.usd

        ctx = omni.usd.get_context()
        if hasattr(ctx, "post_notification"):
            ctx.post_notification(omni.usd.NotificationType.ASSETS_CHANGED)
    except Exception:
        pass
