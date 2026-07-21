# -*- coding: utf-8 -*-
"""抓取盒视觉加载：烘焙 USD > 直接构建 mesh > 转换 USD 回退。"""

from __future__ import annotations

import os
from typing import Optional

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from ..global_variables import (
    BOX_COLLISION_GEO_ROOT,
    BOX_COLLISION_PATH,
    BOX_LINK_PATH,
    BOX_VISUAL_PATH,
)
from ..paths import (
    ensure_box_texture_sidecar,
    find_box_obj_path,
    resolve_box_texture_path,
)
from .box_mesh_builder import (
    build_box_mesh_on_stage,
    remove_legacy_box_collision_cube,
    setup_box_collision,
)


def _existing_baked(box_dir: str) -> Optional[str]:
    obj_path = find_box_obj_path(box_dir)
    if not obj_path:
        return None
    baked = os.path.join(box_dir, "grasp_box_baked.usdc")
    if not os.path.isfile(baked):
        return None
    stamp = os.path.getmtime(obj_path)
    for sub in ("model", "mesh", "textures"):
        folder = os.path.join(box_dir, sub)
        if not os.path.isdir(folder):
            continue
        for name in os.listdir(folder):
            if name.lower().endswith(".png"):
                stamp = max(stamp, os.path.getmtime(os.path.join(folder, name)))
    meta = os.path.join(box_dir, "grasp_box_meta.json")
    if os.path.isfile(meta):
        stamp = max(stamp, os.path.getmtime(meta))
    if os.path.getmtime(baked) >= stamp:
        return baked
    return None


def clear_box_visual(stage) -> None:
    """清除 grasp_box 视觉/碰撞子树，便于切换纸盒↔料盒后重新构建。"""
    # Remove visual children first (mesh references / live OBJ), then legacy paths.
    visual = stage.GetPrimAtPath(BOX_VISUAL_PATH)
    if visual and visual.IsValid():
        for child in list(visual.GetChildren()):
            path = child.GetPath().pathString
            try:
                stage.RemovePrim(path)
            except Exception as exc:
                print(f"box_visual_loader: remove {path} failed: {exc}")
    for path in (
        f"{BOX_VISUAL_PATH}/mesh",
        f"{BOX_VISUAL_PATH}/placeholder",
        BOX_COLLISION_GEO_ROOT,
        BOX_COLLISION_PATH,
    ):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            try:
                stage.RemovePrim(path)
            except Exception as exc:
                print(f"box_visual_loader: remove {path} failed: {exc}")
    # Drop Looks so previous OmniPBR / materials do not stick across kinds.
    looks = stage.GetPrimAtPath(f"{BOX_LINK_PATH}/Looks")
    if looks and looks.IsValid():
        try:
            stage.RemovePrim(f"{BOX_LINK_PATH}/Looks")
        except Exception as exc:
            print(f"box_visual_loader: remove Looks failed: {exc}")


def _box_has_visible_mesh(stage, root_path: str) -> bool:
    """检查 visual/mesh 引用下是否已有可渲染的 Mesh。"""
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return False
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if points and len(points) > 0:
            return True
    return False


def _remove_box_visual_placeholder(stage) -> None:
    path = f"{BOX_VISUAL_PATH}/placeholder"
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)


def _create_box_omnipbr_material(stage, texture_path: Optional[str]):
    """创建 OmniPBR 材质（Isaac RTX 显示贴图；采集 paper_box 同款）。"""
    mat_path = f"{BOX_LINK_PATH}/Looks/box_material"
    material = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
    shader = UsdShade.Shader.Define(stage, Sdf.Path(f"{mat_path}/Shader"))
    shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(1.0, 1.0, 1.0)
    )

    if texture_path and os.path.isfile(texture_path):
        tex_abs = os.path.abspath(texture_path).replace("\\", "/")
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(tex_abs)
        )
    elif texture_path:
        print(f"box_visual_loader: texture file missing: {texture_path}")

    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    material.CreateDisplacementOutput("mdl").ConnectToSource(
        shader.ConnectableAPI(), "out"
    )
    material.CreateVolumeOutput("mdl").ConnectToSource(shader.ConnectableAPI(), "out")
    return material


def _patch_baked_omnipbr_texture(stage, texture_abs: str) -> int:
    """修正烘焙引用内 OmniPBR 的 diffuse_texture 路径。"""
    patched = 0
    asset = Sdf.AssetPath(texture_abs)
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith(BOX_VISUAL_PATH) and not path.startswith(
            f"{BOX_LINK_PATH}/Looks"
        ):
            continue
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        try:
            src = shader.GetSourceAsset("mdl")
        except Exception:
            src = None
        if not src or "OmniPBR" not in str(src.path):
            # 也可能尚未解析 sourceAsset，直接看 diffuse_texture
            pass
        tex_in = shader.GetInput("diffuse_texture")
        if not tex_in:
            # 旧 PreviewSurface 烘焙：跳过
            continue
        tex_in.Set(asset)
        patched += 1
    return patched


def apply_box_textures(stage, box_dir: str) -> None:
    """对已加载 mesh 绑定 OmniPBR 贴图（覆盖烘焙内无效的 UsdPreviewSurface）。"""
    ensure_box_texture_sidecar(box_dir)
    texture_path = resolve_box_texture_path(box_dir)
    tex_abs = None
    if texture_path and os.path.isfile(texture_path):
        tex_abs = os.path.abspath(texture_path).replace("\\", "/")
    elif texture_path:
        print(f"box_visual_loader: texture file missing: {texture_path}")

    for load_path in (BOX_LINK_PATH, BOX_VISUAL_PATH):
        try:
            stage.Load(Sdf.Path(load_path), Usd.LoadWithDescendants)
        except Exception:
            pass

    patched = _patch_baked_omnipbr_texture(stage, tex_abs) if tex_abs else 0
    material = _create_box_omnipbr_material(stage, texture_path if tex_abs else None)

    bound = 0
    visual_root = stage.GetPrimAtPath(BOX_VISUAL_PATH)
    if visual_root and visual_root.IsValid():
        for prim in Usd.PrimRange(visual_root):
            if prim.GetPath().pathString.endswith("/placeholder"):
                continue
            if prim.IsA(UsdGeom.Mesh):
                if not prim.HasAPI(UsdShade.MaterialBindingAPI):
                    UsdShade.MaterialBindingAPI.Apply(prim)
                UsdShade.MaterialBindingAPI(prim).Bind(
                    material, UsdShade.Tokens.strongerThanDescendants
                )
                bound += 1

    if bound:
        _remove_box_visual_placeholder(stage)
        print(
            f"box_visual_loader: OmniPBR texture on {bound} mesh(es)"
            + (f", patched {patched} shader(s)" if patched else "")
            + (f": {tex_abs}" if tex_abs else "")
        )
        _notify_assets_changed()
    elif patched:
        _remove_box_visual_placeholder(stage)
        print(f"box_visual_loader: patched {patched} OmniPBR shader(s)")
        _notify_assets_changed()
    else:
        print(
            "box_visual_loader: WARN no visual mesh to bind — "
            "check data/box/grasp_box_baked.usdc and model/*.png"
        )


def load_box_visual(stage, box_dir: str, *, force: bool = False) -> bool:
    """Load 抓取盒视觉：baked USD 或现场从 OBJ 构建带贴图 mesh。

    Args:
        force: True 时先清空现有 visual/collision（切换物体类型时用）。
    """
    from isaacsim.core.utils.stage import add_reference_to_stage

    if force:
        clear_box_visual(stage)

    UsdGeom.Xform.Define(stage, Sdf.Path(BOX_VISUAL_PATH))

    baked = _existing_baked(box_dir)
    if baked:
        mesh_ref = f"{BOX_VISUAL_PATH}/mesh"
        if not _box_has_visible_mesh(stage, mesh_ref):
            add_reference_to_stage(usd_path=baked, prim_path=mesh_ref)
            prim = stage.GetPrimAtPath(mesh_ref)
            if prim and prim.IsValid():
                try:
                    prim.Load()
                    stage.Load(Sdf.Path(mesh_ref), Usd.LoadWithDescendants)
                except Exception:
                    pass
        if _box_has_visible_mesh(stage, mesh_ref):
            print(f"box_visual_loader: visual from baked {baked}")
            remove_legacy_box_collision_cube(stage)
            try:
                setup_box_collision(stage, box_dir=box_dir)
            except Exception as exc:
                print(f"box_visual_loader: setup_box_collision failed: {exc}")
            apply_box_textures(stage, box_dir)
            return True
        print(f"box_visual_loader: baked reference failed at {mesh_ref}")

    obj_path = find_box_obj_path(box_dir)
    if obj_path:
        print(
            f"box_visual_loader: building mesh from OBJ "
            f"({os.path.basename(obj_path)}; may take ~30-60s) ..."
        )
        if build_box_mesh_on_stage(stage, box_dir, obj_path):
            apply_box_textures(stage, box_dir)
            return True
        print("box_visual_loader: live OBJ build failed")

    print(
        "box_visual_loader: failed — no baked USD / OBJ under "
        f"{box_dir} (paper: model/; cassette: mesh/)"
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
