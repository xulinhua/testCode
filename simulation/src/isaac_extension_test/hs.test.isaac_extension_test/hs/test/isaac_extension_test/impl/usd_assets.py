# -*- coding: utf-8 -*-
"""
修复桌子 mz.usd 内失效贴图路径（改为相对路径 textures/*.png）。

相对路径以 mz.usd 所在目录为基准解析。
勿在 session 层写相对 override（会相对 stage 根解析失败）。
"""

import os
from typing import Dict, Optional

REL_TEXTURE_COLOR = "textures/GenericClassicTable001_color.png"
REL_TEXTURE_NORMAL = "textures/GenericClassicTable001_normal.png"
REL_TEXTURE_ROUGHNESS = "textures/GenericClassicTable001_roughness.png"


def _relative_texture_map(model_dir: str) -> Dict[str, str]:
    model_dir = os.path.abspath(model_dir)
    mapping = {
        "color": REL_TEXTURE_COLOR,
        "normal": REL_TEXTURE_NORMAL,
        "roughness": REL_TEXTURE_ROUGHNESS,
    }
    for key, rel in mapping.items():
        full = os.path.join(model_dir, rel)
        if not os.path.isfile(full):
            print(f"Warning: missing table texture {full}")
    return mapping


def _texture_for_input_name_exact(input_name: str, rel_map: Dict[str, str]) -> Optional[str]:
    key = input_name.lower()
    if key == "diffuse_texture":
        return rel_map.get("color")
    if key == "normalmap_texture":
        return rel_map.get("normal")
    if key == "reflectionroughness_texture":
        return rel_map.get("roughness")
    return None


def _rel_path_from_old_path(old_path: str, rel_map: Dict[str, str]) -> str:
    low = os.path.basename(old_path).lower()
    if "normal" in low:
        return rel_map["normal"]
    if "rough" in low:
        return rel_map["roughness"]
    if "color" in low:
        return rel_map["color"]
    return rel_map["color"]


def _needs_texture_remap(path_str: str) -> bool:
    if not path_str:
        return True
    if "IsaacsimExtGrasping" in path_str or "/home/yz/" in path_str:
        return True
    if os.path.isabs(path_str):
        return True
    return False


def _set_asset_path(attr, asset_path: str) -> bool:
    from pxr import Sdf

    try:
        if not attr or not attr.IsValid() or not attr.IsWritable():
            return False
        if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
            return False
        asset_path = asset_path.replace("\\", "/")
        current = attr.Get()
        if isinstance(current, Sdf.AssetPath) and current.path == asset_path:
            return False
        attr.Set(Sdf.AssetPath(asset_path))
        return True
    except Exception:
        return False


def _force_shader_inputs_relative(root_prim, rel_map: Dict[str, str]) -> int:
    """按 MDL 输入名强制写入三张相对路径贴图（不依赖旧路径内容）。"""
    from pxr import Usd, UsdShade

    fixed = 0
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsA(UsdShade.Shader):
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            target = _texture_for_input_name_exact(inp.GetBaseName(), rel_map)
            if not target:
                continue
            attr = inp.GetAttr()
            if attr and _set_asset_path(attr, target):
                fixed += 1
    return fixed


def _modify_layer_asset_paths(layer, model_dir: str, rel_map: Dict[str, str]) -> int:
    """用 UsdUtils 扫 layer 内所有 asset 路径，替换旧机器绝对路径。"""
    from pxr import Sdf, UsdUtils

    changed = [0]

    def rewrite(asset_path):
        if not asset_path or not asset_path.path:
            return asset_path
        p = asset_path.path.replace("\\", "/")
        if not _needs_texture_remap(p):
            return asset_path
        new_p = _rel_path_from_old_path(p, rel_map)
        if new_p != p:
            changed[0] += 1
        return Sdf.AssetPath(new_p)

    try:
        UsdUtils.ModifyAssetPaths(layer, rewrite)
    except Exception as exc:
        print(f"ModifyAssetPaths failed: {exc}")
    return changed[0]


def register_mz_texture_search_path(mz_dir: str) -> None:
    try:
        from pxr import Ar

        mz_dir = os.path.abspath(mz_dir)
        textures_dir = os.path.join(mz_dir, "textures")
        resolver = Ar.GetResolver()
        for path in (mz_dir, textures_dir):
            if os.path.isdir(path):
                resolver.AddDefaultSearchPath(path)
    except Exception as exc:
        print(f"register_mz_texture_search_path: {exc}")


def verify_mz_texture_files(model_dir: str) -> bool:
    model_dir = os.path.abspath(model_dir)
    ok = True
    for rel in (REL_TEXTURE_COLOR, REL_TEXTURE_NORMAL, REL_TEXTURE_ROUGHNESS):
        full = os.path.join(model_dir, rel)
        exists = os.path.isfile(full)
        print(f"  texture file {rel}: {'OK' if exists else 'MISSING'}")
        ok = ok and exists
    return ok


def repair_mz_usd_on_disk(mz_dir: str) -> int:
    """修补 mz.usd 源文件：强制三通道相对路径 + 替换 layer 内旧绝对路径。"""
    try:
        from pxr import Sdf, Usd
    except ImportError:
        return 0

    try:
        mz_dir = os.path.abspath(mz_dir)
        usd_path = os.path.join(mz_dir, "mz.usd")
        if not os.path.isfile(usd_path):
            print(f"repair_mz_usd_on_disk: missing {usd_path}")
            return 0
        if not verify_mz_texture_files(mz_dir):
            return 0

        rel_map = _relative_texture_map(mz_dir)
        stage = Usd.Stage.Open(usd_path)
        if not stage:
            return 0

        fixed = _force_shader_inputs_relative(stage.GetPseudoRoot(), rel_map)
        layer = stage.GetRootLayer()
        fixed += _modify_layer_asset_paths(layer, mz_dir, rel_map)

        if fixed:
            layer.Save()
            print(
                f"repair_mz_usd_on_disk: saved {fixed} update(s), "
                f"relative paths under {mz_dir}"
            )
        else:
            print("repair_mz_usd_on_disk: no updates (check Shader prims in mz.usd)")
        return fixed
    except Exception as exc:
        import traceback

        traceback.print_exc()
        print(f"repair_mz_usd_on_disk failed: {exc}")
        return 0


def repair_table_reference_layers(stage, prim_path: str, model_dir: str) -> int:
    """reference 后修补引用的 mz.usd layer（避免 Fabric 仍读旧绝对路径）。"""
    try:
        from pxr import Usd
    except ImportError:
        return 0

    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            return 0

        rel_map = _relative_texture_map(model_dir)
        total = 0
        query = Usd.PrimCompositionQuery.GetDirectReferences(prim)
        for arc in query.GetCompositionArcs():
            layer = arc.GetTargetLayer()
            if not layer or not layer.realPath:
                continue
            if "mz.usd" not in layer.realPath:
                continue
            sub_stage = Usd.Stage.Open(layer.realPath)
            if sub_stage:
                total += _force_shader_inputs_relative(sub_stage.GetPseudoRoot(), rel_map)
                total += _modify_layer_asset_paths(layer, model_dir, rel_map)
                layer.Save()
                print(f"repair_table_reference_layers: patched {layer.realPath}")
        return total
    except Exception as exc:
        print(f"repair_table_reference_layers: {exc}")
        return 0


def _force_shader_inputs_absolute(root_prim, model_dir: str) -> int:
    """在 prim 子树内按输入名写入扩展内贴图的绝对路径（session 层可用）。"""
    from pxr import Usd, UsdShade

    rel_map = _relative_texture_map(model_dir)
    fixed = 0
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsA(UsdShade.Shader):
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            rel = _texture_for_input_name_exact(inp.GetBaseName(), rel_map)
            if not rel:
                continue
            abs_path = os.path.abspath(os.path.join(model_dir, rel))
            if os.path.isfile(abs_path) and _set_asset_path(inp.GetAttr(), abs_path):
                fixed += 1
    return fixed


def apply_table_texture_session_absolute(stage, prim_path: str, model_dir: str) -> int:
    """Session 层绝对路径 override，确保 Hydra/Fabric 能解析贴图。"""
    try:
        from pxr import Usd
    except ImportError:
        return 0

    try:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            return 0
        session = stage.GetSessionLayer()
        if not session:
            return 0
        with Usd.EditContext(stage, Usd.EditTarget(session)):
            n = _force_shader_inputs_absolute(prim, model_dir)
        if n:
            print(f"apply_table_texture_session_absolute: {n} override(s) on {prim_path}")
        return n
    except Exception as exc:
        print(f"apply_table_texture_session_absolute: {exc}")
        return 0


def notify_stage_assets_changed() -> None:
    try:
        import omni.usd

        ctx = omni.usd.get_context()
        if hasattr(ctx, "post_notification"):
            ctx.post_notification(omni.usd.NotificationType.ASSETS_CHANGED)
    except Exception:
        pass
