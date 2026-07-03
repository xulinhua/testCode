#!/usr/bin/env python3
"""从 data/box/model/*.obj 生成 grasp_box.usda 与 grasp_box_meta.json。

用法（start_isaac.sh 会自动调用）::

    python3 scripts/prepare_box_usd.py

输出：
  - grasp_box.usda：引用 OBJ，mm→m 缩放并几何居中
  - grasp_box_meta.json：包围盒尺寸，供 SceneLoader 碰撞体
  - grasp_box_prepared.usda（可选）：Isaac asset_converter 转换后带材质
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path


def _obj_bbox_mm(obj_path: Path) -> tuple[list[float], list[float], list[float], list[float]]:
    """扫描 OBJ ``v`` 行，计算毫米单位下的 min/max/center/size。"""
    mins = [1e18, 1e18, 1e18]
    maxs = [-1e18, -1e18, -1e18]
    with obj_path.open(encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if not line.startswith("v "):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            mins[0] = min(mins[0], x)
            mins[1] = min(mins[1], y)
            mins[2] = min(mins[2], z)
            maxs[0] = max(maxs[0], x)
            maxs[1] = max(maxs[1], y)
            maxs[2] = max(maxs[2], z)
    center = [(mins[i] + maxs[i]) * 0.5 for i in range(3)]
    size = [maxs[i] - mins[i] for i in range(3)]
    return mins, maxs, center, size


def _find_obj(model_dir: Path) -> Path | None:
    """返回 model 目录下第一个 ``.obj`` 文件。"""
    objs = sorted(model_dir.glob("*.obj"))
    return objs[0] if objs else None


def _write_wrapper_usda(
    out_usda: Path,
    obj_rel: str,
    center_mm: list[float],
    scale: float,
) -> None:
    """写 USDA 包装：引用 OBJ + translate 居中 + uniform scale（不内嵌 mesh 顶点）。"""
    tx = -center_mm[0] * scale
    ty = -center_mm[1] * scale
    tz = -center_mm[2] * scale
    text = f'''#usda 1.0
(
    defaultPrim = "visual"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "visual" (
    prepend references = @{obj_rel}@
    assetInfo = {{
        string name = "GraspNet grasp box visual"
    }}
)
{{
    double3 xformOp:translate = ({tx:.9f}, {ty:.9f}, {tz:.9f})
    double3 xformOp:scale = ({scale}, {scale}, {scale})
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
}}
'''
    out_usda.write_text(text, encoding="utf-8")


def _warn_missing_texture(model_dir: Path, obj_path: Path) -> None:
    """检查 MTL 引用的 map_Kd 贴图是否存在于 model 目录。"""
    mtl_path = obj_path.with_suffix(obj_path.suffix + ".mtl")
    if not mtl_path.is_file():
        mtl_path = model_dir / f"{obj_path.stem}.obj.mtl"
    if not mtl_path.is_file():
        return
    for line in mtl_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        stripped = line.strip()
        if not stripped.startswith("map_Kd"):
            continue
        parts = stripped.split()
        if len(parts) < 2:
            continue
        tex_path = model_dir / Path(parts[-1]).name
        if tex_path.is_file():
            print(f"prepare_box_usd: texture OK -> {tex_path.name}")
            return
        print(f"prepare_box_usd: WARNING missing texture file: {tex_path}")
        print("  Copy the PNG next to the OBJ (same folder as the .mtl) for box textures.")
        return


def _try_isaac_convert(obj_path: Path, out_usd: Path) -> bool:
    """在 Isaac Sim 环境中用 asset_converter 将 OBJ 转为 USD（含材质，可选）。"""
    try:
        from isaacsim import SimulationApp
    except ImportError:
        return False

    app = SimulationApp({"headless": True, "width": 128, "height": 128})
    try:
        import omni.kit.asset_converter as asset_converter

        ctx = asset_converter.AssetConverterContext()
        ctx.ignore_materials = False
        ctx.ignore_animations = True
        ctx.ignore_camera = True
        ctx.ignore_lights = True
        ctx.single_mesh = False
        ctx.smooth_normals = True
        ctx.export_preview_surface = True
        ctx.use_meter_as_world_unit = False

        instance = asset_converter.get_instance()
        task = instance.create_converter_task(str(obj_path), str(out_usd), ctx)
        ok = instance.wait_until_task_done(task, 300.0)
        if not ok:
            print(f"prepare_box_usd: Isaac converter failed for {obj_path}")
            return False
        print(f"prepare_box_usd: converted via Isaac -> {out_usd}")
        return out_usd.is_file()
    except Exception as exc:
        print(f"prepare_box_usd: Isaac converter error: {exc}")
        return False
    finally:
        app.close()


def main() -> int:
    """入口：解析 bbox → 写 meta + wrapper；若 Isaac 可用则再写 prepared 版。"""
    root = Path(__file__).resolve().parents[1]
    box_dir = root / "data" / "box"
    model_dir = box_dir / "model"
    obj_path = _find_obj(model_dir)
    if obj_path is None:
        print(f"prepare_box_usd: no .obj under {model_dir}")
        return 1

    scale = 0.001  # OBJ 顶点单位为 mm
    _, _, center_mm, size_mm = _obj_bbox_mm(obj_path)
    size_m = [s * scale for s in size_mm]
    center_m = [c * scale for c in center_mm]

    meta = {
        "source_obj": str(obj_path.relative_to(box_dir)),
        "scale": scale,
        "size_m": {
            "x": size_m[0],
            "y": size_m[1],
            "z": size_m[2],
        },
        "center_m": {
            "x": center_m[0],
            "y": center_m[1],
            "z": center_m[2],
        },
    }
    meta_path = box_dir / "grasp_box_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(f"prepare_box_usd: wrote {meta_path}")

    obj_rel = obj_path.relative_to(box_dir).as_posix()
    wrapper_path = box_dir / "grasp_box.usda"
    _write_wrapper_usda(wrapper_path, obj_rel, center_mm, scale)
    print(f"prepare_box_usd: wrote {wrapper_path}")

    _warn_missing_texture(model_dir, obj_path)

    converted = box_dir / "grasp_box_converted.usd"
    use_isaac = os.environ.get("NOVA_PREPARE_BOX_ISAAC", "").strip() == "1"
    if use_isaac and _try_isaac_convert(obj_path, converted):
        prepared = box_dir / "grasp_box_prepared.usda"
        prepared.write_text(
            f'''#usda 1.0
(
    defaultPrim = "visual"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "visual" (
    prepend references = @grasp_box_converted.usd@
)
{{
    double3 xformOp:translate = ({-center_m[0]:.9f}, {-center_m[1]:.9f}, {-center_m[2]:.9f})
    double3 xformOp:scale = ({scale}, {scale}, {scale})
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
}}
''',
            encoding="utf-8",
        )
        print(f"prepare_box_usd: wrote {prepared}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
