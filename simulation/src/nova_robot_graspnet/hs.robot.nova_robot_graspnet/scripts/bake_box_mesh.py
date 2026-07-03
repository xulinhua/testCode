#!/usr/bin/env python3
"""离线烘焙 grasp_box_baked.usdc：mesh + UV + 贴图材质。

由 ``start_isaac.sh`` 在启动 Isaac 前自动调用（OBJ/PNG 更新时重新烘焙）。
也可手动运行::

    python3 scripts/bake_box_mesh.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hs"))


def _bootstrap_pxr():
    """系统 python 无 pxr 时，用 headless SimulationApp 拉起 USD 环境。"""
    try:
        from pxr import Usd  # noqa: F401

        return None
    except ImportError:
        pass

    print("bake_box_mesh: starting headless Isaac (pxr bootstrap) ...")
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": 128, "height": 128})
    return app


def bake_box_mesh(root: Path | None = None) -> int:
    """解析 OBJ → 写入 ``data/box/grasp_box_baked.usdc``。"""
    from pxr import Usd, UsdGeom

    from robot.nova_robot_graspnet.impl.box_mesh_builder import build_box_mesh_on_stage
    from robot.nova_robot_graspnet.paths import load_box_meta

    root = root or ROOT
    box_dir = root / "data" / "box"
    model_dir = box_dir / "model"
    objs = sorted(model_dir.glob("*.obj"))
    if not objs:
        print(f"bake_box_mesh: no OBJ in {model_dir}")
        return 1

    obj_path = str(objs[0])
    baked_path = box_dir / "grasp_box_baked.usdc"

    stage = Usd.Stage.CreateNew(str(baked_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    if not build_box_mesh_on_stage(
        stage,
        str(box_dir),
        obj_path,
        mesh_path="/mesh",
        looks_root="/mesh/Looks",
    ):
        print("bake_box_mesh: build failed")
        return 1

    stage.SetDefaultPrim(stage.GetPrimAtPath("/mesh"))
    stage.GetRootLayer().Save()
    size_kb = baked_path.stat().st_size // 1024
    print(f"bake_box_mesh: wrote {baked_path} ({size_kb} KB)")
    meta = load_box_meta(str(box_dir))
    if meta:
        print(f"  size_m={meta.get('size_m')}")
    return 0


def main() -> int:
    app = _bootstrap_pxr()
    try:
        return bake_box_mesh()
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
