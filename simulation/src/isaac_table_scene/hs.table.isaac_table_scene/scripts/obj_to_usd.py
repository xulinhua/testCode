#!/usr/bin/env python3
"""Headless OBJ -> USD（cassette 扫描 mesh）。"""

from __future__ import annotations

import argparse
import os
import sys


def _parse_args() -> argparse.Namespace:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    default_obj = os.path.join(root, "cassette", "mesh", "20260518.obj")
    default_usd = os.path.join(root, "data", "cassette.usd")
    p = argparse.ArgumentParser(description="Convert cassette OBJ to USD using Isaac Sim")
    p.add_argument("--obj", default=default_obj, help="Input OBJ path")
    p.add_argument("--usd", default=default_usd, help="Output USD path")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    obj_path = os.path.abspath(args.obj)
    dest_path = os.path.abspath(args.usd)

    if not os.path.isfile(obj_path):
        print(f"OBJ not found: {obj_path}", file=sys.stderr)
        return 1

    os.makedirs(os.path.dirname(dest_path), exist_ok=True)

    # 允许从扩展根目录 import cassette_assets
    ext_python = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "python"))
    if ext_python not in sys.path:
        sys.path.insert(0, ext_python)

    from cassette_assets import parse_obj_mesh

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True, "create_new_stage": True})

    try:
        from pxr import Gf, Sdf, Usd, UsdGeom

        for _ in range(8):
            simulation_app.update()

        stage = Usd.Stage.CreateNew(dest_path)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        mesh = UsdGeom.Mesh.Define(stage, Sdf.Path("/cassette"))
        stage.SetDefaultPrim(mesh.GetPrim())

        data = parse_obj_mesh(obj_path)
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in data.points])
        mesh.CreateFaceVertexCountsAttr(data.face_vertex_counts)
        mesh.CreateFaceVertexIndicesAttr(data.face_vertex_indices)
        mesh.CreateSubdivisionSchemeAttr("none")

        stage.GetRootLayer().Save()
        for _ in range(4):
            simulation_app.update()

        if not os.path.isfile(dest_path):
            print(f"ERROR: USD not created: {dest_path}", file=sys.stderr)
            return 1

        print(f"OK obj={obj_path}")
        print(f"OK usd={dest_path} bytes={os.path.getsize(dest_path)} verts={len(data.points)}")
        return 0
    except Exception as exc:
        import traceback

        traceback.print_exc()
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
