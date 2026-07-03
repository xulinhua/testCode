#!/usr/bin/env python3
"""OBJ/STL/PLY mesh -> USD via OpenUSD (no Isaac Sim)."""

from __future__ import annotations

import argparse
import os
import sys


def _parse_obj(path: str):
    points = []
    faces = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0] == "v" and len(parts) >= 4:
                points.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif parts[0] == "f" and len(parts) >= 4:
                idx = []
                for tok in parts[1:]:
                    v = tok.split("/")[0]
                    i = int(v)
                    if i < 0:
                        i = len(points) + i + 1
                    idx.append(i - 1)
                if len(idx) == 3:
                    faces.append(idx)
                elif len(idx) == 4:
                    faces.append([idx[0], idx[1], idx[2]])
                    faces.append([idx[0], idx[2], idx[3]])
    face_counts = [3] * len(faces)
    face_indices = [i for tri in faces for i in tri]
    return points, face_counts, face_indices


def _load_mesh(path: str):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".obj":
        return _parse_obj(path)
    try:
        import trimesh
    except ImportError as exc:
        raise RuntimeError(
            f"非 OBJ 格式需要 trimesh: pip install trimesh ({exc})"
        ) from exc
    mesh = trimesh.load(path, force="mesh")
    if hasattr(mesh, "geometry"):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    points = [tuple(map(float, v)) for v in mesh.vertices]
    faces = mesh.faces.tolist()
    face_counts = [3] * len(faces)
    face_indices = [i for tri in faces for i in tri]
    return points, face_counts, face_indices


def main() -> int:
    p = argparse.ArgumentParser(description="Convert mesh to USD using OpenUSD")
    p.add_argument("--input", required=True, help="Input mesh (.obj/.stl/.ply)")
    p.add_argument("--usd", required=True, help="Output USD path")
    p.add_argument("--root-prim", default="mesh", help="Root prim name")
    args = p.parse_args()

    src = os.path.abspath(args.input)
    dest = os.path.abspath(args.usd)
    if not os.path.isfile(src):
        print(f"Input not found: {src}", file=sys.stderr)
        return 1
    os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)

    try:
        from pxr import Gf, Sdf, Usd, UsdGeom
    except ImportError:
        print("缺少 pxr (OpenUSD)。请: pip install usd-core", file=sys.stderr)
        return 1

    points, face_counts, face_indices = _load_mesh(src)
    if not points or not face_indices:
        print("Mesh 为空或解析失败", file=sys.stderr)
        return 1

    stage = Usd.Stage.CreateNew(dest)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root_path = Sdf.Path(f"/{args.root_prim.strip('/') or 'mesh'}")
    mesh = UsdGeom.Mesh.Define(stage, root_path)
    stage.SetDefaultPrim(mesh.GetPrim())
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr(face_counts)
    mesh.CreateFaceVertexIndicesAttr(face_indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    stage.GetRootLayer().Save()

    if not os.path.isfile(dest):
        print(f"USD not created: {dest}", file=sys.stderr)
        return 1
    print(f"OK input={src}")
    print(f"OK usd={dest} bytes={os.path.getsize(dest)} verts={len(points)} faces={len(face_counts)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
