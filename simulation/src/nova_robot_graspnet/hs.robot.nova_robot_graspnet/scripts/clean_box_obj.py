#!/usr/bin/env python3
"""清理 data/box/model 抓取盒 OBJ 周边杂点。

扫描纸箱常见问题：极少数尖刺顶点把 AABB/convexHull 撑大。
步骤：
  1. 最大连通面片（去掉游离碎片）
  2. 可选百分位裁剪（默认关闭；扫描纸箱边角顶点少，裁剪易出洞）
  3. 可选 SOR（默认关闭）

默认从 ``model/org/*.obj`` 读入（若存在），写出到 ``model/*.obj``，
不覆盖 org。完成后可再跑 ``prepare_box_usd.py`` / ``bake_box_mesh.py``。

用法::

    python3 scripts/clean_box_obj.py
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parents[1]


class _UF:
    def __init__(self, n: int):
        self.p = np.arange(n, dtype=np.int32)
        self.r = np.zeros(n, dtype=np.int8)

    def find(self, x: int) -> int:
        p = self.p
        while p[x] != x:
            p[x] = p[p[x]]
            x = p[x]
        return int(x)

    def union(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return
        if self.r[ra] < self.r[rb]:
            self.p[ra] = rb
        elif self.r[ra] > self.r[rb]:
            self.p[rb] = ra
        else:
            self.p[rb] = ra
            self.r[ra] += 1


def _parse_obj(path: Path):
    verts: list[tuple[float, float, float]] = []
    uvs: list[tuple[float, float]] = []
    norms: list[tuple[float, float, float]] = []
    faces: list[list[tuple[int, int, int]]] = []  # (vi, vti, vni) 0-based, -1 if absent
    header: list[str] = []
    mtllib = "Texture_20260324164131.obj.mtl"
    usemtl = "material_0"

    with path.open("r", encoding="utf-8", errors="ignore") as fh:
        for raw in fh:
            line = raw.rstrip("\n")
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("vt "):
                p = line.split()
                uvs.append((float(p[1]), float(p[2])))
            elif line.startswith("vn "):
                p = line.split()
                norms.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("f "):
                corners: list[tuple[int, int, int]] = []
                for tok in line.split()[1:]:
                    parts = tok.split("/")
                    vi = int(parts[0]) - 1
                    vti = int(parts[1]) - 1 if len(parts) > 1 and parts[1] else -1
                    vni = int(parts[2]) - 1 if len(parts) > 2 and parts[2] else -1
                    corners.append((vi, vti, vni))
                if len(corners) >= 3:
                    faces.append(corners)
            elif line.startswith("mtllib "):
                mtllib = line.split(None, 1)[1].strip()
            elif line.startswith("usemtl "):
                usemtl = line.split(None, 1)[1].strip()
            elif not verts and not faces:
                header.append(line)

    return {
        "verts": np.asarray(verts, dtype=np.float64),
        "uvs": np.asarray(uvs, dtype=np.float64) if uvs else np.zeros((0, 2)),
        "norms": np.asarray(norms, dtype=np.float64) if norms else np.zeros((0, 3)),
        "faces": faces,
        "mtllib": mtllib,
        "usemtl": usemtl,
        "header": header,
    }


def _largest_face_component(n_verts: int, faces) -> np.ndarray:
    """返回保留的 face 布尔 mask。"""
    uf = _UF(n_verts)
    for corners in faces:
        v0 = corners[0][0]
        for c in corners[1:]:
            uf.union(v0, c[0])

    roots = np.fromiter((uf.find(i) for i in range(n_verts)), dtype=np.int32, count=n_verts)
    # count faces per root
    counts: dict[int, int] = {}
    face_roots = []
    for corners in faces:
        r = uf.find(corners[0][0])
        face_roots.append(r)
        counts[r] = counts.get(r, 0) + 1
    best = max(counts, key=counts.get) if counts else 0
    mask = np.array([r == best for r in face_roots], dtype=bool)
    return mask


def _sor_keep_vertices(points: np.ndarray, k: int, std_ratio: float) -> np.ndarray:
    """统计离群点：保留 True。"""
    n = len(points)
    if n == 0:
        return np.zeros(0, dtype=bool)
    kk = min(max(k + 1, 2), n)
    tree = cKDTree(points)
    dists, _ = tree.query(points, k=kk, workers=-1)
    # 第 0 列是自身距离 0
    mean_nn = dists[:, 1:].mean(axis=1)
    mu = float(mean_nn.mean())
    sigma = float(mean_nn.std())
    thresh = mu + std_ratio * sigma
    keep = mean_nn <= thresh
    return keep


def _percentile_keep_vertices(points: np.ndarray, lo: float, hi: float) -> np.ndarray:
    keep = np.ones(len(points), dtype=bool)
    for axis in range(3):
        a, b = np.percentile(points[:, axis], [lo, hi])
        keep &= (points[:, axis] >= a) & (points[:, axis] <= b)
    return keep


def _dense_keep_vertices(
    points: np.ndarray,
    *,
    bins: int = 256,
    dens_frac: float = 0.03,
    margin_mm: float = 0.3,
) -> np.ndarray:
    """按各轴直方图稠密区间裁剪，去掉两端稀疏尖刺（扫描顶/底毛刺常用）。"""
    keep = np.ones(len(points), dtype=bool)
    for axis in range(3):
        hist, edges = np.histogram(points[:, axis], bins=bins)
        thresh = max(1.0, float(hist.max()) * dens_frac)
        dense = np.where(hist >= thresh)[0]
        if len(dense) == 0:
            continue
        lo = float(edges[dense[0]]) - margin_mm
        hi = float(edges[dense[-1] + 1]) + margin_mm
        keep &= (points[:, axis] >= lo) & (points[:, axis] <= hi)
    return keep


def _filter_faces(faces, vert_keep: np.ndarray):
    out = []
    for corners in faces:
        if all(vert_keep[c[0]] for c in corners):
            out.append(corners)
    return out


def _remap_and_write(out_path: Path, data, faces) -> tuple[int, int]:
    verts = data["verts"]
    uvs = data["uvs"]
    norms = data["norms"]

    used_v = sorted({c[0] for f in faces for c in f})
    used_vt = sorted({c[1] for f in faces for c in f if c[1] >= 0})
    used_vn = sorted({c[2] for f in faces for c in f if c[2] >= 0})

    v_map = {old: i for i, old in enumerate(used_v)}
    vt_map = {old: i for i, old in enumerate(used_vt)}
    vn_map = {old: i for i, old in enumerate(used_vn)}

    with out_path.open("w", encoding="utf-8") as fh:
        fh.write("# Cleaned by scripts/clean_box_obj.py\n")
        fh.write(f"mtllib {data['mtllib']}\n")
        fh.write(f"usemtl {data['usemtl']}\n")
        for i in used_v:
            x, y, z = verts[i]
            fh.write(f"v {x:.7f} {y:.7f} {z:.7f}\n")
        for i in used_vt:
            u, vv = uvs[i]
            fh.write(f"vt {u:.7f} {vv:.7f}\n")
        for i in used_vn:
            x, y, z = norms[i]
            fh.write(f"vn {x:.7f} {y:.7f} {z:.7f}\n")
        for corners in faces:
            toks = []
            for vi, vti, vni in corners:
                a = v_map[vi] + 1
                if vti >= 0 and vni >= 0:
                    toks.append(f"{a}/{vt_map[vti]+1}/{vn_map[vni]+1}")
                elif vti >= 0:
                    toks.append(f"{a}/{vt_map[vti]+1}")
                elif vni >= 0:
                    toks.append(f"{a}//{vn_map[vni]+1}")
                else:
                    toks.append(str(a))
            fh.write("f " + " ".join(toks) + "\n")

    return len(used_v), len(faces)


def clean_obj(
    src: Path,
    dst: Path,
    *,
    k: int = 20,
    std_ratio: float = 0.0,
    percentile: float = 0.0,
    dense_frac: float = 0.0,
) -> None:
    print(f"clean_box_obj: load {src}")
    data = _parse_obj(src)
    n0 = len(data["verts"])
    f0 = len(data["faces"])
    pts_all = data["verts"]
    size0 = pts_all.max(0) - pts_all.min(0)
    print(f"  input: {n0} verts, {f0} faces, bbox={size0}")

    faces = data["faces"]
    cc_mask = _largest_face_component(n0, faces)
    faces = [f for f, m in zip(faces, cc_mask) if m]
    print(f"  after largest CC: {len(faces)} faces (dropped {f0 - len(faces)})")

    if percentile > 0:
        used = sorted({c[0] for f in faces for c in f})
        pts = data["verts"][used]
        local_keep = _percentile_keep_vertices(pts, percentile, 100.0 - percentile)
        vert_keep = np.zeros(n0, dtype=bool)
        for i, u in enumerate(used):
            vert_keep[u] = local_keep[i]
        before = len(faces)
        faces = _filter_faces(faces, vert_keep)
        n_drop_v = int((~local_keep).sum())
        print(
            f"  after percentile±{percentile}%: dropped {n_drop_v} verts, "
            f"{before - len(faces)} faces"
        )

    if dense_frac > 0:
        used = sorted({c[0] for f in faces for c in f})
        pts = data["verts"][used]
        local_keep = _dense_keep_vertices(pts, dens_frac=dense_frac)
        vert_keep = np.zeros(n0, dtype=bool)
        for i, u in enumerate(used):
            vert_keep[u] = local_keep[i]
        before = len(faces)
        faces = _filter_faces(faces, vert_keep)
        n_drop_v = int((~local_keep).sum())
        print(
            f"  after dense-hist(frac={dense_frac}): dropped {n_drop_v} verts, "
            f"{before - len(faces)} faces"
        )

    if std_ratio > 0:
        used = sorted({c[0] for f in faces for c in f})
        pts = data["verts"][used]
        local_keep = _sor_keep_vertices(pts, k=k, std_ratio=std_ratio)
        vert_keep = np.zeros(n0, dtype=bool)
        for i, u in enumerate(used):
            vert_keep[u] = local_keep[i]
        n_sor_drop = int((~local_keep).sum())
        before = len(faces)
        faces = _filter_faces(faces, vert_keep)
        print(
            f"  after SOR(k={k}, std={std_ratio}): dropped {n_sor_drop} verts, "
            f"{before - len(faces)} faces"
        )

    if not faces:
        raise SystemExit("clean_box_obj: all faces removed — loosen filters")

    used = sorted({c[0] for f in faces for c in f})
    pts = data["verts"][used]
    size = pts.max(axis=0) - pts.min(axis=0)
    print(f"  bbox size (obj units): {size}  (delta {size - size0})")

    dst.parent.mkdir(parents=True, exist_ok=True)
    nv, nf = _remap_and_write(dst, data, faces)
    print(f"clean_box_obj: wrote {dst} ({nv} verts, {nf} faces)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--k", type=int, default=20, help="SOR neighbor count")
    ap.add_argument(
        "--std-ratio",
        type=float,
        default=0.0,
        help="SOR std multiplier (0=disable; 尖刺场景建议保持 0)",
    )
    ap.add_argument(
        "--percentile",
        type=float,
        default=0.0,
        help="drop verts outside [p, 100-p] per axis (0=disable；默认关，避免边角漏)",
    )
    ap.add_argument(
        "--dense-frac",
        type=float,
        default=0.0,
        help="histogram dense-bin fraction for spike trim (0=disable；默认关，避免边角漏)",
    )
    ap.add_argument(
        "--from-org",
        action="store_true",
        default=True,
        help="read from model/org (default)",
    )
    ap.add_argument("--no-from-org", action="store_true", help="read current model OBJ")
    args = ap.parse_args()

    model = ROOT / "data" / "box" / "model"
    org = model / "org"
    src_dir = org if (not args.no_from_org and org.is_dir()) else model
    objs = sorted(src_dir.glob("*.obj"))
    if not objs:
        print(f"clean_box_obj: no OBJ under {src_dir}")
        return 1
    src = objs[0]
    dst = model / src.name

    # ensure mtl/png present alongside dst
    for name in (src.with_suffix(".obj.mtl").name, src.with_suffix(".png").name):
        # Meshlab style: Texture_xxx.obj.mtl
        pass
    mtl_src = src_dir / f"{src.name}.mtl"
    if not mtl_src.is_file():
        mtl_src = src_dir / src.with_suffix(".mtl").name
    png_candidates = list(src_dir.glob("*.png"))
    if mtl_src.is_file() and mtl_src.resolve() != (model / mtl_src.name).resolve():
        shutil.copy2(mtl_src, model / mtl_src.name)
    for png in png_candidates:
        target = model / png.name
        if png.resolve() != target.resolve():
            shutil.copy2(png, target)

    clean_obj(
        src,
        dst,
        k=args.k,
        std_ratio=args.std_ratio,
        percentile=0.0 if args.percentile <= 0 else args.percentile,
        dense_frac=0.0 if args.dense_frac <= 0 else args.dense_frac,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
