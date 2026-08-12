#!/usr/bin/env python3
"""把 Simple Room 贴图压到小分辨率，显著加快 Load（几何不变，够障碍建图）。"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MAX_EDGE = 256  # 足够深度/点云建图，远小于原 4K 贴图


def _resize_png(src: Path, dst: Path, max_edge: int) -> None:
    try:
        from PIL import Image
    except ImportError:
        # 无 Pillow 时用 ImageMagick convert
        import subprocess

        dst.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "convert",
            str(src),
            "-resize",
            f"{max_edge}x{max_edge}>",
            f"png:{dst}",
        ]
        subprocess.check_call(cmd)
        return

    dst.parent.mkdir(parents=True, exist_ok=True)
    with Image.open(src) as im:
        im = im.convert("RGBA") if im.mode in ("P", "LA") else im
        w, h = im.size
        scale = min(1.0, float(max_edge) / float(max(w, h)))
        if scale < 1.0:
            nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
            # Pillow<9: Image.BILINEAR; Pillow>=9: Image.Resampling.BILINEAR
            resample = getattr(getattr(Image, "Resampling", Image), "BILINEAR", Image.BILINEAR)
            im = im.resize((nw, nh), resample)
        im.save(dst, format="PNG", optimize=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-edge", type=int, default=MAX_EDGE)
    parser.add_argument("--force", action="store_true", help="即使已 lite 也重压")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    tex = root / "data" / "scenes" / "simple_room" / "Materials" / "Textures"
    backup = root / "data" / "scenes" / "simple_room" / "Materials" / "Textures_hires"
    marker = root / "data" / "scenes" / "simple_room" / ".lite_textures"

    if not tex.is_dir():
        print(f"Textures not found: {tex}", file=sys.stderr)
        return 1

    if marker.is_file() and not args.force:
        print(f"Already lite: {marker}")
        return 0

    pngs = sorted(tex.glob("*.png"))
    if not pngs:
        print("No PNG textures found", file=sys.stderr)
        return 1

    # 首次：备份原图
    if not backup.is_dir():
        print(f"Backing up hires textures -> {backup}")
        shutil.copytree(tex, backup)

    before = sum(p.stat().st_size for p in pngs)
    print(f"Compressing {len(pngs)} textures (max_edge={args.max_edge}) ...")
    for src in pngs:
        # 优先从 hires 读，避免反复压缩糊掉
        hires = backup / src.name
        src_in = hires if hires.is_file() else src
        tmp = src.with_suffix(".png.tmp")
        _resize_png(src_in, tmp, args.max_edge)
        tmp.replace(src)
        print(f"  {src.name}: {src_in.stat().st_size // 1024}KB -> {src.stat().st_size // 1024}KB")

    after = sum(p.stat().st_size for p in tex.glob("*.png"))
    marker.write_text(f"max_edge={args.max_edge}\nbefore={before}\nafter={after}\n")
    print(f"Done: {before // 1024 // 1024}MB -> {after // 1024}KB  marker={marker}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
