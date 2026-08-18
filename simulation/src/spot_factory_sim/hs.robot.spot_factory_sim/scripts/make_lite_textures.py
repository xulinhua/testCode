#!/usr/bin/env python3
"""把工厂仓库贴图压到小分辨率，显著加快 Load（几何不变，够障碍建图）。"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

MAX_EDGE = 256  # 足够深度/点云建图，远小于原 4K 贴图
_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg"}


def _resize_image(src: Path, dst: Path, max_edge: int) -> None:
    try:
        from PIL import Image
    except ImportError:
        import subprocess

        dst.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "convert",
            str(src),
            "-resize",
            f"{max_edge}x{max_edge}>",
            str(dst),
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
            resample = getattr(getattr(Image, "Resampling", Image), "BILINEAR", Image.BILINEAR)
            im = im.resize((nw, nh), resample)
        suffix = dst.suffix.lower()
        if suffix in {".jpg", ".jpeg"}:
            if im.mode == "RGBA":
                im = im.convert("RGB")
            im.save(dst, format="JPEG", quality=75, optimize=True)
        else:
            im.save(dst, format="PNG", optimize=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-edge", type=int, default=MAX_EDGE)
    parser.add_argument("--force", action="store_true", help="即使已 lite 也重压")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    scene = root / "data" / "scenes" / "simple_warehouse"
    backup = scene / "_hires_textures"
    marker = scene / ".lite_textures"

    if not scene.is_dir():
        print(f"Scene not found: {scene}", file=sys.stderr)
        return 1

    if marker.is_file() and not args.force:
        print(f"Already lite: {marker}")
        return 0

    images = sorted(
        p
        for p in scene.rglob("*")
        if p.is_file()
        and p.suffix.lower() in _IMAGE_SUFFIXES
        and "_hires_textures" not in p.parts
        and ".thumbs" not in p.parts
    )
    if not images:
        print("No PNG/JPEG textures found (scene may still load)", file=sys.stderr)
        marker.write_text("max_edge={}\nbefore=0\nafter=0\nno_images=1\n".format(args.max_edge))
        return 0

    before = sum(p.stat().st_size for p in images)
    print(f"Compressing {len(images)} textures (max_edge={args.max_edge}) ...")
    for src in images:
        rel = src.relative_to(scene)
        hires = backup / rel
        if not hires.is_file():
            hires.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, hires)
        src_in = hires if hires.is_file() else src
        tmp = src.with_suffix(src.suffix + ".tmp")
        _resize_image(src_in, tmp, args.max_edge)
        tmp.replace(src)
        print(f"  {rel}: {src_in.stat().st_size // 1024}KB -> {src.stat().st_size // 1024}KB")

    after = sum(
        p.stat().st_size
        for p in images
        if p.is_file()
    )
    marker.write_text(f"max_edge={args.max_edge}\nbefore={before}\nafter={after}\n")
    print(f"Done: {before // 1024 // 1024}MB -> {after // 1024}KB  marker={marker}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
