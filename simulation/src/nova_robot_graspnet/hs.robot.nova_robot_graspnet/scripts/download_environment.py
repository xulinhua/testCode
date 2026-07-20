#!/usr/bin/env python3
"""下载 Isaac Grid 地面环境 USD + Wireframe 贴图到 data/env/（需网络）。"""

from __future__ import annotations

import sys
import urllib.request
from pathlib import Path

ENV_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.0/Isaac/Environments/Grid/default_environment.usd"
)
TEXTURE_BASE = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.0/Isaac/Environments/Grid/Materials/Textures"
)
TEXTURE_FILES = (
    "Wireframe_blue.png",
    "WireframeBlur_basecolor.png",
    "WireframeBlur_blue.png",
)


def _download(url: str, out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"  GET {url}")
    urllib.request.urlretrieve(url, out_path)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    out_dir = root / "data" / "env"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "default_environment.usd"
    tex_dir = out_dir / "Materials" / "Textures"

    ok = True
    if out_path.is_file() and out_path.stat().st_size > 5_000:
        print(f"Already present: {out_path} ({out_path.stat().st_size // 1024} KB)")
    else:
        print(f"Downloading Isaac grid environment -> {out_path}")
        try:
            _download(ENV_URL, out_path)
            print(f"Done: {out_path} ({out_path.stat().st_size // 1024} KB)")
        except Exception as exc:
            print(f"download_environment USD failed: {exc}", file=sys.stderr)
            ok = False

    print(f"Ensuring grid textures under {tex_dir}")
    for name in TEXTURE_FILES:
        dest = tex_dir / name
        if dest.is_file() and dest.stat().st_size > 1000:
            print(f"  already: {dest.name} ({dest.stat().st_size // 1024} KB)")
            continue
        try:
            _download(f"{TEXTURE_BASE}/{name}", dest)
            print(f"  done: {dest.name} ({dest.stat().st_size // 1024} KB)")
        except Exception as exc:
            print(f"  texture {name} failed: {exc}", file=sys.stderr)
            ok = False

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
