#!/usr/bin/env python3
"""把 Simple Room / Spot / Spot policy 从 Isaac S3 下载到 data/（一次，可离线复用）。"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

S3_BUCKET = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
# 与当前 isaac_env（5.0）对齐；可用 --isaac-version 覆盖
DEFAULT_ISAAC_VERSION = "5.0"
S3_NS = {"s3": "http://s3.amazonaws.com/doc/2006-03-01/"}

# prefix under Assets/Isaac/<ver>/Isaac/  ->  local dir under data/
ASSET_GROUPS = (
    ("Environments/Simple_Room/", "scenes/simple_room"),
    ("Robots/BostonDynamics/spot/", "robots/spot"),
    ("Samples/Policies/Spot_Policies/", "policies/spot"),
)


def _list_keys(prefix: str) -> list[tuple[str, int]]:
    keys: list[tuple[str, int]] = []
    token = None
    while True:
        url = f"{S3_BUCKET}/?list-type=2&prefix={urllib.parse.quote(prefix)}&max-keys=1000"
        if token:
            url += f"&continuation-token={urllib.parse.quote(token)}"
        with urllib.request.urlopen(url, timeout=120) as resp:
            root = ET.fromstring(resp.read())
        for c in root.findall("s3:Contents", S3_NS):
            key = c.find("s3:Key", S3_NS).text or ""
            size = int(c.find("s3:Size", S3_NS).text or "0")
            if not key or key.endswith("/") or "/.thumbs/" in key:
                continue
            keys.append((key, size))
        truncated = root.find("s3:IsTruncated", S3_NS)
        if truncated is not None and truncated.text == "true":
            nxt = root.find("s3:NextContinuationToken", S3_NS)
            token = None if nxt is None else nxt.text
            if not token:
                break
        else:
            break
    return keys


def _download(url: str, out_path: Path, expected_size: int | None = None) -> None:
    """优先 curl 断点续传；无 curl 时回退 urllib。"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(str(out_path) + ".partial")

    curl = shutil.which("curl")
    if curl:
        cmd = [
            curl,
            "-fL",
            "--retry",
            "5",
            "--retry-delay",
            "2",
            "--connect-timeout",
            "30",
            "--max-time",
            "600",
            "-C",
            "-",
            "-o",
            str(tmp),
            url,
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            # 部分 curl 在文件已完整时对 -C - 返回非 0；再检查尺寸
            if not (tmp.is_file() and expected_size is not None and tmp.stat().st_size == expected_size):
                err = (proc.stderr or proc.stdout or "").strip()
                raise RuntimeError(f"curl failed ({proc.returncode}): {err[:300]}")
    else:
        with urllib.request.urlopen(url, timeout=300) as resp, open(tmp, "wb") as f:
            while True:
                chunk = resp.read(1024 * 1024)
                if not chunk:
                    break
                f.write(chunk)

    if expected_size is not None and tmp.stat().st_size != expected_size:
        raise RuntimeError(
            f"size mismatch for {out_path.name}: got {tmp.stat().st_size}, want {expected_size}"
        )
    tmp.replace(out_path)


def download_group(isaac_version: str, remote_rel: str, local_rel: str, data_dir: Path) -> tuple[int, int]:
    prefix = f"Assets/Isaac/{isaac_version}/Isaac/{remote_rel}"
    out_root = data_dir / local_rel
    keys = _list_keys(prefix)
    if not keys:
        raise RuntimeError(f"no S3 objects under {prefix}")

    done = skipped = 0
    total = sum(s for _, s in keys)
    print(f"\n[{local_rel}] {len(keys)} files, {total / 1e6:.1f} MB  <- {prefix}", flush=True)
    for key, size in keys:
        rel = key[len(prefix) :]
        dest = out_root / rel
        if dest.is_file() and dest.stat().st_size == size:
            skipped += 1
            continue
        # lite 贴图：勿用 S3 原图覆盖已压缩的 Textures
        if "Materials/Textures/" in rel.replace("\\", "/") and (out_root / ".lite_textures").is_file():
            skipped += 1
            continue
        # 清掉坏的 partial，避免 curl -C 续传坏文件
        partial = Path(str(dest) + ".partial")
        if partial.is_file():
            if size and partial.stat().st_size == size:
                partial.replace(dest)
                done += 1
                print(f"  PROMOTE {rel}", flush=True)
                continue
            partial.unlink(missing_ok=True)
        url = f"{S3_BUCKET}/{key}"
        print(f"  GET {rel} ({size / 1e6:.2f} MB)", flush=True)
        try:
            _download(url, dest, expected_size=size)
            done += 1
        except Exception as exc:
            print(f"  FAIL {rel}: {exc}", file=sys.stderr, flush=True)
            raise
    print(f"  done={done} skipped={skipped} -> {out_root}", flush=True)
    return done, skipped


def verify_local(data_dir: Path) -> bool:
    required = [
        data_dir / "scenes/simple_room/simple_room.usd",
        data_dir / "robots/spot/spot.usd",
        data_dir / "policies/spot/spot_policy.pt",
        data_dir / "policies/spot/spot_env.yaml",
    ]
    ok = True
    for path in required:
        if path.is_file() and path.stat().st_size > 100:
            print(f"OK  {path} ({path.stat().st_size // 1024} KB)")
        else:
            print(f"MISSING  {path}", file=sys.stderr)
            ok = False
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--isaac-version",
        default=DEFAULT_ISAAC_VERSION,
        help=f"Isaac asset pack version (default {DEFAULT_ISAAC_VERSION})",
    )
    parser.add_argument("--verify-only", action="store_true", help="Only check local files")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    data_dir = root / "data"
    data_dir.mkdir(parents=True, exist_ok=True)

    if args.verify_only:
        return 0 if verify_local(data_dir) else 1

    print(f"Downloading Isaac {args.isaac_version} assets -> {data_dir}", flush=True)
    try:
        for remote_rel, local_rel in ASSET_GROUPS:
            download_group(args.isaac_version, remote_rel, local_rel, data_dir)
    except (urllib.error.URLError, TimeoutError, RuntimeError, OSError) as exc:
        print(f"download_assets failed: {exc}", file=sys.stderr)
        return 1

    print("\nVerify:", flush=True)
    return 0 if verify_local(data_dir) else 1


if __name__ == "__main__":
    raise SystemExit(main())
