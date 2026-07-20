#!/usr/bin/env python3
"""一次性下载 Intel RSD455 USD 到 data/robot/sensors/（需网络）。"""

from __future__ import annotations

import sys
import urllib.request
from pathlib import Path

RSD455_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.0/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    out_dir = root / "data" / "robot" / "sensors"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "rsd455.usd"

    if out_path.is_file() and out_path.stat().st_size > 100_000:
        print(f"Already present: {out_path} ({out_path.stat().st_size // 1024} KB)")
        return 0

    print(f"Downloading RSD455 -> {out_path}")
    print(f"  URL: {RSD455_URL}")
    try:
        urllib.request.urlretrieve(RSD455_URL, out_path)
    except Exception as exc:
        print(f"download_rsd455 failed: {exc}", file=sys.stderr)
        return 1

    size_kb = out_path.stat().st_size // 1024
    print(f"Done: {out_path} ({size_kb} KB)")
    print("Next: python3 scripts/prepare_robot_usd.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
