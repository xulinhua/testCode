#!/usr/bin/env python3
"""Strip ArUco boards from copied URDF and rewrite mesh paths to local meshes/."""

from __future__ import annotations

import re
import sys
from pathlib import Path


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    urdf_path = root / "data" / "robot" / "nova_robot_position.urdf"
    if not urdf_path.is_file():
        print(f"URDF not found: {urdf_path}", file=sys.stderr)
        return 1

    text = urdf_path.read_text(encoding="utf-8")

    # Remove aruco joint + link blocks.
    text = re.sub(
        r"\s*<joint name=\"aruco_board_[^\"]+\"[\s\S]*?</joint>\s*",
        "\n",
        text,
    )
    text = re.sub(
        r"\s*<link name=\"aruco_board_[^\"]+\"[\s\S]*?</link>\s*",
        "\n",
        text,
    )

    meshes_dir = (root / "data" / "robot" / "meshes").resolve()
    text = re.sub(
        r'filename="file:///[^"]+/([^/"]+\.stl)"',
        lambda m: f'filename="file://{meshes_dir / m.group(1)}"',
        text,
    )

    urdf_path.write_text(text, encoding="utf-8")
    print(f"Patched URDF: {urdf_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
