#!/usr/bin/env python3
"""离线预处理 nova_robot.usda：移除 ActionGraph，弱化相机碰撞（不依赖 pxr）。"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path


def _strip_action_graph(text: str) -> str:
    """删除 USD 文本中的 ``def OmniGraph "ActionGraph"`` 整块。"""
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    skipping = False
    depth = 0
    for line in lines:
        if not skipping:
            if 'def OmniGraph "ActionGraph"' in line:
                skipping = True
                depth = 0
                if "{" in line:
                    depth += line.count("{") - line.count("}")
                continue
            out.append(line)
            continue
        depth += line.count("{")
        depth -= line.count("}")
        if depth <= 0:
            skipping = False
    return "".join(out)


def _strip_matching_def_blocks(text: str, name_needles: tuple[str, ...]) -> str:
    """删除 USD 文本中 def 名包含指定子串的整块（按花括号配对）。"""
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    i = 0
    n = len(lines)
    while i < n:
        line = lines[i]
        if any(f'def ' in line and needle in line for needle in name_needles):
            depth = 0
            started = False
            while i < n:
                chunk = lines[i]
                if "{" in chunk:
                    depth += chunk.count("{")
                    started = True
                if "}" in chunk:
                    depth -= chunk.count("}")
                i += 1
                if started and depth <= 0:
                    break
            continue
        out.append(line)
        i += 1
    return "".join(out)


def _strip_aruco_calibration(text: str) -> str:
    """移除标定板 mesh、link 上的 board、以及 Looks 内 aruco_mat 材质。"""
    needles = (
        '"aruco_board',
        '"aruco_mat',
        '"/meshes/aruco_board',
        "</meshes/aruco_board",
    )
    text = _strip_matching_def_blocks(text, needles)
    # 兜底：去掉残留贴图引用行
    text = re.sub(r'^\s*asset inputs:diffuse_texture = @\./materials/textures/6x6_1000-.*\n', "", text, flags=re.MULTILINE)
    return text


def _patch_camera_mounts(text: str) -> str:
    """在 cam0/1/2 的 RSD455 over 块内关闭 collision / rigidBody（勿加 resetXformStack）。"""

    def patch_cam_block(match: re.Match) -> str:
        block = match.group(0)
        if "physics:rigidBodyEnabled" not in block:
            block = block.replace('over "RSD455"\n            {', 'over "RSD455"\n            {\n                bool physics:rigidBodyEnabled = 0', 1)
            block = block.replace('over "RSD455" {\n', 'over "RSD455" {\n                bool physics:rigidBodyEnabled = 0\n', 1)
        block = re.sub(r"bool physics:collisionEnabled = 1", "bool physics:collisionEnabled = 0", block)
        # 原 2.usda 无 resetXformStack；加上会导致相机不随 link / 龙门 / 关节运动
        block = re.sub(r"\n\s*uniform bool xformOp:resetXformStack = 1", "", block)
        return block

    pattern = re.compile(
        r'        def "cam[012]" \([\s\S]*?\n        \}\n',
        re.MULTILINE,
    )
    return pattern.sub(patch_cam_block, text)


def main() -> int:
    """读取 nova_robot.usda，去 ActionGraph/ArUco/相机物理，写出 nova_robot_prepared.usda。"""
    root = Path(__file__).resolve().parents[1]
    src = root / "data" / "robot" / "nova_robot.usda"
    backup = root / "data" / "robot" / "nova_robot_source.usda"
    out = root / "data" / "robot" / "nova_robot_prepared.usda"

    if not src.is_file():
        print(f"Missing: {src}", file=sys.stderr)
        return 1

    if not backup.is_file():
        shutil.copy2(src, backup)
        print(f"Backup: {backup}")

    print(f"Reading {src} ({src.stat().st_size // (1024*1024)} MB) ...")
    text = src.read_text(encoding="utf-8")
    text = _strip_action_graph(text)
    text = _strip_aruco_calibration(text)
    text = _patch_camera_mounts(text)
    out.write_text(text, encoding="utf-8")
    print(f"Wrote {out} ({out.stat().st_size // (1024*1024)} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
