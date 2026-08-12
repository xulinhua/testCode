# -*- coding: utf-8 -*-
"""程序化生成标定板纹理（PNG）并给出物理尺寸。"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class BoardSpec:
    board_type: str
    squares_x: int  # inner corners / circle / marker cols
    squares_y: int
    square_length_m: float
    marker_length_m: float = 0.018

    @property
    def is_trihedral(self) -> bool:
        return self.board_type.startswith("trihedral_")

    @property
    def face_pattern(self) -> str:
        """Pattern drawn on each face: chess | charuco | aruco | circles_* | ..."""
        if self.board_type.startswith("trihedral_"):
            return self.board_type[len("trihedral_") :]
        return self.board_type

    @property
    def physical_size_xy(self) -> Tuple[float, float]:
        """Outer pattern size in meters (one face for trihedral)."""
        if self.is_trihedral:
            pat = self.face_pattern
            n = max(int(self.squares_x), int(self.squares_y), 3)
            margin = self.square_length_m * 1.0
            if pat == "aruco":
                side = n * self.square_length_m + 2 * margin
            else:
                side = (n + 1) * self.square_length_m + 2 * margin
            return (side, side)
        if self.board_type == "circles_asymmetric":
            w = self.squares_x * self.square_length_m
            h = (self.squares_y * 0.5 + 0.5) * self.square_length_m
            return (w + self.square_length_m, h + self.square_length_m)
        nx = self.squares_x + 1
        ny = self.squares_y + 1
        if self.board_type in ("circles_symmetric", "aruco_grid"):
            margin = self.square_length_m * 0.5
            return (
                self.squares_x * self.square_length_m + 2 * margin,
                self.squares_y * self.square_length_m + 2 * margin,
            )
        return (nx * self.square_length_m, ny * self.square_length_m)

    def face_squares(self) -> Tuple[int, int]:
        """Number of black/white cells on a chess/ChArUco face."""
        return self.squares_x + 1, self.squares_y + 1


def _save_rgb_png(path: str, img: np.ndarray) -> str:
    """保存为 8-bit RGB PNG（Isaac UsdUVTexture 对单通道支持差）。"""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    path = os.path.abspath(path)
    if img.ndim == 2:
        rgb = np.stack([img, img, img], axis=-1)
    elif img.shape[-1] == 1:
        rgb = np.repeat(img, 3, axis=-1)
    else:
        rgb = img
    rgb = np.ascontiguousarray(rgb.astype(np.uint8))
    try:
        import cv2

        # cv2 写 BGR
        cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            return path
    except Exception:
        pass
    try:
        from PIL import Image

        Image.fromarray(rgb, mode="RGB").save(path)
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            return path
    except Exception:
        pass
    # 最后手段：写未压缩 PPM(P6) 再尝试；仍失败则返回 path 供调试
    ppm = path.rsplit(".", 1)[0] + ".ppm"
    h, w = rgb.shape[:2]
    with open(ppm, "wb") as f:
        f.write(f"P6\n{w} {h}\n255\n".encode("ascii"))
        f.write(rgb.tobytes())
    return ppm if os.path.isfile(ppm) else path


def _save_gray_png(path: str, img: np.ndarray) -> str:
    return _save_rgb_png(path, img)


def _make_aruco_4x4(block: int, marker_id: int) -> np.ndarray:
    """OpenCV DICT_4X4_250 marker as a square grayscale patch (row 0 = top)."""
    from .aruco_dict_4x4_250 import marker_bits_4x4

    bits = marker_bits_4x4(int(marker_id))
    n = 4
    grid = n + 2
    block = max(grid, int(block))
    canvas = np.zeros((block, block), dtype=np.uint8)
    for gy in range(grid):
        for gx in range(grid):
            y0 = gy * block // grid
            y1 = (gy + 1) * block // grid
            x0 = gx * block // grid
            x1 = (gx + 1) * block // grid
            if gy == 0 or gy == grid - 1 or gx == 0 or gx == grid - 1:
                continue  # black border
            if bits[gy - 1][gx - 1]:
                canvas[y0:y1, x0:x1] = 255
    return canvas


def generate_trihedral_face_texture(
    spec: BoardSpec,
    pattern: str,
    out_path: str,
    *,
    flip_u: bool = False,
    px_per_square: int = 128,
    marker_id0: int = 0,
) -> str:
    """One face PNG: white margin + chess/ChArUco/ArUco. Optional U-flip for side faces."""
    px = max(32, int(px_per_square))
    n_inner = max(int(spec.squares_x), int(spec.squares_y), 3)
    s = float(spec.square_length_m)
    mk = float(spec.marker_length_m)
    if pattern == "aruco":
        cols = rows = n_inner
    else:
        cols = rows = n_inner + 1
    margin_px = px  # ≈ one square white border
    side = cols * px + 2 * margin_px
    img = np.ones((side, side), dtype=np.uint8) * 255
    mid = int(marker_id0)
    marker_frac = min(0.95, max(0.4, mk / max(s, 1e-6)))

    if pattern in ("chess", "charuco"):
        origin_black = pattern == "charuco"
        for r in range(rows):
            for c in range(cols):
                # Mirror layout in U for side faces; keep marker bits OpenCV-correct
                # (do NOT fliplr the whole image — that mirrors the codes).
                c_draw = (cols - 1 - c) if flip_u else c
                even = (r + c) % 2 == 0
                black = even if origin_black else (not even)
                y0 = margin_px + r * px
                x0 = margin_px + c_draw * px
                cell = 0 if black else 255
                img[y0 : y0 + px, x0 : x0 + px] = cell
                if pattern == "charuco" and not black:
                    msz = max(16, int(round(px * marker_frac)))
                    m = _make_aruco_4x4(msz, mid)
                    yo = y0 + (px - msz) // 2
                    xo = x0 + (px - msz) // 2
                    img[yo : yo + msz, xo : xo + msz] = m
                    mid += 1
    else:
        for r in range(rows):
            for c in range(cols):
                c_draw = (cols - 1 - c) if flip_u else c
                y0 = margin_px + r * px
                x0 = margin_px + c_draw * px
                msz = max(16, int(round(px * 0.90)))
                m = _make_aruco_4x4(msz, mid)
                yo = y0 + (px - msz) // 2
                xo = x0 + (px - msz) // 2
                img[yo : yo + msz, xo : xo + msz] = m
                mid += 1

    return _save_gray_png(out_path, img)


def generate_board_texture(spec: BoardSpec, out_path: str, px_per_square: int = 64) -> str:
    """生成灰度纹理并写到 out_path，返回实际文件路径。"""
    t = spec.board_type
    sx, sy = spec.squares_x, spec.squares_y
    px = max(16, int(px_per_square))

    if t == "chessboard":
        cols, rows = sx + 1, sy + 1
        img = np.zeros((rows * px, cols * px), dtype=np.uint8)
        for r in range(rows):
            for c in range(cols):
                if (r + c) % 2 == 0:
                    img[r * px : (r + 1) * px, c * px : (c + 1) * px] = 255
        return _save_gray_png(out_path, img)

    if t == "charuco":
        cols, rows = sx, sy  # OpenCV CharucoBoard cell count; (0,0) black
        img = np.zeros((rows * px, cols * px), dtype=np.uint8)
        marker_id = 0
        for r in range(rows):
            for c in range(cols):
                y0, y1 = r * px, (r + 1) * px
                x0, x1 = c * px, (c + 1) * px
                if (r + c) % 2 == 0:
                    img[y0:y1, x0:x1] = 0
                else:
                    img[y0:y1, x0:x1] = 255
                    m = _make_aruco_4x4(px, marker_id)
                    img[y0:y1, x0:x1] = m
                    marker_id += 1
        return _save_gray_png(out_path, img)

    if t == "circles_symmetric":
        margin = px // 2
        img = np.ones((sy * px + 2 * margin, sx * px + 2 * margin), dtype=np.uint8) * 255
        radius = max(3, int(px * 0.28))
        yy, xx = np.ogrid[: img.shape[0], : img.shape[1]]
        for r in range(sy):
            for c in range(sx):
                cy = margin + r * px + px // 2
                cx = margin + c * px + px // 2
                mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius**2
                img[mask] = 0
        return _save_gray_png(out_path, img)

    if t == "circles_asymmetric":
        # OpenCV asymmetric：奇数行偏移半格
        margin = px // 2
        h = int((sy * 0.5 + 0.5) * px) + 2 * margin
        w = sx * px + 2 * margin
        img = np.ones((h, w), dtype=np.uint8) * 255
        radius = max(3, int(px * 0.22))
        yy, xx = np.ogrid[:h, :w]
        for r in range(sy):
            for c in range(sx):
                cy = margin + int((r * 0.5 + 0.5) * px)
                cx = margin + c * px + px // 2
                if r % 2 == 1:
                    cx += px // 2
                if cx >= w - margin:
                    continue
                mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius**2
                img[mask] = 0
        return _save_gray_png(out_path, img)

    if t == "aruco_grid":
        margin = px // 2
        img = np.ones((sy * px + 2 * margin, sx * px + 2 * margin), dtype=np.uint8) * 255
        pad = max(2, px // 10)
        mid = 0
        for r in range(sy):
            for c in range(sx):
                y0 = margin + r * px + pad
                x0 = margin + c * px + pad
                size = px - 2 * pad
                if size < 8:
                    continue
                marker = _make_aruco_4x4(size, mid)
                img[y0 : y0 + size, x0 : x0 + size] = marker
                mid += 1
        return _save_gray_png(out_path, img)

    # 未知类型回退棋盘
    fallback = BoardSpec("chessboard", sx, sy, spec.square_length_m, spec.marker_length_m)
    return generate_board_texture(fallback, out_path, px_per_square)
