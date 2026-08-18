# -*- coding: utf-8 -*-
"""ArUco / ChArUco dictionary helpers (OpenCV-backed, DICT_4X4_250 fallback)."""

from __future__ import annotations

from typing import List, Sequence, Tuple

# Common dicts shown in Isaac UI / matching hs_calib_suite
ARUCO_DICTIONARY_NAMES: Tuple[str, ...] = (
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_6X6_1000",
    "DICT_7X7_50",
    "DICT_7X7_100",
    "DICT_7X7_250",
    "DICT_7X7_1000",
    "DICT_ARUCO_ORIGINAL",
)

DEFAULT_ARUCO_DICTIONARY = "DICT_4X4_250"


def normalize_dictionary_name(name: str) -> str:
    n = (name or "").strip()
    if not n:
        return DEFAULT_ARUCO_DICTIONARY
    if n in ARUCO_DICTIONARY_NAMES:
        return n
    upper = n.upper()
    for cand in ARUCO_DICTIONARY_NAMES:
        if cand == upper or cand.replace("DICT_", "") == upper:
            return cand
    return DEFAULT_ARUCO_DICTIONARY


def dictionary_marker_count(name: str) -> int:
    """Max number of markers in the dictionary (id in [0, count))."""
    n = normalize_dictionary_name(name)
    if n.endswith("_50"):
        return 50
    if n.endswith("_100"):
        return 100
    if n.endswith("_250"):
        return 250
    if n.endswith("_1000"):
        return 1000
    if n == "DICT_ARUCO_ORIGINAL":
        return 1024
    return 250


def _opencv_dict_id(name: str) -> int:
    import cv2

    n = normalize_dictionary_name(name)
    if not hasattr(cv2.aruco, n):
        return int(cv2.aruco.DICT_4X4_250)
    return int(getattr(cv2.aruco, n))


def get_predefined_dictionary(name: str):
    import cv2

    return cv2.aruco.getPredefinedDictionary(_opencv_dict_id(name))


def marker_bits(dictionary_name: str, marker_id: int) -> Tuple[List[List[int]], int]:
    """Return (bits[n][n], n) with 1=white. Row 0 = OpenCV marker top."""
    name = normalize_dictionary_name(dictionary_name)
    mid = int(marker_id) % max(1, dictionary_marker_count(name))
    try:
        import cv2
        import numpy as np

        dictionary = get_predefined_dictionary(name)
        n = int(getattr(dictionary, "markerSize", 0) or 0)
        if n <= 0:
            if "4X4" in name:
                n = 4
            elif "5X5" in name:
                n = 5
            elif "6X6" in name:
                n = 6
            elif "7X7" in name:
                n = 7
            else:
                n = 4
        grid = n + 2
        side = grid * 16
        img = None
        if hasattr(cv2.aruco, "generateImageMarker"):
            img = np.zeros((side, side), dtype=np.uint8)
            cv2.aruco.generateImageMarker(dictionary, mid, side, img, 1)
        elif hasattr(cv2.aruco, "drawMarker"):
            img = cv2.aruco.drawMarker(dictionary, mid, side)
        if img is None:
            raise RuntimeError("no marker draw API")
        cell = side // grid
        bits: List[List[int]] = []
        for r in range(1, n + 1):
            row: List[int] = []
            for c in range(1, n + 1):
                cy = r * cell + cell // 2
                cx = c * cell + cell // 2
                row.append(1 if int(img[cy, cx]) > 127 else 0)
            bits.append(row)
        return bits, n
    except Exception:
        from .aruco_dict_4x4_250 import marker_bits_4x4

        return marker_bits_4x4(mid), 4


def make_marker_patch(dictionary_name: str, marker_id: int, block: int) -> "object":
    """Square grayscale uint8 patch (row 0 = top), size≈block."""
    import numpy as np

    bits, n = marker_bits(dictionary_name, marker_id)
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
                continue
            if bits[gy - 1][gx - 1]:
                canvas[y0:y1, x0:x1] = 255
    return canvas


def clamp_marker_id(dictionary_name: str, marker_id: int) -> int:
    n = dictionary_marker_count(dictionary_name)
    return int(marker_id) % max(1, n)
