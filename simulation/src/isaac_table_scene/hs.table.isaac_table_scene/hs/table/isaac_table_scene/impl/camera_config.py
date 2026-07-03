# Copyright (c) 2024
"""相机内参计算。"""

from typing import List


def compute_intrinsics_k(
    width: int,
    height: int,
    focal_length_mm: float,
    horiz_aperture_mm: float,
    vert_aperture_mm: float,
) -> List[List[float]]:
    fx = (focal_length_mm * width) / horiz_aperture_mm
    fy = (focal_length_mm * height) / vert_aperture_mm
    cx = width / 2.0
    cy = height / 2.0
    return [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]


def usd_camera_attrs_from_fov(width: int, height: int, horizontal_fov_deg: float):
    import math

    fov_rad = math.radians(float(horizontal_fov_deg))
    horiz_ap_mm = 20.955
    focal_mm = (horiz_ap_mm / 2.0) / math.tan(fov_rad / 2.0)
    vert_ap_mm = horiz_ap_mm * (float(height) / float(width))
    return focal_mm, horiz_ap_mm, vert_ap_mm


def usd_camera_attrs_from_hv_fov(
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    horiz_ap_mm: float = 20.955,
):
    """按水平/垂直 FOV 分别设置（Orbbec Gemini 335 深度 90°×65°）。"""
    import math

    fov_h = math.radians(float(horizontal_fov_deg))
    fov_v = math.radians(float(vertical_fov_deg))
    focal_mm = (horiz_ap_mm / 2.0) / math.tan(fov_h / 2.0)
    vert_ap_mm = 2.0 * focal_mm * math.tan(fov_v / 2.0)
    return focal_mm, horiz_ap_mm, vert_ap_mm
