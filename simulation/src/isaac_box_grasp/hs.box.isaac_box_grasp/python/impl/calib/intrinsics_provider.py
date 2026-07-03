# -*- coding: utf-8 -*-
"""从分辨率与水平 FOV 计算内参及 USD 相机属性。"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from ..interfaces import CameraIntrinsics

DEFAULT_FOCAL_LENGTH_MM = 24.0
# Legacy defaults: focal=24 mm, horiz_aperture=20.955 mm
DEFAULT_HORIZONTAL_FOV_DEG = math.degrees(
    2.0 * math.atan(20.955 / (2.0 * DEFAULT_FOCAL_LENGTH_MM))
)


def usd_camera_attrs_from_fov(
    width: int,
    height: int,
    horizontal_fov_deg: float,
    focal_length_mm: float = DEFAULT_FOCAL_LENGTH_MM,
) -> Tuple[float, float, float]:
    """Derive USD focal length and sensor apertures from horizontal FOV."""
    half_fov_rad = math.radians(horizontal_fov_deg) / 2.0
    horiz_aperture_mm = 2.0 * focal_length_mm * math.tan(half_fov_rad)
    vert_aperture_mm = horiz_aperture_mm * (height / width)
    return focal_length_mm, horiz_aperture_mm, vert_aperture_mm


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


def compute_intrinsics_k_from_fov(
    width: int,
    height: int,
    horizontal_fov_deg: float,
    focal_length_mm: float = DEFAULT_FOCAL_LENGTH_MM,
) -> List[List[float]]:
    focal, horiz_ap, vert_ap = usd_camera_attrs_from_fov(
        width, height, horizontal_fov_deg, focal_length_mm
    )
    return compute_intrinsics_k(width, height, focal, horiz_ap, vert_ap)


class StageIntrinsicsProvider:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        horizontal_fov_deg: float = DEFAULT_HORIZONTAL_FOV_DEG,
    ):
        self.width = width
        self.height = height
        self.horizontal_fov_deg = horizontal_fov_deg

    def get_usd_camera_attrs(self) -> Tuple[float, float, float]:
        return usd_camera_attrs_from_fov(self.width, self.height, self.horizontal_fov_deg)

    def get_intrinsics(self) -> CameraIntrinsics:
        k = compute_intrinsics_k_from_fov(self.width, self.height, self.horizontal_fov_deg)
        return CameraIntrinsics(
            width=self.width,
            height=self.height,
            k=k,
            distortion=[0.0, 0.0, 0.0, 0.0, 0.0],
        )

    def get_camera_info_dict(self) -> Dict[str, Any]:
        intr = self.get_intrinsics()
        fx, fy, cx, cy = intr.k[0][0], intr.k[1][1], intr.k[0][2], intr.k[1][2]
        return {
            "width": intr.width,
            "height": intr.height,
            "k": intr.k,
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "horizontal_fov_deg": self.horizontal_fov_deg,
            "distortion_model": intr.distortion_model,
            "d": intr.distortion,
        }
