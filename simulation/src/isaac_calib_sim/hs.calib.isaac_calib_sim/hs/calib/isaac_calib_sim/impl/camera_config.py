# -*- coding: utf-8 -*-
"""相机内参 / FOV → USD 相机属性。"""

from __future__ import annotations

import math
from typing import Tuple


def usd_camera_attrs_from_fov(
    width: int, height: int, horizontal_fov_deg: float
) -> Tuple[float, float, float]:
    fov_rad = math.radians(float(horizontal_fov_deg))
    horiz_ap_mm = 20.955
    focal_mm = (horiz_ap_mm / 2.0) / math.tan(fov_rad / 2.0)
    vert_ap_mm = horiz_ap_mm * (float(height) / float(width))
    return focal_mm, horiz_ap_mm, vert_ap_mm
