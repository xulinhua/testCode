# Copyright (c) 2024
"""相机内参计算与 camera_params.json 写入。"""

import json
import os
from typing import Any, Dict, List


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


def write_camera_params_json(
    output_dir: str,
    settings: Dict[str, Any],
    frame_poses: List[Dict[str, Any]],
) -> str:
    path = os.path.join(output_dir, "camera_params.json")
    payload = dict(settings)
    payload["frames"] = frame_poses
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(payload, fp, indent=2, ensure_ascii=False)
    return path
