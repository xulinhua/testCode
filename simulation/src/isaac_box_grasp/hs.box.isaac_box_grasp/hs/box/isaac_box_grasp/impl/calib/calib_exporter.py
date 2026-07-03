# -*- coding: utf-8 -*-
"""标定结果导出（yaml/json），不依赖 calib_sim_isaac。"""

from __future__ import annotations

import json
import os
from datetime import datetime

from ..interfaces import CalibExportResult, CameraIntrinsics, Transform6D


def export_calibration(
    output_dir: str,
    intrinsics: CameraIntrinsics,
    extrinsics: Transform6D,
    calibrated: bool,
) -> CalibExportResult:
    try:
        os.makedirs(output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        intr_path = os.path.join(output_dir, f"camera_intrinsics_{stamp}.json")
        extr_path = os.path.join(output_dir, f"camera_extrinsics_{stamp}.json")

        intr_payload = {
            "timestamp": stamp,
            "width": intrinsics.width,
            "height": intrinsics.height,
            "camera_matrix": intrinsics.k,
            "distortion_model": intrinsics.distortion_model,
            "distortion_coefficients": intrinsics.distortion,
        }
        extr_payload = {
            "timestamp": stamp,
            "calibrated": calibrated,
            "target_frame": extrinsics.target_frame,
            "source_frame": extrinsics.source_frame,
            "translation": extrinsics.translation,
            "rotation_xyzw": extrinsics.rotation_xyzw,
            "note": "T_target_source: p_target = R @ p_source + t",
        }

        with open(intr_path, "w", encoding="utf-8") as fp:
            json.dump(intr_payload, fp, indent=2, ensure_ascii=False)
        with open(extr_path, "w", encoding="utf-8") as fp:
            json.dump(extr_payload, fp, indent=2, ensure_ascii=False)

        print(f"Calibration exported:\n  {intr_path}\n  {extr_path}")
        return CalibExportResult(
            ok=True,
            intrinsics_path=intr_path,
            extrinsics_path=extr_path,
            message="export ok",
        )
    except Exception as exc:
        print(f"export_calibration failed: {exc}")
        return CalibExportResult(ok=False, message=str(exc))
