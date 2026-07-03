# -*- coding: utf-8 -*-
"""手眼标定采样：采集外参样本并写回 StageExtrinsicsProvider。"""

from __future__ import annotations

from typing import List, Optional

from ..interfaces import CalibExportResult, ICalibrationSession, Transform6D
from .calib_exporter import export_calibration
from .extrinsics_provider import StageExtrinsicsProvider
from .intrinsics_provider import StageIntrinsicsProvider


class HandEyeCalibSession(ICalibrationSession):
    def __init__(
        self,
        intrinsics: StageIntrinsicsProvider,
        extrinsics: StageExtrinsicsProvider,
    ):
        self._intrinsics = intrinsics
        self._extrinsics = extrinsics
        self._samples: List[Transform6D] = []
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def start_sampling(self) -> None:
        self._samples.clear()
        self._active = True
        print("HandEyeCalibSession: sampling started")

    def capture_sample(self) -> bool:
        if not self._active:
            print("HandEyeCalibSession: start sampling first")
            return False
        sample = self._extrinsics.get_T_world_camera()
        self._samples.append(sample)
        print(f"HandEyeCalibSession: captured sample #{len(self._samples)} t={sample.translation}")
        return True

    def cancel_sampling(self) -> None:
        self._samples.clear()
        self._active = False
        print("HandEyeCalibSession: sampling cancelled")

    def finish_and_apply(self) -> bool:
        if not self._samples:
            print("HandEyeCalibSession: no samples")
            return False
        t_sum = [0.0, 0.0, 0.0]
        q_sum = [0.0, 0.0, 0.0, 0.0]
        for s in self._samples:
            t_sum[0] += s.translation[0]
            t_sum[1] += s.translation[1]
            t_sum[2] += s.translation[2]
            q_sum[0] += s.rotation_xyzw[0]
            q_sum[1] += s.rotation_xyzw[1]
            q_sum[2] += s.rotation_xyzw[2]
            q_sum[3] += s.rotation_xyzw[3]
        n = float(len(self._samples))
        avg = Transform6D(
            translation=[t_sum[0] / n, t_sum[1] / n, t_sum[2] / n],
            rotation_xyzw=[q_sum[0] / n, q_sum[1] / n, q_sum[2] / n, q_sum[3] / n],
            source_frame=self._samples[0].source_frame,
            target_frame=self._samples[0].target_frame,
        )
        self._extrinsics.set_T_world_camera(avg)
        self._active = False
        print(f"HandEyeCalibSession: applied average of {int(n)} samples")
        return True

    def finish_and_export(self, output_dir: str) -> CalibExportResult:
        if self._active and self._samples:
            self.finish_and_apply()
        return export_calibration(
            output_dir,
            self._intrinsics.get_intrinsics(),
            self._extrinsics.get_T_world_camera(),
            calibrated=self._extrinsics.is_calibrated,
        )
