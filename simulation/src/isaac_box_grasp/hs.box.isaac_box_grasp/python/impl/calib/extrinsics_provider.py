# -*- coding: utf-8 -*-
"""外参：未标定时从 Stage 读取 T_world_camera。"""

from __future__ import annotations

from typing import Optional

from ..interfaces import Transform6D


class StageExtrinsicsProvider:
    def __init__(self, world_frame: str = "world", camera_frame: str = "camera_optical_frame"):
        self._world_frame = world_frame
        self._camera_frame = camera_frame
        self._override: Optional[Transform6D] = None

    @property
    def is_calibrated(self) -> bool:
        return self._override is not None

    def get_T_world_camera(self) -> Transform6D:
        if self._override is not None:
            return self._override
        return self._read_from_stage()

    def set_T_world_camera(self, transform: Transform6D) -> None:
        self._override = transform

    def clear_calibration_override(self) -> None:
        self._override = None

    def _read_from_stage(self) -> Transform6D:
        try:
            from pxr import Gf, Usd, UsdGeom
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return self._identity_fallback("no stage")

            cam_path = self._find_camera_prim_path(stage)
            if not cam_path:
                return self._identity_fallback("camera prim not found")

            cam_prim = stage.GetPrimAtPath(cam_path)
            if not cam_prim:
                return self._identity_fallback(cam_path)

            xformable = UsdGeom.Xformable(cam_prim)
            world_xf = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            if isinstance(world_xf, Gf.Matrix4d):
                mat = world_xf
            else:
                mat = Gf.Matrix4d(world_xf)

            trans = mat.ExtractTranslation()
            rot = mat.ExtractRotation().GetQuat()
            imag = rot.GetImaginary()
            return Transform6D(
                translation=[float(trans[0]), float(trans[1]), float(trans[2])],
                rotation_xyzw=[
                    float(imag[0]),
                    float(imag[1]),
                    float(imag[2]),
                    float(rot.GetReal()),
                ],
                source_frame=self._camera_frame,
                target_frame=self._world_frame,
            )
        except Exception as exc:
            print(f"StageExtrinsicsProvider: {exc}")
            return self._identity_fallback(str(exc))

    def _find_camera_prim_path(self, stage) -> Optional[str]:
        for path in (
            "/World/box_grasp_camera",
            "/World/Camera",
            "/Camera",
        ):
            if stage.GetPrimAtPath(path):
                return path
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Camera":
                return str(prim.GetPath())
        return None

    @staticmethod
    def _identity_fallback(reason: str) -> Transform6D:
        print(f"StageExtrinsicsProvider fallback identity ({reason})")
        return Transform6D(
            translation=[0.0, 0.0, 0.0],
            rotation_xyzw=[0.0, 0.0, 0.0, 1.0],
            source_frame="camera_optical_frame",
            target_frame="world",
        )
