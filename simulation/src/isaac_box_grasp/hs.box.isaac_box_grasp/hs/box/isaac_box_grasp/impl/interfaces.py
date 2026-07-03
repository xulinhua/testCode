# -*- coding: utf-8 -*-
"""抽象接口：抓取、标定、机器人加载与位姿修正（预留）。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Protocol


@dataclass
class CameraIntrinsics:
    width: int
    height: int
    k: List[List[float]]
    distortion: List[float]
    distortion_model: str = "plumb_bob"


@dataclass
class Transform6D:
    """T_target_source: p_target = R @ p_source + t"""

    translation: List[float]
    rotation_xyzw: List[float]
    source_frame: str = ""
    target_frame: str = ""


@dataclass
class GraspResult:
    ok: bool
    message: str = ""
    request_id: str = ""


@dataclass
class CalibExportResult:
    ok: bool
    intrinsics_path: str = ""
    extrinsics_path: str = ""
    message: str = ""


class ICameraModelProvider(Protocol):
    def get_intrinsics(self) -> CameraIntrinsics: ...

    def get_camera_info_dict(self) -> Dict[str, Any]: ...


class ITransformProvider(Protocol):
    def get_T_world_camera(self) -> Transform6D: ...

    def set_T_world_camera(self, transform: Transform6D) -> None: ...

    def is_calibrated(self) -> bool: ...


class IGraspExecutor(Protocol):
    def execute_grasp(self, pose_world: Transform6D) -> GraspResult: ...


class IRobotLoader(Protocol):
    def load_robot(self, asset_name: str, prim_path: str = "") -> bool: ...


class ICalibrationSession(Protocol):
    def start_sampling(self) -> None: ...

    def capture_sample(self) -> bool: ...

    def cancel_sampling(self) -> None: ...

    def finish_and_apply(self) -> bool: ...

    def finish_and_export(self, output_dir: str) -> CalibExportResult: ...


class IPoseCorrector(Protocol):
    """预留：标定完成后对外部位姿做修正。"""

    def correct_pose_world(self, pose: Transform6D) -> Transform6D: ...
