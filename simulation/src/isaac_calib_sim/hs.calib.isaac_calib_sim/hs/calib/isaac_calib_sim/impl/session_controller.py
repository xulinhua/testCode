# -*- coding: utf-8 -*-
"""Session: load scene, orbit camera on Play (no PhysX), ROS stream @ 30 FPS."""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import Callable, Optional

import omni.timeline

from ..global_variables import (
    DEFAULT_AZIM_RATE_DEG,
    DEFAULT_DIST_MAX,
    DEFAULT_DIST_MIN,
    DEFAULT_DIST_PERIOD_S,
    DEFAULT_ELEV_MAX_DEG,
    DEFAULT_ELEV_MIN_DEG,
    DEFAULT_ELEV_PERIOD_S,
    DEFAULT_HFOV_DEG,
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_IMAGE_WIDTH,
    DEFAULT_LOOKAT_OFFSET_M,
    DEFAULT_LOOKAT_PERIOD_S,
    DEFAULT_ROLL_AMP_DEG,
    DEFAULT_ROLL_PERIOD_S,
    DEFAULT_STREAM_FPS,
    TRI_AZIM_RATE_DEG,
    TRI_DIST_MAX,
    TRI_DIST_MIN,
    TRI_DIST_PERIOD_S,
    TRI_ELEV_MAX_DEG,
    TRI_ELEV_MIN_DEG,
    TRI_ELEV_PERIOD_S,
    TRI_LOOKAT_OFFSET_M,
    TRI_LOOKAT_PERIOD_S,
    TRI_ROLL_AMP_DEG,
    TRI_ROLL_PERIOD_S,
)
from .board_factory import BoardSpec
from .camera_motion import CameraOrbitMotion, OrbitParams
from .ros_io.camera_publisher import CameraPublisher
from .scene_loader import SceneLoader
from .topic_config import TopicConfig


class SessionState(Enum):
    IDLE = auto()
    LOADED = auto()
    STREAMING = auto()


class SessionController:
    def __init__(
        self,
        scene_loader: SceneLoader,
        on_status: Optional[Callable[[str], None]] = None,
    ):
        self.scene_loader = scene_loader
        self.camera_publisher = CameraPublisher()
        self.motion = CameraOrbitMotion()
        self._timeline = omni.timeline.get_timeline_interface()
        self._state = SessionState.IDLE
        self._topic_config = TopicConfig()
        self._resolution = (DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT)
        self._hfov = DEFAULT_HFOV_DEG
        self._stream_fps = DEFAULT_STREAM_FPS
        self._on_status = on_status or (lambda s: print(f"[CalibSim] {s}"))
        self._last_t = None
        self._motion_enabled = True

    @property
    def state(self) -> SessionState:
        return self._state

    def status_text(self) -> str:
        names = {
            SessionState.IDLE: "IDLE",
            SessionState.LOADED: "LOADED — Press Play to orbit and stream",
            SessionState.STREAMING: f"STREAMING — {self._stream_fps:.0f} FPS, no physics",
        }
        return names.get(self._state, str(self._state))

    def configure(
        self,
        topic_config: TopicConfig,
        resolution: tuple,
        hfov_deg: float,
        orbit: OrbitParams,
        stream_fps: float = DEFAULT_STREAM_FPS,
    ) -> None:
        self._topic_config = topic_config
        self._resolution = resolution
        self._hfov = hfov_deg
        self._stream_fps = max(1.0, float(stream_fps))
        self.motion.set_params(orbit)
        self._motion_enabled = orbit.enabled

    def load_scene(self, board_spec: BoardSpec) -> bool:
        self.stop_all()
        ok = self.scene_loader.load(
            board_spec=board_spec,
            camera_resolution=self._resolution,
            horizontal_fov_deg=self._hfov,
        )
        if ok:
            self.motion.reset(azim0_deg=20.0)
            self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()
        return ok

    def switch_board(self, board_spec: BoardSpec) -> bool:
        if not self.scene_loader.is_loaded:
            self._on_status("Load scene first")
            return False
        ok = self.scene_loader.switch_board(board_spec)
        self._notify(extra=f"board={board_spec.board_type}" if ok else "switch failed")
        return ok

    def start_stream(self) -> bool:
        if not self.scene_loader.is_loaded:
            self._on_status("Load scene first")
            return False
        self._apply_stream_fps()
        ok = self.camera_publisher.start(
            self.scene_loader.camera_prim_path,
            self._resolution,
            self._topic_config,
            stream_fps=self._stream_fps,
        )
        if ok:
            self._state = SessionState.STREAMING
            if not self._timeline.is_playing():
                self._timeline.play()
        self._notify()
        return ok

    def stop_streaming_only(self) -> None:
        self.camera_publisher.stop()
        if self._state == SessionState.STREAMING:
            self._state = SessionState.LOADED if self.scene_loader.is_loaded else SessionState.IDLE
        self._notify()

    def stop_all(self) -> None:
        self.camera_publisher.stop()
        self.scene_loader.unload()
        self._state = SessionState.IDLE
        self._last_t = None
        self._notify()

    def on_timeline_event(self, event) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._apply_stream_fps()
            if self.scene_loader.is_loaded and not self.camera_publisher.is_active:
                self.camera_publisher.start(
                    self.scene_loader.camera_prim_path,
                    self._resolution,
                    self._topic_config,
                    stream_fps=self._stream_fps,
                )
                self._state = SessionState.STREAMING
                self._notify()
            self._last_t = None
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.camera_publisher.stop()
            if self.scene_loader.is_loaded:
                self._state = SessionState.LOADED
            self._last_t = None
            self._notify()

    def on_physics_step(self, step) -> None:
        # Physics disabled — orbit runs from on_app_update.
        _ = step

    def on_app_update(self) -> None:
        """Drive camera orbit from app tick (no PhysX / collision)."""
        if not self.scene_loader.is_loaded or not self._motion_enabled:
            return
        if not self._timeline.is_playing():
            return
        now = time.perf_counter()
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        self._last_t = now
        # Clamp dt to avoid huge jumps after pause
        dt = max(0.0, min(dt, 0.1))
        eye, lookat, dist, elev, azim, roll = self.motion.step(
            dt, self.scene_loader.board_target
        )
        self.scene_loader.set_camera_look_at(eye, lookat=lookat, roll_deg=roll)

    def _apply_stream_fps(self) -> None:
        fps = max(1.0, float(self._stream_fps))
        try:
            # Prefer explicit target framerate when available
            if hasattr(self._timeline, "set_target_framerate"):
                self._timeline.set_target_framerate(fps)
        except Exception as exc:
            print(f"[CalibSim] set_target_framerate skipped: {exc}")
        try:
            if hasattr(self._timeline, "set_ticks_per_second"):
                self._timeline.set_ticks_per_second(fps)
        except Exception as exc:
            print(f"[CalibSim] set_ticks_per_second skipped: {exc}")
        try:
            # Time codes / second ≈ publish cadence for OnPlaybackTick
            if hasattr(self._timeline, "set_time_codes_per_second"):
                self._timeline.set_time_codes_per_second(fps)
        except Exception as exc:
            print(f"[CalibSim] set_time_codes_per_second skipped: {exc}")
        print(f"[CalibSim] stream target FPS={fps:.0f}")

    def _notify(self, extra: str = "") -> None:
        msg = self.status_text()
        if extra:
            msg = f"{msg} | {extra}"
        self._on_status(msg)


def orbit_params_for_board(board_type: str, enabled: bool = True) -> OrbitParams:
    """Planar boards: wide motion. Trihedral: stay in 3-face viewing cone."""
    if str(board_type).startswith("trihedral_"):
        return OrbitParams(
            dist_min=TRI_DIST_MIN,
            dist_max=TRI_DIST_MAX,
            elev_min_deg=TRI_ELEV_MIN_DEG,
            elev_max_deg=TRI_ELEV_MAX_DEG,
            azim_rate_deg=TRI_AZIM_RATE_DEG,
            dist_period_s=TRI_DIST_PERIOD_S,
            elev_period_s=TRI_ELEV_PERIOD_S,
            lookat_offset_m=TRI_LOOKAT_OFFSET_M,
            lookat_period_s=TRI_LOOKAT_PERIOD_S,
            roll_amp_deg=TRI_ROLL_AMP_DEG,
            roll_period_s=TRI_ROLL_PERIOD_S,
            enabled=enabled,
        )
    return OrbitParams(
        dist_min=DEFAULT_DIST_MIN,
        dist_max=DEFAULT_DIST_MAX,
        elev_min_deg=DEFAULT_ELEV_MIN_DEG,
        elev_max_deg=DEFAULT_ELEV_MAX_DEG,
        azim_rate_deg=DEFAULT_AZIM_RATE_DEG,
        dist_period_s=DEFAULT_DIST_PERIOD_S,
        elev_period_s=DEFAULT_ELEV_PERIOD_S,
        lookat_offset_m=DEFAULT_LOOKAT_OFFSET_M,
        lookat_period_s=DEFAULT_LOOKAT_PERIOD_S,
        roll_amp_deg=DEFAULT_ROLL_AMP_DEG,
        roll_period_s=DEFAULT_ROLL_PERIOD_S,
        enabled=enabled,
    )


def default_orbit_params() -> OrbitParams:
    return orbit_params_for_board("chessboard", enabled=True)
