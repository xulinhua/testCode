# -*- coding: utf-8 -*-
"""会话状态：Load / Timeline 出流 / Start 抓取 / cmd_vel 底盘。"""

from __future__ import annotations

from enum import Enum, auto
from typing import Callable, Optional

import omni.timeline

from ..defaults import DEFAULT_ROBOT_USD, DEFAULT_SCENE_USD
from .calib.calib_session import HandEyeCalibSession
from .calib.extrinsics_provider import StageExtrinsicsProvider
from .calib.intrinsics_provider import StageIntrinsicsProvider
from .camera_pipeline import CameraPipeline
from .grasp.grasp_controller import IsaacGraspController
from .ros_io.camera_publisher import CameraPublisher
from .ros_io.cmd_vel_subscriber import CmdVelSubscriber
from .ros_io.pose_subscriber import PoseSubscriber
from .ros_io.tf_publisher import TfPublisher
from .scene_loader import CAMERA_PRIM_PATH, SceneLoader
from .topic_config import TopicConfig


class SessionState(Enum):
    IDLE = auto()
    LOADED = auto()
    STREAMING = auto()
    GRASP_ARMED = auto()


class SessionController:
    def __init__(
        self,
        scene_loader: SceneLoader,
        on_status: Optional[Callable[[str], None]] = None,
    ):
        self.scene_loader = scene_loader
        self.camera_pipeline = CameraPipeline()
        self.camera_publisher = CameraPublisher()
        self.tf_publisher = TfPublisher()
        self.pose_subscriber = PoseSubscriber()
        self.cmd_vel_subscriber = CmdVelSubscriber()
        self.grasp_controller = IsaacGraspController()
        self.intrinsics_provider = StageIntrinsicsProvider()
        self.extrinsics_provider = StageExtrinsicsProvider()
        self.calib_session = HandEyeCalibSession(
            self.intrinsics_provider, self.extrinsics_provider
        )
        self._timeline = omni.timeline.get_timeline_interface()
        self._state = SessionState.IDLE
        self._topic_config = TopicConfig()
        self._camera_prim_path = CAMERA_PRIM_PATH
        self._resolution = (640, 480)
        self._on_status = on_status or (lambda s: print(f"[Session] {s}"))
        self._cmd_vel_active = False

    @property
    def state(self) -> SessionState:
        return self._state

    def status_text(self) -> str:
        scene = "scene+robot" if self.scene_loader.scene_loaded else "robot only"
        names = {
            SessionState.IDLE: "IDLE",
            SessionState.LOADED: f"LOADED ({scene}, Press Play to stream)",
            SessionState.STREAMING: "STREAMING (ROS camera + cmd_vel)",
            SessionState.GRASP_ARMED: "GRASP_ARMED (waiting PoseStamped)",
        }
        return names.get(self._state, str(self._state))

    def configure(
        self,
        topic_config: TopicConfig,
        intrinsics: StageIntrinsicsProvider,
        camera_prim_path: str,
        resolution: tuple,
    ) -> None:
        self._topic_config = topic_config
        self.intrinsics_provider = intrinsics
        self.extrinsics_provider = StageExtrinsicsProvider(
            world_frame=topic_config.tf_world_frame,
            camera_frame=topic_config.tf_camera_frame,
        )
        self.calib_session = HandEyeCalibSession(
            self.intrinsics_provider, self.extrinsics_provider
        )
        self._camera_prim_path = camera_prim_path
        self._resolution = resolution

    def load_scene(
        self,
        scene_usd: str = DEFAULT_SCENE_USD,
        robot_usd: str = DEFAULT_ROBOT_USD,
        camera_position=(0, 0, 1.2),
    ) -> bool:
        self.stop_all()
        ok = self.scene_loader.load(
            scene_usd=scene_usd,
            robot_usd=robot_usd,
            camera_position=camera_position,
            camera_resolution=self._resolution,
            horizontal_fov_deg=self.intrinsics_provider.horizontal_fov_deg,
        )
        if ok:
            self._camera_prim_path = self.scene_loader.camera_prim_path
            self.camera_pipeline.setup(self._camera_prim_path, self._resolution)
            self.grasp_controller.bind_robot_runtime(self.scene_loader.robot_runtime)
            self._state = SessionState.LOADED
            self._notify()
        else:
            self.grasp_controller.bind_robot_runtime(None)
            self._state = SessionState.IDLE
            self._notify()
        return ok

    def on_timeline_event(self, event) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if self.scene_loader.is_loaded:
                self._start_streaming()
                self._start_cmd_vel()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._stop_streaming()
            self._stop_cmd_vel()

    def start_grasp_mode(self) -> bool:
        if not self.scene_loader.is_loaded:
            print("SessionController: Load scene first")
            return False
        if self._state == SessionState.IDLE:
            return False
        if not self.camera_publisher.is_active and not self._timeline.is_playing():
            print("SessionController: Press Play to start camera stream before grasp")
        self.pose_subscriber.start(self._topic_config, self._on_pose_received)
        self._state = SessionState.GRASP_ARMED
        self._notify()
        return True

    def stop_grasp_mode(self) -> None:
        self.pose_subscriber.stop()
        if self.camera_publisher.is_active:
            self._state = SessionState.STREAMING
        elif self.scene_loader.is_loaded:
            self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()

    def stop_all(self) -> None:
        self.pose_subscriber.stop()
        self._stop_cmd_vel()
        self._stop_streaming()
        self.scene_loader.unload()
        self.camera_pipeline.teardown()
        self.grasp_controller.bind_robot_runtime(None)
        self.calib_session.cancel_sampling()
        self._state = SessionState.IDLE
        self._notify()

    def on_update(self) -> None:
        if self.pose_subscriber.is_active:
            self.pose_subscriber.spin_once()
        if self.cmd_vel_subscriber.is_active:
            self.cmd_vel_subscriber.spin_once()

    def on_physics_step(self, _step) -> None:
        if self.scene_loader.robot_runtime.is_ready and self._cmd_vel_active:
            self.scene_loader.robot_runtime.apply_wheel_velocities()

    def _start_cmd_vel(self) -> None:
        if not self.scene_loader.is_loaded:
            return
        rt = self.scene_loader.robot_runtime

        def _on_twist(lin: float, ang: float) -> None:
            rt.set_cmd_vel(lin, ang)

        ok = self.cmd_vel_subscriber.start(self._topic_config, _on_twist)
        self._cmd_vel_active = ok

    def _stop_cmd_vel(self) -> None:
        self.cmd_vel_subscriber.stop()
        self._cmd_vel_active = False
        if self.scene_loader.robot_runtime.is_ready:
            self.scene_loader.robot_runtime.set_cmd_vel(0.0, 0.0)
            self.scene_loader.robot_runtime.apply_wheel_velocities()

    def _start_streaming(self) -> None:
        if not self.scene_loader.is_loaded:
            return
        extr = self.extrinsics_provider.get_T_world_camera()
        cam_ok = self.camera_publisher.start(
            self._camera_prim_path,
            self._resolution,
            self._topic_config,
        )
        tf_ok = self.tf_publisher.start(self._topic_config, extr)
        if cam_ok or tf_ok:
            self._state = (
                SessionState.STREAMING
                if not self.pose_subscriber.is_active
                else SessionState.GRASP_ARMED
            )
        self._notify()

    def _stop_streaming(self) -> None:
        self.camera_publisher.stop()
        self.tf_publisher.stop()
        if self.pose_subscriber.is_active:
            self._state = SessionState.GRASP_ARMED
        elif self.scene_loader.is_loaded:
            self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()

    def _on_pose_received(self, pose_world) -> None:
        result = self.grasp_controller.execute_grasp(pose_world)
        print(f"Grasp result: ok={result.ok} {result.message}")

    def _notify(self) -> None:
        self._on_status(self.status_text())
