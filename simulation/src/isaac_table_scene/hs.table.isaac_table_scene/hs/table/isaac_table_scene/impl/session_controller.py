# -*- coding: utf-8 -*-
"""会话：Load / Play 出流 / 运行时 6D 位姿应用。"""

from __future__ import annotations

from enum import Enum, auto
from typing import Callable, Optional

import omni.timeline

from ..cassette_assets import center_height_in_link
from ..global_variables import (
    CAMERA_LINK_PATH,
    CUBOID_GEOM_PATH,
    CUBOID_LINK_PATH,
    DEFAULT_CAMERA_HEIGHT_Z,
    DEFAULT_CAMERA_ROTATION_DEG,
    DEFAULT_CUBOID_CENTER_XY,
    DEFAULT_TABLE_TOP_Z,
    ORBBEC_G335_DEPTH_HEIGHT,
    ORBBEC_G335_DEPTH_WIDTH,
)
from .pose_utils import Pose6D, parse_pose_from_fields, read_pose, set_pose
from .ros_io.camera_publisher import CameraPublisher
from .ros_io.tf_publisher import TfPublisher
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
        self.tf_publisher = TfPublisher()
        self._timeline = omni.timeline.get_timeline_interface()
        self._state = SessionState.IDLE
        self._topic_config = TopicConfig()
        self._resolution = (ORBBEC_G335_DEPTH_WIDTH, ORBBEC_G335_DEPTH_HEIGHT)
        self._on_status = on_status or (lambda s: print(f"[TableScene] {s}"))

        _obj_z = DEFAULT_TABLE_TOP_Z + center_height_in_link()
        self._object_pose_defaults = Pose6D(
            (DEFAULT_CUBOID_CENTER_XY[0], DEFAULT_CUBOID_CENTER_XY[1], _obj_z),
            (0.0, 0.0, 0.0),
        )
        self._camera_pose_defaults = Pose6D(
            (DEFAULT_CUBOID_CENTER_XY[0], 0.52, DEFAULT_CAMERA_HEIGHT_Z),
            DEFAULT_CAMERA_ROTATION_DEG,
        )
        self._live_apply_camera = False
        self._live_apply_object = True
        self._kinematic_object = False

        self._pose_reader: Optional[Callable[[], dict]] = None

    @property
    def state(self) -> SessionState:
        return self._state

    def status_text(self) -> str:
        names = {
            SessionState.IDLE: "IDLE",
            SessionState.LOADED: "LOADED (Press Play to publish ROS topics)",
            SessionState.STREAMING: "STREAMING (color / depth / pointcloud)",
        }
        return names.get(self._state, str(self._state))

    def configure(
        self,
        topic_config: TopicConfig,
        resolution: tuple,
        pose_reader: Callable[[], dict],
    ) -> None:
        self._topic_config = topic_config
        self._resolution = resolution
        self._pose_reader = pose_reader

    def set_live_apply(self, camera: bool, obj: bool, kinematic_object: bool) -> None:
        self._live_apply_camera = camera
        self._live_apply_object = obj
        self._kinematic_object = kinematic_object

    def load_scene(self, load_kwargs: dict) -> bool:
        self.stop_all()
        ok = self.scene_loader.load(**load_kwargs)
        if ok:
            self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()
        return ok

    def apply_poses_from_ui(self, force_object: bool = True, force_camera: bool = True) -> None:
        if not self.scene_loader.is_loaded or not self._pose_reader:
            return
        data = self._pose_reader()
        import omni.timeline
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        playing = omni.timeline.get_timeline_interface().is_playing()

        if force_camera or self._live_apply_camera:
            cam_pose = parse_pose_from_fields(
                data.get("cam_tx", "0"),
                data.get("cam_ty", "0"),
                data.get("cam_tz", "0"),
                data.get("cam_roll", str(DEFAULT_CAMERA_ROTATION_DEG[0])),
                data.get("cam_pitch", str(DEFAULT_CAMERA_ROTATION_DEG[1])),
                data.get("cam_yaw", str(DEFAULT_CAMERA_ROTATION_DEG[2])),
                self._camera_pose_defaults,
            )
            set_pose(CAMERA_LINK_PATH, cam_pose, stage)

        if force_object or self._live_apply_object:
            obj_pose = parse_pose_from_fields(
                data.get("obj_tx", "0"),
                data.get("obj_ty", "0"),
                data.get("obj_tz", f"{DEFAULT_TABLE_TOP_Z + 0.02:.3f}"),
                data.get("obj_roll", "0"),
                data.get("obj_pitch", "0"),
                data.get("obj_yaw", "0"),
                self._object_pose_defaults,
            )
            self._set_object_pose(obj_pose, stage, playing=playing)

    def sync_poses_to_ui(self) -> dict:
        """从 Stage 读取当前位姿，供 UI 回填。"""
        cam = read_pose(CAMERA_LINK_PATH)
        obj = read_pose(CUBOID_LINK_PATH)
        result = {}
        if cam:
            result.update(
                {
                    "cam_tx": f"{cam.translation[0]:.4f}",
                    "cam_ty": f"{cam.translation[1]:.4f}",
                    "cam_tz": f"{cam.translation[2]:.4f}",
                    "cam_roll": f"{cam.rotation_deg[0]:.2f}",
                    "cam_pitch": f"{cam.rotation_deg[1]:.2f}",
                    "cam_yaw": f"{cam.rotation_deg[2]:.2f}",
                }
            )
        if obj:
            result.update(
                {
                    "obj_tx": f"{obj.translation[0]:.4f}",
                    "obj_ty": f"{obj.translation[1]:.4f}",
                    "obj_tz": f"{obj.translation[2]:.4f}",
                    "obj_roll": f"{obj.rotation_deg[0]:.2f}",
                    "obj_pitch": f"{obj.rotation_deg[1]:.2f}",
                    "obj_yaw": f"{obj.rotation_deg[2]:.2f}",
                }
            )
        return result

    def _get_camera_pose_for_tf(self) -> Pose6D:
        pose = read_pose(CAMERA_LINK_PATH)
        if pose:
            return pose
        if self._pose_reader:
            data = self._pose_reader()
            return parse_pose_from_fields(
                data.get("cam_tx", "0"),
                data.get("cam_ty", "0"),
                data.get("cam_tz", "0"),
                data.get("cam_roll", str(DEFAULT_CAMERA_ROTATION_DEG[0])),
                data.get("cam_pitch", str(DEFAULT_CAMERA_ROTATION_DEG[1])),
                data.get("cam_yaw", str(DEFAULT_CAMERA_ROTATION_DEG[2])),
                self._camera_pose_defaults,
            )
        return self._camera_pose_defaults

    def _get_object_pose_for_tf(self) -> Pose6D:
        pose = read_pose(CUBOID_LINK_PATH)
        if pose:
            return pose
        if self._pose_reader:
            data = self._pose_reader()
            return parse_pose_from_fields(
                data.get("obj_tx", "0"),
                data.get("obj_ty", "0"),
                data.get("obj_tz", f"{DEFAULT_TABLE_TOP_Z + 0.02:.3f}"),
                data.get("obj_roll", "0"),
                data.get("obj_pitch", "0"),
                data.get("obj_yaw", "0"),
                self._object_pose_defaults,
            )
        return self._object_pose_defaults

    def _set_object_pose(self, pose: Pose6D, stage, *, playing: bool = False) -> None:
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(CUBOID_GEOM_PATH)
        if not prim or not prim.IsValid():
            prim = stage.GetPrimAtPath(CUBOID_LINK_PATH)
        if prim and prim.IsValid():
            rb = UsdPhysics.RigidBodyAPI(prim)
            if rb:
                # Play 时 Live apply 需 kinematic，否则 USD 改 transform 无效且易触发 PhysX 错误
                rb.CreateKinematicEnabledAttr().Set(bool(self._kinematic_object or playing))
        set_pose(CUBOID_LINK_PATH, pose, stage)

    def on_timeline_event(self, event) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if self.scene_loader.is_loaded:
                self._start_streaming()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._stop_streaming()

    def on_app_update(self) -> None:
        if not self.scene_loader.is_loaded:
            return
        if self._timeline.is_playing():
            if self._live_apply_camera or self._live_apply_object:
                self.apply_poses_from_ui(
                    force_object=self._live_apply_object,
                    force_camera=self._live_apply_camera,
                )
            if self.tf_publisher:
                self.tf_publisher.update()

    def stop_streaming_only(self) -> None:
        self._stop_streaming()

    def stop_all(self) -> None:
        self._stop_streaming()
        self.scene_loader.unload()
        self._state = SessionState.IDLE
        self._notify()

    def start_streaming_manual(self) -> bool:
        """不依赖 Timeline 事件，手动启动 ROS 出流。"""
        if not self.scene_loader.is_loaded:
            print("SessionController: Load scene first")
            return False
        self._start_streaming()
        return self._state == SessionState.STREAMING

    def _start_streaming(self) -> None:
        if not self.scene_loader.is_loaded:
            return
        if self._state == SessionState.STREAMING:
            return
        # 相机位姿已在 Load 时布置；勿用 UI 旧默认值覆盖
        self.apply_poses_from_ui(force_camera=False, force_object=True)
        cam_ok = self.camera_publisher.start(
            self.scene_loader.camera_prim_path,
            self._resolution,
            self._topic_config,
        )
        tf_ok = self.tf_publisher.start(
            self._topic_config,
            self.scene_loader.camera_link_path,
            self.scene_loader.cuboid_link_path,
        )
        if cam_ok or tf_ok:
            self._state = SessionState.STREAMING
        self._notify()

    def _stop_streaming(self) -> None:
        self.camera_publisher.stop()
        self.tf_publisher.stop()
        if self.scene_loader.is_loaded:
            self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()

    def _notify(self) -> None:
        self._on_status(self.status_text())
