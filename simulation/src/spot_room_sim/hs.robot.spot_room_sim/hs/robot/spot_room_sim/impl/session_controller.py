# -*- coding: utf-8 -*-
"""会话：Load → Play 出流 / 步态；Stop 停流。"""

from __future__ import annotations

from enum import Enum, auto
from typing import Callable, Optional

import omni.physx as _physx
import omni.timeline

from .kit_extensions import ensure_ros2_bridge_enabled
from .ros_io.camera_publisher import CameraPublisher
from .ros_io.cmd_vel_subscriber import CmdVelSubscriber
from .ros_io.joint_state_publisher import JointStatePublisher
from .ros_io.odom_tf_publisher import OdomTfPublisher
from .scene_loader import SceneLoader
from .topic_config import TopicConfig


class SessionState(Enum):
    IDLE = auto()
    LOADING = auto()
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
        self.joint_state_publisher = JointStatePublisher()
        self.odom_tf_publisher = OdomTfPublisher()
        self.cmd_vel_subscriber = CmdVelSubscriber()
        self._timeline = omni.timeline.get_timeline_interface()
        self._state = SessionState.IDLE
        self._topic_config = TopicConfig()
        self._on_status = on_status or (lambda s: print(f"[SpotRoomSim] {s}"))
        self._cmd_vel_active = False
        self._physx = _physx.acquire_physx_interface()
        self._physx_sub = None
        self._ui_driving = False
        self._ui_cmd = (0.0, 0.0, 0.0)

    @property
    def state(self) -> SessionState:
        return self._state

    def status_text(self) -> str:
        names = {
            SessionState.IDLE: "IDLE",
            SessionState.LOADING: "LOADING (async World init)...",
            SessionState.LOADED: "LOADED (Press Play to stream + walk)",
            SessionState.STREAMING: "STREAMING (camera + cmd_vel + odom/tf/joint_states)",
        }
        return names.get(self._state, str(self._state))

    def configure(self, topic_config: TopicConfig) -> None:
        self._topic_config = topic_config

    async def load_scene_async(self) -> bool:
        self.stop_streaming_only()
        self._clear_physx_sub()
        self._state = SessionState.LOADING
        self._notify()
        cfg = self._topic_config
        ok = await self.scene_loader.load_async(
            image_size=(cfg.image_width, cfg.image_height),
        )
        if ok:
            self._state = SessionState.LOADED
            self._ensure_physx_sub()
        else:
            self._state = SessionState.IDLE
        self._notify()
        return ok

    def stop_all(self) -> None:
        self.stop_streaming_only()
        self._clear_physx_sub()
        self.scene_loader.unload()
        self._state = SessionState.IDLE
        self._notify()

    def stop_streaming_only(self) -> None:
        self._ui_driving = False
        self._ui_cmd = (0.0, 0.0, 0.0)
        self._stop_cmd_vel()
        self.camera_publisher.stop()
        self.joint_state_publisher.stop()
        self.odom_tf_publisher.stop()
        if self.scene_loader.is_loaded:
            self.scene_loader.spot_runtime.set_cmd_vel(0.0, 0.0, 0.0)
            self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE

    def set_ui_cmd_vel(self, vx: float, vy: float, wz: float, *, active: bool = True) -> None:
        """面板遥控；开启时优先于 ROS /cmd_vel。"""
        self._ui_cmd = (float(vx), float(vy), float(wz))
        self._ui_driving = bool(active)
        if self.scene_loader.is_loaded:
            self.scene_loader.spot_runtime.set_cmd_vel(*self._ui_cmd)

    def stop_ui_drive(self) -> None:
        self.set_ui_cmd_vel(0.0, 0.0, 0.0, active=False)

    def on_timeline_event(self, event) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if self.scene_loader.is_loaded:
                self._ensure_physx_sub()
                # Stop→Play 后 SimulationView 必换新：无条件硬初始化
                self.scene_loader.spot_runtime.mark_needs_init()
                self._start_streaming()
                self._start_cmd_vel()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.stop_streaming_only()
            if self.scene_loader.is_loaded:
                self.scene_loader.spot_runtime.mark_needs_init()
            self._notify()

    def on_update(self) -> None:
        # 面板驾驶时覆盖 ROS，避免被空闲 cmd_vel（默认零速）冲掉
        if self._ui_driving and self.scene_loader.is_loaded:
            self.scene_loader.spot_runtime.set_cmd_vel(*self._ui_cmd)
        elif self.cmd_vel_subscriber.is_active:
            self.cmd_vel_subscriber.spin_once()
        if self.odom_tf_publisher.is_active:
            self.odom_tf_publisher.spin_once()

    def _clear_physx_sub(self) -> None:
        """释放 PhysX 步进订阅，避免 Unload/Reload 时旧回调踩到已销毁 World。"""
        if self._physx_sub is not None:
            print("SessionController: clear PhysX step subscription")
        self._physx_sub = None

    def _ensure_physx_sub(self) -> None:
        if self._physx_sub is not None:
            return
        self._physx_sub = self._physx.subscribe_physics_step_events(self.on_physics_step)
        print("SessionController: physics callback via PhysX (timeline-safe)")

    def _start_streaming(self) -> None:
        if not self.scene_loader.is_loaded:
            return
        if not ensure_ros2_bridge_enabled():
            print("SessionController: ROS2 bridge unavailable")
            return
        cfg = self._topic_config
        cam_ok = self.camera_publisher.start(self.scene_loader.camera_prim_path, cfg)
        js_ok = self.joint_state_publisher.start(cfg, self.scene_loader.spot_runtime.prim_path)
        chassis = self.scene_loader.body_prim_path or self.scene_loader.spot_runtime.prim_path
        odom_ok = self.odom_tf_publisher.start(
            cfg,
            self.scene_loader.cam_xyz_base,
            self.scene_loader.cam_link_quat_xyzw_base,
            self.scene_loader.cam_optical_from_link_quat_xyzw,
            chassis_prim_path=chassis,
        )
        if cam_ok or js_ok or odom_ok:
            self._state = SessionState.STREAMING
            self._notify()
            # RenderProduct 可能把 Viewport 切到 front_cam，再拉回俯视 Spot
            try:
                spawn = getattr(
                    self.scene_loader.spot_runtime,
                    "_spawn_pos",
                    None,
                )
                if spawn is None:
                    from ..global_variables import SPOT_SPAWN_POS

                    spawn = SPOT_SPAWN_POS
                self.scene_loader._frame_viewport_in_room(
                    (float(spawn[0]), float(spawn[1]), float(spawn[2]))
                )
            except Exception as exc:
                print(f"SessionController: re-frame viewport skipped: {exc}")
        else:
            print("SessionController: no ROS stream started")

    def on_physics_step(self, step_size) -> None:
        if not self.scene_loader.is_loaded:
            return
        # 物理步频率高于 UI update：驱动中每步重刷，避免被 ROS 默认零速冲掉
        if self._ui_driving:
            self.scene_loader.spot_runtime.set_cmd_vel(*self._ui_cmd)
        self.scene_loader.spot_runtime.on_physics_step(float(step_size))

    def _start_cmd_vel(self) -> None:
        if not self.scene_loader.is_loaded:
            return
        rt = self.scene_loader.spot_runtime

        def _on_twist(vx: float, vy: float, wz: float) -> None:
            if self._ui_driving:
                return
            rt.set_cmd_vel(vx, vy, wz)

        self._cmd_vel_active = self.cmd_vel_subscriber.start(self._topic_config, _on_twist)

    def _stop_cmd_vel(self) -> None:
        self.cmd_vel_subscriber.stop()
        self._cmd_vel_active = False
        if self.scene_loader.is_loaded:
            self.scene_loader.spot_runtime.set_cmd_vel(0.0, 0.0, 0.0)

    def _notify(self) -> None:
        self._on_status(self.status_text())
