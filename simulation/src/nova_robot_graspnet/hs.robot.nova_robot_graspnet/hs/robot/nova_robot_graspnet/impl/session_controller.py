# -*- coding: utf-8 -*-
"""会话控制器：Load/Unload、Timeline Play 触发 ROS 出流、抓取盒位姿同步。"""

from __future__ import annotations

from enum import Enum, auto
from typing import Callable, Optional

import asyncio
import omni.timeline

from ..global_variables import BOX_LINK_PATH, DEFAULT_BOX_CENTER
from .pose_utils import Pose6D, parse_pose_from_fields, read_pose, set_pose
from .ros_io.multi_camera_publisher import MultiCameraPublisher
from .kit_extensions import ensure_ros2_bridge_enabled
from .ros_io.robot_ros_publisher import RobotRosPublisher
from .scene_loader import SceneLoader
from .topic_config import SessionTopicConfig


class SessionState(Enum):
    """扩展会话状态机。"""

    IDLE = auto()       # 未 Load 或已 Unload
    LOADED = auto()     # 场景在 Stage 上，Timeline 可 Play
    STREAMING = auto()  # OmniGraph 已创建，正在发布 ROS


class SessionController:
    """协调 SceneLoader 与 ROS 发布器，响应 UI / Timeline 事件。

    典型流程：
      Load scene → LOADED → Play / Start stream → STREAMING → Stop → LOADED
    """

    def __init__(
        self,
        scene_loader: SceneLoader,
        on_status: Optional[Callable[[str], None]] = None,
    ):
        """Args:
            scene_loader: 场景加载器实例。
            on_status: 状态变更回调（UI 状态栏）。
        """
        self.scene_loader = scene_loader
        self.multi_camera_publisher = MultiCameraPublisher()
        self.robot_ros_publisher = RobotRosPublisher()
        self._timeline = omni.timeline.get_timeline_interface()
        self._state = SessionState.IDLE
        self._topic_config = SessionTopicConfig.default()
        self._on_status = on_status or (lambda s: print(f"[NovaGraspNet] {s}"))

        self._object_pose_defaults = Pose6D(DEFAULT_BOX_CENTER, (0.0, 0.0, 0.0))
        self._live_apply_object = False
        self._kinematic_object = False
        self._pose_reader: Optional[Callable[[], dict]] = None

    @property
    def state(self) -> SessionState:
        """当前会话状态。"""
        return self._state

    def status_text(self) -> str:
        """供 UI 显示的人类可读状态字符串。"""
        names = {
            SessionState.IDLE: "IDLE",
            SessionState.LOADED: "LOADED (Start ROS stream or Press Play)",
            SessionState.STREAMING: "STREAMING (cameras + joint_states + TF)",
        }
        return names.get(self._state, str(self._state))

    def configure(
        self,
        topic_config: SessionTopicConfig,
        pose_reader: Callable[[], dict],
    ) -> None:
        """从 UI 同步话题配置与位姿字段读取器，并刷新相机 prim 路径。"""
        self._topic_config = topic_config
        self._pose_reader = pose_reader
        paths = self.scene_loader.camera_prim_paths
        for cam in self._topic_config.cameras:
            if cam.key in paths:
                cam.camera_prim_path = paths[cam.key]

    def set_live_apply(self, obj: bool, kinematic_object: bool) -> None:
        """设置 Play 期间是否实时写盒子位姿，以及写位姿时是否临时 kinematic。"""
        self._live_apply_object = obj
        self._kinematic_object = kinematic_object

    def load_scene(self, box_pose: Optional[Pose6D] = None) -> bool:
        """Unload 旧场景并 Load 新场景；成功后延迟设置关节零位。

        Returns:
            ``scene_loader.load`` 是否成功。
        """
        self.stop_all()
        ok = self.scene_loader.load(box_pose=box_pose)
        if ok:
            self._state = SessionState.LOADED
            cx, cy, cz = self.scene_loader.default_box_center
            self._object_pose_defaults = Pose6D((cx, cy, cz), (0.0, 0.0, 0.0))
            paths = self.scene_loader.camera_prim_paths
            for cam in self._topic_config.cameras:
                if cam.key in paths:
                    cam.camera_prim_path = paths[cam.key]
            asyncio.ensure_future(self._deferred_post_load())
        else:
            self._state = SessionState.IDLE
        self._notify()
        return ok

    async def _deferred_post_load(self) -> None:
        """等几帧渲染稳定后再设关节零位、补剥离相机物理（避免 Load 时 GPU 竞争）。"""
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(15):
            await app.next_update_async()

        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage and self.scene_loader.is_loaded:
            self.scene_loader.build_box_visual_if_needed(stage)
            self.scene_loader.fix_camera_physics(stage)
            self.scene_loader.fix_box_visual_material(stage)

    def apply_box_pose_from_ui(self, force: bool = True) -> None:
        """将 UI 中 obj_tx/ty/tz/roll/pitch/yaw 写入 ``BOX_LINK_PATH``。

        Args:
            force: ``False`` 且未开启 live apply 时跳过。
        """
        if not self.scene_loader.is_loaded or not self._pose_reader:
            return
        if not force and not self._live_apply_object:
            return
        data = self._pose_reader()
        import omni.timeline
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        playing = omni.timeline.get_timeline_interface().is_playing()
        obj_pose = parse_pose_from_fields(
            data.get("obj_tx", "0"),
            data.get("obj_ty", "0"),
            data.get("obj_tz", f"{DEFAULT_BOX_CENTER[2]:.3f}"),
            data.get("obj_roll", "0"),
            data.get("obj_pitch", "0"),
            data.get("obj_yaw", "0"),
            self._object_pose_defaults,
        )
        self._set_box_pose(obj_pose, stage, playing=playing)

    def enable_physics_grasp(self) -> None:
        """关闭摆位模式，让抓取盒受重力（Play 前调用）。"""
        self._live_apply_object = False
        self._kinematic_object = False
        self._ensure_box_dynamic()
        print("SessionController: physics grasp enabled (gravity ON, live pose OFF)")

    def _physics_grasp_blocked_reason(self) -> str:
        if self._kinematic_object and self._live_apply_object:
            return "box pinned in air: kinematic + Live apply are ON"
        if self._kinematic_object:
            return "box ignores gravity: Box kinematic is ON"
        if self._live_apply_object:
            return "box pose overwritten every frame: Live apply is ON"
        return ""

    def _ensure_box_dynamic(self) -> None:
        """Play 出流前确保抓取盒为动态刚体（受重力），除非用户显式勾选 kinematic。"""
        if self._kinematic_object:
            return
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        prim = stage.GetPrimAtPath(BOX_LINK_PATH)
        if prim and prim.IsValid():
            rb = UsdPhysics.RigidBodyAPI(prim)
            if rb:
                rb.CreateKinematicEnabledAttr(False)

    def sync_poses_to_ui(self) -> dict:
        """从 Stage 读取抓取盒位姿，返回 UI 字段键值 ``obj_tx`` 等。"""
        obj = read_pose(BOX_LINK_PATH)
        result = {}
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

    def _set_box_pose(self, pose: Pose6D, stage, *, playing: bool = False) -> None:
        """写盒子位姿；仅当 UI 勾选 kinematic 时才冻结刚体（便于摆位）。"""
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(BOX_LINK_PATH)
        if prim and prim.IsValid():
            rb = UsdPhysics.RigidBodyAPI(prim)
            if rb:
                rb.CreateKinematicEnabledAttr().Set(bool(self._kinematic_object))
        set_pose(BOX_LINK_PATH, pose, stage)

    def on_timeline_event(self, event) -> None:
        """Timeline Play → 立即 soft reset，再延迟启 ROS；Stop → 停出流。"""
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if self.scene_loader.is_loaded:
                # 必须在首帧物理步进前 reset，否则会先下落再被弹回初始高度
                self.scene_loader.ensure_physics_on_play()
                asyncio.ensure_future(self._deferred_play_start())
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._stop_streaming()
            self.scene_loader.invalidate_physics()

    async def _deferred_play_start(self) -> None:
        """等几帧 PhysX 稳定后再创建 OmniGraph（不再 reset / 写盒子位姿）。"""
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(8):
            await app.next_update_async()
            if not self._timeline.is_playing():
                return

        if self.scene_loader.is_loaded and self._timeline.is_playing():
            self._start_streaming()

    def on_app_update(self) -> None:
        """每帧回调：Play + live apply 时持续写盒子位姿。"""
        if not self.scene_loader.is_loaded:
            return
        if self._timeline.is_playing() and self._live_apply_object:
            self.apply_box_pose_from_ui(force=True)

    def stop_streaming_only(self) -> None:
        """停止 ROS OmniGraph，保留 Stage 场景与 Timeline 状态。"""
        self._stop_streaming()

    def stop_streaming_manual(self) -> None:
        """停止 ROS 出流并 Stop 仿真（与面板 Stop 按钮 / Timeline Stop 一致）。"""
        if self._timeline.is_playing():
            self._timeline.stop()
        else:
            self._stop_streaming()
            self.scene_loader.invalidate_physics()

    def stop_all(self) -> None:
        """停止出流并 Unload 场景，状态回到 IDLE。"""
        self._stop_streaming()
        self.scene_loader.unload()
        self._state = SessionState.IDLE
        self._notify()

    def start_streaming_manual(self) -> bool:
        """启动仿真并 ROS 出流；未 Play 时自动 timeline.play()。

        Returns:
            是否已进入或即将进入 STREAMING 状态。
        """
        if not self.scene_loader.is_loaded:
            print("SessionController: Load scene first")
            return False
        if self._state == SessionState.STREAMING and self._timeline.is_playing():
            print("SessionController: already streaming")
            return True
        if not self._timeline.is_playing():
            self._timeline.play()
            asyncio.ensure_future(self._deferred_play_start())
            return True
        self.scene_loader.ensure_physics_on_play()
        self._start_streaming()
        return self._state == SessionState.STREAMING

    def _start_streaming(self) -> None:
        """创建三路相机 + 机器人 ROS OmniGraph。"""
        if not self.scene_loader.is_loaded:
            return
        if self._state == SessionState.STREAMING:
            return

        paths = self.scene_loader.camera_prim_paths
        for cam in self._topic_config.cameras:
            if cam.key in paths:
                cam.camera_prim_path = paths[cam.key]

        if not ensure_ros2_bridge_enabled():
            print("SessionController: ROS2 bridge not available")
            self._notify()
            return

        reason = self._physics_grasp_blocked_reason()
        if reason:
            print(f"SessionController: WARN {reason}")
        if not self._kinematic_object:
            self._ensure_box_dynamic()
        cam_ok = self.multi_camera_publisher.start(self._topic_config)
        robot_ok = self.robot_ros_publisher.start(
            self._topic_config.robot,
            self.scene_loader.box_link_path,
        )
        if cam_ok or robot_ok:
            self._state = SessionState.STREAMING
        self._notify()

    def _stop_streaming(self) -> None:
        """销毁 OmniGraph；若场景仍在则回到 LOADED。"""
        self.multi_camera_publisher.stop()
        self.robot_ros_publisher.stop()
        if self.scene_loader.is_loaded:
            self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()

    def _notify(self) -> None:
        """触发状态栏更新。"""
        self._on_status(self.status_text())
