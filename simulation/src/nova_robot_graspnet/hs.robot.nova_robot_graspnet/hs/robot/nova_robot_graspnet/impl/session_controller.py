# -*- coding: utf-8 -*-
"""会话控制器：Load/Unload、Timeline Play 触发 ROS 出流、抓取盒位姿同步。"""

from __future__ import annotations

from enum import Enum, auto
from typing import Callable, Dict, Optional

import asyncio
import omni.timeline

from ..global_variables import (
    BOX_LINK_PATH,
    BOX_POSE_FRAME,
    BOX_POSE_TOPIC,
    DATA_LOG_DIRNAME,
    DEFAULT_ARM1_RESET_XYZRPY,
    DEFAULT_ARM2_RESET_XYZRPY,
    DEFAULT_BOX_CENTER,
    DEFAULT_BOX_POSE_RPY,
    DEFAULT_SETTLE_STEPS,
    GRASP_POSE_ARRAY_TOPIC,
    ROBOT_PRIM_PATH,
)
from .box_physics import configure_box_usd_dynamic, pin_box_kinematic_pose, release_box_for_gravity
from .data_collect import BoxPoseRange, DataCollector
from .grasp.grasp_controller import NovaGraspController
from .grasp.interfaces import Transform6D
from .grasp.robot_runtime import NovaRobotRuntime
from .pose_utils import (
    Pose6D,
    is_plausible_gripper_pose,
    looks_like_joint_degrees_text,
    parse_pose_from_fields,
    read_dual_arm_ee_from_stage,
    read_pose,
    set_pose,
    invalidate_box_pose_cache,
    xyzrpy_tuple_to_pose6d,
)
from .ros_io.box_pose_ros import BoxPosePublisher, BoxPoseSubscriber
from .ros_io.joint_command_graph import JointCommandGraph
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
    COLLECTING = auto() # Replicator 批量采集中
    GRASP_ARMED = auto()  # 订阅 PoseArray 抓取位姿，可执行抓取


class SessionController:
    """协调 SceneLoader 与 ROS 发布器，响应 UI / Timeline 事件。

    典型流程：
      Load scene → LOADED → Play / Start stream → STREAMING → Stop → LOADED
    """

    def __init__(
        self,
        scene_loader: SceneLoader,
        on_status: Optional[Callable[[str], None]] = None,
        on_collect_progress: Optional[Callable[[int, int], None]] = None,
    ):
        """Args:
            scene_loader: 场景加载器实例。
            on_status: 状态变更回调（UI 状态栏）。
            on_collect_progress: 采集进度回调（当前序号, 总数；0/0 表示空闲）。
        """
        self.scene_loader = scene_loader
        self.multi_camera_publisher = MultiCameraPublisher()
        self.robot_ros_publisher = RobotRosPublisher()
        self.data_collector = DataCollector()
        self.robot_runtime = NovaRobotRuntime()
        self.grasp_controller = NovaGraspController()
        self.box_pose_publisher = BoxPosePublisher()
        self.box_pose_subscriber = BoxPoseSubscriber()
        self.joint_command_graph = JointCommandGraph()
        self._timeline = omni.timeline.get_timeline_interface()
        self._state = SessionState.IDLE
        self._topic_config = SessionTopicConfig.default()
        self._on_status = on_status or (lambda s: print(f"[NovaGraspNet] {s}"))
        self._on_collect_progress = on_collect_progress or (lambda _c, _t: None)
        self._collect_current = 0
        self._collect_total = 0

        self._object_pose_defaults = Pose6D(DEFAULT_BOX_CENTER, DEFAULT_BOX_POSE_RPY)
        self._live_apply_object = False
        self._kinematic_object = False
        self._box_physics_released = False
        self._box_pose_block_log_time = 0.0
        self._pose_reader: Optional[Callable[[], dict]] = None
        self._ext_root: str = ""
        self._publish_box_pose_gt = True
        self._grasp_arm_mode = "auto"
        self._grasp_pose_topic = GRASP_POSE_ARRAY_TOPIC
        self._box_pose_topic = BOX_POSE_TOPIC
        self._pending_grasp_pose: Optional[Transform6D] = None
        self._last_received_grasp_pose: Optional[Transform6D] = None
        self._grasp_task: Optional[asyncio.Task] = None
        self._reset_before_motion = True
        self._external_grasp = False
        self._arm1_reset_pose = xyzrpy_tuple_to_pose6d(DEFAULT_ARM1_RESET_XYZRPY)
        self._arm2_reset_pose = xyzrpy_tuple_to_pose6d(DEFAULT_ARM2_RESET_XYZRPY)
        self._last_runtime_ensure = 0.0
        self._collection_box_pose: Optional[Pose6D] = None

    @property
    def state(self) -> SessionState:
        """当前会话状态。"""
        return self._state

    def status_text(self) -> str:
        """供 UI 显示的人类可读状态字符串。"""
        if self._state == SessionState.COLLECTING and self._collect_total > 0:
            return f"COLLECTING sample {self._collect_current}/{self._collect_total}"
        names = {
            SessionState.IDLE: "IDLE",
            SessionState.LOADED: "LOADED (Start ROS stream or Press Play)",
            SessionState.STREAMING: "STREAMING (cameras + joint_states + TF)",
            SessionState.COLLECTING: "COLLECTING (Replicator dataset)",
            SessionState.GRASP_ARMED: "GRASP_ARMED (PoseArray + arm IK)",
        }
        return names.get(self._state, str(self._state))

    def _set_collect_progress(self, current: int, total: int) -> None:
        self._collect_current = max(0, int(current))
        self._collect_total = max(0, int(total))
        self._on_collect_progress(self._collect_current, self._collect_total)
        if self._state == SessionState.COLLECTING:
            self._on_status(self.status_text())

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

    def set_extension_root(self, ext_root: str) -> None:
        """设置扩展根目录（用于 data_log 默认路径）。"""
        self._ext_root = ext_root

    def set_grasp_options(
        self,
        *,
        publish_gt: bool,
        arm_mode: str,
        grasp_pose_topic: str,
        box_pose_topic: str,
    ) -> None:
        self._publish_box_pose_gt = bool(publish_gt)
        self._grasp_arm_mode = arm_mode if arm_mode in ("auto", "arm1", "arm2") else "auto"
        self._grasp_pose_topic = (grasp_pose_topic or GRASP_POSE_ARRAY_TOPIC).strip()
        self._box_pose_topic = (box_pose_topic or BOX_POSE_TOPIC).strip()
        self.grasp_controller.set_arm_mode(self._grasp_arm_mode)

    def set_external_grasp(self, enabled: bool) -> None:
        """True：不在 Isaac 内执行 IK，改由 nova_grasp_moveit + MoveIt2 抓取。"""
        self._external_grasp = bool(enabled)

    @property
    def external_grasp(self) -> bool:
        return self._external_grasp

    def set_arm_reset_config(
        self,
        arm1_pose: Pose6D,
        arm2_pose: Pose6D,
        *,
        reset_before_motion: bool = True,
    ) -> None:
        self._arm1_reset_pose = arm1_pose
        self._arm2_reset_pose = arm2_pose
        self._reset_before_motion = bool(reset_before_motion)
        self.grasp_controller.set_reset_config(
            self._arm1_reset_pose,
            self._arm2_reset_pose,
            enabled=self._reset_before_motion,
        )

    def get_ee_pose_snapshot(self) -> Dict[str, dict]:
        """实时末端位姿：优先 articulation，回退 Stage USD。"""
        if not self.scene_loader.is_loaded:
            return {}
        if self._timeline.is_playing():
            import time

            now = time.monotonic()
            if now - self._last_runtime_ensure > 0.4:
                self._last_runtime_ensure = now
                self._ensure_robot_runtime()
        if self.robot_runtime.is_ready:
            snap = self.robot_runtime.get_live_arm_status()
            if snap:
                return snap
        return read_dual_arm_ee_from_stage()

    def validate_reset_poses(self) -> Optional[str]:
        """检查 UI 复位 xyzrpy 是否合法；返回错误说明或 None。"""
        for label, pose in (("J1", self._arm1_reset_pose), ("J2", self._arm2_reset_pose)):
            if not is_plausible_gripper_pose(pose):
                return (
                    f"{label} reset pose out of workspace: "
                    f"{pose.translation[0]:.3f},{pose.translation[1]:.3f},{pose.translation[2]:.3f} — "
                    "use Sync reset (not joint angles)"
                )
        return None

    def get_arm_joint_positions_deg(self, arm: str) -> Optional[list]:
        if not self.robot_runtime.is_ready and self._timeline.is_playing():
            self._ensure_robot_runtime()
        joints = self.robot_runtime.get_arm_joint_positions_rad(arm)
        if joints is None:
            return None
        import math

        return [math.degrees(j) for j in joints]

    def reset_arms_now(self) -> bool:
        """手动复位双臂（不执行抓取）。"""
        print("SessionController: Reset arms requested")
        if self._external_grasp:
            print("SessionController: external grasp ON — in-sim reset disabled")
            return False
        if not self._timeline.is_playing():
            print("SessionController: press Play (▶) before Reset arms")
            return False
        if not self._ensure_robot_runtime():
            print("SessionController: robot runtime not ready for reset")
            return False
        err = self.validate_reset_poses()
        if err:
            print(f"SessionController: {err}")
            return False
        self._schedule_reset_async()
        return True

    def _cancel_arm_task(self) -> None:
        if self._grasp_task is not None and not self._grasp_task.done():
            print("SessionController: cancel in-flight arm motion")
            self._grasp_task.cancel()
        self._grasp_task = None

    def _ensure_robot_runtime(self) -> bool:
        if not self.scene_loader.is_loaded:
            print("SessionController: robot runtime skip — scene not loaded")
            return False
        if not self._timeline.is_playing():
            print("SessionController: robot runtime skip — timeline not playing")
            return False
        if self.robot_runtime.is_ready and not self.robot_runtime.validate_ready():
            print("SessionController: robot runtime stale — rebinding")
            self.robot_runtime.shutdown()
        if not self.scene_loader.robot_articulation:
            ok = self.scene_loader.register_physics_prims(kinematic_box=self._kinematic_object)
            print(f"SessionController: register_physics_prims -> {ok}")
        art = self.scene_loader.robot_articulation
        if art is None:
            print("SessionController: robot articulation missing after register")
            return False
        if not self.robot_runtime.is_ready:
            if not self.robot_runtime.bind_articulation(art):
                print("SessionController: bind_articulation failed")
                return False
        self.grasp_controller.bind_runtime(self.robot_runtime)
        self.robot_runtime.log_diagnostics()
        return True

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
        self._box_physics_released = False
        ok = self.scene_loader.load(box_pose=box_pose)
        if ok:
            self._state = SessionState.LOADED
            self._object_pose_defaults = Pose6D(DEFAULT_BOX_CENTER, DEFAULT_BOX_POSE_RPY)
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
            print("SessionController: deferred post-load (lighting / box visual / camera physics)")
            self.scene_loader.apply_default_light_rig()
            self.scene_loader.build_box_visual_if_needed(stage)
            self.scene_loader.fix_camera_physics(stage)
            self.scene_loader.fix_box_visual_material(stage)
            # 画面不动；只把 TF cam0 对齐到 Gemini 出流（约 30°），消除图/TF 外参差。
            try:
                self.scene_loader.sync_cam0_tf_to_gemini_rgb(stage)
            except Exception as exc:
                print(f"SessionController: cam0 TF sync skipped: {exc}")
            print("SessionController: deferred post-load done")
            self._notify()

    def apply_box_pose_from_ui(self, force: bool = True) -> bool:
        """将 UI 中 obj_tx/ty/tz/roll/pitch/yaw 写入 ``BOX_LINK_PATH``。

        Args:
            force: ``False`` 且未开启 live apply 时跳过。

        Returns:
            是否成功写入（物理释放期间未 force 时为 False）。
        """
        if not self.scene_loader.is_loaded or not self._pose_reader:
            return False
        if not force and not self._live_apply_object:
            return False
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
        return self._set_box_pose(obj_pose, stage, playing=playing, force=force)

    def pin_box_for_posing(self, *, quiet: bool = False) -> None:
        """摆位模式：允许再次写 USD 坐标（Apply / Live apply / 采集用）。"""
        self._box_physics_released = False
        if not quiet:
            print("SessionController: box posing mode — USD pose write enabled")

    def reposition_box_for_collection(self, pose: Pose6D) -> bool:
        """采集专用：kinematic 钉扎到采样位姿（严格按 UI 范围，不释放重力、不抬高 Z）。"""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return False

        self.pin_box_for_posing(quiet=True)
        self.scene_loader.build_box_visual_if_needed(stage)
        # 采集每帧不要重写 collision_geo：会 invalidate PhysX，盒子看起来钉不住
        from .box_mesh_builder import has_box_collision, ensure_box_collision

        if not has_box_collision(stage):
            if not ensure_box_collision(stage, self.scene_loader._box_dir):
                print("SessionController: WARN box collision missing — collection may fail")

        if not pin_box_kinematic_pose(pose, stage, log=True):
            print("SessionController: failed to pin box pose for collection")
            return False

        self._collection_box_pose = pose
        self._box_physics_released = False
        return True

    def enable_physics_grasp(self) -> None:
        """关闭摆位模式，释放盒子给 PhysX 重力（不再每帧写 USD xform）。"""
        self._live_apply_object = False
        self._kinematic_object = False
        self._box_physics_released = True
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        self.scene_loader.build_box_visual_if_needed(stage)
        from .box_mesh_builder import ensure_box_collision

        if not ensure_box_collision(stage, self.scene_loader._box_dir):
            print("SessionController: WARN no box collision — fix before Physics ON")
            return

        surface_z = self.scene_loader.workspace_surface_z_world
        if self._timeline.is_playing():
            if not self.scene_loader._physics_initialized:
                self.scene_loader.ensure_physics_on_play()
            else:
                self.scene_loader._register_table_physics()
                configure_box_usd_dynamic(kinematic=False)
                release_box_for_gravity(stage, surface_z_world=surface_z)
        else:
            configure_box_usd_dynamic(kinematic=False)
            release_box_for_gravity(stage, surface_z_world=surface_z)
        print(
            "SessionController: physics grasp enabled "
            "(gravity ON, stop UI pose pinning)"
        )

    def _physics_grasp_blocked_reason(self) -> str:
        if self._kinematic_object and self._live_apply_object:
            return "box pinned in air: kinematic + Live apply are ON"
        if self._kinematic_object:
            return "box ignores gravity: Box kinematic is ON"
        if self._live_apply_object:
            return "box pose overwritten every frame: Live apply is ON"
        return ""

    def _ensure_box_dynamic(self) -> None:
        """确保抓取盒 USD 为动态（实际 PhysX 注册在 Play + register_physics_prims）。"""
        if self._kinematic_object:
            return
        configure_box_usd_dynamic(kinematic=False)

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

    def _set_box_pose(
        self,
        pose: Pose6D,
        stage,
        *,
        playing: bool = False,
        force: bool = False,
        kinematic: Optional[bool] = None,
    ) -> bool:
        """写盒子位姿。Play 且已释放物理后禁止写 USD xform（会与 PhysX 冲突把盒子拉回 UI 坐标）。"""
        import time

        kin = self._kinematic_object if kinematic is None else kinematic
        if (
            playing
            and self._box_physics_released
            and not kin
            and not force
        ):
            now = time.monotonic()
            if now - self._box_pose_block_log_time > 2.0:
                self._box_pose_block_log_time = now
                print(
                    "SessionController: BLOCKED box pose write during physics "
                    f"(ui z={pose.translation[2]:.3f}) — uncheck Physics or use kinematic to reposition"
                )
            return False

        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(BOX_LINK_PATH)
        if prim and prim.IsValid():
            rb = UsdPhysics.RigidBodyAPI(prim)
            if rb:
                rb.CreateKinematicEnabledAttr().Set(bool(kin))
        set_pose(BOX_LINK_PATH, pose, stage)
        return True

    def on_timeline_event(self, event) -> None:
        """Timeline Play → 立即 soft reset，再延迟启 ROS；Stop → 停出流。"""
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if self.scene_loader.is_loaded:
                if self.scene_loader.ensure_physics_on_play():
                    self._box_physics_released = True
                asyncio.ensure_future(self._deferred_play_start())
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            # 先丢掉 PhysX view，避免 Stop 过程中再走 RigidPrim/orient 路径刷屏
            from .box_physics import invalidate_box_physx_view

            invalidate_box_physx_view()
            self._collection_box_pose = None
            self._stop_streaming()
            self.scene_loader.invalidate_physics()
            self.robot_runtime.shutdown()

    async def _deferred_play_start(self) -> None:
        """等几帧 PhysX 稳定后再创建 OmniGraph（不再 reset / 写盒子位姿）。"""
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(8):
            await app.next_update_async()
            if not self._timeline.is_playing():
                return

        if self.scene_loader.is_loaded and self._timeline.is_playing():
            self._ensure_robot_runtime()
            self._start_streaming()

    def on_app_update(self) -> None:
        """每帧回调：仅在 live apply / GT 发布 / 抓取订阅活跃时工作。"""
        if not self.scene_loader.is_loaded:
            return
        need_work = (
            (self._timeline.is_playing() and self._live_apply_object)
            or self.box_pose_publisher.is_active
            or self.box_pose_subscriber.is_active
        )
        if not need_work:
            return
        if self._timeline.is_playing() and self._live_apply_object:
            self.apply_box_pose_from_ui(force=True)
        if self.box_pose_publisher.is_active:
            self.box_pose_publisher.spin_once()
        if self.box_pose_subscriber.is_active:
            self.box_pose_subscriber.spin_once()
            if self._pending_grasp_pose is not None and self._grasp_task is None:
                pose = self._pending_grasp_pose
                self._pending_grasp_pose = None
                if self._external_grasp:
                    print(
                        "SessionController: external grasp mode — "
                        "use nova_grasp_moveit (ros2 service call /grasp_moveit_node/execute)"
                    )
                elif self._ensure_robot_runtime():
                    self._schedule_grasp_async(pose)
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
        self.stop_grasp_mode()
        self._stop_streaming()
        self.joint_command_graph.stop()
        self.robot_runtime.shutdown()
        self.grasp_controller.bind_runtime(None)
        self._box_physics_released = False
        invalidate_box_pose_cache()
        self._pending_grasp_pose = None
        self._last_received_grasp_pose = None
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
        elif self._state == SessionState.GRASP_ARMED:
            print("SessionController: box kinematic ON — gravity disabled until unchecked")
        cam_ok = self.multi_camera_publisher.start(self._topic_config)
        robot_ok = self.robot_ros_publisher.start(
            self._topic_config.robot,
            self.scene_loader.box_link_path,
        )
        if cam_ok or robot_ok:
            if self._state not in (SessionState.GRASP_ARMED, SessionState.COLLECTING):
                self._state = SessionState.STREAMING
            if robot_ok and self._timeline.is_playing():
                self._start_joint_command_graph()
            self._start_box_pose_publish()
        self._notify()

    def _start_box_pose_publish(self) -> None:
        """仿真启动后自动持续发布盒子相对 base_link 的 PoseStamped。"""
        self._publish_box_pose_gt = True
        topic = (self._box_pose_topic or BOX_POSE_TOPIC).strip() or BOX_POSE_TOPIC
        ok = self.box_pose_publisher.start(topic, BOX_POSE_FRAME, continuous=True)
        if ok:
            print(f"SessionController: auto-publish {topic} (frame={BOX_POSE_FRAME})")
        else:
            print("SessionController: WARN box_pose auto-publish failed to start")

    def _stop_streaming(self) -> None:
        """销毁 OmniGraph；若场景仍在则回到 LOADED。"""
        self.multi_camera_publisher.stop()
        self.robot_ros_publisher.stop()
        if self._state != SessionState.GRASP_ARMED:
            self.joint_command_graph.stop()
            self.box_pose_publisher.stop()
        elif self._timeline.is_playing():
            self._start_joint_command_graph()
        if self._state == SessionState.COLLECTING:
            return
        if self.scene_loader.is_loaded:
            if self._state not in (SessionState.GRASP_ARMED,):
                self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()

    def _snapshot_robot_joints_usd(self, stage) -> Dict[str, float]:
        """采集前保存关节角（避免 world.reset 把臂拉回零位）。"""
        from pxr import Usd, UsdPhysics

        out: Dict[str, float] = {}
        robot = stage.GetPrimAtPath(self.scene_loader.robot_prim_path or ROBOT_PRIM_PATH)
        if not robot or not robot.IsValid():
            return out
        for prim in Usd.PrimRange(robot):
            path = str(prim.GetPath())
            if prim.IsA(UsdPhysics.RevoluteJoint):
                for name in (
                    "state:angular:physics:position",
                    "drive:angular:physics:targetPosition",
                ):
                    attr = prim.GetAttribute(name)
                    if attr and attr.IsValid():
                        out[path] = float(attr.Get())
                        break
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                for name in (
                    "state:linear:physics:position",
                    "drive:linear:physics:targetPosition",
                ):
                    attr = prim.GetAttribute(name)
                    if attr and attr.IsValid():
                        out[path] = float(attr.Get())
                        break
        return out

    def _restore_robot_joints_usd(self, stage, snapshot: Dict[str, float]) -> None:
        """把快照写回 USD 关节 state / drive target。"""
        if not snapshot:
            return
        for path, val in snapshot.items():
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            for name in (
                "state:angular:physics:position",
                "drive:angular:physics:targetPosition",
                "state:linear:physics:position",
                "drive:linear:physics:targetPosition",
            ):
                attr = prim.GetAttribute(name)
                if attr and attr.IsValid():
                    attr.Set(float(val))

    async def _prepare_collection_only(self) -> bool:
        """采集前仅初始化 PhysX；机械臂保持当前姿态，不做 IK 运动。"""
        import omni.kit.app
        import omni.usd

        self._cancel_arm_task()
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            timeline.play()
        app = omni.kit.app.get_app()
        for _ in range(15):
            await app.next_update_async()

        stage = omni.usd.get_context().get_stage()
        joint_snapshot: Dict[str, float] = {}
        if stage and not self.scene_loader._physics_initialized:
            joint_snapshot = self._snapshot_robot_joints_usd(stage)

        if not self.scene_loader._physics_initialized:
            self.scene_loader.ensure_physics_on_play(
                release_box=False,
                kinematic_box=True,
            )
            for _ in range(15):
                await app.next_update_async()
            if stage and joint_snapshot:
                self._restore_robot_joints_usd(stage, joint_snapshot)
                for _ in range(5):
                    await app.next_update_async()
                self.robot_runtime.shutdown()
                if self._ensure_robot_runtime():
                    self.robot_runtime.hold_current_configuration()

        print("SessionController: collection ready — robot arms held at current pose")
        return bool(self.scene_loader._physics_initialized)

    def _hold_collection_sample(self) -> None:
        """拍图前再次钉扎盒子位姿。"""
        if self._collection_box_pose is None:
            return
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage:
            pin_box_kinematic_pose(self._collection_box_pose, stage, log=False)

    def start_data_collection(
        self,
        *,
        num_samples: int,
        pose_range: BoxPoseRange,
        output_dir: str = "",
        settle_steps: int = DEFAULT_SETTLE_STEPS,
    ) -> None:
        """异步启动 Replicator 数据采集。"""
        if not self.scene_loader.is_loaded:
            print("SessionController: Load scene first")
            return
        if self.data_collector.is_collecting:
            print("SessionController: collection already running")
            return
        import os

        root = output_dir.strip() or os.path.join(self._ext_root, DATA_LOG_DIRNAME)
        os.makedirs(root, exist_ok=True)
        was_streaming = self._state == SessionState.STREAMING
        if was_streaming:
            self._stop_streaming()
        self._state = SessionState.COLLECTING
        self._set_collect_progress(0, num_samples)
        self._notify()
        asyncio.ensure_future(
            self._run_data_collection(
                num_samples=num_samples,
                pose_range=pose_range,
                output_root=root,
                settle_steps=settle_steps,
                resume_streaming=was_streaming,
            )
        )

    async def _run_data_collection(
        self,
        *,
        num_samples: int,
        pose_range: BoxPoseRange,
        output_root: str,
        settle_steps: int,
        resume_streaming: bool,
    ) -> None:
        self._collection_box_pose = None
        try:
            if not await self._prepare_collection_only():
                print("SessionController: WARN collection physics not ready")

            def _prepare_sample(pose: Pose6D) -> None:
                self.reposition_box_for_collection(pose)

            run_dir = await self.data_collector.collect(
                output_root=output_root,
                num_samples=num_samples,
                pose_range=pose_range,
                topic_config=self._topic_config,
                prepare_sample=_prepare_sample,
                hold_sample=self._hold_collection_sample,
                settle_steps=settle_steps,
                on_progress=lambda p: self._set_collect_progress(p.index, p.total),
            )
            if run_dir:
                print(f"SessionController: dataset saved to {run_dir}")
        finally:
            self._collection_box_pose = None
            self._set_collect_progress(0, 0)
            if self.scene_loader.is_loaded:
                self._state = SessionState.LOADED
                if resume_streaming and self._timeline.is_playing():
                    self._start_streaming()
            else:
                self._state = SessionState.IDLE
            self._notify()

    def start_grasp_mode(self) -> bool:
        """启用 PoseArray 订阅与可选 box_pose GT 发布。"""
        if not self.scene_loader.is_loaded:
            print("SessionController: Load scene first")
            return False
        if not ensure_ros2_bridge_enabled():
            print("SessionController: ROS2 bridge required for grasp mode")
            return False
        if not self._timeline.is_playing():
            self._timeline.play()
        asyncio.ensure_future(self._deferred_start_grasp_mode())
        return True

    async def _deferred_start_grasp_mode(self) -> None:
        """等 PhysX 就绪后再初始化 articulation 并订阅抓取话题。"""
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(12):
            await app.next_update_async()
            if not self.scene_loader.is_loaded:
                return
            if not self._timeline.is_playing():
                print("SessionController: grasp mode aborted (timeline stopped)")
                self._notify()
                return

        if not self._ensure_robot_runtime():
            print("SessionController: robot runtime init failed")
            self._notify()
            return
        self._start_joint_command_graph()
        self.enable_physics_grasp()
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(8):
            await app.next_update_async()
        if not self.scene_loader._physics_initialized:
            self.scene_loader.register_physics_prims(kinematic_box=False)
        self.box_pose_publisher.start(
            self._box_pose_topic,
            BOX_POSE_FRAME,
            continuous=True,
        )
        if not self.box_pose_publisher.is_active:
            print("SessionController: WARN box_pose publisher failed to start")
        else:
            self._publish_box_pose_gt = True
            print(
                f"SessionController: publish {self._box_pose_topic} "
                f"(frame={BOX_POSE_FRAME})"
            )
        self.box_pose_subscriber.start(self._on_grasp_pose_received, self._grasp_pose_topic)
        self._state = SessionState.GRASP_ARMED
        print("SessionController: grasp mode armed")
        self._notify()

    def stop_grasp_mode(self) -> None:
        self.box_pose_subscriber.stop()
        self.box_pose_publisher.stop()
        if not self.robot_ros_publisher.is_active:
            self.joint_command_graph.stop()
        self._pending_grasp_pose = None
        self._last_received_grasp_pose = None
        if self.scene_loader.is_loaded:
            if self.multi_camera_publisher.is_active:
                self._state = SessionState.STREAMING
            else:
                self._state = SessionState.LOADED
        else:
            self._state = SessionState.IDLE
        self._notify()

    def _on_grasp_pose_received(self, pose: Transform6D) -> None:
        """收到 PoseArray 后排队到下一帧执行（避免在回调中阻塞）。"""
        self._last_received_grasp_pose = pose
        self._pending_grasp_pose = pose

    def _start_joint_command_graph(self) -> None:
        """启动 /joint_command OmniGraph（与 joint_states 同一 ROS2 栈）。"""
        if self.robot_ros_publisher.is_active and self._topic_config.robot.enable_joint_command:
            self.joint_command_graph.stop()
            return
        if self.joint_command_graph.is_active:
            return
        if not self._timeline.is_playing():
            print("SessionController: joint command graph skipped — timeline paused")
            return
        self.joint_command_graph.start(self._topic_config.robot.sub_joint_command)

    def ensure_playing(self) -> None:
        """确保 Timeline 处于 Play（抓取 / 采集需要 PhysX）。"""
        if not self._timeline.is_playing():
            self._timeline.play()

    def execute_grasp_from_stage_gt(self) -> bool:
        """用 Stage 上盒子当前世界位姿执行一次抓取。"""
        print("SessionController: Execute grasp requested")
        if self._external_grasp:
            print(
                "SessionController: external grasp ON — run nova_grasp_moveit "
                "(ros2 launch nova_grasp_moveit grasp_stack.launch.py)"
            )
            return False
        if not self._timeline.is_playing():
            print("SessionController: press Play (▶) before Execute grasp")
            return False
        if not self._ensure_robot_runtime():
            print("SessionController: robot runtime not ready for grasp")
            return False
        pose = self.grasp_controller.pose_from_stage_box()
        if not pose:
            print("SessionController: box pose unavailable")
            return False
        print(
            f"SessionController: box world pose t={pose.translation} frame={pose.target_frame}"
        )
        self._schedule_grasp_async(pose)
        return True

    def execute_move_to_box_only(self) -> bool:
        """仅移动机械臂到料盒位姿，不闭合夹爪（用于核对 Stage 盒位与 IK）。"""
        print("SessionController: Move only requested")
        if self._external_grasp:
            print("SessionController: external grasp ON — in-sim move disabled")
            return False
        if not self._timeline.is_playing():
            print("SessionController: press Play (▶) before Move only")
            return False
        if not self._ensure_robot_runtime():
            print("SessionController: robot runtime not ready for move")
            return False

        pose: Optional[Transform6D] = None
        source = ""
        if self.box_pose_subscriber.is_active and self._last_received_grasp_pose is not None:
            pose = self._last_received_grasp_pose
            source = "last ROS PoseArray (Grasp ON)"
        else:
            pose = self.grasp_controller.pose_from_stage_box()
            source = "Stage grasp_box (world, TF 一致)"
        if pose is None:
            print("SessionController: box pose unavailable for move-only")
            return False

        print(
            f"SessionController: move-only source={source} "
            f"frame={pose.target_frame} t={pose.translation} q={pose.rotation_xyzw}"
        )
        self._schedule_move_async(pose)
        return True

    def _schedule_reset_async(self) -> None:
        self._cancel_arm_task()
        self.grasp_controller.set_reset_config(
            self._arm1_reset_pose,
            self._arm2_reset_pose,
            enabled=True,
        )
        self._grasp_task = asyncio.ensure_future(self._run_reset_async())

    async def _run_reset_async(self) -> None:
        try:
            if not self._ensure_robot_runtime():
                print("SessionController: reset aborted — robot runtime not ready")
                return
            ok = await self.robot_runtime.reset_arms_to_poses_async(
                self._arm1_reset_pose,
                self._arm2_reset_pose,
            )
            if ok:
                print("SessionController: reset arms done")
            else:
                print("SessionController: reset arms failed — try Sync reset then Reset arms")
        except asyncio.CancelledError:
            print("SessionController: reset cancelled")
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SessionController: reset failed: {exc}")
        finally:
            self._grasp_task = None
            self._notify()

    def _schedule_move_async(self, pose: Transform6D) -> None:
        if self._grasp_task is not None and not self._grasp_task.done():
            print("SessionController: arm motion already running")
            return
        self.grasp_controller.set_arm_mode(self._grasp_arm_mode)
        self.grasp_controller.set_reset_config(
            self._arm1_reset_pose,
            self._arm2_reset_pose,
            enabled=self._reset_before_motion,
        )
        self._grasp_task = asyncio.ensure_future(self._run_move_async(pose))

    async def _run_move_async(self, pose: Transform6D) -> None:
        try:
            if not self._ensure_robot_runtime():
                print("SessionController: move-only aborted — robot runtime not ready")
                return
            await self.grasp_controller.move_to_pose_async(pose)
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SessionController: move-only failed: {exc}")
        finally:
            self._grasp_task = None
            self._notify()

    def _schedule_grasp_async(self, pose) -> None:
        if self._grasp_task is not None and not self._grasp_task.done():
            print("SessionController: grasp already running")
            return
        self.grasp_controller.set_arm_mode(self._grasp_arm_mode)
        self.grasp_controller.set_reset_config(
            self._arm1_reset_pose,
            self._arm2_reset_pose,
            enabled=self._reset_before_motion,
        )
        self._grasp_task = asyncio.ensure_future(self._run_grasp_async(pose))

    async def _run_grasp_async(self, pose) -> None:
        try:
            if not self._ensure_robot_runtime():
                print("SessionController: grasp aborted — robot runtime not ready")
                return
            await self.grasp_controller.execute_grasp_async(pose)
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SessionController: grasp failed: {exc}")
        finally:
            self._grasp_task = None
            self._notify()

    def publish_box_pose_once(self, topic: str = BOX_POSE_TOPIC) -> bool:
        """调试：从 Stage 读取盒子位姿，单次发布 PoseStamped（异步，不阻塞 UI）。"""
        if not self.scene_loader.is_loaded:
            print("SessionController: Load scene first")
            return False
        if not ensure_ros2_bridge_enabled():
            print("SessionController: ROS2 bridge required to publish box pose")
            return False
        if not self._timeline.is_playing():
            print("SessionController: WARN timeline paused — box pose will publish but physics is frozen")
        pub_topic = (topic or self._box_pose_topic).strip() or BOX_POSE_TOPIC
        if not self.box_pose_publisher.is_active:
            self.box_pose_publisher.start(pub_topic, BOX_POSE_FRAME, continuous=False)
        self.box_pose_publisher.publish_once_deferred(pub_topic, BOX_POSE_FRAME)
        print(f"SessionController: scheduling debug publish box pose -> {pub_topic}")
        return True

    def _notify(self) -> None:
        """触发状态栏更新。"""
        self._on_status(self.status_text())
