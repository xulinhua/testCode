# -*- coding: utf-8 -*-
"""/box_pose GT 发布（OmniGraph PoseStamped）与外部抓取位姿订阅（PoseArray）。"""

from __future__ import annotations

from typing import Callable, Optional

from ...global_variables import (
    BOX_POSE_FRAME,
    BOX_POSE_TOPIC,
    GRASP_POSE_ARRAY_TOPIC,
)
from ..grasp.interfaces import Transform6D
from .box_pose_omni_publisher import BoxPoseOmniPublisher


class BoxPosePublisher:
    """从 Stage 读取盒子位姿并发布 GT（OmniGraph，与 joint_states 同一 ROS 栈）。"""

    def __init__(self):
        self._impl = BoxPoseOmniPublisher()
        self._topic = BOX_POSE_TOPIC
        self._frame_id = BOX_POSE_FRAME
        self._continuous = False

    @property
    def is_active(self) -> bool:
        return self._impl.is_active

    def start(
        self,
        topic: str = BOX_POSE_TOPIC,
        frame_id: str = BOX_POSE_FRAME,
        *,
        continuous: bool = True,
    ) -> bool:
        self._topic = (topic or BOX_POSE_TOPIC).strip()
        self._frame_id = (frame_id or BOX_POSE_FRAME).strip()
        self._continuous = bool(continuous)
        ok = self._impl.start(self._topic, self._frame_id, continuous=self._continuous)
        if ok and self._continuous:
            self._impl.publish_from_stage(send=True)
        return ok

    def spin_once(self) -> None:
        self._impl.spin_once()

    def publish_from_stage(self) -> bool:
        return self._impl.publish_from_stage(send=True)

    def publish_once(self, topic: str = BOX_POSE_TOPIC, frame_id: str = BOX_POSE_FRAME) -> bool:
        topic = (topic or BOX_POSE_TOPIC).strip()
        frame_id = (frame_id or BOX_POSE_FRAME).strip()
        if not self._impl.is_active:
            if not self.start(topic, frame_id, continuous=False):
                return False
        return self._impl.publish_from_stage(send=True)

    def publish_once_deferred(
        self, topic: str = BOX_POSE_TOPIC, frame_id: str = BOX_POSE_FRAME
    ) -> None:
        topic = (topic or BOX_POSE_TOPIC).strip()
        frame_id = (frame_id or BOX_POSE_FRAME).strip()
        if not self._impl.is_active or topic != self._topic or frame_id != self._frame_id:
            self.start(topic, frame_id, continuous=False)
        self._impl.publish_once_deferred()

    def stop(self) -> None:
        self._impl.stop()
        self._continuous = False


class BoxPoseSubscriber:
    """订阅外部发布的 geometry_msgs/PoseArray 抓取位姿。"""

    def __init__(self):
        self._node = None
        self._active = False
        self._callback: Optional[Callable[[Transform6D], None]] = None
        self._expected_frame = BOX_POSE_FRAME
        self._topic = GRASP_POSE_ARRAY_TOPIC
        self._pose_index = 0

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        on_pose: Callable[[Transform6D], None],
        topic: str = GRASP_POSE_ARRAY_TOPIC,
        expected_frame: str = BOX_POSE_FRAME,
        pose_index: int = 0,
    ) -> bool:
        self.stop()
        self._callback = on_pose
        self._topic = topic.strip() or GRASP_POSE_ARRAY_TOPIC
        self._expected_frame = expected_frame.strip() or BOX_POSE_FRAME
        self._pose_index = max(0, int(pose_index))
        try:
            from geometry_msgs.msg import PoseArray
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            if not rclpy.ok():
                rclpy.init(args=None)
            parent = self

            class _Node(Node):
                def __init__(self):
                    super().__init__("nova_graspnet_grasp_pose_sub")
                    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                    self.create_subscription(PoseArray, parent._topic, parent._on_msg, qos)

            self._node = _Node()
            self._active = True
            print(
                f"BoxPoseSubscriber: PoseArray {self._topic} "
                f"(frame_id={self._expected_frame}, index={self._pose_index})"
            )
            return True
        except Exception as exc:
            print(f"BoxPoseSubscriber.start failed: {exc}")
            self._active = False
            return False

    def spin_once(self) -> None:
        if not self._active:
            return
        try:
            import rclpy

            rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:
            pass

    def stop(self) -> None:
        self._callback = None
        self._active = False
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._node = None

    def _on_msg(self, msg) -> None:
        if not msg.poses:
            return
        frame = (msg.header.frame_id or "").strip("/")
        expected = self._expected_frame.strip("/")
        if frame and expected and frame != expected:
            print(f"BoxPoseSubscriber: skip frame_id={frame} (expected {expected})")
            return
        idx = min(self._pose_index, len(msg.poses) - 1)
        p = msg.poses[idx]
        pose = Transform6D(
            translation=[
                float(p.position.x),
                float(p.position.y),
                float(p.position.z),
            ],
            rotation_xyzw=[
                float(p.orientation.x),
                float(p.orientation.y),
                float(p.orientation.z),
                float(p.orientation.w),
            ],
            source_frame="grasp_target",
            target_frame=self._expected_frame,
            pose_role="gripper_pose",
        )
        if self._callback:
            self._callback(pose)
