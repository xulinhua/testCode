# -*- coding: utf-8 -*-
"""订阅 world 系 PoseStamped，回调抓取控制器。"""

from __future__ import annotations

from typing import Callable, Optional

from ..interfaces import Transform6D
from ..topic_config import TopicConfig


class PoseSubscriber:
    def __init__(self):
        self._sub = None
        self._active = False
        self._callback: Optional[Callable[[Transform6D], None]] = None
        self._expected_frame = "world"

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        topic_config: TopicConfig,
        on_pose: Callable[[Transform6D], None],
    ) -> bool:
        self.stop()
        self._callback = on_pose
        self._expected_frame = topic_config.tf_world_frame
        try:
            from geometry_msgs.msg import PoseStamped
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            if not rclpy.ok():
                rclpy.init(args=None)

            topic = topic_config.sub_grasp_pose
            parent = self

            class _Node(Node):
                def __init__(self):
                    super().__init__("isaac_box_grasp_pose_sub")
                    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                    self.create_subscription(PoseStamped, topic, parent._on_msg, qos)
                    self.get_logger().info(f"Listening PoseStamped on {topic}")

            self._node = _Node()
            self._active = True
            print(f"PoseSubscriber: subscribed {topic} (frame_id={self._expected_frame})")
            return True
        except Exception as exc:
            print(f"PoseSubscriber.start failed: {exc}")
            print("  (Ensure ROS2 bridge / rclpy is available in Isaac Sim)")
            self._active = False
            return False

    def stop(self) -> None:
        self._sub = None
        self._callback = None
        self._active = False
        try:
            if hasattr(self, "_node") and self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        self._node = None

    def spin_once(self) -> None:
        if not self._active:
            return
        try:
            import rclpy

            rclpy.spin_once(self._node, timeout_sec=0.0)
        except Exception:
            pass

    def _on_msg(self, msg) -> None:
        frame = (msg.header.frame_id or "").strip("/")
        expected = self._expected_frame.strip("/")
        if frame and expected and frame != expected:
            print(f"PoseSubscriber: skip frame_id={frame} (expected {expected})")
            return
        pose = Transform6D(
            translation=[
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            rotation_xyzw=[
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ],
            source_frame="grasp_target",
            target_frame=self._expected_frame,
        )
        if self._callback:
            self._callback(pose)
