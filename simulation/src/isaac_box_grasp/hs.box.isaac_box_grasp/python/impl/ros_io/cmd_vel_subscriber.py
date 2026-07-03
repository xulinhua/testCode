# -*- coding: utf-8 -*-
"""订阅 cmd_vel，转发给 RobotRuntime 差速控制。"""

from __future__ import annotations

from typing import Callable, Optional

from ..topic_config import TopicConfig


class CmdVelSubscriber:
    def __init__(self):
        self._node = None
        self._active = False
        self._on_twist: Optional[Callable[[float, float], None]] = None

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        topic_config: TopicConfig,
        on_twist: Callable[[float, float], None],
    ) -> bool:
        self.stop()
        self._on_twist = on_twist
        try:
            from geometry_msgs.msg import Twist
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            if not rclpy.ok():
                rclpy.init(args=None)

            topic = topic_config.sub_cmd_vel
            parent = self

            class _Node(Node):
                def __init__(self):
                    super().__init__("isaac_box_grasp_cmd_vel_sub")
                    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                    self.create_subscription(Twist, topic, parent._on_msg, qos)
                    self.get_logger().info(f"Listening Twist on {topic}")

            self._node = _Node()
            self._active = True
            print(f"CmdVelSubscriber: subscribed {topic}")
            return True
        except Exception as exc:
            print(f"CmdVelSubscriber.start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        self._on_twist = None
        self._active = False
        try:
            if self._node is not None:
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
        if self._on_twist:
            self._on_twist(float(msg.linear.x), float(msg.angular.z))
