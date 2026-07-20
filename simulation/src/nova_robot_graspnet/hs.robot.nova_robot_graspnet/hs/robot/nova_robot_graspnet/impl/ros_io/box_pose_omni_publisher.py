# -*- coding: utf-8 -*-
"""通过 OmniGraph 发布 /box_pose（与 joint_states 同源 ROS2Context，可被 ros2 topic list 发现）。"""

from __future__ import annotations

import time
from typing import Optional

from ...global_variables import (
    BASE_LINK_PATH,
    BOX_LINK_PATH,
    BOX_POSE_FRAME,
    BOX_POSE_PUBLISH_HZ,
    BOX_POSE_TOPIC,
)
from ..pose_utils import read_box_pose_in_frame
from .graph_utils import destroy_omni_graph

GRAPH_PATH = "/NovaGraspNet/BoxPoseGraph"
PUBLISHER_NODE = "PublishBoxPose"


class BoxPoseOmniPublisher:
    """Isaac ROS2 bridge OmniGraph 版 PoseStamped 发布器。"""

    def __init__(self):
        self._active = False
        self._topic = BOX_POSE_TOPIC
        self._frame_id = BOX_POSE_FRAME
        self._continuous = False
        self._publish_interval = 1.0 / max(float(BOX_POSE_PUBLISH_HZ), 0.1)
        self._last_publish_mono = 0.0

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        topic: str = BOX_POSE_TOPIC,
        frame_id: str = BOX_POSE_FRAME,
        *,
        continuous: bool = False,
    ) -> bool:
        """创建 BoxPose OmniGraph；continuous=True 时按 BOX_POSE_PUBLISH_HZ 定时发布。"""
        self.stop()
        self._topic = (topic or BOX_POSE_TOPIC).strip()
        self._frame_id = (frame_id or BOX_POSE_FRAME).strip()
        self._continuous = bool(continuous)
        try:
            import omni.graph.core as og

            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": GRAPH_PATH, "evaluator_name": "push"},
                {
                    keys.CREATE_NODES: [
                        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("BoxPoseQoS", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                        (PUBLISHER_NODE, "isaacsim.ros2.bridge.ROS2Publisher"),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:useDomainIDEnvVar", True),
                        ("BoxPoseQoS.inputs:createProfile", "Custom"),
                        ("BoxPoseQoS.inputs:durability", "transientLocal"),
                        ("BoxPoseQoS.inputs:reliability", "reliable"),
                        ("BoxPoseQoS.inputs:depth", 1),
                        ("BoxPoseQoS.inputs:history", "keepLast"),
                        (f"{PUBLISHER_NODE}.inputs:topicName", self._topic),
                        (f"{PUBLISHER_NODE}.inputs:messagePackage", "geometry_msgs"),
                        (f"{PUBLISHER_NODE}.inputs:messageName", "PoseStamped"),
                        (f"{PUBLISHER_NODE}.inputs:messageSubfolder", "msg"),
                    ],
                    keys.CONNECT: [
                        ("ROS2Context.outputs:context", f"{PUBLISHER_NODE}.inputs:context"),
                        ("BoxPoseQoS.outputs:qosProfile", f"{PUBLISHER_NODE}.inputs:qosProfile"),
                    ],
                },
            )
            self._active = True
            self._last_publish_mono = 0.0
            print(
                f"BoxPoseOmniPublisher: graph ready topic={self._topic} "
                f"frame={self._frame_id} continuous={self._continuous} "
                f"hz={BOX_POSE_PUBLISH_HZ}"
            )
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"BoxPoseOmniPublisher.start failed: {exc}")
            self._active = False
            return False

    def _set_input(self, suffix: str, value) -> bool:
        import omni.graph.core as og

        candidates = (
            f"{GRAPH_PATH}/{PUBLISHER_NODE}.inputs:{suffix}",
            f"{GRAPH_PATH}/{PUBLISHER_NODE}.inputs:{suffix.replace(':', '.')}",
        )
        for path in candidates:
            try:
                og.Controller.attribute(path).set(value)
                return True
            except Exception:
                continue
        return False

    @staticmethod
    def _current_stamp() -> tuple[int, int]:
        """返回 (sec, nanosec) 供 PoseStamped.header.stamp 使用。"""
        try:
            import omni.timeline

            t = float(omni.timeline.get_timeline_interface().get_current_time())
        except Exception:
            import time

            t = time.time()
        sec = int(t)
        nanosec = int((t - sec) * 1_000_000_000)
        return sec, nanosec

    def publish_from_stage(self, *, send: bool = False) -> bool:
        if not self._active:
            return False
        data = read_box_pose_in_frame(BOX_LINK_PATH, BASE_LINK_PATH)
        if not data:
            print(f"BoxPoseOmniPublisher: no box pose ({BOX_LINK_PATH})")
            return False
        pos = data["position"]
        quat = data["orientation_xyzw"]
        sec, nanosec = self._current_stamp()
        ok = True
        ok &= self._set_input("header:frame_id", self._frame_id)
        ok &= self._set_input("header:stamp:sec", sec)
        ok &= self._set_input("header:stamp:nanosec", nanosec)
        ok &= self._set_input("pose:position:x", float(pos[0]))
        ok &= self._set_input("pose:position:y", float(pos[1]))
        ok &= self._set_input("pose:position:z", float(pos[2]))
        ok &= self._set_input("pose:orientation:x", float(quat[0]))
        ok &= self._set_input("pose:orientation:y", float(quat[1]))
        ok &= self._set_input("pose:orientation:z", float(quat[2]))
        ok &= self._set_input("pose:orientation:w", float(quat[3]))
        if not ok:
            print("BoxPoseOmniPublisher: WARN some PoseStamped inputs missing on ROS2Publisher")
        if send:
            self._evaluate_publish()
            print(
                f"BoxPoseOmniPublisher: published {self._topic} "
                f"pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) frame={self._frame_id}"
            )
        return True

    def _evaluate_publish(self) -> None:
        import omni.graph.core as og

        try:
            og.Controller.evaluate_sync(GRAPH_PATH)
        except Exception as exc:
            print(f"BoxPoseOmniPublisher: evaluate_sync: {exc}")

    def spin_once(self) -> None:
        """连续模式：按 BOX_POSE_PUBLISH_HZ 刷新并发布。"""
        if not self._active or not self._continuous:
            return
        now = time.monotonic()
        if now - self._last_publish_mono < self._publish_interval:
            return
        self._last_publish_mono = now
        self.publish_from_stage(send=True)

    def publish_once_deferred(self) -> None:
        import asyncio

        asyncio.ensure_future(self._publish_once_deferred())

    async def _publish_once_deferred(self) -> bool:
        import omni.kit.app

        if not self._active:
            if not self.start(self._topic, self._frame_id, continuous=False):
                return False
        app = omni.kit.app.get_app()
        for _ in range(5):
            await app.next_update_async()
        if not self.publish_from_stage(send=True):
            return False
        print(f"BoxPoseOmniPublisher: one-shot publish -> {self._topic}")
        return True

    def stop(self) -> None:
        destroy_omni_graph(GRAPH_PATH)
        self._active = False
        self._continuous = False
