# -*- coding: utf-8 -*-
"""发布 world→camera TF（未标定时用 Stage 初值）。"""

from __future__ import annotations

from typing import Optional

from ..interfaces import Transform6D
from ..topic_config import TopicConfig


class TfPublisher:
    GRAPH_PATH = "/BoxGrasp/ROS2TfGraph"

    def __init__(self):
        self._active = False

    def start(self, topic_config: TopicConfig, transform: Transform6D) -> bool:
        self.stop()
        if not topic_config.enable_tf:
            return False
        try:
            self._create_graph(topic_config, transform)
            self._active = True
            print(
                f"TfPublisher: world→{topic_config.tf_camera_frame} "
                f"t={transform.translation}"
            )
            return True
        except Exception as exc:
            print(f"TfPublisher.start failed: {exc}")
            return False

    def stop(self) -> None:
        try:
            import omni.graph.core as og

            if og.Controller.graph(self.GRAPH_PATH):
                og.Controller.destroy_graph(self.GRAPH_PATH)
        except Exception as exc:
            print(f"TfPublisher.stop: {exc}")
        self._active = False

    def _create_graph(self, cfg: TopicConfig, transform: Transform6D) -> None:
        import omni.graph.core as og

        tx, ty, tz = transform.translation
        qx, qy, qz, qw = transform.rotation_xyzw
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "push"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishRawTransform"),
                ],
                keys.SET_VALUES: [
                    ("PublishTF.inputs:parentFrameId", cfg.tf_world_frame),
                    ("PublishTF.inputs:childFrameId", cfg.tf_camera_frame),
                    ("PublishTF.inputs:translation", [float(tx), float(ty), float(tz)]),
                    ("PublishTF.inputs:rotation", [float(qx), float(qy), float(qz), float(qw)]),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ],
            },
        )
