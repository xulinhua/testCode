# -*- coding: utf-8 -*-
"""Timeline Play 时通过 OmniGraph + ROS2CameraHelper 持续出流。"""

from __future__ import annotations

from typing import Optional

from ..topic_config import TopicConfig


GRAPH_PATH = "/BoxGrasp/ROS2CameraGraph"


class CameraPublisher:
    """Play 时创建 ROS2 相机图；Stop 时销毁。"""

    def __init__(self):
        self._active = False
        self._graph_path = GRAPH_PATH

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        camera_prim_path: str,
        resolution: tuple,
        topic_config: TopicConfig,
    ) -> bool:
        if self._active:
            self.stop()
        try:
            ok = self._create_graph(camera_prim_path, resolution, topic_config)
            self._active = ok
            if ok:
                print("CameraPublisher: ROS2 stream graph started (Play timeline)")
            return ok
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"CameraPublisher.start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        try:
            import omni.graph.core as og

            if og.Controller.graph(self._graph_path):
                og.Controller.destroy_graph(self._graph_path)
        except Exception as exc:
            print(f"CameraPublisher.stop: {exc}")
        self._active = False

    def _create_graph(
        self,
        camera_prim_path: str,
        resolution: tuple,
        cfg: TopicConfig,
    ) -> bool:
        import omni.graph.core as og

        width, height = resolution
        self.stop()

        keys = og.Controller.Keys
        nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ]
        values = [
            ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
            ("CreateRenderProduct.inputs:width", int(width)),
            ("CreateRenderProduct.inputs:height", int(height)),
        ]
        connections = [
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        ]

        def add_helper(name: str, helper_type: str, topic: str):
            nodes.append((name, "isaacsim.ros2.bridge.ROS2CameraHelper"))
            values.extend(
                [
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", cfg.tf_camera_frame),
                    (f"{name}.inputs:type", helper_type),
                    (f"{name}.inputs:enableSemanticLabels", False),
                ]
            )
            connections.extend(
                [
                    ("CreateRenderProduct.outputs:execOut", f"{name}.inputs:execIn"),
                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        f"{name}.inputs:renderProductPath",
                    ),
                ]
            )

        if cfg.enable_color:
            add_helper("PublishRGB", "rgb", cfg.pub_color)
        if cfg.enable_depth:
            add_helper("PublishDepth", "depth", cfg.pub_depth)
        if cfg.enable_points:
            # Isaac ROS2 bridge publishes sensor_msgs/PointCloud2 for depth_pcl
            add_helper("PublishPointCloud", "depth_pcl", cfg.pub_points)

        og.Controller.edit(
            {"graph_path": self._graph_path, "evaluator_name": "push"},
            {
                keys.CREATE_NODES: nodes,
                keys.SET_VALUES: values,
                keys.CONNECT: connections,
            },
        )
        print(
            f"CameraPublisher graph={self._graph_path} camera={camera_prim_path} "
            f"color={cfg.enable_color} depth={cfg.enable_depth} points={cfg.enable_points}"
        )
        return True
