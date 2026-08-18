# -*- coding: utf-8 -*-
"""RTX 激光雷达 ROS2 OmniGraph：/lidar/points 与可选 /scan。"""

from __future__ import annotations

from ...global_variables import (
    FRAME_LIDAR,
    LIDAR_FRAME_SKIP,
    LIDAR_GRAPH_PATH,
)
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph


class LidarPublisher:
    def __init__(self, graph_path: str = LIDAR_GRAPH_PATH):
        self._graph_path = graph_path
        self._active = False
        self._render_product = None
        self._render_product_path = None

    @property
    def is_active(self) -> bool:
        return self._active

    def start(self, lidar_prim_path: str, cfg: TopicConfig) -> bool:
        self.stop()
        enable_points = bool(getattr(cfg, "enable_lidar_points", True))
        enable_scan = bool(getattr(cfg, "enable_lidar_scan", False))
        if not enable_points and not enable_scan:
            return False
        if not lidar_prim_path:
            print("LidarPublisher: missing lidar prim path")
            return False
        try:
            import omni.graph.core as og
            import omni.replicator.core as rep

            self._render_product = rep.create.render_product(
                lidar_prim_path,
                resolution=(128, 128),
                render_vars=["GenericModelOutput", "RtxSensorMetadata"],
            )
            self._render_product_path = getattr(self._render_product, "path", None) or str(
                self._render_product
            )
            if not self._render_product_path:
                print("LidarPublisher: failed to create RTX lidar render product")
                self._destroy_render_product()
                return False

            keys = og.Controller.Keys
            frame_id = getattr(cfg, "frame_lidar", None) or FRAME_LIDAR
            skip = int(LIDAR_FRAME_SKIP)
            nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
            values = []
            connections = []

            if enable_points:
                topic = (cfg.pub_lidar_points or "/lidar/points").lstrip("/") or "lidar/points"
                nodes.append(("PubPCL", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
                values.extend(
                    [
                        ("PubPCL.inputs:renderProductPath", self._render_product_path),
                        ("PubPCL.inputs:topicName", topic),
                        ("PubPCL.inputs:frameId", frame_id),
                        ("PubPCL.inputs:type", "point_cloud"),
                        ("PubPCL.inputs:fullScan", True),
                        ("PubPCL.inputs:resetSimulationTimeOnStop", False),
                        ("PubPCL.inputs:frameSkipCount", skip),
                        ("PubPCL.inputs:showDebugView", False),
                    ]
                )
                connections.append(("OnPlaybackTick.outputs:tick", "PubPCL.inputs:execIn"))

            if enable_scan:
                topic = (cfg.pub_lidar_scan or "/scan").lstrip("/") or "scan"
                nodes.append(("PubScan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
                values.extend(
                    [
                        ("PubScan.inputs:renderProductPath", self._render_product_path),
                        ("PubScan.inputs:topicName", topic),
                        ("PubScan.inputs:frameId", frame_id),
                        ("PubScan.inputs:type", "laser_scan"),
                        ("PubScan.inputs:resetSimulationTimeOnStop", False),
                        ("PubScan.inputs:frameSkipCount", skip),
                        ("PubScan.inputs:showDebugView", False),
                    ]
                )
                connections.append(("OnPlaybackTick.outputs:tick", "PubScan.inputs:execIn"))

            og.Controller.edit(
                {"graph_path": self._graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: nodes,
                    keys.SET_VALUES: values,
                    keys.CONNECT: connections,
                },
            )
            self._active = True
            print(
                f"LidarPublisher OK prim={lidar_prim_path} rp={self._render_product_path} "
                f"frame={frame_id} points={enable_points} scan={enable_scan}"
            )
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"LidarPublisher.start failed: {exc}")
            self.stop()
            return False

    def stop(self) -> None:
        destroy_omni_graph(self._graph_path)
        self._destroy_render_product()
        self._active = False

    def _destroy_render_product(self) -> None:
        rp = self._render_product
        self._render_product = None
        self._render_product_path = None
        if rp is None:
            return
        for meth in ("destroy", "detach"):
            fn = getattr(rp, meth, None)
            if callable(fn):
                try:
                    fn()
                    return
                except Exception:
                    pass
        try:
            import omni.replicator.core as rep

            path = getattr(rp, "path", None)
            if path:
                rep.destroy.render_product(path)
        except Exception:
            pass
