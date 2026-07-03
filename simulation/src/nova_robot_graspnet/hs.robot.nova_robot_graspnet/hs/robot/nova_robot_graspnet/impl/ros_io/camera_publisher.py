# -*- coding: utf-8 -*-
"""单路相机 ROS2 OmniGraph：RenderProduct → Image/CameraInfo/PointCloud2。"""

from __future__ import annotations

from ..topic_config import CameraStreamConfig
from .graph_utils import destroy_omni_graph


class SingleCameraPublisher:
    """为单个 ``Camera_Pseudo_Depth`` prim 创建 Isaac ROS2 相机发布图。"""

    def __init__(self, graph_path: str):
        """Args:
            graph_path: OmniGraph 根路径，如 ``/NovaGraspNet/CameraGraph/cam0``。
        """
        self._graph_path = graph_path
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    def start(self, cfg: CameraStreamConfig) -> bool:
        """按 ``cfg`` 中 enable_* 与话题名创建 OmniGraph。

        Returns:
            至少启用一种流且 ``camera_prim_path`` 有效时为 True。
        """
        if self._active:
            self.stop()
        if not cfg.camera_prim_path:
            print(f"SingleCameraPublisher[{cfg.key}]: missing camera prim path")
            return False
        try:
            ok = self._create_graph(cfg)
            self._active = ok
            return ok
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SingleCameraPublisher[{cfg.key}].start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        """销毁本路 OmniGraph。"""
        destroy_omni_graph(self._graph_path)
        self._active = False

    def _create_graph(self, cfg: CameraStreamConfig) -> bool:
        """组装 OnPlaybackTick → CreateRenderProduct → ROS2CameraHelper 节点链。"""
        import omni.graph.core as og

        width, height = int(cfg.width), int(cfg.height)
        frame_id = cfg.frame_id

        image_helpers = []
        if cfg.enable_color:
            image_helpers.append((f"PubRGB_{cfg.key}", "rgb", cfg.pub_color))
        if cfg.enable_depth:
            image_helpers.append((f"PubDepth_{cfg.key}", "depth", cfg.pub_depth))
        point_helpers = []
        if cfg.enable_points:
            point_helpers.append((f"PubPCL_{cfg.key}", "depth_pcl", cfg.pub_points))

        if not image_helpers and not point_helpers and not cfg.enable_camera_info:
            print(f"SingleCameraPublisher[{cfg.key}]: no stream enabled")
            return False

        keys = og.Controller.Keys
        nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ]
        values = [
            ("CreateRenderProduct.inputs:cameraPrim", cfg.camera_prim_path),
            ("CreateRenderProduct.inputs:width", width),
            ("CreateRenderProduct.inputs:height", height),
        ]
        connections = [
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        ]

        def add_ros(name: str, extra: list) -> None:
            values.extend(extra)

        if cfg.enable_camera_info:
            nodes.append(("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"))
            add_ros(
                "CameraInfoPublish",
                [
                    ("CameraInfoPublish.inputs:topicName", cfg.pub_camera_info),
                    ("CameraInfoPublish.inputs:frameId", frame_id),
                    ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                ],
            )
            connections.extend(
                [
                    ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        "CameraInfoPublish.inputs:renderProductPath",
                    ),
                ]
            )

        for name, helper_type, topic in image_helpers:
            nodes.append((name, "isaacsim.ros2.bridge.ROS2CameraHelper"))
            add_ros(
                name,
                [
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", frame_id),
                    (f"{name}.inputs:type", helper_type),
                    (f"{name}.inputs:enableSemanticLabels", False),
                    (f"{name}.inputs:resetSimulationTimeOnStop", True),
                ],
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

        for name, helper_type, topic in point_helpers:
            nodes.append((name, "isaacsim.ros2.bridge.ROS2CameraHelper"))
            add_ros(
                name,
                [
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", frame_id),
                    (f"{name}.inputs:type", helper_type),
                    (f"{name}.inputs:enableSemanticLabels", False),
                    (f"{name}.inputs:resetSimulationTimeOnStop", True),
                ],
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

        og.Controller.edit(
            {"graph_path": self._graph_path, "evaluator_name": "push"},
            {
                keys.CREATE_NODES: nodes,
                keys.SET_VALUES: values,
                keys.CONNECT: connections,
            },
        )
        print(
            f"SingleCameraPublisher OK key={cfg.key} graph={self._graph_path} "
            f"camera={cfg.camera_prim_path} res={width}x{height} frame={frame_id}"
        )
        return True
