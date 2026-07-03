# -*- coding: utf-8 -*-
"""ROS2 相机出流：对齐官方 ROS2 Camera Graph（CameraInfo + RGB/Depth/PointCloud）。"""

from __future__ import annotations

from ...global_variables import ROS_NODE_NAMESPACE
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph

GRAPH_PATH = "/TableScene/ROS2CameraGraph"


class CameraPublisher:
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
            return ok
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"CameraPublisher.start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        destroy_omni_graph(self._graph_path)
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

        link_frame = cfg.tf_camera_link_frame
        optical_frame = cfg.tf_camera_optical_frame
        # color / depth / CameraInfo → camera_link（与项目 Gazebo RGB 一致）
        # PointCloud2 → camera_optical_frame（3D 点云用光学系）
        image_helpers = []
        if cfg.enable_color:
            image_helpers.append(("PublishRGB", "rgb", cfg.pub_color))
        if cfg.enable_depth:
            image_helpers.append(("PublishDepth", "depth", cfg.pub_depth))
        point_helpers = []
        if cfg.enable_points:
            point_helpers.append(("PublishPointCloud", "depth_pcl", cfg.pub_points))
        helpers = image_helpers + point_helpers
        if not helpers and not cfg.enable_camera_info:
            print("CameraPublisher: no stream enabled")
            return False

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

        def add_ros_inputs(name: str, extra: list) -> None:
            values.extend(
                [
                    (f"{name}.inputs:nodeNamespace", ROS_NODE_NAMESPACE),
                    *extra,
                ]
            )

        if cfg.enable_camera_info:
            nodes.append(("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"))
            add_ros_inputs(
                "CameraInfoPublish",
                [
                    ("CameraInfoPublish.inputs:topicName", cfg.pub_camera_info),
                    ("CameraInfoPublish.inputs:frameId", link_frame),
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
            add_ros_inputs(
                name,
                [
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", link_frame),
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
            add_ros_inputs(
                name,
                [
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", optical_frame),
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
            f"CameraPublisher OK graph={self._graph_path} camera={camera_prim_path} "
            f"image_frame={link_frame} pointcloud_frame={optical_frame} "
            f"namespace={ROS_NODE_NAMESPACE} topics={[t for _, _, t in helpers]}"
        )
        return True
