# -*- coding: utf-8 -*-
"""前方相机 ROS2 OmniGraph：color / depth / points / camera_info。

消息 frame_id 使用光学系（与 cam_mgr / OpenCV / geometry_lib 反投影一致）：
- color → cam_0_color_optical_frame
- depth / points / camera_info → cam_0_depth_optical_frame
"""

from __future__ import annotations

from ...global_variables import (
    CAMERA_GRAPH_PATH,
    FRAME_COLOR_OPTICAL,
    FRAME_DEPTH_OPTICAL,
)
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph


class CameraPublisher:
    def __init__(self, graph_path: str = CAMERA_GRAPH_PATH):
        self._graph_path = graph_path
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    def start(self, camera_prim_path: str, cfg: TopicConfig) -> bool:
        if self._active:
            self.stop()
        if not camera_prim_path:
            print("CameraPublisher: missing camera prim path")
            return False
        if not (
            cfg.enable_color
            or cfg.enable_depth
            or cfg.enable_points
            or cfg.enable_camera_info
        ):
            print("CameraPublisher: no camera stream enabled")
            return False
        try:
            ok = self._create_graph(camera_prim_path, cfg)
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

    def _create_graph(self, camera_prim_path: str, cfg: TopicConfig) -> bool:
        import omni.graph.core as og
        import usdrt.Sdf

        width, height = int(cfg.image_width), int(cfg.image_height)
        depth_optical = getattr(cfg, "frame_camera", None) or FRAME_DEPTH_OPTICAL
        color_optical = FRAME_COLOR_OPTICAL
        keys = og.Controller.Keys

        nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ]
        # 先建图再写 target 型 cameraPrim，避免 Fabric “attribute not found”
        values = [
            ("CreateRenderProduct.inputs:width", width),
            ("CreateRenderProduct.inputs:height", height),
        ]
        connections = [
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        ]

        helpers = []
        if cfg.enable_color:
            helpers.append(("PubRGB", "rgb", cfg.pub_color, color_optical))
        if cfg.enable_depth:
            helpers.append(("PubDepth", "depth", cfg.pub_depth, depth_optical))
        if cfg.enable_points:
            helpers.append(("PubPCL", "depth_pcl", cfg.pub_points, depth_optical))

        if cfg.enable_camera_info:
            nodes.append(("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"))
            values.extend(
                [
                    ("CameraInfoPublish.inputs:topicName", cfg.pub_camera_info),
                    ("CameraInfoPublish.inputs:frameId", depth_optical),
                    # 必须与 OdomTf 的 ReadSimTime.resetOnStop=False 一致。
                    # True 时 Stop/Play 后相机戳会清零，而 /tf 继续累加，RViz 查 TF 会报
                    # “timestamp earlier than all the data in the transform cache”。
                    ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", False),
                ]
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

        for name, helper_type, topic, frame_id in helpers:
            nodes.append((name, "isaacsim.ros2.bridge.ROS2CameraHelper"))
            values.extend(
                [
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", frame_id),
                    (f"{name}.inputs:type", helper_type),
                    (f"{name}.inputs:enableSemanticLabels", False),
                    (f"{name}.inputs:resetSimulationTimeOnStop", False),
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

        og.Controller.edit(
            {"graph_path": self._graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: nodes,
                keys.SET_VALUES: values,
                keys.CONNECT: connections,
            },
        )
        cam_attr = og.Controller.attribute(
            f"{self._graph_path}/CreateRenderProduct.inputs:cameraPrim"
        )
        og.Controller.set(cam_attr, [usdrt.Sdf.Path(camera_prim_path)])
        print(
            f"CameraPublisher OK graph={self._graph_path} camera={camera_prim_path} "
            f"res={width}x{height} color_frame={color_optical} depth_frame={depth_optical}"
        )
        return True
