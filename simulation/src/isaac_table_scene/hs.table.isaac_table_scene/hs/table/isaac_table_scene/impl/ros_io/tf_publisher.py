# -*- coding: utf-8 -*-
"""ROS2 /tf：map→world 静态 + world 下 camera_link/optical/cuboid 树。"""

from __future__ import annotations

import usdrt.Sdf

from ...global_variables import (
    CAMERA_LINK_PATH,
    CAMERA_OPTICAL_PATH,
    CUBOID_LINK_PATH,
    OPTICAL_FRAME_RPY_DEG,
    ROS_NODE_NAMESPACE,
)
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph

GRAPH_PATH = "/TableScene/ROS2TfGraph"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"


def _optical_quat_xyzw() -> list:
    from pxr import Gf

    rot = Gf.Rotation(Gf.Vec3d.XAxis(), OPTICAL_FRAME_RPY_DEG[0])
    rot *= Gf.Rotation(Gf.Vec3d.YAxis(), OPTICAL_FRAME_RPY_DEG[1])
    rot *= Gf.Rotation(Gf.Vec3d.ZAxis(), OPTICAL_FRAME_RPY_DEG[2])
    q = rot.GetQuat()
    imag = q.GetImaginary()
    return [float(imag[0]), float(imag[1]), float(imag[2]), float(q.GetReal())]


class TfPublisher:
    def __init__(self):
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        topic_config: TopicConfig,
        camera_link_path: str = CAMERA_LINK_PATH,
        object_link_path: str = CUBOID_LINK_PATH,
    ) -> bool:
        self.stop()
        if not topic_config.enable_camera_tf and not topic_config.enable_object_tf:
            return False
        try:
            ok = self._create_graph(topic_config, camera_link_path, object_link_path)
            self._active = ok
            return ok
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"TfPublisher.start failed: {exc}")
            return False

    def stop(self) -> None:
        destroy_omni_graph(GRAPH_PATH)
        self._active = False

    def update(self) -> None:
        pass

    def _create_graph(
        self,
        cfg: TopicConfig,
        camera_link_path: str,
        object_link_path: str,
    ) -> bool:
        import omni.graph.core as og

        targets = []
        if cfg.enable_camera_tf:
            targets.append(usdrt.Sdf.Path(camera_link_path))
        if cfg.enable_object_tf:
            targets.append(usdrt.Sdf.Path(object_link_path))
        if not targets:
            return False

        link_frame = cfg.tf_camera_link_frame
        optical_frame = cfg.tf_camera_optical_frame
        optical_q = _optical_quat_xyzw()

        keys = og.Controller.Keys
        nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("PublishMapWorld", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ]
        values = [
            ("ReadSimTime.inputs:resetOnStop", False),
            ("PublishTF.inputs:nodeNamespace", ROS_NODE_NAMESPACE),
            ("PublishTF.inputs:topicName", TF_TOPIC),
            ("PublishTF.inputs:targetPrims", targets),
            ("PublishTF.inputs:parentPrim", usdrt.Sdf.Path("/World")),
            ("PublishMapWorld.inputs:nodeNamespace", ROS_NODE_NAMESPACE),
            ("PublishMapWorld.inputs:topicName", TF_STATIC_TOPIC),
            ("PublishMapWorld.inputs:parentFrameId", cfg.tf_world_frame),
            ("PublishMapWorld.inputs:childFrameId", "world"),
            ("PublishMapWorld.inputs:translation", [0.0, 0.0, 0.0]),
            ("PublishMapWorld.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
            ("PublishMapWorld.inputs:staticPublisher", True),
        ]
        connections = [
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishMapWorld.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        ]

        if cfg.enable_camera_tf:
            nodes.append(("PublishCameraOptical", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
            values.extend(
                [
                    ("PublishCameraOptical.inputs:nodeNamespace", ROS_NODE_NAMESPACE),
                    ("PublishCameraOptical.inputs:topicName", TF_STATIC_TOPIC),
                    ("PublishCameraOptical.inputs:parentFrameId", link_frame),
                    ("PublishCameraOptical.inputs:childFrameId", optical_frame),
                    ("PublishCameraOptical.inputs:translation", [0.0, 0.0, 0.0]),
                    ("PublishCameraOptical.inputs:rotation", optical_q),
                    ("PublishCameraOptical.inputs:staticPublisher", True),
                ]
            )
            connections.append(
                ("OnPlaybackTick.outputs:tick", "PublishCameraOptical.inputs:execIn")
            )

        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "push"},
            {
                keys.CREATE_NODES: nodes,
                keys.SET_VALUES: values,
                keys.CONNECT: connections,
            },
        )
        print(
            f"TfPublisher OK graph={GRAPH_PATH} targets={[str(p) for p in targets]} "
            f"namespace={ROS_NODE_NAMESPACE} static {cfg.tf_world_frame}->world "
            f"{link_frame}->{optical_frame}"
        )
        _ = CAMERA_OPTICAL_PATH  # USD 层级与静态 TF 一致
        return True
