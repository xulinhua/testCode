# -*- coding: utf-8 -*-
"""ROS2 RGB + CameraInfo streaming (gated to target FPS, no physics)."""

from __future__ import annotations

from ...global_variables import DEFAULT_STREAM_FPS, GRAPH_PATH, ROS_NODE_NAMESPACE
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph


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
        stream_fps: float = DEFAULT_STREAM_FPS,
    ) -> bool:
        if self._active:
            self.stop()
        try:
            fps = max(1.0, float(stream_fps))
            gate_step = max(1, int(round(60.0 / fps)))
            ok = self._create_graph(
                camera_prim_path, resolution, topic_config, fps, gate_step, use_gate=True
            )
            if not ok:
                ok = self._create_graph(
                    camera_prim_path,
                    resolution,
                    topic_config,
                    fps,
                    gate_step=1,
                    use_gate=False,
                )
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
        fps: float,
        gate_step: int,
        use_gate: bool,
    ) -> bool:
        import omni.graph.core as og

        width, height = resolution
        self.stop()
        if not cfg.enable_color and not cfg.enable_camera_info:
            print("CameraPublisher: no stream enabled")
            return False

        keys = og.Controller.Keys
        nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
        values = []
        connections = []

        if use_gate:
            nodes.append(("SimGate", "isaacsim.core.nodes.IsaacSimulationGate"))
            values.append(("SimGate.inputs:step", int(gate_step)))
            connections.append(("OnPlaybackTick.outputs:tick", "SimGate.inputs:execIn"))
            tick_out = "SimGate.outputs:execOut"
        else:
            tick_out = "OnPlaybackTick.outputs:tick"

        nodes.append(("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"))
        values.extend(
            [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:width", int(width)),
                ("CreateRenderProduct.inputs:height", int(height)),
            ]
        )
        connections.append((tick_out, "CreateRenderProduct.inputs:execIn"))

        if cfg.enable_camera_info:
            nodes.append(("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"))
            values.extend(
                [
                    ("CameraInfoPublish.inputs:nodeNamespace", ROS_NODE_NAMESPACE),
                    ("CameraInfoPublish.inputs:topicName", cfg.pub_camera_info),
                    ("CameraInfoPublish.inputs:frameId", cfg.tf_camera_optical_frame),
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

        if cfg.enable_color:
            nodes.append(("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"))
            values.extend(
                [
                    ("PublishRGB.inputs:nodeNamespace", ROS_NODE_NAMESPACE),
                    ("PublishRGB.inputs:topicName", cfg.pub_color),
                    ("PublishRGB.inputs:frameId", cfg.tf_camera_optical_frame),
                    ("PublishRGB.inputs:type", "rgb"),
                    ("PublishRGB.inputs:enableSemanticLabels", False),
                    ("PublishRGB.inputs:resetSimulationTimeOnStop", False),
                ]
            )
            connections.extend(
                [
                    ("CreateRenderProduct.outputs:execOut", "PublishRGB.inputs:execIn"),
                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        "PublishRGB.inputs:renderProductPath",
                    ),
                ]
            )

        try:
            og.Controller.edit(
                {"graph_path": self._graph_path, "evaluator_name": "push"},
                {
                    keys.CREATE_NODES: nodes,
                    keys.SET_VALUES: values,
                    keys.CONNECT: connections,
                },
            )
        except Exception as exc:
            print(f"CameraPublisher: graph edit failed (gate={use_gate}): {exc}")
            destroy_omni_graph(self._graph_path)
            return False

        print(
            f"CameraPublisher OK camera={camera_prim_path} "
            f"res={width}x{height} target_fps={fps:.0f} "
            f"gate_step={gate_step if use_gate else 'off'} "
            f"color={cfg.pub_color if cfg.enable_color else '-'} "
            f"info={cfg.pub_camera_info if cfg.enable_camera_info else '-'}"
        )
        return True
