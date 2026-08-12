# -*- coding: utf-8 -*-
"""OmniGraph 发布 /joint_states。"""

from __future__ import annotations

import usdrt.Sdf

from ...global_variables import JOINT_GRAPH_PATH, SPOT_PRIM_PATH
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph


class JointStatePublisher:
    def __init__(self, graph_path: str = JOINT_GRAPH_PATH):
        self._graph_path = graph_path
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    def start(self, cfg: TopicConfig, robot_prim_path: str = SPOT_PRIM_PATH) -> bool:
        self.stop()
        if not cfg.enable_joint_states:
            return False
        try:
            import omni.graph.core as og

            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": self._graph_path, "evaluator_name": "push"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadSimTime.inputs:resetOnStop", False),
                        ("ROS2Context.inputs:useDomainIDEnvVar", True),
                        ("PublishJointState.inputs:topicName", cfg.pub_joint_states),
                        ("PublishJointState.inputs:targetPrim", usdrt.Sdf.Path(robot_prim_path)),
                        ("PublishJointState.inputs:queueSize", 10),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "PublishJointState.inputs:timeStamp",
                        ),
                    ],
                },
            )
            self._active = True
            print(f"JointStatePublisher OK topic={cfg.pub_joint_states} prim={robot_prim_path}")
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"JointStatePublisher.start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        destroy_omni_graph(self._graph_path)
        self._active = False
