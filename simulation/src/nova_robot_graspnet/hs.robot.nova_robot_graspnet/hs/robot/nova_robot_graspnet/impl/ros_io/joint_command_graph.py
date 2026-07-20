# -*- coding: utf-8 -*-
"""OmniGraph：订阅 /joint_command 并驱动 Isaac ArticulationController。"""

from __future__ import annotations

import usdrt.Sdf

from ...global_variables import JOINT_COMMAND_TOPIC, ROBOT_PRIM_PATH, ROBOT_ROOT_JOINT_PATH
from .graph_utils import destroy_omni_graph

GRAPH_PATH = "/NovaGraspNet/JointCommandGraph"

class JointCommandGraph:
    """与 RobotRosPublisher 共用同一 ROS2Context 栈，接收外部关节命令。"""

    def __init__(self):
        self._active = False
        self._topic = JOINT_COMMAND_TOPIC

    @property
    def is_active(self) -> bool:
        return self._active

    def start(self, topic: str = JOINT_COMMAND_TOPIC) -> bool:
        self.stop()
        self._topic = (topic or JOINT_COMMAND_TOPIC).strip() or JOINT_COMMAND_TOPIC
        try:
            import omni.graph.core as og

            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": GRAPH_PATH, "evaluator_name": "push"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:useDomainIDEnvVar", True),
                        ("SubscribeJointState.inputs:topicName", self._topic),
                        ("SubscribeJointState.inputs:queueSize", 10),
                        ("ArticulationController.inputs:robotPath", ROBOT_ROOT_JOINT_PATH),
                        ("ArticulationController.inputs:targetPrim", usdrt.Sdf.Path(ROBOT_PRIM_PATH)),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"),
                        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                },
            )
            self._active = True
            print(f"JointCommandGraph: listening on {self._topic} via OmniGraph")
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"JointCommandGraph.start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        destroy_omni_graph(GRAPH_PATH)
        self._active = False
