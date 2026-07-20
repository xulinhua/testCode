# -*- coding: utf-8 -*-
"""机器人 joint_states 与 TF 树 ROS 出流。"""

from __future__ import annotations

from typing import List

import usdrt.Sdf

from ...global_variables import ROBOT_PRIM_PATH, ROBOT_ROOT_JOINT_PATH
from ..topic_config import RobotRosConfig
from .graph_utils import destroy_omni_graph

GRAPH_PATH = "/NovaGraspNet/RobotRosGraph"

# 与 nova_robot.usda 中 isaac:physics:robotLinks 对齐（前缀在运行时补上）
ROBOT_LINK_SUFFIXES = (
    "base_link",
    "J1_1",
    "J1_2",
    "J1_3",
    "J1_4",
    "J1_5",
    "J1_6",
    "J1_7",
    "J1_8",
    "J2_1",
    "J2_2",
    "J2_3",
    "J2_4",
    "J2_5",
    "J2_6",
    "J2_7",
    "J2_8",
    "J3_1",
    "J3_2",
    "J3_3",
    "J3_4",
    "J3_5",
    "J3_6",
    "J4_1",
    "J4_2",
    "J4_3",
    "J4_4",
    "J4_5",
    "J4_6",
    "base_link/gantry_left_column",
    "base_link/gantry_left_column/gantry_beam",
    "base_link/gantry_left_column/gantry_beam/gantry_connector",
    "base_link/gantry_right_column",
    "base_link/cam0",
    "J1_6/cam1",
    "J2_6/cam2",
)


class RobotRosPublisher:
    """发布 ``/joint_states``、机器人 link TF 树、抓取盒 TF 及 map→world 静态 TF。"""

    def __init__(self):
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    def start(self, cfg: RobotRosConfig, box_link_path: str) -> bool:
        """创建 ``GRAPH_PATH`` 下的 OmniGraph。

        Args:
            cfg: 话题名与各 enable_* 开关。
            box_link_path: 抓取盒 prim 路径，用于 box TF。

        Returns:
            至少一种流启用且图创建成功时为 True。
        """
        self.stop()
        if not cfg.enable_joint_states and not cfg.enable_robot_tf and not cfg.enable_box_tf:
            return False
        try:
            ok = self._create_graph(cfg, box_link_path)
            self._active = ok
            return ok
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"RobotRosPublisher.start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        """销毁机器人 ROS OmniGraph。"""
        destroy_omni_graph(GRAPH_PATH)
        self._active = False

    def _collect_robot_tf_targets(self) -> List[usdrt.Sdf.Path]:
        """收集 Stage 中实际存在的 robot link prim，供 PublishTransformTree 使用。"""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        targets: List[usdrt.Sdf.Path] = []
        if not stage:
            return targets
        for suffix in ROBOT_LINK_SUFFIXES:
            path = f"{ROBOT_PRIM_PATH}/{suffix}"
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                targets.append(usdrt.Sdf.Path(path))
        return targets

    def _create_graph(self, cfg: RobotRosConfig, box_link_path: str) -> bool:
        """按需添加 JointState / RobotTF / BoxTF / map→world 静态 TF 节点。"""
        import omni.graph.core as og

        keys = og.Controller.Keys
        nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
        ]
        values = [
            ("ReadSimTime.inputs:resetOnStop", False),
            ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ]
        connections = []

        if cfg.enable_joint_states:
            nodes.append(("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"))
            values.extend(
                [
                    ("PublishJointState.inputs:topicName", cfg.pub_joint_states),
                    ("PublishJointState.inputs:targetPrim", ROBOT_ROOT_JOINT_PATH),
                    ("PublishJointState.inputs:queueSize", 10),
                ]
            )
            connections.extend(
                [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ]
            )

        if cfg.enable_joint_command:
            nodes.extend(
                [
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ]
            )
            values.extend(
                [
                    ("SubscribeJointState.inputs:topicName", cfg.sub_joint_command),
                    ("SubscribeJointState.inputs:queueSize", 10),
                    ("ArticulationController.inputs:robotPath", ROBOT_ROOT_JOINT_PATH),
                    ("ArticulationController.inputs:targetPrim", usdrt.Sdf.Path(ROBOT_PRIM_PATH)),
                ]
            )
            connections.extend(
                [
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:effortCommand",
                        "ArticulationController.inputs:effortCommand",
                    ),
                ]
            )

        if cfg.enable_robot_tf:
            tf_targets = self._collect_robot_tf_targets()
            if tf_targets:
                nodes.append(("PublishRobotTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
                values.extend(
                    [
                        ("PublishRobotTF.inputs:topicName", cfg.pub_tf),
                        ("PublishRobotTF.inputs:parentPrim", usdrt.Sdf.Path("/World")),
                        ("PublishRobotTF.inputs:targetPrims", tf_targets),
                        ("PublishRobotTF.inputs:queueSize", 10),
                    ]
                )
                connections.extend(
                    [
                        ("OnPlaybackTick.outputs:tick", "PublishRobotTF.inputs:execIn"),
                        ("ROS2Context.outputs:context", "PublishRobotTF.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishRobotTF.inputs:timeStamp"),
                    ]
                )

        if cfg.enable_box_tf and box_link_path:
            nodes.append(("PublishBoxTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
            values.extend(
                [
                    ("PublishBoxTF.inputs:topicName", cfg.pub_tf),
                    ("PublishBoxTF.inputs:parentPrim", usdrt.Sdf.Path("/World")),
                    ("PublishBoxTF.inputs:targetPrims", [usdrt.Sdf.Path(box_link_path)]),
                    ("PublishBoxTF.inputs:queueSize", 10),
                ]
            )
            connections.extend(
                [
                    ("OnPlaybackTick.outputs:tick", "PublishBoxTF.inputs:execIn"),
                    ("ROS2Context.outputs:context", "PublishBoxTF.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishBoxTF.inputs:timeStamp"),
                ]
            )

        # map → world 静态 TF（与 calib 约定一致）
        nodes.append(("PublishMapWorld", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        values.extend(
            [
                ("PublishMapWorld.inputs:topicName", cfg.pub_tf_static),
                ("PublishMapWorld.inputs:parentFrameId", "map"),
                ("PublishMapWorld.inputs:childFrameId", cfg.tf_world_frame),
                ("PublishMapWorld.inputs:translation", [0.0, 0.0, 0.0]),
                ("PublishMapWorld.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
                ("PublishMapWorld.inputs:staticPublisher", True),
            ]
        )
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", "PublishMapWorld.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishMapWorld.inputs:context"),
            ]
        )

        if len(nodes) <= 3:
            print("RobotRosPublisher: nothing to publish")
            return False

        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "push"},
            {
                keys.CREATE_NODES: nodes,
                keys.SET_VALUES: values,
                keys.CONNECT: connections,
            },
        )
        print(
            f"RobotRosPublisher OK graph={GRAPH_PATH} "
            f"joint_states={cfg.enable_joint_states} joint_command={cfg.enable_joint_command}"
        )
        return True
