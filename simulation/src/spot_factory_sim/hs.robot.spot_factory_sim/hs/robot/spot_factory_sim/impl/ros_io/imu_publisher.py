# -*- coding: utf-8 -*-
"""OmniGraph 发布 sensor_msgs/Imu（/imu，frame=imu_link）。"""

from __future__ import annotations

import usdrt.Sdf

from ...global_variables import FRAME_IMU, IMU_GRAPH_PATH
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph


class ImuPublisher:
    def __init__(self, graph_path: str = IMU_GRAPH_PATH):
        self._graph_path = graph_path
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    def start(self, imu_prim_path: str, cfg: TopicConfig) -> bool:
        self.stop()
        if not getattr(cfg, "enable_imu", True):
            return False
        if not imu_prim_path:
            print("ImuPublisher: missing IMU prim path")
            return False
        try:
            import omni.graph.core as og

            keys = og.Controller.Keys
            topic = (cfg.pub_imu or "/imu").lstrip("/") or "imu"
            frame_id = getattr(cfg, "frame_imu", None) or FRAME_IMU
            og.Controller.edit(
                {"graph_path": self._graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                        ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
                    ],
                    keys.SET_VALUES: [
                        ("ReadSimTime.inputs:resetOnStop", False),
                        ("ROS2Context.inputs:useDomainIDEnvVar", True),
                        ("ReadIMU.inputs:readGravity", True),
                        ("ReadIMU.inputs:useLatestData", True),
                        ("PublishIMU.inputs:topicName", topic),
                        ("PublishIMU.inputs:frameId", frame_id),
                        ("PublishIMU.inputs:queueSize", 10),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                        ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                        ("ROS2Context.outputs:context", "PublishIMU.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                        ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                        ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                        ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                    ],
                },
            )
            og.Controller.set(
                og.Controller.attribute(f"{self._graph_path}/ReadIMU.inputs:imuPrim"),
                [usdrt.Sdf.Path(imu_prim_path)],
            )
            self._active = True
            print(f"ImuPublisher OK topic=/{topic} frame={frame_id} prim={imu_prim_path}")
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"ImuPublisher.start failed: {exc}")
            self.stop()
            return False

    def stop(self) -> None:
        destroy_omni_graph(self._graph_path)
        self._active = False
