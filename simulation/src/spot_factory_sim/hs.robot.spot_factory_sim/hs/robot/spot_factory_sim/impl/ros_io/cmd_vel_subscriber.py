# -*- coding: utf-8 -*-
"""OmniGraph 订阅 /cmd_vel（ROS2SubscribeTwist），不依赖系统 rclpy。"""

from __future__ import annotations

from typing import Callable, Optional

from ...global_variables import GRAPH_ROOT
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph

CMD_VEL_GRAPH_PATH = f"{GRAPH_ROOT}/CmdVelGraph"


class CmdVelSubscriber:
    def __init__(self, graph_path: str = CMD_VEL_GRAPH_PATH):
        self._graph_path = graph_path
        self._active = False
        self._on_twist: Optional[Callable[[float, float, float], None]] = None

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        topic_config: TopicConfig,
        on_twist: Callable[[float, float, float], None],
    ) -> bool:
        self.stop()
        self._on_twist = on_twist
        try:
            import omni.graph.core as og

            topic = topic_config.sub_cmd_vel.lstrip("/") or "cmd_vel"
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": self._graph_path, "evaluator_name": "push"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:useDomainIDEnvVar", True),
                        ("SubscribeTwist.inputs:topicName", topic),
                        ("SubscribeTwist.inputs:queueSize", 10),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                        ("ROS2Context.outputs:context", "SubscribeTwist.inputs:context"),
                    ],
                },
            )
            self._active = True
            print(f"CmdVelSubscriber (OmniGraph): subscribed {topic}")
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"CmdVelSubscriber.start failed: {exc}")
            self._active = False
            return False

    def stop(self) -> None:
        self._on_twist = None
        destroy_omni_graph(self._graph_path)
        self._active = False

    def spin_once(self) -> None:
        """每帧读取最新 Twist，转给 SpotRuntime。

        注意：无新消息时 OmniGraph 输出仍是默认 [0,0,0]。若每帧无条件灌零速，
        会冲掉面板遥控；调用方应在 UI 驾驶时跳过，或仅在有订阅回调时应用。
        """
        if not self._active or self._on_twist is None:
            return
        try:
            import omni.graph.core as og

            lin = og.Controller.get(
                og.Controller.attribute(f"{self._graph_path}/SubscribeTwist.outputs:linearVelocity")
            )
            ang = og.Controller.get(
                og.Controller.attribute(f"{self._graph_path}/SubscribeTwist.outputs:angularVelocity")
            )
            if lin is None or ang is None:
                return
            self._on_twist(float(lin[0]), float(lin[1]), float(ang[2]))
        except Exception:
            pass
