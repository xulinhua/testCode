# -*- coding: utf-8 -*-
"""OmniGraph 发布 /odom 与完整 TF：

- /tf: odom→base_link（里程计）
- /tf: Spot 整棵 articulation 连杆树（body/腿等）
- /tf_static: base_link→body、base_link→cam_0_link、base_link→imu_link、base_link→lidar_link、以及光学系链
"""

from __future__ import annotations

from typing import Optional, Sequence

import usdrt.Sdf

from ...global_variables import (
    ENABLE_ROBOT_LINK_TF,
    FRAME_CAMERA_LINK,
    FRAME_COLOR,
    FRAME_COLOR_OPTICAL,
    FRAME_DEPTH,
    FRAME_DEPTH_OPTICAL,
    FRAME_IMU,
    FRAME_LIDAR,
    GRAPH_ROOT,
    SPOT_PRIM_PATH,
    TOPIC_TF_STATIC,
)
from ..topic_config import TopicConfig
from .graph_utils import destroy_omni_graph

ODOM_TF_GRAPH_PATH = f"{GRAPH_ROOT}/OdomTfGraph"


def _add_static_tf(
    nodes,
    values,
    connections,
    *,
    name: str,
    topic: str,
    parent: str,
    child: str,
    xyz: Sequence[float],
    quat_xyzw: Sequence[float],
) -> None:
    nodes.append((name, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
    values.extend(
        [
            (f"{name}.inputs:topicName", topic),
            (f"{name}.inputs:parentFrameId", parent),
            (f"{name}.inputs:childFrameId", child),
            (f"{name}.inputs:translation", [float(xyz[0]), float(xyz[1]), float(xyz[2])]),
            (
                f"{name}.inputs:rotation",
                [float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2]), float(quat_xyzw[3])],
            ),
            (f"{name}.inputs:staticPublisher", True),
            (f"{name}.inputs:queueSize", 1),
        ]
    )
    connections.extend(
        [
            ("OnPlaybackTick.outputs:tick", f"{name}.inputs:execIn"),
            ("ROS2Context.outputs:context", f"{name}.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", f"{name}.inputs:timeStamp"),
        ]
    )


class OdomTfPublisher:
    def __init__(self, graph_path: str = ODOM_TF_GRAPH_PATH):
        self._graph_path = graph_path
        self._active = False
        self._cfg: Optional[TopicConfig] = None

    @property
    def is_active(self) -> bool:
        return self._active

    def start(
        self,
        cfg: TopicConfig,
        cam_xyz_base: Sequence[float],
        cam_link_quat_xyzw_base: Sequence[float],
        cam_optical_from_link_quat_xyzw: Sequence[float],
        chassis_prim_path: str = SPOT_PRIM_PATH,
        imu_xyz_base: Optional[Sequence[float]] = None,
        imu_link_quat_xyzw_base: Optional[Sequence[float]] = None,
        lidar_xyz_base: Optional[Sequence[float]] = None,
        lidar_link_quat_xyzw_base: Optional[Sequence[float]] = None,
    ) -> bool:
        self.stop()
        if not cfg.enable_odom and not cfg.enable_tf:
            return False
        self._cfg = cfg
        try:
            import omni.graph.core as og

            keys = og.Controller.Keys
            odom_topic = cfg.pub_odom.lstrip("/") or "odom"
            tf_topic = cfg.pub_tf.lstrip("/") or "tf"
            tf_static_topic = (getattr(cfg, "pub_tf_static", None) or TOPIC_TF_STATIC).lstrip("/") or "tf_static"

            cx, cy, cz = [float(v) for v in cam_xyz_base]
            lqx, lqy, lqz, lqw = [float(v) for v in cam_link_quat_xyzw_base]
            oqx, oqy, oqz, oqw = [float(v) for v in cam_optical_from_link_quat_xyzw]
            identity = (0.0, 0.0, 0.0, 1.0)

            nodes = [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ]
            values = [
                # 与相机 ROS2CameraHelper.resetSimulationTimeOnStop=False 对齐，避免 Stop/Play 后
                # 相机戳归零而 /tf、/odom 继续累加，导致下游 MessageFilter 丢图。
                ("ReadSimTime.inputs:resetOnStop", False),
                ("ROS2Context.inputs:useDomainIDEnvVar", True),
                ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(chassis_prim_path)]),
            ]
            connections = [
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ]

            if cfg.enable_odom:
                nodes.append(("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"))
                values.extend(
                    [
                        ("PublishOdom.inputs:topicName", odom_topic),
                        ("PublishOdom.inputs:odomFrameId", cfg.frame_odom),
                        ("PublishOdom.inputs:chassisFrameId", cfg.frame_base),
                        ("PublishOdom.inputs:queueSize", 10),
                    ]
                )
                connections.extend(
                    [
                        ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
                        ("ROS2Context.outputs:context", "PublishOdom.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                        ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                        ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                        ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                        ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                    ]
                )

            if cfg.enable_tf:
                # 1) odom → base_link
                nodes.append(("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
                values.extend(
                    [
                        ("PublishOdomTF.inputs:topicName", tf_topic),
                        ("PublishOdomTF.inputs:parentFrameId", cfg.frame_odom),
                        ("PublishOdomTF.inputs:childFrameId", cfg.frame_base),
                        ("PublishOdomTF.inputs:queueSize", 10),
                    ]
                )
                connections.extend(
                    [
                        ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
                        ("ROS2Context.outputs:context", "PublishOdomTF.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                        ("ComputeOdom.outputs:position", "PublishOdomTF.inputs:translation"),
                        ("ComputeOdom.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                    ]
                )

                # 2) Spot articulation link TF (optional; expensive every frame)
                if ENABLE_ROBOT_LINK_TF:
                    nodes.append(("PublishRobotTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
                    values.extend(
                        [
                            ("PublishRobotTF.inputs:topicName", tf_topic),
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

                # 3) base_link → body identity (ROS name ↔ USD articulation root)
                body_name = chassis_prim_path.rstrip("/").split("/")[-1] or "body"
                _add_static_tf(
                    nodes,
                    values,
                    connections,
                    name="PublishBaseAliasTF",
                    topic=tf_static_topic,
                    parent=cfg.frame_base,
                    child=body_name,
                    xyz=(0.0, 0.0, 0.0),
                    quat_xyzw=identity,
                )

                # 4) 与 cam_mgr_ros / Gazebo 一致：
                #    base→cam_0_link（Z 上）→ color/depth_frame → *_optical_frame（Z 前）
                #    optical = rpy(-π/2, 0, -π/2)
                link = FRAME_CAMERA_LINK
                color_opt = FRAME_COLOR_OPTICAL
                depth_opt = FRAME_DEPTH_OPTICAL

                _add_static_tf(
                    nodes,
                    values,
                    connections,
                    name="PublishCamLinkTF",
                    topic=tf_static_topic,
                    parent=cfg.frame_base,
                    child=link,
                    xyz=(cx, cy, cz),
                    quat_xyzw=(lqx, lqy, lqz, lqw),
                )
                _add_static_tf(
                    nodes,
                    values,
                    connections,
                    name="PublishColorFrameTF",
                    topic=tf_static_topic,
                    parent=link,
                    child=FRAME_COLOR,
                    xyz=(0.0, 0.0, 0.0),
                    quat_xyzw=identity,
                )
                _add_static_tf(
                    nodes,
                    values,
                    connections,
                    name="PublishDepthFrameTF",
                    topic=tf_static_topic,
                    parent=link,
                    child=FRAME_DEPTH,
                    xyz=(0.0, 0.0, 0.0),
                    quat_xyzw=identity,
                )
                _add_static_tf(
                    nodes,
                    values,
                    connections,
                    name="PublishColorOpticalTF",
                    topic=tf_static_topic,
                    parent=FRAME_COLOR,
                    child=color_opt,
                    xyz=(0.0, 0.0, 0.0),
                    quat_xyzw=(oqx, oqy, oqz, oqw),
                )
                _add_static_tf(
                    nodes,
                    values,
                    connections,
                    name="PublishDepthOpticalTF",
                    topic=tf_static_topic,
                    parent=FRAME_DEPTH,
                    child=depth_opt,
                    xyz=(0.0, 0.0, 0.0),
                    quat_xyzw=(oqx, oqy, oqz, oqw),
                )

                if imu_xyz_base is not None:
                    iqx, iqy, iqz, iqw = [float(v) for v in (imu_link_quat_xyzw_base or identity)]
                    _add_static_tf(
                        nodes,
                        values,
                        connections,
                        name="PublishImuLinkTF",
                        topic=tf_static_topic,
                        parent=cfg.frame_base,
                        child=getattr(cfg, "frame_imu", None) or FRAME_IMU,
                        xyz=tuple(float(v) for v in imu_xyz_base),
                        quat_xyzw=(iqx, iqy, iqz, iqw),
                    )
                if lidar_xyz_base is not None:
                    qlx, qly, qlz, qlw = [float(v) for v in (lidar_link_quat_xyzw_base or identity)]
                    _add_static_tf(
                        nodes,
                        values,
                        connections,
                        name="PublishLidarLinkTF",
                        topic=tf_static_topic,
                        parent=cfg.frame_base,
                        child=getattr(cfg, "frame_lidar", None) or FRAME_LIDAR,
                        xyz=tuple(float(v) for v in lidar_xyz_base),
                        quat_xyzw=(qlx, qly, qlz, qlw),
                    )

            og.Controller.edit(
                {"graph_path": self._graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: nodes,
                    keys.SET_VALUES: values,
                    keys.CONNECT: connections,
                },
            )

            if cfg.enable_tf and ENABLE_ROBOT_LINK_TF:
                try:
                    og.Controller.set(
                        og.Controller.attribute(f"{self._graph_path}/PublishRobotTF.inputs:targetPrims"),
                        [usdrt.Sdf.Path(SPOT_PRIM_PATH)],
                    )
                    og.Controller.set(
                        og.Controller.attribute(f"{self._graph_path}/PublishRobotTF.inputs:parentPrim"),
                        [usdrt.Sdf.Path(chassis_prim_path)],
                    )
                except Exception as exc:
                    print(f"OdomTfPublisher: set PublishRobotTF prims failed: {exc}")

            self._active = True
            body_note = ""
            if cfg.enable_tf:
                body_note = chassis_prim_path.rstrip("/").split("/")[-1] or "body"
            print(
                f"OdomTfPublisher: odom={cfg.enable_odom} tf={cfg.enable_tf} "
                f"chassis={chassis_prim_path} "
                f"frames {cfg.frame_odom}->{cfg.frame_base}->[{body_note}|{FRAME_CAMERA_LINK}"
                f"->*_optical] topics /{tf_topic} + /{tf_static_topic} "
                f"cam_link_xyz=({cx:.3f},{cy:.3f},{cz:.3f}) "
                f"optical_from_link_xyzw=({oqx:.3f},{oqy:.3f},{oqz:.3f},{oqw:.3f}) | "
                f"光学系+Z=机器狗前方(+X)"
            )
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"OdomTfPublisher.start failed: {exc}")
            self.stop()
            return False

    def stop(self) -> None:
        destroy_omni_graph(self._graph_path)
        self._active = False
        self._cfg = None

    def publish(self, *args, **kwargs) -> None:
        return

    def spin_once(self) -> None:
        return
