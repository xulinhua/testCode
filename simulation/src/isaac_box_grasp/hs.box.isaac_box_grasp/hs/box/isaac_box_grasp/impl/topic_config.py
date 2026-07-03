# -*- coding: utf-8 -*-
"""ROS 话题名与流开关（UI 可读写的默认值）。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from ..defaults import DEFAULT_SUB_CMD_VEL


@dataclass
class TopicConfig:
    pub_color: str = "/box_grasp/camera/color/image_raw"
    pub_depth: str = "/box_grasp/camera/depth/image_raw"
    pub_points: str = "/box_grasp/camera/depth/points"
    pub_camera_info: str = "/box_grasp/camera/color/camera_info"
    sub_grasp_pose: str = "/box_grasp/pose_world"
    sub_cmd_vel: str = DEFAULT_SUB_CMD_VEL
    pub_grasp_result: str = "/box_grasp/grasp/result"
    tf_camera_frame: str = "camera_optical_frame"
    tf_world_frame: str = "world"

    enable_color: bool = True
    enable_depth: bool = True
    enable_points: bool = True
    enable_camera_info: bool = True
    enable_tf: bool = True


TOPIC_FIELD_SPECS = (
    ("pub_color", "Color image topic"),
    ("pub_depth", "Depth image topic"),
    ("pub_points", "PointCloud2 topic"),
    ("pub_camera_info", "CameraInfo topic"),
    ("sub_grasp_pose", "Grasp PoseStamped (world) subscribe"),
    ("sub_cmd_vel", "Mobile base cmd_vel (Twist) subscribe"),
    ("pub_grasp_result", "Grasp result topic"),
    ("tf_camera_frame", "Camera optical frame id"),
    ("tf_world_frame", "World frame id"),
)

STREAM_TOGGLE_SPECS = (
    ("enable_color", "Publish color"),
    ("enable_depth", "Publish depth"),
    ("enable_points", "Publish PointCloud2"),
    ("enable_camera_info", "Publish CameraInfo"),
    ("enable_tf", "Publish TF world→camera"),
)


def topic_config_from_ui(fields: Dict[str, str], toggles: Dict[str, bool]) -> TopicConfig:
    cfg = TopicConfig()
    for key, _label in TOPIC_FIELD_SPECS:
        val = fields.get(key)
        if val:
            setattr(cfg, key, val.strip())
    for key, _label in STREAM_TOGGLE_SPECS:
        if key in toggles:
            setattr(cfg, key, bool(toggles[key]))
    return cfg
