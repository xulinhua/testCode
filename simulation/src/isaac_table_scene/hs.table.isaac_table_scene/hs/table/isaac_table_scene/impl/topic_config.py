# -*- coding: utf-8 -*-
"""ROS 话题名与流开关。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass
class TopicConfig:
    pub_color: str = "/camera/color/image_raw"
    pub_depth: str = "/camera/depth/image_raw"
    pub_points: str = "/camera/depth/points"
    pub_camera_info: str = "/camera/color/camera_info"
    tf_camera_link_frame: str = "camera_link"
    tf_camera_optical_frame: str = "camera_optical_frame"
    tf_object_frame: str = "cuboid_link"
    tf_world_frame: str = "map"

    enable_color: bool = True
    enable_depth: bool = True
    enable_points: bool = True
    enable_camera_info: bool = True
    enable_camera_tf: bool = True
    enable_object_tf: bool = True


TOPIC_FIELD_SPECS = (
    ("pub_color", "Color image topic"),
    ("pub_depth", "Depth image topic"),
    ("pub_points", "PointCloud2 topic"),
    ("pub_camera_info", "CameraInfo topic"),
    ("tf_camera_link_frame", "Camera link frame (color/depth/CameraInfo)"),
    ("tf_camera_optical_frame", "Camera optical frame (PointCloud2)"),
    ("tf_object_frame", "Cuboid link frame id"),
    ("tf_world_frame", "World frame id (Fixed Frame)"),
)

STREAM_TOGGLE_SPECS = (
    ("enable_color", "Publish color"),
    ("enable_depth", "Publish depth"),
    ("enable_points", "Publish PointCloud2"),
    ("enable_camera_info", "Publish CameraInfo"),
    ("enable_camera_tf", "Publish TF (camera + cuboid)"),
    ("enable_object_tf", "Publish TF cuboid"),
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
