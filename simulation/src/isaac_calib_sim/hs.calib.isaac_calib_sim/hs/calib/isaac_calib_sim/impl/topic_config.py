# -*- coding: utf-8 -*-
"""ROS 话题配置（仅 RGB + CameraInfo，供标定 GUI 订阅）。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass
class TopicConfig:
    pub_color: str = "/camera/image_raw"
    pub_camera_info: str = "/camera/camera_info"
    tf_camera_link_frame: str = "camera_link"
    tf_camera_optical_frame: str = "camera_optical_frame"
    enable_color: bool = True
    enable_camera_info: bool = True


TOPIC_FIELD_SPECS = (
    ("pub_color", "Color image topic"),
    ("pub_camera_info", "CameraInfo topic"),
    ("tf_camera_link_frame", "Camera link frame"),
    ("tf_camera_optical_frame", "Camera optical frame"),
)


def topic_config_from_ui(fields: Dict[str, str], enable_color: bool, enable_info: bool) -> TopicConfig:
    cfg = TopicConfig()
    for key, _ in TOPIC_FIELD_SPECS:
        val = fields.get(key)
        if val:
            setattr(cfg, key, val.strip())
    cfg.enable_color = bool(enable_color)
    cfg.enable_camera_info = bool(enable_info)
    return cfg
