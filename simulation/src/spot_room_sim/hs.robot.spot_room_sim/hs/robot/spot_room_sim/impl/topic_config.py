# -*- coding: utf-8 -*-
"""话题与流开关配置。"""

from __future__ import annotations

from dataclasses import dataclass, fields

from ..global_variables import (
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_IMAGE_WIDTH,
    FRAME_BASE,
    FRAME_CAMERA,
    FRAME_ODOM,
    TOPIC_CAMERA_INFO,
    TOPIC_CMD_VEL,
    TOPIC_COLOR,
    TOPIC_DEPTH,
    TOPIC_JOINT_STATES,
    TOPIC_ODOM,
    TOPIC_POINTS,
    TOPIC_TF,
    TOPIC_TF_STATIC,
)


@dataclass
class TopicConfig:
    sub_cmd_vel: str = TOPIC_CMD_VEL
    pub_odom: str = TOPIC_ODOM
    pub_tf: str = TOPIC_TF
    pub_tf_static: str = TOPIC_TF_STATIC
    pub_joint_states: str = TOPIC_JOINT_STATES
    pub_color: str = TOPIC_COLOR
    pub_depth: str = TOPIC_DEPTH
    pub_points: str = TOPIC_POINTS
    pub_camera_info: str = TOPIC_CAMERA_INFO

    frame_odom: str = FRAME_ODOM
    frame_base: str = FRAME_BASE
    frame_camera: str = FRAME_CAMERA

    enable_color: bool = True
    enable_depth: bool = True
    enable_points: bool = True
    enable_camera_info: bool = True
    enable_odom: bool = True
    enable_tf: bool = True
    enable_joint_states: bool = True

    image_width: int = DEFAULT_IMAGE_WIDTH
    image_height: int = DEFAULT_IMAGE_HEIGHT


TOPIC_FIELD_SPECS = (
    ("sub_cmd_vel", "cmd_vel"),
    ("pub_odom", "odom"),
    ("pub_tf", "tf"),
    ("pub_joint_states", "joint_states"),
    ("pub_color", "color"),
    ("pub_depth", "depth"),
    ("pub_points", "points"),
    ("pub_camera_info", "camera_info"),
    ("frame_odom", "frame odom"),
    ("frame_base", "frame base"),
    ("frame_camera", "depth optical frame"),
)

STREAM_TOGGLE_SPECS = (
    ("enable_color", "color"),
    ("enable_depth", "depth"),
    ("enable_points", "points"),
    ("enable_camera_info", "camera_info"),
    ("enable_odom", "odom"),
    ("enable_tf", "tf"),
    ("enable_joint_states", "joint_states"),
)


def topic_config_from_ui(topic_fields: dict, stream_toggles: dict, width: int, height: int) -> TopicConfig:
    cfg = TopicConfig()
    for f in fields(cfg):
        if f.name in topic_fields:
            widget = topic_fields[f.name]
            try:
                setattr(cfg, f.name, widget.model.get_value_as_string().strip())
            except Exception:
                pass
        if f.name in stream_toggles:
            widget = stream_toggles[f.name]
            try:
                setattr(cfg, f.name, bool(widget.model.get_value_as_bool()))
            except Exception:
                pass
    cfg.image_width = int(width)
    cfg.image_height = int(height)
    return cfg
