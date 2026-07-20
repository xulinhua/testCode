# -*- coding: utf-8 -*-
"""ROS 话题与流开关配置（dataclass + UI 字段映射）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from ..global_variables import CAMERA_DEFS, DEFAULT_CAMERA_HEIGHT, DEFAULT_CAMERA_WIDTH


@dataclass
class CameraStreamConfig:
    """单路相机的分辨率、话题名与发布开关（cam0=Gemini335，cam1/2=RSD455）。"""

    key: str = "cam0"
    label: str = ""
    camera_prim_path: str = ""  # Load 后由 SceneLoader 填入
    frame_id: str = "gemini335"
    width: int = DEFAULT_CAMERA_WIDTH
    height: int = DEFAULT_CAMERA_HEIGHT
    pub_color: str = "/camera0_rgb_sensor/image_raw"
    pub_depth: str = "/camera0_depth_sensor/depth/image_raw"
    pub_points: str = "/camera0_depth_sensor/depth/points"
    pub_camera_info: str = "/camera0_rgb_sensor/camera_info"
    enable_color: bool = True
    enable_depth: bool = True
    enable_points: bool = False
    enable_camera_info: bool = True


@dataclass
class RobotRosConfig:
    """机器人 joint_states、TF 树与抓取盒 TF 的 ROS 配置。"""

    pub_joint_states: str = "/joint_states"
    sub_joint_command: str = "/joint_command"
    pub_tf: str = "/tf"
    pub_tf_static: str = "/tf_static"
    tf_world_frame: str = "world"
    enable_joint_states: bool = True
    enable_joint_command: bool = True
    enable_robot_tf: bool = True
    enable_box_tf: bool = True
    tf_box_frame: str = "grasp_box"


@dataclass
class SessionTopicConfig:
    """一次 Play 会话使用的完整 ROS 话题配置。"""

    cameras: List[CameraStreamConfig] = field(default_factory=list)
    robot: RobotRosConfig = field(default_factory=RobotRosConfig)

    @staticmethod
    def default() -> "SessionTopicConfig":
        """从 ``CAMERA_DEFS`` 构造 cam0/cam1/cam2 默认配置。"""
        cameras = []
        for spec in CAMERA_DEFS:
            cameras.append(
                CameraStreamConfig(
                    key=spec["key"],
                    label=spec["label"],
                    frame_id=spec["frame_id"],
                    pub_color=spec["pub_color"],
                    pub_depth=spec["pub_depth"],
                    pub_points=spec["pub_points"],
                    pub_camera_info=spec["pub_camera_info"],
                )
            )
        return SessionTopicConfig(cameras=cameras)


# UI 字段名 ↔ dataclass 属性（供 ui_builder 遍历）
CAMERA_TOPIC_FIELD_SPECS = (
    ("pub_color", "RGB"),
    ("pub_depth", "Depth"),
    ("pub_points", "Points"),
    ("pub_camera_info", "CamInfo"),
    ("frame_id", "frame_id"),
)

CAMERA_STREAM_TOGGLE_SPECS = (
    ("enable_color", "RGB"),
    ("enable_depth", "Depth"),
    ("enable_points", "Points"),
    ("enable_camera_info", "CamInfo"),
)

CAMERA_RESOLUTION_KEYS = (
    ("width", "W (px)"),
    ("height", "H (px)"),
)

ROBOT_TOPIC_FIELD_SPECS = (
    ("pub_joint_states", "joint_states"),
    ("sub_joint_command", "joint_command"),
    ("pub_tf", "TF"),
    ("pub_tf_static", "TF static"),
    ("tf_world_frame", "world"),
    ("tf_box_frame", "box"),
)

ROBOT_STREAM_TOGGLE_SPECS = (
    ("enable_joint_states", "joint_states"),
    ("enable_joint_command", "joint_command"),
    ("enable_robot_tf", "robot TF"),
    ("enable_box_tf", "box TF"),
)


def _parse_int(val: str, default: int) -> int:
    """安全解析 UI 整数字符串。"""
    try:
        return int(str(val).strip())
    except (TypeError, ValueError):
        return default


def topic_config_from_ui(
    camera_fields: Dict[str, Dict[str, str]],
    camera_toggles: Dict[str, Dict[str, bool]],
    camera_resolutions: Dict[str, Dict[str, int]],
    robot_fields: Dict[str, str],
    robot_toggles: Dict[str, bool],
    camera_prim_paths: Dict[str, str],
) -> SessionTopicConfig:
    """将 UI 控件当前值合并为 ``SessionTopicConfig``。

    Args:
        camera_fields: 每路相机的 topic/frame_id 字符串字段。
        camera_toggles: 每路相机的 enable_* 勾选状态。
        camera_resolutions: 每路相机的 width/height。
        robot_fields: 机器人 ROS 话题字符串字段。
        robot_toggles: 机器人 enable_* 勾选状态。
        camera_prim_paths: Load 后解析的相机 prim 路径。

    Returns:
        可直接传给 ``MultiCameraPublisher.start`` / ``RobotRosPublisher.start`` 的配置。
    """
    cfg = SessionTopicConfig.default()
    for cam in cfg.cameras:
        key = cam.key
        fields = camera_fields.get(key, {})
        toggles = camera_toggles.get(key, {})
        res = camera_resolutions.get(key, {})
        cam.camera_prim_path = camera_prim_paths.get(key, cam.camera_prim_path)
        for fk, _ in CAMERA_TOPIC_FIELD_SPECS:
            if fields.get(fk):
                setattr(cam, fk, fields[fk].strip())
        for tk, _ in CAMERA_STREAM_TOGGLE_SPECS:
            if tk in toggles:
                setattr(cam, tk, bool(toggles[tk]))
        if "width" in res:
            cam.width = _parse_int(str(res["width"]), cam.width)
        if "height" in res:
            cam.height = _parse_int(str(res["height"]), cam.height)
    robot = cfg.robot
    for fk, _ in ROBOT_TOPIC_FIELD_SPECS:
        if robot_fields.get(fk):
            setattr(robot, fk, robot_fields[fk].strip())
    for tk, _ in ROBOT_STREAM_TOGGLE_SPECS:
        if tk in robot_toggles:
            setattr(robot, tk, bool(robot_toggles[tk]))
    return cfg
