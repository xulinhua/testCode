# -*- coding: utf-8 -*-
"""抓取相关数据结构。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Transform6D:
    """平移 + 四元数位姿。"""

    translation: List[float]
    rotation_xyzw: List[float]
    source_frame: str = ""
    target_frame: str = ""
    # box_center: Stage/盒心顶抓；gripper_pose: 外部夹爪 6D 位姿
    pose_role: str = ""

    def is_gripper_pose(self) -> bool:
        role = (self.pose_role or "").strip().lower()
        if role == "gripper_pose":
            return True
        if role == "box_center":
            return False
        return (self.source_frame or "").strip().lower() in ("grasp_target", "gripper")


@dataclass
class GraspResult:
    """一次抓取执行结果。"""

    ok: bool
    message: str = ""
    arm: str = ""
