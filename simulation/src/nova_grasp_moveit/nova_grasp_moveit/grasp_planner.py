# -*- coding: utf-8 -*-
"""抓取位姿规划：臂选择、预抓取/接近/抬升偏移。"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import List, Tuple

from geometry_msgs.msg import Pose, PoseStamped


@dataclass
class GraspPlan:
    """一次抓取的三段末端目标（均在同一 frame 下）。"""

    arm_id: int
    frame_id: str
    pregrasp: Pose
    grasp: Pose
    lift: Pose


def choose_arm_id(target_x: float, split_x: float = 0.53) -> int:
    """与 Isaac ``NovaRobotRuntime.choose_arm`` 一致：x 大的一侧用 J2 (arm_id=1)。"""
    return 1 if float(target_x) > float(split_x) else 0


def _offset_pose_z(pose: Pose, dz: float) -> Pose:
    out = copy.deepcopy(pose)
    out.position.z += float(dz)
    return out


def _top_down_orientation(yaw_offset_deg: float = 90.0) -> Tuple[float, float, float, float]:
    """垂直向下：RPY(0, -90°, yaw)。本臂爪口沿 -X，故 pitch=-90 时爪口朝下。"""
    import math

    roll = 0.0
    pitch = -math.pi / 2.0
    yaw = math.radians(float(yaw_offset_deg))
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return (0.0, -0.70710678118, 0.0, 0.70710678118)
    return (x / n, y / n, z / n, w / n)


def plan_grasp_from_pose(
    grasp_pose: Pose,
    *,
    frame_id: str = "base_link",
    arm_split_x: float = 0.53,
    pregrasp_z_offset: float = 0.18,
    lift_z_offset: float = 0.08,
    grasp_yaw_offset_deg: float = 90.0,
    use_top_down_if_identity: bool = True,
) -> GraspPlan:
    """由单条抓取位姿生成 pregrasp → grasp → lift 序列。

    姿态固定垂直向下（RPY 0,-90°,yaw），默认 yaw=90° 夹短边。
    """
    _ = use_top_down_if_identity  # 兼容旧参数；始终朝下
    grasp = copy.deepcopy(grasp_pose)
    qx, qy, qz, qw = _top_down_orientation(grasp_yaw_offset_deg)
    grasp.orientation.x = qx
    grasp.orientation.y = qy
    grasp.orientation.z = qz
    grasp.orientation.w = qw
    arm_id = choose_arm_id(grasp.position.x, split_x=arm_split_x)
    pre = _offset_pose_z(grasp, pregrasp_z_offset)
    lift = _offset_pose_z(grasp, max(float(lift_z_offset), 0.05))
    return GraspPlan(
        arm_id=arm_id,
        frame_id=frame_id or "base_link",
        pregrasp=pre,
        grasp=grasp,
        lift=lift,
    )


def pose_stamped_from_plan_step(plan: GraspPlan, pose: Pose, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = plan.frame_id
    msg.header.stamp = stamp
    msg.pose = copy.deepcopy(pose)
    return msg


def format_plan(plan: GraspPlan) -> str:
    g = plan.grasp.position
    return (
        f"arm_id={plan.arm_id} frame={plan.frame_id} "
        f"grasp=({g.x:.3f},{g.y:.3f},{g.z:.3f})"
    )
