# -*- coding: utf-8 -*-
"""单臂 URDF 正/逆运动学（阻尼最小二乘，供 Isaac 抓取闭环）。"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from ...global_variables import ARM1_JOINTS, ARM2_JOINTS


def _parse_floats(text: str, n: int) -> Tuple[float, ...]:
    parts = [float(x) for x in text.split()]
    if len(parts) < n:
        raise ValueError(f"expected {n} floats, got {text!r}")
    return tuple(parts[:n])


def _rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def _axis_angle_matrix(axis: Sequence[float], theta: float) -> np.ndarray:
    ax = np.array(axis, dtype=np.float64)
    n = float(np.linalg.norm(ax))
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    ax /= n
    x, y, z = ax
    c, s = math.cos(theta), math.sin(theta)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=np.float64,
    )


def _make_transform(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = _rpy_matrix(rpy)
    t[:3, 3] = np.array(xyz, dtype=np.float64)
    return t


@dataclass
class RevoluteJointSpec:
    name: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


@dataclass
class ArmChainSpec:
    arm: str
    joint_names: Tuple[str, ...]
    joints: Tuple[RevoluteJointSpec, ...]
    ee_offset: np.ndarray


def _origin_of(elem) -> Tuple[np.ndarray, np.ndarray]:
    origin = elem.find("origin")
    if origin is None:
        return np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)
    xyz = origin.get("xyz", "0 0 0")
    rpy = origin.get("rpy", "0 0 0")
    return np.array(_parse_floats(xyz, 3), dtype=np.float64), np.array(_parse_floats(rpy, 3), dtype=np.float64)


def _load_joint_map(urdf_path: str) -> dict:
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if not name:
            continue
        joints[name] = joint
    return joints


def _build_chain_to_link(
    joints: dict, root_link: str, tip_link: str, joint_name_order: Sequence[str]
) -> Tuple[RevoluteJointSpec, ...]:
    specs: List[RevoluteJointSpec] = []
    link = root_link
    for jname in joint_name_order:
        joint = joints.get(jname)
        if joint is None:
            raise KeyError(f"joint missing in URDF: {jname}")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        if parent != link:
            raise ValueError(f"chain break at {jname}: parent {parent} != {link}")
        xyz, rpy = _origin_of(joint)
        axis_text = joint.find("axis").get("xyz", "0 0 1")
        axis = np.array(_parse_floats(axis_text, 3), dtype=np.float64)
        lower, upper = -math.pi, math.pi
        limit = joint.find("limit")
        if limit is not None:
            lower = float(limit.get("lower", str(-math.pi)))
            upper = float(limit.get("upper", str(math.pi)))
        specs.append(
            RevoluteJointSpec(
                name=jname,
                origin=_make_transform(xyz, rpy),
                axis=axis,
                lower=lower,
                upper=upper,
            )
        )
        link = child
    if link != tip_link:
        raise ValueError(f"chain ends at {link}, expected {tip_link}")
    return tuple(specs)


def _gripper_offset_in_ee(joints: dict, ee_link: str, finger_joints: Sequence[str]) -> np.ndarray:
    """夹爪 TCP：两指 prismatic 在张开中位时，指尖中点在腕部坐标系下的位置。"""
    points = []
    for jname in finger_joints:
        joint = joints.get(jname)
        if joint is None:
            continue
        if joint.find("parent").get("link") != ee_link:
            continue
        xyz, rpy = _origin_of(joint)
        t_joint = _make_transform(xyz, rpy)
        # prismatic 张开中位（与 GRIPPER*_OPEN 一致）
        q_open = 0.0
        limit = joint.find("limit")
        if limit is not None:
            lo = float(limit.get("lower", "0"))
            hi = float(limit.get("upper", "0"))
            q_open = 0.5 * (lo + hi)
        axis_text = joint.find("axis").get("xyz", "0 0 1")
        axis = np.array(_parse_floats(axis_text, 3), dtype=np.float64)
        t_slide = np.eye(4, dtype=np.float64)
        t_slide[:3, :3] = _axis_angle_matrix(axis, q_open)
        t_child = t_joint @ t_slide
        points.append(t_child[:3, 3])
    if not points:
        return np.array([0.0, 0.0, 0.115], dtype=np.float64)
    return np.mean(np.stack(points, axis=0), axis=0)


def load_arm_chains(urdf_path: str) -> dict:
    """从 URDF 构建 arm1 / arm2 运动链（base_link → J*_6 + 夹爪偏移）。"""
    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(urdf_path)
    joints = _load_joint_map(urdf_path)
    chains = {}
    for arm, joint_names, ee_link, fingers in (
        ("arm1", ARM1_JOINTS, "J1_6", ("J1_7_joint", "J1_8_joint")),
        ("arm2", ARM2_JOINTS, "J2_6", ("J2_7_joint", "J2_8_joint")),
    ):
        revolute = _build_chain_to_link(joints, "base_link", ee_link, joint_names)
        offset = _gripper_offset_in_ee(joints, ee_link, fingers)
        t_off = np.eye(4, dtype=np.float64)
        t_off[:3, 3] = offset
        chains[arm] = ArmChainSpec(
            arm=arm,
            joint_names=tuple(joint_names),
            joints=revolute,
            ee_offset=t_off,
        )
    return chains


class ArmKinematics:
    """单臂 FK / 数值 IK（腕部坐标系为链末端，再加工夹爪 TCP 偏移）。"""

    def __init__(self, chain: ArmChainSpec):
        self.chain = chain

    def fk_tcp(self, q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (tcp_position, rotation_matrix) ，相对 base_link。"""
        t = np.eye(4, dtype=np.float64)
        for qi, spec in zip(q, self.chain.joints):
            t = t @ spec.origin
            rot = np.eye(4, dtype=np.float64)
            rot[:3, :3] = _axis_angle_matrix(spec.axis, float(qi))
            t = t @ rot
        t = t @ self.chain.ee_offset
        return t[:3, 3].copy(), t[:3, :3].copy()

    def fk_wrist(self, q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
        """腕部 J*_6 原点，相对 base_link（不含夹爪 TCP 偏移）。"""
        t = np.eye(4, dtype=np.float64)
        for qi, spec in zip(q, self.chain.joints):
            t = t @ spec.origin
            rot = np.eye(4, dtype=np.float64)
            rot[:3, :3] = _axis_angle_matrix(spec.axis, float(qi))
            t = t @ rot
        return t[:3, 3].copy(), t[:3, :3].copy()

    def fk_jacobian_position(self, q: Sequence[float], eps: float = 1e-5) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64)
        p0, _ = self.fk_tcp(q)
        jac = np.zeros((3, len(q)), dtype=np.float64)
        for i in range(len(q)):
            dq = q.copy()
            dq[i] += eps
            p1, _ = self.fk_tcp(dq)
            jac[:, i] = (p1 - p0) / eps
        return jac

    def ik_position(
        self,
        target_pos: Sequence[float],
        seed: Sequence[float],
        *,
        max_iter: int = 80,
        pos_tol: float = 0.008,
        damping: float = 0.05,
    ) -> Tuple[Optional[np.ndarray], float]:
        """仅位置 IK，返回 (关节角, 末端位置误差米)。"""
        q = np.array(seed, dtype=np.float64)
        target = np.array(target_pos, dtype=np.float64)
        best_q = q.copy()
        best_err = float("inf")

        for _ in range(max_iter):
            pos, _ = self.fk_tcp(q)
            err = target - pos
            dist = float(np.linalg.norm(err))
            if dist < best_err:
                best_err = dist
                best_q = q.copy()
            if dist < pos_tol:
                return best_q, dist

            j = self.fk_jacobian_position(q)
            jj_t = j @ j.T + (damping ** 2) * np.eye(3)
            dq = j.T @ np.linalg.solve(jj_t, err)
            dq = np.clip(dq, -0.12, 0.12)
            q = q + dq
            for i, spec in enumerate(self.chain.joints):
                q[i] = float(np.clip(q[i], spec.lower, spec.upper))

        if best_err < pos_tol * 3.0:
            return best_q, best_err
        return None, best_err

    def ik_pose(
        self,
        target_pos: Sequence[float],
        target_rot: np.ndarray,
        seed: Sequence[float],
        *,
        rot_weight: float = 0.35,
        max_iter: int = 100,
        pos_tol: float = 0.01,
        damping: float = 0.06,
    ) -> Tuple[Optional[np.ndarray], float, float]:
        """位置 + 姿态（旋转矩阵）IK。"""
        q = np.array(seed, dtype=np.float64)
        target_p = np.array(target_pos, dtype=np.float64)
        target_r = np.array(target_rot, dtype=np.float64)
        best_q = q.copy()
        best_pos_err = float("inf")

        for _ in range(max_iter):
            pos, rot = self.fk_tcp(q)
            pos_err = target_p - pos
            rot_err = _orientation_error(target_r, rot)
            pos_dist = float(np.linalg.norm(pos_err))
            rot_dist = float(np.linalg.norm(rot_err))
            if pos_dist < best_pos_err:
                best_pos_err = pos_dist
                best_q = q.copy()
            if pos_dist < pos_tol and rot_dist < 0.15:
                return best_q, pos_dist, rot_dist

            j_pos = self.fk_jacobian_position(q)
            j_rot = np.zeros((3, len(q)), dtype=np.float64)
            eps = 1e-5
            for i in range(len(q)):
                dq = q.copy()
                dq[i] += eps
                _, r1 = self.fk_tcp(dq)
                j_rot[:, i] = _orientation_error(r1, rot) / eps

            j = np.vstack([j_pos, rot_weight * j_rot])
            err = np.concatenate([pos_err, rot_weight * rot_err])
            jj_t = j @ j.T + (damping ** 2) * np.eye(6)
            dq = j.T @ np.linalg.solve(jj_t, err)
            dq = np.clip(dq, -0.1, 0.1)
            q = q + dq
            for i, spec in enumerate(self.chain.joints):
                q[i] = float(np.clip(q[i], spec.lower, spec.upper))

        if best_pos_err < pos_tol * 3.0:
            return best_q, best_pos_err, float("inf")
        return None, best_pos_err, float("inf")


def _orientation_error(desired: np.ndarray, current: np.ndarray) -> np.ndarray:
    """旋转误差向量（近似李代数）。"""
    r_err = desired @ current.T
    return np.array(
        [
            r_err[2, 1] - r_err[1, 2],
            r_err[0, 2] - r_err[2, 0],
            r_err[1, 0] - r_err[0, 1],
        ],
        dtype=np.float64,
    ) * 0.5


def top_down_rotation() -> np.ndarray:
    """夹爪朝下（TCP -Z 指向 world -Z）的常用腕部姿态近似。"""
    return np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64)


def quat_xyzw_to_matrix(quat_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = [float(v) for v in quat_xyzw]
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
