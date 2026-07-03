# -*- coding: utf-8 -*-
"""USD Prim 6D 位姿读写（平移 + XYZ 欧拉角，度）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from pxr import Gf, Usd, UsdGeom


@dataclass
class Pose6D:
    """世界或本地坐标系下的 6D 位姿（平移 + XYZ 欧拉角）。"""

    translation: Tuple[float, float, float]
    rotation_deg: Tuple[float, float, float]  # roll, pitch, yaw (XYZ degrees)


def _get_or_add_xform_op(xformable: UsdGeom.Xformable, op_type):
    """获取已有 xform op 或按类型新建（避免重复 Add 导致 op 堆叠）。"""
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == op_type and not op.IsInverseOp():
            return op
    if op_type == UsdGeom.XformOp.TypeTranslate:
        return xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    if op_type == UsdGeom.XformOp.TypeRotateXYZ:
        return xformable.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat)
    if op_type == UsdGeom.XformOp.TypeScale:
        return xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
    return None


def set_pose(prim_path: str, pose: Pose6D, stage=None, use_world: bool = False) -> bool:
    """写入 Prim 位姿。始终通过 USD xform 属性，避免 Play 时 XFormPrim 触发 PhysX 失效错误。"""
    _ = use_world  # 保留参数以兼容旧调用
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return False

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return False

    xformable = UsdGeom.Xformable(prim)
    translate_op = _get_or_add_xform_op(xformable, UsdGeom.XformOp.TypeTranslate)
    rotate_op = _get_or_add_xform_op(xformable, UsdGeom.XformOp.TypeRotateXYZ)
    if translate_op is not None:
        translate_op.Set(Gf.Vec3d(*[float(v) for v in pose.translation]))
    if rotate_op is not None:
        rotate_op.Set(Gf.Vec3f(*[float(v) for v in pose.rotation_deg]))
    return True


def read_pose_local(prim_path: str, stage=None) -> Optional[Pose6D]:
    """读取 Prim 本地 Translate + RotateXYZ（与 Property 面板一致）。"""
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return None

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None

    xformable = UsdGeom.Xformable(prim)
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and not op.IsInverseOp():
            t = op.Get()
            translation = (float(t[0]), float(t[1]), float(t[2]))
        elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ and not op.IsInverseOp():
            r = op.Get()
            rotation = (float(r[0]), float(r[1]), float(r[2]))
    return Pose6D(translation=translation, rotation_deg=rotation)


def read_pose(prim_path: str, stage=None) -> Optional[Pose6D]:
    """读取 Prim 位姿：优先本地 xform；无显式 op 时回退世界矩阵分解。"""
    local = read_pose_local(prim_path, stage)
    if local is not None:
        return local

    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return None

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None

    xformable = UsdGeom.Xformable(prim)
    world_xform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = world_xform.ExtractTranslation()
    rotation = world_xform.ExtractRotation()
    euler = rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    return Pose6D(
        translation=(float(translation[0]), float(translation[1]), float(translation[2])),
        rotation_deg=(float(euler[0]), float(euler[1]), float(euler[2])),
    )


def pose_to_tf_lists(pose: Pose6D) -> Tuple[List[float], List[float]]:
    """返回 (translation xyz, quaternion xyzw) 供 ROS TF 使用。"""
    rot = Gf.Rotation(Gf.Vec3d.XAxis(), pose.rotation_deg[0])
    rot *= Gf.Rotation(Gf.Vec3d.YAxis(), pose.rotation_deg[1])
    rot *= Gf.Rotation(Gf.Vec3d.ZAxis(), pose.rotation_deg[2])
    quat = rot.GetQuat()
    imag = quat.GetImaginary()
    return (
        [float(v) for v in pose.translation],
        [float(imag[0]), float(imag[1]), float(imag[2]), float(quat.GetReal())],
    )


def _rotation_from_rpy_deg(rpy_deg: Tuple[float, float, float]) -> Gf.Rotation:
    rot = Gf.Rotation(Gf.Vec3d.XAxis(), float(rpy_deg[0]))
    rot *= Gf.Rotation(Gf.Vec3d.YAxis(), float(rpy_deg[1]))
    rot *= Gf.Rotation(Gf.Vec3d.ZAxis(), float(rpy_deg[2]))
    return rot


def _normalize_vec3(v: Gf.Vec3d) -> Gf.Vec3d:
    out = Gf.Vec3d(v)
    length = out.GetLength()
    if length < 1e-12:
        return Gf.Vec3d(0.0, 0.0, 1.0)
    out /= length
    return out


def camera_link_rpy_deg_look_at(
    eye: Tuple[float, float, float],
    target: Tuple[float, float, float],
    optical_rpy_deg: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """
    计算 camera_link 的 RotateXYZ（度），使光学子系视线对准 target。
    考虑 camera_optical_frame 固定旋转与 USD Camera 沿 -Z 成像；保持水平（world Z 为上）。
    """
    view = _normalize_vec3(
        Gf.Vec3d(
            float(target[0]) - float(eye[0]),
            float(target[1]) - float(eye[1]),
            float(target[2]) - float(eye[2]),
        )
    )

    r_opt = _rotation_from_rpy_deg(optical_rpy_deg)
    view_in_link = _normalize_vec3(r_opt.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0)))
    # USD Camera 成像 +Y 在 link 下的方向（与 optical_frame 固定旋转一致）
    img_up_in_link = _normalize_vec3(r_opt.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0)))

    world_up = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(Gf.Dot(view, world_up)) > 0.99:
        world_up = Gf.Vec3d(0.0, 1.0, 0.0)

    right_w = _normalize_vec3(Gf.Cross(view, world_up))
    up_w = _normalize_vec3(Gf.Cross(right_w, view))

    # Gf.Rotation 不能从 Matrix3d 构造：先对准视线，再绕视线扭转使成像 +Y 对齐地平线
    rot_link = Gf.Rotation()
    rot_link.SetRotateInto(view_in_link, view)

    up_current = rot_link.TransformDir(img_up_in_link)
    sin_a = Gf.Dot(Gf.Cross(up_current, up_w), view)
    cos_a = Gf.Dot(up_current, up_w)
    angle_deg = math.degrees(math.atan2(float(sin_a), float(cos_a)))
    if abs(angle_deg) > 1e-6:
        twist = Gf.Rotation(view, angle_deg)
        rot_link = twist * rot_link

    euler = rot_link.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    return (float(euler[0]), float(euler[1]), float(euler[2]))


def camera_pose_look_at_cuboid(
    eye: Tuple[float, float, float],
    cuboid_center: Tuple[float, float, float],
    optical_rpy_deg: Tuple[float, float, float],
) -> Pose6D:
    """构造相机 link 位姿：光心 ``eye`` 朝向 ``cuboid_center``（预留标定/外参工具）。"""
    rpy = camera_link_rpy_deg_look_at(eye, cuboid_center, optical_rpy_deg)
    return Pose6D(eye, rpy)


def parse_pose_from_fields(
    tx: str, ty: str, tz: str, roll: str, pitch: str, yaw: str, defaults: Pose6D
) -> Pose6D:
    """从 UI 字符串字段解析 6D 位姿，非法值回退到 ``defaults`` 对应分量。"""
    def _f(val: str, default: float) -> float:
        try:
            return float(str(val).strip())
        except (TypeError, ValueError):
            return default

    return Pose6D(
        translation=(
            _f(tx, defaults.translation[0]),
            _f(ty, defaults.translation[1]),
            _f(tz, defaults.translation[2]),
        ),
        rotation_deg=(
            _f(roll, defaults.rotation_deg[0]),
            _f(pitch, defaults.rotation_deg[1]),
            _f(yaw, defaults.rotation_deg[2]),
        ),
    )
