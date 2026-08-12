# -*- coding: utf-8 -*-
"""位姿与 look-at 工具。"""

from __future__ import annotations

import math
from typing import Tuple

from pxr import Gf, UsdGeom


def rpy_deg_to_quatf(roll: float, pitch: float, yaw: float) -> Gf.Quatf:
    """ZYX 外旋（度）→ GfQuatf（UsdGeom.XformOp orient 默认精度）。"""
    r, p, y = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yv = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return Gf.Quatf(float(w), float(x), float(yv), float(z))


def set_translate_rotate(
    stage,
    prim_path: str,
    xyz: Tuple[float, float, float],
    rpy_deg: Tuple[float, float, float],
) -> None:
    """设置平移 + 欧拉角。优先 RotateXYZ，避免 Quatd/Quatf 精度不匹配。"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2])))
    # RotateXYZ 用 Vec3f，Isaac/USD 上比 orient(Quat) 更稳
    xform.AddRotateXYZOp().Set(
        Gf.Vec3f(float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2]))
    )


def look_at_matrix(
    eye: Tuple[float, float, float],
    target: Tuple[float, float, float],
    up: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    roll_deg: float = 0.0,
) -> Gf.Matrix4d:
    """Optical-frame world matrix: USD Camera -Z toward target, optional roll."""
    eye_v = Gf.Vec3d(*eye)
    tgt_v = Gf.Vec3d(*target)
    up_v = Gf.Vec3d(*up)
    forward = (tgt_v - eye_v).GetNormalized()
    z_axis = -forward
    x_axis = Gf.Cross(up_v, z_axis)
    if x_axis.GetLength() < 1e-6:
        up_v = Gf.Vec3d(0.0, 1.0, 0.0)
        x_axis = Gf.Cross(up_v, z_axis)
    x_axis = x_axis.GetNormalized()
    y_axis = Gf.Cross(z_axis, x_axis).GetNormalized()

    # Roll about optical forward (-Z in camera / +looking direction = -z_axis)
    if abs(float(roll_deg)) > 1e-6:
        ang = math.radians(float(roll_deg))
        c, s = math.cos(ang), math.sin(ang)
        x_rot = Gf.Vec3d(
            c * x_axis[0] + s * y_axis[0],
            c * x_axis[1] + s * y_axis[1],
            c * x_axis[2] + s * y_axis[2],
        )
        y_rot = Gf.Vec3d(
            -s * x_axis[0] + c * y_axis[0],
            -s * x_axis[1] + c * y_axis[1],
            -s * x_axis[2] + c * y_axis[2],
        )
        x_axis, y_axis = x_rot.GetNormalized(), y_rot.GetNormalized()

    m = Gf.Matrix4d(1.0)
    m.SetRow(0, Gf.Vec4d(x_axis[0], x_axis[1], x_axis[2], 0.0))
    m.SetRow(1, Gf.Vec4d(y_axis[0], y_axis[1], y_axis[2], 0.0))
    m.SetRow(2, Gf.Vec4d(z_axis[0], z_axis[1], z_axis[2], 0.0))
    m.SetRow(3, Gf.Vec4d(eye_v[0], eye_v[1], eye_v[2], 1.0))
    return m


def set_world_matrix(stage, prim_path: str, matrix: Gf.Matrix4d) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    op = xform.AddTransformOp()
    op.Set(matrix)
