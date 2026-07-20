# -*- coding: utf-8 -*-
"""USD Prim 6D 位姿读写（平移 + XYZ 欧拉角，度）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

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


def pose6d_to_quat_xyzw(pose: Pose6D) -> Tuple[float, float, float, float]:
    """6D 欧拉角位姿 → 四元数 xyzw。"""
    _, quat = pose_to_tf_lists(pose)
    return (quat[0], quat[1], quat[2], quat[3])


def xyzrpy_tuple_to_pose6d(xyzrpy: Sequence[float]) -> Pose6D:
    """(x,y,z,roll,pitch,yaw) → Pose6D（米 + 度）。"""
    vals = [float(x) for x in xyzrpy[:6]]
    while len(vals) < 6:
        vals.append(0.0)
    return Pose6D(
        translation=(vals[0], vals[1], vals[2]),
        rotation_deg=(vals[3], vals[4], vals[5]),
    )


def format_xyzrpy(pose: Pose6D) -> str:
    """格式化为 ``x,y,z,roll,pitch,yaw`` 字符串（m, deg）。"""
    t = pose.translation
    r = pose.rotation_deg
    return (
        f"{t[0]:.3f},{t[1]:.3f},{t[2]:.3f},"
        f"{r[0]:.1f},{r[1]:.1f},{r[2]:.1f}"
    )


def parse_xyzrpy_string(text: str, defaults: Pose6D) -> Pose6D:
    """解析 ``x,y,z,roll,pitch,yaw``（逗号/空格/分号分隔，m + deg）。"""
    import re

    raw = str(text or "").strip()
    if not raw:
        return defaults
    parts = [p for p in re.split(r"[,;\s]+", raw) if p]
    vals: list[float] = []
    default_vals = (
        defaults.translation[0],
        defaults.translation[1],
        defaults.translation[2],
        defaults.rotation_deg[0],
        defaults.rotation_deg[1],
        defaults.rotation_deg[2],
    )
    for i in range(6):
        if i < len(parts):
            try:
                vals.append(float(parts[i]))
            except (TypeError, ValueError):
                vals.append(float(default_vals[i]))
        else:
            vals.append(float(default_vals[i]))
    return Pose6D(
        translation=(vals[0], vals[1], vals[2]),
        rotation_deg=(vals[3], vals[4], vals[5]),
    )


def base_link_pose_to_world(
    pose: Pose6D,
    *,
    base_link_path: str,
    world_path: str = "/World",
    stage=None,
) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
    """base_link 下 Pose6D → world 下 (translation, quat_xyzw)。"""
    quat = pose6d_to_quat_xyzw(pose)
    return transform_pose_between_frames(
        pose.translation,
        quat,
        base_link_path,
        world_path,
        stage=stage,
    )


def is_plausible_gripper_pose(pose: Pose6D) -> bool:
    """复位 xyzrpy 是否在合理工作空间内（base_link，米 + 度）。"""
    x, y, z = pose.translation
    if not (-0.25 <= x <= 1.55 and -0.95 <= y <= 0.95 and 0.04 <= z <= 1.85):
        return False
    for ang in pose.rotation_deg:
        if abs(ang) > 360.0:
            return False
    return True


def looks_like_joint_degrees_text(text: str) -> bool:
    """启发式：字段内容是否像旧的关节角（度）而非 xyzrpy。"""
    import re

    parts = [p for p in re.split(r"[,;\s]+", str(text or "").strip()) if p]
    if len(parts) != 6:
        return False
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        return False
    # 关节角：6 个数都在 ±180 内，且 z 不像夹爪高度（米）
    if all(abs(v) <= 180.0 for v in vals) and abs(vals[2]) < 1.5:
        return True
    return False


def read_arm_ee_from_stage(arm: str, stage=None) -> Optional[dict]:
    """从 Stage USD 读夹爪位姿（不依赖 articulation 绑定）。"""
    import numpy as np

    from ..global_variables import (
        ARM1_EE_LINK,
        ARM1_GRIPPER_FINGERS,
        ARM2_EE_LINK,
        ARM2_GRIPPER_FINGERS,
        BASE_LINK_PATH,
        ROBOT_PRIM_PATH,
    )

    fingers = ARM1_GRIPPER_FINGERS if arm == "arm1" else ARM2_GRIPPER_FINGERS
    ee_link = ARM1_EE_LINK if arm == "arm1" else ARM2_EE_LINK

    pts = []
    for name in fingers:
        mat = get_prim_world_matrix(f"{ROBOT_PRIM_PATH}/{name}", stage)
        if mat is not None:
            t, _ = matrix_to_translation_quat(mat)
            pts.append(np.array(t, dtype=np.float64))
    if pts:
        grip_w = np.mean(np.stack(pts, axis=0), axis=0)
    else:
        mat = get_prim_world_matrix(f"{ROBOT_PRIM_PATH}/{ee_link}", stage)
        if mat is None:
            return None
        t, _ = matrix_to_translation_quat(mat)
        grip_w = np.array(t, dtype=np.float64)

    mat_base = get_prim_world_matrix(BASE_LINK_PATH, stage)
    mat_ee = get_prim_world_matrix(f"{ROBOT_PRIM_PATH}/{ee_link}", stage)
    if mat_base is None or mat_ee is None:
        return None

    grip_base_v = mat_base.GetInverse().Transform(
        Gf.Vec3d(float(grip_w[0]), float(grip_w[1]), float(grip_w[2]))
    )
    grip_base = [float(grip_base_v[0]), float(grip_base_v[1]), float(grip_base_v[2])]

    # USD 行向量：p_base = p_ee_local * mat_ee * inv(mat_base)
    mat_rel = mat_ee * mat_base.GetInverse()
    rot = mat_rel.ExtractRotation()
    euler = rot.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    rpy_deg = (float(euler[0]), float(euler[1]), float(euler[2]))

    return {
        "gripper_world": grip_w.tolist(),
        "gripper_base": grip_base,
        "tcp_rpy_deg": rpy_deg,
        "joints_deg": None,
        "source": "stage",
    }


def read_dual_arm_ee_from_stage(stage=None) -> Dict[str, dict]:
    out: Dict[str, dict] = {}
    for arm in ("arm1", "arm2"):
        data = read_arm_ee_from_stage(arm, stage)
        if data:
            out[arm] = data
    return out


_BOX_POSE_CACHE: dict = {"mono": 0.0, "center": None}
_BOX_POSE_CACHE_TTL_S = 0.2


def invalidate_box_pose_cache() -> None:
    """场景卸载 / 盒子重载后清缓存。"""
    _BOX_POSE_CACHE["mono"] = 0.0
    _BOX_POSE_CACHE["center"] = None


def _visual_bbox_center(stage, prim_paths: Sequence[str]) -> Optional[Tuple[float, float, float]]:
    """带短时缓存的视觉包围盒中心（避免每帧新建 BBoxCache）。"""
    import time

    now = time.monotonic()
    cached = _BOX_POSE_CACHE.get("center")
    if cached is not None and now - float(_BOX_POSE_CACHE.get("mono", 0.0)) < _BOX_POSE_CACHE_TTL_S:
        return cached

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            continue
        bound = cache.ComputeWorldBound(prim)
        if bound.GetRange().IsEmpty():
            continue
        center = bound.ComputeCentroid()
        result = (float(center[0]), float(center[1]), float(center[2]))
        _BOX_POSE_CACHE["mono"] = now
        _BOX_POSE_CACHE["center"] = result
        return result
    return None


def get_box_world_pose(stage=None) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
    """读取抓取盒世界位姿：中心优先用视觉 mesh 包围盒，旋转用刚体根节点。

    Play 期间只用 USD（``BBoxCache`` / ``ComputeLocalToWorldTransform``），
    禁止 ``SingleRigidPrim``，否则会触发 simulation view invalidated。
    """
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return None

    from ..global_variables import BOX_LINK_PATH, BOX_VISUAL_PATH

    link_mat = get_prim_world_matrix(BOX_LINK_PATH, stage)
    if link_mat is None:
        return None
    t, q = matrix_to_translation_quat(link_mat)

    center = _visual_bbox_center(
        stage,
        (BOX_VISUAL_PATH, f"{BOX_VISUAL_PATH}/mesh", BOX_LINK_PATH),
    )
    if center is not None:
        return (center, q)
    return (t, q)


def get_prim_world_matrix(prim_path: str, stage=None) -> Optional[Gf.Matrix4d]:
    """读取 Prim 世界变换矩阵。"""
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return None
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())


def matrix_to_translation_quat(mat: Gf.Matrix4d) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """4x4 矩阵 → (translation xyz, quaternion xyzw)。"""
    t = mat.ExtractTranslation()
    rot = mat.ExtractRotation().GetQuat()
    imag = rot.GetImaginary()
    return (
        (float(t[0]), float(t[1]), float(t[2])),
        (float(imag[0]), float(imag[1]), float(imag[2]), float(rot.GetReal())),
    )


def transform_pose_between_frames(
    translation: Tuple[float, float, float],
    quat_xyzw: Tuple[float, float, float, float],
    source_frame_path: str,
    target_frame_path: str,
    stage=None,
) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
    """将位姿从 source 坐标系变换到 target 坐标系（两者均为 Stage prim 路径）。"""
    mat_src = get_prim_world_matrix(source_frame_path, stage)
    mat_tgt = get_prim_world_matrix(target_frame_path, stage)
    if mat_src is None or mat_tgt is None:
        return None
    q = Gf.Quatd(quat_xyzw[3], Gf.Vec3d(*quat_xyzw[:3]))
    pose_m = Gf.Matrix4d(1.0)
    pose_m.SetRotateOnly(q)
    pose_m.SetTranslateOnly(Gf.Vec3d(*translation))
    # USD 行向量：p_world = p_local * pose_m * mat_src
    mat_pose_world = pose_m * mat_src
    mat_rel = mat_pose_world * mat_tgt.GetInverse()
    return matrix_to_translation_quat(mat_rel)


def read_box_pose_in_frame(box_link_path: str, frame_prim_path: str, stage=None) -> Optional[dict]:
    """读取抓取盒在指定坐标系下的位姿，返回 JSON 可序列化字典。"""
    from ..global_variables import BOX_POSE_FRAME

    world_pose = get_box_world_pose(stage)
    if world_pose is not None:
        t_world, q_world = world_pose
        mat_box = Gf.Matrix4d(1.0)
        mat_box.SetRotateOnly(Gf.Quatd(q_world[3], Gf.Vec3d(*q_world[:3])))
        mat_box.SetTranslateOnly(Gf.Vec3d(*t_world))
    else:
        mat_box = get_prim_world_matrix(box_link_path, stage)
        if mat_box is None:
            return None

    mat_frame = get_prim_world_matrix(frame_prim_path, stage)
    if mat_frame is None:
        return None
    # USD 行向量约定：相对位姿 = M_child * inv(M_parent)
    # 误写成 inv(M_parent)*M_child 时，盒子 pitch=90° 会把中心 (0.53,0,0.26)
    # 错算成约 (-0.64,0,0.37)，导致 MoveIt IK 全失败。
    mat_rel = mat_box * mat_frame.GetInverse()
    t, q = matrix_to_translation_quat(mat_rel)
    return {
        "frame_id": BOX_POSE_FRAME if "base_link" in frame_prim_path else frame_prim_path.split("/")[-1],
        "position": list(t),
        "orientation_xyzw": list(q),
    }
