# -*- coding: utf-8 -*-
"""抓取盒刚体：动态 + 重力；释放时停止 UI 钉扎，由 PhysX 接管。"""

from __future__ import annotations

import math
from typing import Optional, Sequence

from ..global_variables import (
    BOX_LINK_PATH,
    BOX_SIZE_Z,
    BOX_SURFACE_CLEARANCE_M,
    TABLE_TOP_Z,
    WORKSPACE_BASE_LINK_MESH_TOP_Z,
    WORKSPACE_COLLISION_FILL_NAME,
)


def configure_box_usd_dynamic(stage=None, *, kinematic: bool = False) -> bool:
    """写 USD 属性：动态刚体 + 重力。"""
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return False

    from pxr import UsdGeom, UsdPhysics

    prim = stage.GetPrimAtPath(BOX_LINK_PATH)
    if not prim or not prim.IsValid():
        print(f"box_physics: prim missing {BOX_LINK_PATH}")
        return False
    if not prim.IsA(UsdGeom.Xformable):
        print(f"box_physics: skip RigidBodyAPI — not xformable: {BOX_LINK_PATH}")
        return False

    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(bool(kinematic))

    try:
        from pxr import PhysxSchema

        px_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px_rb.CreateSleepThresholdAttr(0.0)
        px_rb.CreateStabilizationThresholdAttr(0.0)
        px_rb.CreateDisableGravityAttr(False)
    except Exception:
        pass

    return True


def _repair_xform_op_order(xformable, prim) -> list:
    """xformOpOrder 被清空但属性仍在时，恢复顺序（避免 AddTranslateOp 重复报错）。"""
    from pxr import UsdGeom

    if xformable.GetOrderedXformOps():
        return list(xformable.GetOrderedXformOps())

    ops = []
    for name in ("xformOp:translate", "xformOp:orient", "xformOp:rotateXYZ", "xformOp:scale"):
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            ops.append(UsdGeom.XformOp(attr))
    if ops:
        xformable.SetXformOpOrder(ops)
    return ops


def _vec3_for_xform_op(op, position: Sequence[float]):
    """按 xformOp 精度写入 translate（float/double）。"""
    from pxr import Gf, UsdGeom

    v = Gf.Vec3d(float(position[0]), float(position[1]), float(position[2]))
    if op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        return Gf.Vec3f(v)
    return v


def _quat_for_xform_op(op, quat: "Gf.Quatd"):
    """按 xformOp 精度写入 orient（常见为 GfQuatf）。"""
    from pxr import Gf, UsdGeom

    if op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        imag = quat.GetImaginary()
        return Gf.Quatf(
            float(quat.GetReal()),
            float(imag[0]),
            float(imag[1]),
            float(imag[2]),
        )
    return quat


def _set_box_local_pose(xformable, prim, position: Sequence[float], quat: "Gf.Quatd") -> None:
    """复用已有 xformOp 写位姿，Play 期间禁止 ClearXformOpOrder / SingleRigidPrim。"""
    from pxr import Gf, UsdGeom

    from .pose_utils import _get_or_add_xform_op

    ops = _repair_xform_op_order(xformable, prim)
    translate_set = False
    rotate_set = False

    for op in ops:
        if op.IsInverseOp():
            continue
        op_type = op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeTranslate:
            op.Set(_vec3_for_xform_op(op, position))
            translate_set = True
        elif op_type == UsdGeom.XformOp.TypeOrient:
            op.Set(_quat_for_xform_op(op, quat))
            rotate_set = True
        elif op_type == UsdGeom.XformOp.TypeRotateXYZ:
            euler = Gf.Rotation(quat).Decompose(
                Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()
            )
            op.Set(
                Gf.Vec3f(
                    math.degrees(float(euler[0])),
                    math.degrees(float(euler[1])),
                    math.degrees(float(euler[2])),
                )
            )
            rotate_set = True

    if not translate_set:
        t_op = _get_or_add_xform_op(xformable, UsdGeom.XformOp.TypeTranslate)
        if t_op is not None:
            t_op.Set(_vec3_for_xform_op(t_op, position))
    if not rotate_set:
        r_op = _get_or_add_xform_op(xformable, UsdGeom.XformOp.TypeRotateXYZ)
        if r_op is not None:
            euler = Gf.Rotation(quat).Decompose(
                Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()
            )
            r_op.Set(
                Gf.Vec3f(
                    math.degrees(float(euler[0])),
                    math.degrees(float(euler[1])),
                    math.degrees(float(euler[2])),
                )
            )


def box_world_bounds_min_z(stage=None, prim_path: str = BOX_LINK_PATH) -> Optional[float]:
    """盒子刚体（含 collision_geo）世界系包围盒最低 z。"""
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return None

    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    rng = cache.ComputeWorldBound(prim).GetRange()
    if rng.IsEmpty():
        return None
    return float(rng.GetMin()[2])


def lift_box_clear_of_surface(
    stage=None,
    *,
    surface_z_world: float,
    clearance: float = BOX_SURFACE_CLEARANCE_M,
    log: bool = True,
) -> bool:
    """按碰撞/视觉包围盒最低点抬高盒子，避免倾斜时穿模。"""
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return False

    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(BOX_LINK_PATH)
    if not prim or not prim.IsValid():
        if log:
            print(f"box_physics: prim missing {BOX_LINK_PATH}")
        return False

    min_z = box_world_bounds_min_z(stage)
    if min_z is None:
        return False

    target_bottom = float(surface_z_world) + float(clearance)
    delta = target_bottom - min_z
    if delta <= 1e-5:
        return True

    xformable = UsdGeom.Xformable(prim)
    world_mat = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = world_mat.ExtractTranslation()
    quat = world_mat.ExtractRotation().GetQuat()
    position = [
        float(translation[0]),
        float(translation[1]),
        float(translation[2] + delta),
    ]
    _set_box_local_pose(xformable, prim, position, quat)
    if log:
        print(
            f"box_physics: lifted box by {delta:.4f} m "
            f"(bottom {min_z:.3f} -> {target_bottom:.3f})"
        )
    return True


def pin_box_kinematic_pose(pose, stage=None, *, log: bool = False) -> bool:
    """采集专用：kinematic 钉扎到指定位姿（写全 xformOp、清零速度、关重力）。"""
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return False

    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    from .pose_utils import Pose6D, get_box_world_pose, invalidate_box_pose_cache

    if not isinstance(pose, Pose6D):
        return False

    prim = stage.GetPrimAtPath(BOX_LINK_PATH)
    if not prim or not prim.IsValid():
        if log:
            print(f"box_physics: prim missing {BOX_LINK_PATH}")
        return False

    configure_box_usd_dynamic(stage, kinematic=True)
    rb = UsdPhysics.RigidBodyAPI(prim)
    if rb:
        rb.CreateKinematicEnabledAttr(True)

    try:
        from pxr import PhysxSchema

        px_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px_rb.CreateDisableGravityAttr(True)
        for attr_name in (
            "physics:velocity",
            "physics:angularVelocity",
            "physics:linearVelocity",
        ):
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid():
                attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
    except Exception:
        pass

    rot = Gf.Rotation(Gf.Vec3d.XAxis(), float(pose.rotation_deg[0]))
    rot *= Gf.Rotation(Gf.Vec3d.YAxis(), float(pose.rotation_deg[1]))
    rot *= Gf.Rotation(Gf.Vec3d.ZAxis(), float(pose.rotation_deg[2]))
    quat = rot.GetQuat()
    position = [float(pose.translation[0]), float(pose.translation[1]), float(pose.translation[2])]

    xformable = UsdGeom.Xformable(prim)
    _set_box_local_pose(xformable, prim, position, quat)
    invalidate_box_pose_cache()
    _teleport_box_physx(position, quat, log=log)

    if log:
        world = get_box_world_pose(stage)
        if world:
            pos, _ = world
            print(
                f"box_physics: pinned kinematic @ "
                f"({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}) "
                f"target z={pose.translation[2]:.3f}"
            )
    return True


_box_rb_view = None


def invalidate_box_physx_view() -> None:
    """Timeline Stop / Unload 后丢掉缓存的 PhysX rigid-body view。"""
    global _box_rb_view
    _box_rb_view = None


def _get_box_rb_view():
    """通过 SimulationManager 创建 tensors RigidBodyView（不碰 USD xformOp）。"""
    global _box_rb_view
    if _box_rb_view is not None:
        try:
            # soft reset 后旧 view 仍非 None，但已 invalidated
            count = int(getattr(_box_rb_view, "count", 0))
            if count >= 1:
                # 轻量探测：失败则重建
                if hasattr(_box_rb_view, "get_transforms"):
                    _ = _box_rb_view.get_transforms()
                return _box_rb_view
        except Exception:
            _box_rb_view = None

    from isaacsim.core.simulation_manager import SimulationManager

    sim_view = SimulationManager.get_physics_sim_view()
    if sim_view is None:
        return None
    try:
        _box_rb_view = sim_view.create_rigid_body_view(BOX_LINK_PATH)
    except Exception:
        _box_rb_view = None
        return None
    if _box_rb_view is None or int(getattr(_box_rb_view, "count", 0)) < 1:
        _box_rb_view = None
        return None
    return _box_rb_view


def _teleport_box_physx(position: Sequence[float], quat: "Gf.Quatd", *, log: bool = False) -> bool:
    """Play 期间经 PhysX tensors 瞬移刚体（避免 RigidPrim→orient 报错）。"""
    try:
        import omni.timeline
        import numpy as np
    except Exception as exc:
        if log:
            print(f"box_physics: PhysX teleport unavailable: {exc}")
        return False

    if not omni.timeline.get_timeline_interface().is_playing():
        return False

    try:
        view = _get_box_rb_view()
        if view is None:
            return False

        imag = quat.GetImaginary()
        # PhysX tensors: (x, y, z, qx, qy, qz, qw)
        pose = np.array(
            [
                [
                    float(position[0]),
                    float(position[1]),
                    float(position[2]),
                    float(imag[0]),
                    float(imag[1]),
                    float(imag[2]),
                    float(quat.GetReal()),
                ]
            ],
            dtype=np.float32,
        )
        indices = np.array([0], dtype=np.uint32)
        # 瞬移 + kinematic 目标，避免下一步又被旧状态拉回
        view.set_transforms(pose, indices)
        try:
            view.set_kinematic_targets(pose, indices)
        except Exception:
            pass
        if log:
            print(
                f"box_physics: PhysX teleport "
                f"({pose[0,0]:.3f},{pose[0,1]:.3f},{pose[0,2]:.3f})"
            )
        return True
    except Exception as exc:
        if log:
            print(f"box_physics: PhysX teleport failed: {exc}")
        invalidate_box_physx_view()
        return False


def release_box_for_gravity(
    stage=None,
    *,
    surface_z_world: float | None = None,
    log: bool = True,
) -> bool:
    """释放盒子给 PhysX：设为动态刚体，一次性写入当前位姿后不再由 UI 每帧覆盖。

    注意：Play 期间不能用 ``ClearXformOpOrder`` 或 ``SingleRigidPrim.set_world_pose``，
    否则会触发 PhysX simulation view invalidated 错误。
    """
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if not stage:
        return False

    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(BOX_LINK_PATH)
    if not prim or not prim.IsValid():
        if log:
            print(f"box_physics: prim missing {BOX_LINK_PATH}")
        return False

    configure_box_usd_dynamic(stage, kinematic=False)

    if surface_z_world is None:
        surface_z_world = TABLE_TOP_Z + WORKSPACE_BASE_LINK_MESH_TOP_Z

    lift_box_clear_of_surface(
        stage,
        surface_z_world=float(surface_z_world),
        clearance=BOX_SURFACE_CLEARANCE_M,
        log=log,
    )

    xformable = UsdGeom.Xformable(prim)
    world_mat = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    translation = world_mat.ExtractTranslation()
    quat = world_mat.ExtractRotation().GetQuat()

    position = [
        float(translation[0]),
        float(translation[1]),
        float(translation[2]),
    ]

    _set_box_local_pose(xformable, prim, position, quat)

    if log:
        import omni.timeline

        playing = omni.timeline.get_timeline_interface().is_playing()
        print(
            f"box_physics: released for gravity @ "
            f"({position[0]:.3f},{position[1]:.3f},{position[2]:.3f})"
            + ("" if playing else " (press Play to simulate)")
        )
    return True


def box_rest_center_on_workspace(
    arm_center_xy: tuple[float, float] = (0.0, 0.0),
    *,
    surface_z_world: float | None = None,
) -> tuple[float, float, float]:
    """盒子中心默认世界坐标：双臂中点 xy，底面贴在 workspace 工作面顶。"""
    clearance = BOX_SURFACE_CLEARANCE_M
    if surface_z_world is None:
        surface_z_world = TABLE_TOP_Z + WORKSPACE_BASE_LINK_MESH_TOP_Z
    z = float(surface_z_world) + BOX_SIZE_Z * 0.5 + clearance
    return (float(arm_center_xy[0]), float(arm_center_xy[1]), z)


def box_rest_center_on_table(
    arm_center_xy: tuple[float, float] = (0.0, 0.0),
    *,
    surface_z_world: float | None = None,
) -> tuple[float, float, float]:
    """兼容旧名；实际落在 workspace 工作面而非裸桌面。"""
    return box_rest_center_on_workspace(
        arm_center_xy, surface_z_world=surface_z_world
    )
