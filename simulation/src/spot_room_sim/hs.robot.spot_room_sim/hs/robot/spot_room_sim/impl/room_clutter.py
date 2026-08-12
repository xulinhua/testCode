# -*- coding: utf-8 -*-
"""房间杂物：带碰撞 + 刚体动力学（Dynamic*），用于障碍建图测试。

- 圆柱 / 箱体：Dynamic + mass，Play 后受重力落到地面/桌面
- 碰撞：Collider API（深度相机可见外形，PhysX 可撞）
不依赖 Nucleus 额外资产。
"""

from __future__ import annotations

from typing import Any, List, Tuple

import numpy as np

from ..global_variables import CLUTTER_ROOT_PATH, ENABLE_ROOM_CLUTTER, SPOT_SPAWN_POS

# 地面约 z=0；中心 z = height/2 + 微小抬起，Play 后靠重力落地
_FLOOR_Z = 0.0
_DROP_EPS = 0.03

# (name, x, y, radius, height, mass_kg, rgb)
_CYLINDERS: Tuple[Tuple[str, float, float, float, float, float, Tuple[float, float, float]], ...] = (
    ("cyl_short", -0.6, 1.55, 0.10, 0.30, 3.0, (0.85, 0.45, 0.20)),
    ("cyl_mid", -0.2, 0.95, 0.14, 0.60, 5.0, (0.25, 0.55, 0.85)),
    ("cyl_tall", 0.35, 1.35, 0.11, 0.90, 6.0, (0.35, 0.75, 0.40)),
    ("cyl_fat", 0.10, 0.35, 0.22, 0.44, 8.0, (0.70, 0.30, 0.55)),
    ("cyl_slim", -1.0, 0.70, 0.07, 0.70, 2.5, (0.90, 0.75, 0.20)),
)

# 简易「椅子」用整块动态箱近似（多块会散架）；带质量与碰撞
# (name, x, y, yaw, size_xyz, mass_kg, rgb)
_STOOLS: Tuple[Tuple[str, float, float, float, Tuple[float, float, float], float, Tuple[float, float, float]], ...] = (
    ("stool_a", 0.6, -0.4, 0.0, (0.40, 0.40, 0.48), 6.0, (0.55, 0.38, 0.22)),
    ("stool_b", -0.9, -0.2, 0.6, (0.38, 0.38, 0.52), 5.5, (0.45, 0.30, 0.18)),
)

# 靠背条：固定碰撞（挂在凳子旁，不散架）
# (name, x, y, yaw, size_xyz, rgb)
_BACKRESTS: Tuple[Tuple[str, float, float, float, Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ("back_a", 0.6, -0.55, 0.0, (0.40, 0.05, 0.45), (0.40, 0.28, 0.16)),
    ("back_b", -1.05, -0.2, 0.6, (0.38, 0.05, 0.45), (0.38, 0.26, 0.15)),
)


def spawn_room_clutter(world: Any) -> List[Any]:
    """生成带碰撞/刚体的杂物，加入 World.scene。"""
    if not ENABLE_ROOM_CLUTTER:
        print("SceneClutter: disabled (ENABLE_ROOM_CLUTTER=False)")
        return []
    if world is None:
        print("SceneClutter: world is None, skip")
        return []

    from isaacsim.core.api.materials.physics_material import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder, FixedCuboid
    from isaacsim.core.utils.prims import define_prim, is_prim_path_valid
    from isaacsim.core.utils.string import find_unique_string_name

    define_prim(CLUTTER_ROOT_PATH, "Xform")
    mat_path = find_unique_string_name(
        initial_name=f"{CLUTTER_ROOT_PATH}/physics_material",
        is_unique_fn=lambda x: not is_prim_path_valid(x),
    )
    phys_mat = PhysicsMaterial(
        prim_path=mat_path,
        static_friction=0.8,
        dynamic_friction=0.6,
        restitution=0.05,
    )

    added: List[Any] = []

    for name, x, y, radius, height, mass, rgb in _CYLINDERS:
        if _too_close_to_spot(x, y):
            print(f"SceneClutter: skip {name} (too close to Spot spawn)")
            continue
        z = _FLOOR_Z + 0.5 * height + _DROP_EPS
        path = f"{CLUTTER_ROOT_PATH}/{name}"
        obj = DynamicCylinder(
            prim_path=path,
            name=name,
            position=np.array([x, y, z], dtype=float),
            radius=float(radius),
            height=float(height),
            color=np.array(rgb, dtype=float),
            mass=float(mass),
            physics_material=phys_mat,
        )
        try:
            obj.set_collision_enabled(True)
        except Exception:
            pass
        world.scene.add(obj)
        added.append(obj)

    import math

    for name, x, y, yaw, size_xyz, mass, rgb in _STOOLS:
        if _too_close_to_spot(x, y):
            print(f"SceneClutter: skip {name} (too close to Spot spawn)")
            continue
        sx, sy, sz = size_xyz
        z = _FLOOR_Z + 0.5 * sz + _DROP_EPS
        qw = math.cos(yaw * 0.5)
        qz = math.sin(yaw * 0.5)
        path = f"{CLUTTER_ROOT_PATH}/{name}"
        obj = DynamicCuboid(
            prim_path=path,
            name=name,
            position=np.array([x, y, z], dtype=float),
            orientation=np.array([qw, 0.0, 0.0, qz], dtype=float),
            scale=np.array([sx, sy, sz], dtype=float),
            color=np.array(rgb, dtype=float),
            mass=float(mass),
            physics_material=phys_mat,
        )
        try:
            obj.set_collision_enabled(True)
        except Exception:
            pass
        world.scene.add(obj)
        added.append(obj)

    # 靠背：固定碰撞体（静态障碍），贴地立住
    for name, x, y, yaw, size_xyz, rgb in _BACKRESTS:
        if _too_close_to_spot(x, y):
            continue
        sx, sy, sz = size_xyz
        z = _FLOOR_Z + 0.5 * sz
        qw = math.cos(yaw * 0.5)
        qz = math.sin(yaw * 0.5)
        path = f"{CLUTTER_ROOT_PATH}/{name}"
        obj = FixedCuboid(
            prim_path=path,
            name=name,
            position=np.array([x, y, z], dtype=float),
            orientation=np.array([qw, 0.0, 0.0, qz], dtype=float),
            scale=np.array([sx, sy, sz], dtype=float),
            color=np.array(rgb, dtype=float),
            physics_material=phys_mat,
        )
        try:
            obj.set_collision_enabled(True)
        except Exception:
            pass
        world.scene.add(obj)
        added.append(obj)

    print(
        f"SceneClutter: spawned {len(added)} props with collision "
        f"(dynamic cylinders/stools + fixed backs) under {CLUTTER_ROOT_PATH}"
    )
    return added


def remove_room_clutter(stage: Any) -> None:
    if stage is None:
        return
    prim = stage.GetPrimAtPath(CLUTTER_ROOT_PATH)
    if prim and prim.IsValid():
        try:
            stage.RemovePrim(CLUTTER_ROOT_PATH)
            print(f"SceneClutter: removed {CLUTTER_ROOT_PATH}")
        except Exception as exc:
            print(f"SceneClutter: remove failed: {exc}")


def _too_close_to_spot(x: float, y: float, min_dist: float = 0.85) -> bool:
    dx = float(x) - float(SPOT_SPAWN_POS[0])
    dy = float(y) - float(SPOT_SPAWN_POS[1])
    return (dx * dx + dy * dy) < (min_dist * min_dist)
