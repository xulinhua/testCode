# -*- coding: utf-8 -*-
"""工厂过道杂物：带碰撞 + 刚体动力学（Dynamic*），用于障碍建图测试。

仓库货架已在 USD 里；这里再放桶/箱，Play 后受重力落地。
不依赖 Nucleus 额外资产。
"""

from __future__ import annotations

from typing import Any, List, Tuple

import numpy as np

from ..global_variables import CLUTTER_ROOT_PATH, ENABLE_FACTORY_CLUTTER, SPOT_SPAWN_POS

# 地面约 z=0；中心 z = height/2 + 微小抬起，Play 后靠重力落地
_FLOOR_Z = 0.0
_DROP_EPS = 0.03

# (name, x, y, radius, height, mass_kg, rgb) — 油桶/立柱
_BARRELS: Tuple[Tuple[str, float, float, float, float, float, Tuple[float, float, float]], ...] = (
    ("barrel_a", 4.6, 1.35, 0.18, 0.88, 12.0, (0.75, 0.28, 0.18)),
    ("barrel_b", 6.2, -1.10, 0.20, 0.92, 14.0, (0.22, 0.42, 0.70)),
    ("barrel_c", -3.8, 1.55, 0.16, 0.80, 10.0, (0.35, 0.55, 0.30)),
    ("post_slim", 7.4, 0.45, 0.08, 1.10, 6.0, (0.55, 0.55, 0.50)),
)

# (name, x, y, yaw, size_xyz, mass_kg, rgb) — 木箱/托盘货
_CRATES: Tuple[Tuple[str, float, float, float, Tuple[float, float, float], float, Tuple[float, float, float]], ...] = (
    ("crate_a", 5.4, 0.85, 0.15, (0.55, 0.45, 0.40), 18.0, (0.62, 0.48, 0.28)),
    ("crate_b", 8.0, -0.70, -0.40, (0.70, 0.50, 0.35), 16.0, (0.50, 0.38, 0.22)),
    ("crate_c", -5.2, -1.20, 0.30, (0.48, 0.48, 0.50), 15.0, (0.58, 0.42, 0.25)),
    ("pallet_box", 4.0, -1.60, 0.0, (0.80, 0.60, 0.28), 20.0, (0.70, 0.55, 0.32)),
)


def spawn_factory_clutter(world: Any) -> List[Any]:
    """生成带碰撞/刚体的杂物，加入 World.scene。"""
    if not ENABLE_FACTORY_CLUTTER:
        print("SceneClutter: disabled (ENABLE_FACTORY_CLUTTER=False)")
        return []
    if world is None:
        print("SceneClutter: world is None, skip")
        return []

    from isaacsim.core.api.materials.physics_material import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder
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

    for name, x, y, radius, height, mass, rgb in _BARRELS:
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

    for name, x, y, yaw, size_xyz, mass, rgb in _CRATES:
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

    print(
        f"SceneClutter: spawned {len(added)} factory props with collision "
        f"(dynamic barrels/crates) under {CLUTTER_ROOT_PATH}"
    )
    return added


def remove_factory_clutter(stage: Any) -> None:
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
