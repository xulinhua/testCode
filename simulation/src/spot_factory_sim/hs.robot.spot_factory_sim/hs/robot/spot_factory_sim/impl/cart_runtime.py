# -*- coding: utf-8 -*-
"""差速小车（NVIDIA Carter V1）运行时：/cmd_vel → DifferentialController → 轮速。"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from ..global_variables import (
    CART_PRIM_PATH,
    CART_SPAWN_ORI,
    CART_SPAWN_POS,
    CART_WHEEL_BASE,
    CART_WHEEL_DOF_NAMES,
    CART_WHEEL_RADIUS,
)
from ..paths import local_cart_usd

_SETTLE_STEPS = 20


class CartRuntime:
    def __init__(self, prim_path: str = CART_PRIM_PATH, ext_root: Optional[str] = None):
        self.prim_path = prim_path
        self._ext_root = ext_root
        self._robot = None
        self._controller = None
        self._command = np.zeros(3, dtype=float)  # vx, vy(ignored), wz
        self._initialized = False
        self._needs_init = False
        self._needs_pose_reset = False
        self._spawn_pos = np.asarray(CART_SPAWN_POS, dtype=float)
        self._spawn_ori = np.asarray(CART_SPAWN_ORI, dtype=float)
        self._settle_left = 0
        self._init_fail_logged = False
        self._bound_sim_view = None
        self._play_warmup_left = 0
        self._usd_path: Optional[str] = None

    @property
    def is_ready(self) -> bool:
        return self._robot is not None

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def robot(self):
        return self._robot

    def spawn(
        self,
        position: Sequence[float] = CART_SPAWN_POS,
        orientation_wxyz: Sequence[float] = CART_SPAWN_ORI,
        usd_path: Optional[str] = None,
    ):
        self.shutdown()

        usd = usd_path or local_cart_usd(self._ext_root)
        if not usd:
            raise FileNotFoundError(
                "Local Carter USD missing. Run: python3 scripts/download_assets.py"
            )

        from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
            DifferentialController,
        )
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot

        robot = WheeledRobot(
            prim_path=self.prim_path,
            name="Cart",
            wheel_dof_names=list(CART_WHEEL_DOF_NAMES),
            create_robot=False,
            position=np.asarray(position, dtype=float),
            orientation=np.asarray(orientation_wxyz, dtype=float),
        )
        self._robot = robot
        self._controller = DifferentialController(
            name="cart_diff",
            wheel_radius=float(CART_WHEEL_RADIUS),
            wheel_base=float(CART_WHEEL_BASE),
        )
        self._usd_path = usd
        self._spawn_pos = np.asarray(position, dtype=float)
        self._spawn_ori = np.asarray(orientation_wxyz, dtype=float)
        self._needs_init = True
        self._needs_pose_reset = False
        self._initialized = False
        self._settle_left = 0
        self._init_fail_logged = False
        self._bound_sim_view = None
        self._play_warmup_left = 0
        self._command[:] = 0.0
        print(
            f"CartRuntime: spawned {self.prim_path} at "
            f"({self._spawn_pos[0]:.2f},{self._spawn_pos[1]:.2f},{self._spawn_pos[2]:.2f}) usd={usd}"
        )

    def shutdown(self) -> None:
        self._robot = None
        self._controller = None
        self._initialized = False
        self._needs_init = False
        self._needs_pose_reset = False
        self._settle_left = 0
        self._bound_sim_view = None
        self._command[:] = 0.0

    def mark_needs_init(self) -> None:
        if self._robot is None:
            return
        self._invalidate_physics_handle()
        self._needs_init = True
        self._needs_pose_reset = True
        self._initialized = False
        self._settle_left = 0
        self._init_fail_logged = False
        self._bound_sim_view = None
        self._play_warmup_left = 8
        self._command[:] = 0.0

    def mark_needs_pose_reset(self) -> None:
        if self._robot is None:
            return
        self._needs_pose_reset = True
        self._settle_left = 0
        self._command[:] = 0.0
        if not self._articulation_handles_valid():
            self.mark_needs_init()

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        # 差速底盘忽略侧向 vy
        self._command[0] = float(vx)
        self._command[1] = 0.0
        self._command[2] = float(wz)

    def _physics_view_ready(self) -> bool:
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            view = SimulationManager.get_physics_sim_view()
            if view is not None:
                return True
            SimulationManager.initialize_physics()
            return SimulationManager.get_physics_sim_view() is not None
        except Exception as exc:
            if not self._init_fail_logged:
                print(f"CartRuntime: physics view not ready yet: {exc}")
                self._init_fail_logged = True
            return False

    def _current_sim_view(self):
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            return SimulationManager.get_physics_sim_view()
        except Exception:
            return None

    def _articulation_handles_valid(self) -> bool:
        if self._robot is None:
            return False
        view = getattr(self._robot, "_articulation_view", None)
        if view is None:
            return False
        sim = self._current_sim_view()
        if sim is None or self._bound_sim_view is None or sim is not self._bound_sim_view:
            return False
        try:
            if hasattr(view, "is_physics_handle_valid") and not bool(view.is_physics_handle_valid()):
                return False
        except Exception:
            return False
        if getattr(view, "_physics_view", None) is None:
            return False
        try:
            _ = self._robot.get_joint_positions()
            return True
        except Exception:
            return False

    def _invalidate_physics_handle(self) -> None:
        self._bound_sim_view = None
        if self._robot is None:
            return
        view = getattr(self._robot, "_articulation_view", None)
        if view is None:
            return
        try:
            view._physics_view = None
        except Exception:
            pass

    def _reset_spawn_pose(self) -> None:
        if self._robot is None:
            return
        try:
            self._robot.set_world_pose(position=self._spawn_pos, orientation=self._spawn_ori)
            self._robot.set_linear_velocity(np.zeros(3, dtype=float))
            self._robot.set_angular_velocity(np.zeros(3, dtype=float))
        except Exception as exc:
            print(f"CartRuntime: set_world_pose failed, try USD xform: {exc}")
            self._reset_spawn_pose_usd()
        try:
            zeros = np.zeros(2, dtype=float)
            self._robot.apply_wheel_actions(
                __import__("isaacsim.core.utils.types", fromlist=["ArticulationAction"]).ArticulationAction(
                    joint_velocities=zeros
                )
            )
        except Exception:
            pass

    def _reset_spawn_pose_usd(self) -> None:
        try:
            import omni.usd
            from pxr import Gf, UsdGeom

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return
            prim = stage.GetPrimAtPath(self.prim_path)
            if not prim or not prim.IsValid():
                return
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(
                    float(self._spawn_pos[0]),
                    float(self._spawn_pos[1]),
                    float(self._spawn_pos[2]),
                )
            )
            qw, qx, qy, qz = [float(v) for v in self._spawn_ori]
            xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))
            print(
                f"CartRuntime: USD pose reset @ "
                f"({self._spawn_pos[0]:.2f},{self._spawn_pos[1]:.2f},{self._spawn_pos[2]:.2f})"
            )
        except Exception as exc:
            print(f"CartRuntime: USD pose reset failed: {exc}")

    def on_physics_step(self, dt: float) -> None:
        if self._robot is None or self._controller is None:
            return

        if self._needs_init or not self._initialized:
            if self._play_warmup_left > 0:
                self._play_warmup_left -= 1
                return
            if not self._physics_view_ready():
                return
            try:
                self._invalidate_physics_handle()
                self._robot.initialize()
                self._bound_sim_view = self._current_sim_view()
                _ = self._robot.get_joint_positions()
                self._initialized = True
                self._needs_init = False
                self._reset_spawn_pose()
                self._settle_left = _SETTLE_STEPS
                print(f"CartRuntime: initialized (settle {_SETTLE_STEPS})")
            except Exception as exc:
                if not self._init_fail_logged:
                    print(f"CartRuntime.initialize failed: {exc}")
                    self._init_fail_logged = True
                self._initialized = False
                self._needs_init = True
            return

        if not self._articulation_handles_valid():
            self.mark_needs_init()
            return

        if self._needs_pose_reset:
            self._reset_spawn_pose()
            self._needs_pose_reset = False
            self._settle_left = _SETTLE_STEPS

        if self._settle_left > 0:
            self._settle_left -= 1
            try:
                from isaacsim.core.utils.types import ArticulationAction

                self._robot.apply_wheel_actions(ArticulationAction(joint_velocities=np.zeros(2, dtype=float)))
            except Exception:
                pass
            return

        try:
            action = self._controller.forward(command=np.array([self._command[0], self._command[2]], dtype=float))
            self._robot.apply_wheel_actions(action)
        except Exception as exc:
            if not self._init_fail_logged:
                print(f"CartRuntime.forward failed: {exc}")
                self._init_fail_logged = True
            self.mark_needs_init()
