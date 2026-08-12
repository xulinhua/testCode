# -*- coding: utf-8 -*-
"""Spot RL 步态运行时：对齐官方 QuadrupedExample 初始化时序。

Timeline Stop / soft reset 会让 PhysX tensor handle（`_physics_view`）失效。
再次 Play 时应清掉 `_physics_view` 再 `initialize()`；**不能**把
`SingleArticulation._articulation_view` 设成 None（initialize 不会重建该对象）。
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

from ..global_variables import SPOT_PRIM_PATH, SPOT_SPAWN_ORI, SPOT_SPAWN_POS
from ..paths import local_spot_policy_files, local_spot_usd
from .policy.policy_controller import PolicyController
from .policy.spot_policy import SpotFlatTerrainPolicy

# Play 后钉住站立若干物理步再放开策略
_SETTLE_STEPS = 50


class SpotRuntime:
    def __init__(self, prim_path: str = SPOT_PRIM_PATH, ext_root: Optional[str] = None):
        self.prim_path = prim_path
        self._ext_root = ext_root
        self._spot = None
        self._command = np.zeros(3, dtype=float)
        self._initialized = False
        self._needs_init = False
        self._needs_pose_reset = False
        self._policy_pt: Optional[str] = None
        self._policy_yaml: Optional[str] = None
        self._policy_loaded = False
        self._spawn_pos = np.asarray(SPOT_SPAWN_POS, dtype=float)
        self._spawn_ori = np.asarray(SPOT_SPAWN_ORI, dtype=float)
        self._settle_left = 0
        self._init_fail_logged = False
        self._fwd_fail_logged = False
        self._bound_sim_view = None
        self._play_warmup_left = 0

    @property
    def is_ready(self) -> bool:
        return self._spot is not None

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def robot(self):
        return None if self._spot is None else self._spot.robot

    def spawn(
        self,
        position: Sequence[float] = SPOT_SPAWN_POS,
        orientation_wxyz: Sequence[float] = SPOT_SPAWN_ORI,
        usd_path: Optional[str] = None,
        policy_pt: Optional[str] = None,
        policy_yaml: Optional[str] = None,
    ):
        self.shutdown()

        usd = usd_path or local_spot_usd(self._ext_root)
        pt = policy_pt
        env_yaml = policy_yaml
        if pt is None or env_yaml is None:
            local_pt, local_yaml = local_spot_policy_files(self._ext_root)
            pt = pt or local_pt
            env_yaml = env_yaml or local_yaml

        if not usd:
            raise FileNotFoundError(
                "Local Spot USD missing. Run: python3 scripts/download_assets.py"
            )
        if not pt or not env_yaml:
            raise FileNotFoundError(
                "Local Spot policy missing. Run: python3 scripts/download_assets.py"
            )

        spot = SpotFlatTerrainPolicy.__new__(SpotFlatTerrainPolicy)
        PolicyController.__init__(
            spot,
            "Spot",
            self.prim_path,
            None,
            usd,
            np.asarray(position, dtype=float),
            np.asarray(orientation_wxyz, dtype=float),
        )
        spot._action_scale = 0.2
        spot._previous_action = np.zeros(12)
        spot._policy_counter = 0
        spot.policy = None
        spot.policy_env_params = None
        spot._decimation = 1
        spot._dt = 1.0 / 500.0
        spot.render_interval = 1
        spot.action = np.zeros(12)

        self._spot = spot
        self._spawn_pos = np.asarray(position, dtype=float)
        self._spawn_ori = np.asarray(orientation_wxyz, dtype=float)
        self._policy_pt = pt
        self._policy_yaml = env_yaml
        self._policy_loaded = False
        self._needs_init = True
        self._needs_pose_reset = False
        self._initialized = False
        self._settle_left = 0
        self._init_fail_logged = False
        self._fwd_fail_logged = False
        self._bound_sim_view = None
        self._play_warmup_left = 0
        self._command[:] = 0.0

        if not self._ensure_policy_loaded():
            raise RuntimeError(f"Failed to load Spot policy: {pt}")

        print(
            f"SpotRuntime: spawned {self.prim_path} at "
            f"({self._spawn_pos[0]:.2f},{self._spawn_pos[1]:.2f},{self._spawn_pos[2]:.2f}) usd={usd}"
        )

    def mark_needs_init(self) -> None:
        """Timeline Stop / Unload 后：使 PhysX handle 失效，下次 Play 硬初始化并复位出生点。"""
        if self._spot is None:
            return
        self._invalidate_physics_handle()
        self._needs_init = True
        self._needs_pose_reset = True
        self._initialized = False
        self._settle_left = 0
        self._init_fail_logged = False
        self._fwd_fail_logged = False
        self._bound_sim_view = None
        # Stop→Play 后 World 会重建 SimulationView；等几帧再 init，避免绑到即将销毁的旧 view
        self._play_warmup_left = 8
        self._command[:] = 0.0

    def mark_needs_pose_reset(self) -> None:
        """仅要求回到出生站立姿（handles 仍有效时走软复位）。"""
        if self._spot is None:
            return
        self._needs_pose_reset = True
        self._settle_left = 0
        self._command[:] = 0.0
        if not self._articulation_handles_valid():
            self.mark_needs_init()

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        self._command[0] = float(vx)
        self._command[1] = float(vy)
        self._command[2] = float(wz)

    def _ensure_policy_loaded(self) -> bool:
        if self._policy_loaded:
            return True
        if self._spot is None or not self._policy_pt or not self._policy_yaml:
            return False
        try:
            self._spot.load_policy(self._policy_pt, self._policy_yaml)
            self._policy_loaded = True
            print(f"SpotRuntime: policy loaded {self._policy_pt}")
            return True
        except Exception as exc:
            print(f"SpotRuntime.load_policy failed: {exc}")
            return False

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
                print(f"SpotRuntime: physics view not ready yet: {exc}")
                self._init_fail_logged = True
            return False

    def _current_sim_view(self):
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            return SimulationManager.get_physics_sim_view()
        except Exception:
            return None

    def _articulation_handles_valid(self) -> bool:
        """Stop/Play 后 SimulationView 会换新对象；旧 _physics_view 非空但 backend 已死。"""
        if self._spot is None or self._spot.robot is None:
            return False
        art = self._spot.robot
        view = getattr(art, "_articulation_view", None)
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
        # 轻量探活：能读关节位姿才算真活（避免 DOF targets from backend 刷屏）
        try:
            _ = self._spot.robot.get_joint_positions()
            return True
        except Exception:
            return False

    def _invalidate_physics_handle(self) -> None:
        """只清 Articulation._physics_view，保留 SingleArticulation._articulation_view。"""
        self._bound_sim_view = None
        if self._spot is None or self._spot.robot is None:
            return
        art = self._spot.robot
        view = getattr(art, "_articulation_view", None)
        if view is None:
            return
        try:
            view._physics_view = None
        except Exception:
            pass

    def _probe_after_init(self) -> None:
        """initialize 后立刻验证 DOF API，失败则抛出让外层重试。"""
        robot = self._spot.robot
        pos = robot.get_joint_positions()
        if pos is None:
            raise RuntimeError("get_joint_positions returned None after initialize")
        # apply_action 内部会 get_dof_position_targets；这里提前暴露 Stop/Play 后的死 handle
        from isaacsim.core.utils.types import ArticulationAction

        robot.apply_action(ArticulationAction(joint_positions=np.asarray(pos, dtype=float)))

    def _reset_standing_pose(self) -> None:
        from isaacsim.core.utils.types import ArticulationAction

        robot = self._spot.robot
        default_pos = np.asarray(self._spot.default_pos, dtype=float)
        try:
            robot.set_joints_default_state(default_pos)
        except Exception:
            pass
        try:
            robot.set_joint_positions(default_pos)
            robot.set_joint_velocities(np.zeros_like(default_pos, dtype=float))
        except Exception:
            pass
        try:
            robot.apply_action(ArticulationAction(joint_positions=default_pos))
        except Exception:
            pass
        try:
            robot.set_world_pose(position=self._spawn_pos, orientation=self._spawn_ori)
            robot.set_linear_velocity(np.zeros(3, dtype=float))
            robot.set_angular_velocity(np.zeros(3, dtype=float))
        except Exception as exc:
            print(f"SpotRuntime: set_world_pose failed, try USD xform: {exc}")
            self._reset_spawn_pose_usd()

    def _reset_spawn_pose_usd(self) -> None:
        """Articulation API 失败时，直接写 USD 位姿作为兜底。"""
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
                f"SpotRuntime: USD pose reset @ "
                f"({self._spawn_pos[0]:.2f},{self._spawn_pos[1]:.2f},{self._spawn_pos[2]:.2f})"
            )
        except Exception as exc:
            print(f"SpotRuntime: USD pose reset failed: {exc}")

    def _begin_settle(self, reason: str) -> None:
        self._reset_standing_pose()
        self._needs_pose_reset = False
        self._settle_left = _SETTLE_STEPS
        print(
            f"SpotRuntime: {reason} → standing @ "
            f"{self._spawn_pos[0]:.2f},{self._spawn_pos[1]:.2f},{self._spawn_pos[2]:.2f} "
            f"(settle {_SETTLE_STEPS})"
        )

    def on_physics_step(self, dt: float) -> None:
        if self._spot is None:
            return

        # 硬初始化：Stop / 首次 Play / PhysX handle 失效
        if self._needs_init or not self._initialized:
            if self._play_warmup_left > 0:
                self._play_warmup_left -= 1
                return
            if not self._ensure_policy_loaded():
                return
            if not self._physics_view_ready():
                return
            try:
                self._invalidate_physics_handle()
                self._spot.initialize()
                self._bound_sim_view = self._current_sim_view()
                self._probe_after_init()
                if not self._articulation_handles_valid():
                    raise RuntimeError("articulation handles still invalid after initialize")
                self._spot.post_reset()
                if not hasattr(self._spot, "default_pos") or self._spot.default_pos is None:
                    raise RuntimeError("default_pos missing after initialize")
                self._needs_init = False
                self._initialized = True
                self._init_fail_logged = False
                self._fwd_fail_logged = False
                self._begin_settle("hard re-init")
            except Exception as exc:
                self._initialized = False
                self._needs_init = True
                self._bound_sim_view = None
                self._invalidate_physics_handle()
                # 下一帧再试，避免卡在死 handle 上狂刷
                self._play_warmup_left = max(self._play_warmup_left, 3)
                if not self._init_fail_logged:
                    import traceback

                    traceback.print_exc()
                    print(f"SpotRuntime.initialize failed (will retry): {exc}")
                    self._init_fail_logged = True
            return

        # 软复位：handles 仍有效时只拉回出生点
        if self._needs_pose_reset:
            if not self._articulation_handles_valid():
                self.mark_needs_init()
                return
            try:
                self._begin_settle("pose reset")
            except Exception as exc:
                print(f"SpotRuntime.pose reset failed → hard re-init: {exc}")
                self.mark_needs_init()
            return

        if self._settle_left > 0:
            try:
                self._reset_standing_pose()
            except Exception as exc:
                # settle 阶段就 DOF 失败 → 直接重绑 sim
                if not self._fwd_fail_logged:
                    print(f"SpotRuntime: settle failed → re-init: {exc}")
                    self._fwd_fail_logged = True
                self.mark_needs_init()
                return
            self._settle_left -= 1
            if self._settle_left == 0:
                print("SpotRuntime: settle done, policy control enabled")
            return

        # 运行中若 PhysX handle / SimulationView 失效，下一帧硬复位
        if not self._articulation_handles_valid():
            if not self._fwd_fail_logged:
                print("SpotRuntime: articulation view lost mid-run → re-init + spawn reset")
                self._fwd_fail_logged = True
            self.mark_needs_init()
            return

        try:
            self._spot.forward(float(dt), self._command)
        except Exception as exc:
            msg = str(exc)
            if not self._fwd_fail_logged:
                print(f"SpotRuntime.forward failed: {exc}")
                self._fwd_fail_logged = True
            if (
                "DOF" in msg
                or "backend" in msg
                or "is_homogeneous" in msg
                or "NoneType" in msg
                or "Physics" in msg
            ):
                self.mark_needs_init()

    def get_world_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self._spot is None or not self._initialized:
            return None
        try:
            pos, quat = self._spot.robot.get_world_pose()
            return np.asarray(pos, dtype=float), np.asarray(quat, dtype=float)
        except Exception:
            return None

    def get_world_velocities(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self._spot is None or not self._initialized:
            return None, None
        try:
            lin = np.asarray(self._spot.robot.get_linear_velocity(), dtype=float)
            ang = np.asarray(self._spot.robot.get_angular_velocity(), dtype=float)
            return lin, ang
        except Exception:
            return None, None

    def get_joint_positions(self) -> Optional[np.ndarray]:
        if self._spot is None or not self._initialized:
            return None
        try:
            return np.asarray(self._spot.robot.get_joint_positions(), dtype=float)
        except Exception:
            return None

    def shutdown(self) -> None:
        self._invalidate_physics_handle()
        self._spot = None
        self._initialized = False
        self._needs_init = False
        self._needs_pose_reset = False
        self._policy_loaded = False
        self._policy_pt = None
        self._policy_yaml = None
        self._settle_left = 0
        self._init_fail_logged = False
        self._fwd_fail_logged = False
        self._bound_sim_view = None
        self._play_warmup_left = 0
        self._command[:] = 0.0
