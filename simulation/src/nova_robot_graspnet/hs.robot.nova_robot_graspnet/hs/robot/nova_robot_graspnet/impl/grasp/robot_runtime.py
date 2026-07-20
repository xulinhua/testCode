# -*- coding: utf-8 -*-
"""Nova 双臂运行时：关节读写、启发式 IK、夹爪。"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ...global_variables import (
    ARM1_EE_LINK,
    ARM1_GRIPPER_FINGERS,
    ARM1_GRIPPER_JOINTS,
    ARM1_JOINTS,
    ARM2_EE_LINK,
    ARM2_GRIPPER_FINGERS,
    ARM2_GRIPPER_JOINTS,
    ARM2_JOINTS,
    BASE_LINK_PATH,
    BOX_LINK_PATH,
    DEFAULT_ARM1_PARK_JOINTS,
    DEFAULT_ARM2_PARK_JOINTS,
    GRIPPER1_CLOSED,
    GRIPPER1_OPEN,
    GRIPPER2_CLOSED,
    GRIPPER2_OPEN,
    ROBOT_PRIM_PATH,
    ROBOT_ROOT_JOINT_PATH,
)
from .arm_kinematics import ArmKinematics, quat_xyzw_to_matrix, top_down_rotation

# 顶抓时腕部关节初值（弧度）：使夹爪大致朝下
WRIST_DOWN_J4_RAD = 0.0
WRIST_DOWN_J5_RAD = -1.35
WRIST_DOWN_J6_RAD = 0.0
# 每帧最大关节增量（弧度）
MAX_JOINT_STEP_RAD = 0.07


def _articulation_ready(art) -> bool:
    """兼容 Isaac Sim 5.x：SingleArticulation 无 is_physics_handle_valid，用 handles_initialized。"""
    if art is None:
        return False
    if hasattr(art, "handles_initialized"):
        try:
            return bool(art.handles_initialized)
        except Exception:
            pass
    view = getattr(art, "_articulation_view", None)
    if view is not None and hasattr(view, "is_physics_handle_valid"):
        try:
            return bool(view.is_physics_handle_valid())
        except Exception:
            pass
    if hasattr(art, "is_physics_handle_valid"):
        try:
            return bool(art.is_physics_handle_valid())
        except Exception:
            pass
    try:
        return len(list(art.dof_names or [])) > 0
    except Exception:
        return False


class NovaRobotRuntime:
    """封装 ``SingleArticulation``，支持 J1/J2 双臂抓取。"""

    def __init__(self, robot_prim_path: str = ROBOT_ROOT_JOINT_PATH):
        self.robot_prim_path = robot_prim_path
        self._articulation = None
        self._dof_name_to_index: Dict[str, int] = {}
        self._arm1_indices: List[int] = []
        self._arm2_indices: List[int] = []
        self._gripper1_indices: List[int] = []
        self._gripper2_indices: List[int] = []
        self._ee_paths: Dict[str, Optional[str]] = {"arm1": None, "arm2": None}
        self._gripper_finger_paths: Dict[str, List[str]] = {"arm1": [], "arm2": []}
        self._kinematics: Dict[str, ArmKinematics] = {}
        self._drive_warned = False

    @property
    def is_ready(self) -> bool:
        return _articulation_ready(self._articulation)

    def validate_ready(self) -> bool:
        return _articulation_ready(self._articulation)

    def bind_articulation(self, articulation) -> bool:
        """绑定 World.scene 已注册的 SingleArticulation。"""
        self._articulation = articulation
        if not _articulation_ready(self._articulation):
            print("NovaRobotRuntime: bind_articulation failed (not ready)")
            self._articulation = None
            return False
        return self._index_dof_and_ee()

    def initialize(self) -> bool:
        """回退：SceneLoader 未注册时自建 articulation。"""
        try:
            from isaacsim.core.prims import SingleArticulation

            candidates = (ROBOT_ROOT_JOINT_PATH, ROBOT_PRIM_PATH)
            last_exc = None
            for prim_path in candidates:
                try:
                    art = SingleArticulation(prim_path=prim_path, name="nova_robot_grasp")
                    art.initialize()
                    if _articulation_ready(art):
                        print(f"NovaRobotRuntime: standalone articulation @ {prim_path}")
                        return self.bind_articulation(art)
                except Exception as exc:
                    last_exc = exc
            if last_exc:
                print(f"NovaRobotRuntime.initialize failed: {last_exc}")
            return False
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"NovaRobotRuntime.initialize failed: {exc}")
            return False

    def _index_dof_and_ee(self) -> bool:
        if not self._articulation:
            return False
        try:
            names = list(self._articulation.dof_names or [])
            self._dof_name_to_index = {n: i for i, n in enumerate(names)}
            self._arm1_indices = [self._dof_name_to_index[n] for n in ARM1_JOINTS if n in self._dof_name_to_index]
            self._arm2_indices = [self._dof_name_to_index[n] for n in ARM2_JOINTS if n in self._dof_name_to_index]
            self._gripper1_indices = [
                self._dof_name_to_index[n] for n in ARM1_GRIPPER_JOINTS if n in self._dof_name_to_index
            ]
            self._gripper2_indices = [
                self._dof_name_to_index[n] for n in ARM2_GRIPPER_JOINTS if n in self._dof_name_to_index
            ]
            self._ee_paths["arm1"] = self._find_link_prim_path(ARM1_EE_LINK)
            self._ee_paths["arm2"] = self._find_link_prim_path(ARM2_EE_LINK)
            self._gripper_finger_paths["arm1"] = [
                p for n in ARM1_GRIPPER_FINGERS if (p := self._find_link_prim_path(n))
            ]
            self._gripper_finger_paths["arm2"] = [
                p for n in ARM2_GRIPPER_FINGERS if (p := self._find_link_prim_path(n))
            ]
            print(
                f"NovaRobotRuntime: ready dofs={len(names)} "
                f"arm1={len(self._arm1_indices)} arm2={len(self._arm2_indices)} "
                f"ee1={self._ee_paths['arm1']} ee2={self._ee_paths['arm2']} "
                f"grip1={self._gripper_finger_paths['arm1']} grip2={self._gripper_finger_paths['arm2']}"
            )
            self._load_kinematics()
            return bool(self._arm1_indices or self._arm2_indices)
        except Exception as exc:
            print(f"NovaRobotRuntime._index_dof_and_ee failed: {exc}")
            return False

    def shutdown(self) -> None:
        self._articulation = None
        self._dof_name_to_index.clear()
        self._arm1_indices.clear()
        self._arm2_indices.clear()
        self._kinematics.clear()
        self._drive_warned = False

    def _load_kinematics(self) -> None:
        self._kinematics.clear()
        try:
            from ...paths import resolve_robot_urdf
            from .arm_kinematics import load_arm_chains

            urdf = resolve_robot_urdf()
            if not urdf:
                print("NovaRobotRuntime: URDF not found — numerical IK disabled")
                return
            for arm, chain in load_arm_chains(urdf).items():
                self._kinematics[arm] = ArmKinematics(chain)
            print(f"NovaRobotRuntime: URDF numerical IK ready ({urdf})")
        except Exception as exc:
            print(f"NovaRobotRuntime: URDF IK init failed: {exc}")

    def log_diagnostics(self) -> None:
        if not self._articulation:
            print("NovaRobotRuntime: diagnostics — articulation is None")
            return
        names = list(self._articulation.dof_names or [])
        pos = self._articulation.get_joint_positions()
        pos_str = "None"
        if pos is not None:
            pos_str = ", ".join(f"{n}={float(pos[i]):.3f}" for i, n in enumerate(names[:8]))
            if len(names) > 8:
                pos_str += ", ..."
        print(
            f"NovaRobotRuntime: diagnostics dofs={len(names)} "
            f"arm1_idx={self._arm1_indices} arm2_idx={self._arm2_indices} "
            f"ee1={self._ee_paths.get('arm1')} ee2={self._ee_paths.get('arm2')}"
        )
        print(f"NovaRobotRuntime: joint sample [{pos_str}]")

    def choose_arm(self, target_world: Sequence[float], mode: str = "auto") -> str:
        """根据目标 x 坐标或 UI 选择 arm1 / arm2。"""
        if mode in ("arm1", "arm2"):
            tx = float(target_world[0])
            if mode == "arm1" and tx > 0.75:
                print(
                    f"NovaRobotRuntime: WARN arm1 forced but target x={tx:.3f} "
                    f"is on arm2 side — consider auto/arm2"
                )
            elif mode == "arm2" and tx < 0.35:
                print(
                    f"NovaRobotRuntime: WARN arm2 forced but target x={tx:.3f} "
                    f"is on arm1 side — consider auto/arm1"
                )
            return mode
        # J2 在 +x 侧（约 1.06m），J1 在 x≈0
        return "arm2" if float(target_world[0]) > 0.40 else "arm1"

    def hold_current_configuration(self) -> None:
        """把当前关节角写回 PD 目标，防止 physics reset 后臂段塌落。"""
        if not self._articulation:
            return
        cur = self._full_joint_positions()
        if cur is None:
            return
        all_indices = list(range(len(cur)))
        self._apply_joint_targets(cur, all_indices)

    def apply_joint_command(self, joint_names: Sequence[str], positions: Sequence[float]) -> bool:
        """应用外部 ``/joint_command``：按关节名更新 PD 目标。"""
        if not self._articulation:
            return False
        all_pos = self._full_joint_positions()
        if all_pos is None:
            return False
        indices = []
        for name, pos in zip(joint_names, positions):
            idx = self._dof_name_to_index.get(str(name))
            if idx is None:
                continue
            all_pos[idx] = float(pos)
            indices.append(idx)
        if not indices:
            return False
        self._apply_joint_targets(all_pos, sorted(set(indices)))
        return True

    async def hold_current_configuration_async(self, frames: int = 8) -> None:
        """连续多帧重发关节目标，等 PD 重新抓住。"""
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(max(1, frames)):
            self.hold_current_configuration()
            await app.next_update_async()

    def _full_joint_positions(self) -> Optional[np.ndarray]:
        if not self._articulation:
            return None
        positions = self._articulation.get_joint_positions()
        if positions is None:
            return None
        return np.array(positions, dtype=np.float64)

    def _arm_base_path(self, arm: str) -> str:
        return f"{ROBOT_PRIM_PATH}/{'J2_1' if arm == 'arm2' else 'J1_1'}"

    def _arm_indices(self, arm: str) -> List[int]:
        return self._arm2_indices if arm == "arm2" else self._arm1_indices

    def _arm_seed_q(self, arm: str) -> np.ndarray:
        indices = self._arm_indices(arm)
        current = self._full_joint_positions()
        if current is None or not indices:
            return np.zeros(len(indices), dtype=np.float64)
        return np.array([float(current[i]) for i in indices], dtype=np.float64)

    def _base_link_to_world_pos(self, p_base: np.ndarray) -> Optional[np.ndarray]:
        from pxr import Gf

        from ..pose_utils import get_prim_world_matrix

        mat = get_prim_world_matrix(BASE_LINK_PATH)
        if mat is None:
            return None
        p = mat.Transform(Gf.Vec3d(float(p_base[0]), float(p_base[1]), float(p_base[2])))
        return np.array([float(p[0]), float(p[1]), float(p[2])], dtype=np.float64)

    def _get_tcp_world(self, arm: str) -> Optional[np.ndarray]:
        """URDF FK 的 TCP 世界坐标。"""
        kin = self._kinematics.get(arm)
        if kin is None:
            return None
        p_base, _ = kin.fk_tcp(self._arm_seed_q(arm))
        return self._base_link_to_world_pos(p_base)

    def _gripper_target_to_ik_world(self, arm: str, gripper_target: np.ndarray) -> np.ndarray:
        """把「夹爪中心目标」换算成 IK 应跟踪的 TCP 世界坐标（补偿仿真与 URDF 偏差）。"""
        grip = self._get_gripper_center_world(arm)
        tcp = self._get_tcp_world(arm)
        if grip is None or tcp is None:
            return np.array(gripper_target, dtype=np.float64)
        offset = tcp - grip
        ik_world = np.array(gripper_target, dtype=np.float64) + offset
        return ik_world

    def log_alignment(self, arm: str, label: str, target_gripper: np.ndarray) -> None:
        """打印夹爪 / TCP / 盒子 / 目标 对齐诊断。"""
        from ..pose_utils import get_prim_world_matrix, matrix_to_translation_quat

        grip = self._get_gripper_center_world(arm)
        tcp = self._get_tcp_world(arm)
        wrist = None
        ee_path = self._ee_paths.get(arm)
        if ee_path:
            wrist = self._get_link_world_position(ee_path)
        box_t = None
        mat = get_prim_world_matrix(BOX_LINK_PATH)
        if mat is not None:
            box_t, _ = matrix_to_translation_quat(mat)
        parts = [f"NovaRobotRuntime: align [{label}] arm={arm}"]
        parts.append(
            f" target_grip=({target_gripper[0]:.3f},{target_gripper[1]:.3f},{target_gripper[2]:.3f})"
        )
        if grip is not None:
            d = float(np.linalg.norm(grip - target_gripper))
            parts.append(f" grip=({grip[0]:.3f},{grip[1]:.3f},{grip[2]:.3f}) err={d:.3f}m")
        if tcp is not None:
            parts.append(f" tcp_fk=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f})")
        if wrist is not None:
            parts.append(f" wrist=({wrist[0]:.3f},{wrist[1]:.3f},{wrist[2]:.3f})")
        if box_t is not None:
            parts.append(f" grasp_box=({box_t[0]:.3f},{box_t[1]:.3f},{box_t[2]:.3f})")
            if grip is not None:
                db = float(np.linalg.norm(grip - np.array(box_t, dtype=np.float64)))
                parts.append(f" |grip-box|={db:.3f}m")
        print("".join(parts))

    @staticmethod
    def passive_arm(active_arm: str) -> str:
        return "arm2" if active_arm == "arm1" else "arm1"

    def get_arm_joint_positions_rad(self, arm: str) -> Optional[List[float]]:
        """读取指定臂 J1..J6 当前关节角（弧度）。"""
        indices = self._arm_indices(arm)
        current = self._full_joint_positions()
        if current is None or not indices:
            return None
        return [float(current[i]) for i in indices]

    @staticmethod
    def _rotmat_to_rpy_deg(rot: np.ndarray) -> Tuple[float, float, float]:
        from pxr import Gf

        # Gf.Rotation 不能从 Matrix3d 直接构造；经 Matrix4d.ExtractRotation 分解。
        m4 = Gf.Matrix4d(1.0)
        for i in range(3):
            for j in range(3):
                m4[i][j] = float(rot[i, j])
        euler = m4.ExtractRotation().Decompose(
            Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis()
        )
        return float(euler[0]), float(euler[1]), float(euler[2])

    def get_live_arm_status(self) -> Dict[str, dict]:
        """供 UI 实时显示：夹爪 world/base_link 位姿与关节角。"""
        if not self.is_ready:
            return {}
        out: Dict[str, dict] = {}
        for arm in ("arm1", "arm2"):
            joints = self.get_arm_joint_positions_rad(arm)
            if joints is None:
                from ..pose_utils import read_arm_ee_from_stage

                stage_data = read_arm_ee_from_stage(arm)
                if stage_data:
                    out[arm] = stage_data
                continue
            import math

            grip_w = self._get_gripper_center_world(arm)
            grip_base = self._world_to_base_link_pos(grip_w) if grip_w is not None else None
            rpy_deg = None
            kin = self._kinematics.get(arm)
            if kin is not None:
                _, rot = kin.fk_tcp(joints)
                rpy_deg = self._rotmat_to_rpy_deg(rot)
            out[arm] = {
                "joints_deg": [math.degrees(j) for j in joints],
                "gripper_world": grip_w.tolist() if grip_w is not None else None,
                "gripper_base": grip_base.tolist() if grip_base is not None else None,
                "tcp_rpy_deg": rpy_deg,
            }
        return out

    async def move_arm_to_joints_async(
        self,
        arm: str,
        target_joints: Sequence[float],
        *,
        max_steps: int = 260,
        tol_rad: float = 0.028,
        open_gripper: Optional[bool] = True,
    ) -> bool:
        """分帧驱动单臂到目标关节角（用于复位 / 避让）。"""
        import omni.kit.app

        if not self._articulation:
            return False
        arm_indices = self._arm_indices(arm)
        if not arm_indices or len(target_joints) < len(arm_indices):
            print(f"NovaRobotRuntime: move_arm_to_joints missing arm={arm}")
            return False
        if open_gripper is not None:
            self.set_gripper(arm, open_gripper=bool(open_gripper))

        targets = [float(target_joints[i]) for i in range(len(arm_indices))]
        app = omni.kit.app.get_app()
        for step in range(max_steps):
            current = self._full_joint_positions()
            if current is None:
                return False
            errs = [abs(float(current[arm_indices[i]]) - targets[i]) for i in range(len(arm_indices))]
            if max(errs) < tol_rad:
                print(f"NovaRobotRuntime: {arm} joints reached in {step} steps max_err={max(errs):.4f}rad")
                return True
            next_pos = current.copy()
            for i, idx in enumerate(arm_indices):
                delta = float(np.clip(targets[i] - current[idx], -MAX_JOINT_STEP_RAD, MAX_JOINT_STEP_RAD))
                next_pos[idx] = current[idx] + delta
            self._apply_joint_targets(next_pos, arm_indices)
            await app.next_update_async()

        current = self._full_joint_positions()
        if current is None:
            return False
        errs = [abs(float(current[arm_indices[i]]) - targets[i]) for i in range(len(arm_indices))]
        ok = max(errs) < tol_rad * 2.5
        if not ok:
            print(f"NovaRobotRuntime: {arm} joint move short max_err={max(errs):.4f}rad")
        return ok

    async def reset_arms_to_poses_async(
        self,
        arm1_pose,
        arm2_pose,
    ) -> bool:
        """双臂同时 IK 到 base_link 下复位 xyzrpy，夹爪张开。"""
        import asyncio

        from ..pose_utils import Pose6D

        self.set_gripper("arm1", open_gripper=True)
        self.set_gripper("arm2", open_gripper=True)
        self._drive_warned = False
        ok1, ok2 = await asyncio.gather(
            self.move_arm_to_base_xyzrpy_async("arm1", arm1_pose),
            self.move_arm_to_base_xyzrpy_async("arm2", arm2_pose),
        )
        print(f"NovaRobotRuntime: reset poses ok1={ok1} ok2={ok2}")
        return bool(ok1 and ok2)

    async def move_arm_to_base_xyzrpy_async(self, arm: str, pose) -> bool:
        """base_link 下夹爪 xyzrpy → world IK 跟踪。"""
        from ..pose_utils import Pose6D, base_link_pose_to_world

        if not isinstance(pose, Pose6D):
            return False
        world_pose = base_link_pose_to_world(pose, base_link_path=BASE_LINK_PATH)
        if world_pose is None:
            print(f"NovaRobotRuntime: {arm} reset pose transform to world failed")
            return False
        target_t, target_q = world_pose
        print(
            f"NovaRobotRuntime: {arm} reset base=({pose.translation[0]:.3f},{pose.translation[1]:.3f},"
            f"{pose.translation[2]:.3f}) rpy=({pose.rotation_deg[0]:.1f},{pose.rotation_deg[1]:.1f},"
            f"{pose.rotation_deg[2]:.1f})°"
        )
        return await self.move_arm_to_pose_async(
            arm,
            target_t,
            target_q,
            use_orientation=True,
            top_down=False,
        ) or await self.move_arm_to_pose_async(
            arm,
            target_t,
            None,
            use_orientation=False,
            top_down=True,
        )

    async def reset_arms_async(
        self,
        arm1_joints: Sequence[float],
        arm2_joints: Sequence[float],
    ) -> bool:
        """双臂同时回到复位关节角，夹爪张开。"""
        import asyncio

        self.set_gripper("arm1", open_gripper=True)
        self.set_gripper("arm2", open_gripper=True)
        self._drive_warned = False
        ok1, ok2 = await asyncio.gather(
            self.move_arm_to_joints_async("arm1", arm1_joints, open_gripper=True),
            self.move_arm_to_joints_async("arm2", arm2_joints, open_gripper=True),
        )
        print(f"NovaRobotRuntime: reset arms ok1={ok1} ok2={ok2}")
        return bool(ok1 and ok2)

    async def park_passive_arm_async(self, active_arm: str) -> bool:
        """非工作臂沿 X 退避到预设 park 关节角。"""
        passive = self.passive_arm(active_arm)
        park = DEFAULT_ARM2_PARK_JOINTS if passive == "arm2" else DEFAULT_ARM1_PARK_JOINTS
        print(f"NovaRobotRuntime: park {passive} (active={active_arm}) joints={park}")
        ok = await self.move_arm_to_joints_async(passive, park, open_gripper=True)
        grip = self._get_gripper_center_world(passive)
        if grip is not None:
            print(
                f"NovaRobotRuntime: {passive} parked gripper=({grip[0]:.3f},{grip[1]:.3f},{grip[2]:.3f})"
            )
        return ok

    def _world_to_base_link_pos(self, world_pos: Sequence[float]) -> Optional[np.ndarray]:
        from pxr import Gf

        from ..pose_utils import get_prim_world_matrix

        mat = get_prim_world_matrix(BASE_LINK_PATH)
        if mat is None:
            return None
        p = mat.GetInverse().Transform(
            Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
        )
        return np.array([float(p[0]), float(p[1]), float(p[2])], dtype=np.float64)

    def _world_rot_to_base_link(self, rot_world: np.ndarray) -> Optional[np.ndarray]:
        from ..pose_utils import get_prim_world_matrix

        mat = get_prim_world_matrix(BASE_LINK_PATH)
        if mat is None:
            return None
        r_bw = np.array(
            [[mat[0, 0], mat[0, 1], mat[0, 2]], [mat[1, 0], mat[1, 1], mat[1, 2]], [mat[2, 0], mat[2, 1], mat[2, 2]]],
            dtype=np.float64,
        )
        return r_bw.T @ rot_world

    def _solve_arm_ik(
        self,
        arm: str,
        world_pos: Sequence[float],
        seed: Sequence[float],
        *,
        world_quat: Optional[Sequence[float]] = None,
        top_down: bool = False,
        verbose: bool = False,
    ) -> Optional[np.ndarray]:
        kin = self._kinematics.get(arm)
        if kin is None:
            return None
        p_base = self._world_to_base_link_pos(world_pos)
        if p_base is None:
            return None

        if top_down:
            q, pos_err = kin.ik_position(p_base, seed)
            if verbose:
                if q is not None:
                    print(f"NovaRobotRuntime: {arm} IK top-down (pos-only) ok err={pos_err:.4f}m")
                else:
                    print(f"NovaRobotRuntime: {arm} IK top-down (pos-only) failed err={pos_err:.4f}m")
            return q

        if world_quat is not None:
            r_base = self._world_rot_to_base_link(quat_xyzw_to_matrix(world_quat))
            if r_base is None:
                return None
            q, pos_err, rot_err = kin.ik_pose(p_base, r_base, seed)
            if verbose:
                if q is not None:
                    print(
                        f"NovaRobotRuntime: {arm} IK 6D ok pos_err={pos_err:.4f}m rot_err={rot_err:.4f}"
                    )
                else:
                    print(f"NovaRobotRuntime: {arm} IK 6D failed pos_err={pos_err:.4f}m")
            return q

        q, pos_err = kin.ik_position(p_base, seed)
        if verbose:
            if q is not None:
                print(f"NovaRobotRuntime: {arm} IK position ok err={pos_err:.4f}m")
            else:
                print(f"NovaRobotRuntime: {arm} IK position failed err={pos_err:.4f}m")
        return q

    def _log_tcp_diagnostics(self, arm: str, target_world: np.ndarray, step: int) -> None:
        grip = self._get_gripper_center_world(arm)
        if grip is None:
            return
        dist = float(np.linalg.norm(target_world - grip))
        kin = self._kinematics.get(arm)
        fk_note = ""
        if kin is not None:
            p_base, _ = kin.fk_tcp(self._arm_seed_q(arm))
            fk_note = f" fk_base=({p_base[0]:.3f},{p_base[1]:.3f},{p_base[2]:.3f})"
        print(
            f"NovaRobotRuntime: {arm} diag step={step} gripper=({grip[0]:.3f},{grip[1]:.3f},{grip[2]:.3f}) "
            f"dist={dist:.3f}m{fk_note}"
        )

    async def move_arm_to_pose_async(
        self,
        arm: str,
        world_pos: Sequence[float],
        world_quat: Optional[Sequence[float]] = None,
        *,
        use_orientation: bool = False,
        top_down: bool = False,
        max_steps: int = 300,
        position_tol: float = 0.04,
    ) -> bool:
        """URDF 数值 IK + 分帧 PD 跟踪（失败时回退启发式）。"""
        import omni.kit.app

        if not self._articulation:
            return False
        arm_indices = self._arm_indices(arm)
        if not arm_indices:
            return False

        target = np.array(world_pos, dtype=np.float64)
        ik_target = self._gripper_target_to_ik_world(arm, target)
        app = omni.kit.app.get_app()
        self.log_alignment(arm, "move_start", target)
        if float(np.linalg.norm(ik_target - target)) > 1e-4:
            print(
                f"NovaRobotRuntime: {arm} IK world target adjusted "
                f"({ik_target[0]:.3f},{ik_target[1]:.3f},{ik_target[2]:.3f}) "
                f"for gripper ({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})"
            )
        grip0 = self._get_gripper_center_world(arm)
        if grip0 is not None:
            print(
                f"NovaRobotRuntime: {arm} move_to_pose gripper_target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
                f"start_dist={float(np.linalg.norm(target - grip0)):.3f}m "
                f"mode={'6D' if use_orientation else 'top-down' if top_down else 'pos'}"
            )

        ik_ok_once = False
        for step in range(max_steps):
            grip = self._get_gripper_center_world(arm)
            if grip is not None and float(np.linalg.norm(target - grip)) < position_tol:
                print(f"NovaRobotRuntime: {arm} pose reached in {step} steps")
                self.log_alignment(arm, "move_done", target)
                return True

            current = self._full_joint_positions()
            if current is None:
                return False
            seed = self._arm_seed_q(arm)
            ik_target = self._gripper_target_to_ik_world(arm, target)
            q_ik = self._solve_arm_ik(
                arm,
                ik_target,
                seed,
                world_quat=world_quat if use_orientation else None,
                top_down=top_down and not use_orientation,
                verbose=(step == 0),
            )
            if q_ik is not None:
                ik_ok_once = True
                next_pos = current.copy()
                for j, idx in enumerate(arm_indices):
                    delta = float(np.clip(float(q_ik[j]) - float(current[idx]), -MAX_JOINT_STEP_RAD, MAX_JOINT_STEP_RAD))
                    next_pos[idx] = float(current[idx]) + delta
                self._apply_joint_targets(next_pos, arm_indices)
            elif step == 0:
                print(f"NovaRobotRuntime: {arm} URDF IK failed at start — heuristic fallback")
                return await self._move_arm_heuristic_async(
                    arm, world_pos, world_quat, max_steps=max_steps, position_tol=position_tol
                )

            if step % 40 == 0:
                self._log_tcp_diagnostics(arm, target, step)
            await app.next_update_async()

        grip_f = self._get_gripper_center_world(arm)
        if grip_f is None:
            return ik_ok_once
        ok = float(np.linalg.norm(target - grip_f)) < position_tol * 2.5
        if not ok:
            print(
                f"NovaRobotRuntime: {arm} pose stopped short dist="
                f"{float(np.linalg.norm(target - grip_f)):.3f}m"
            )
        self.log_alignment(arm, "move_stop", target)
        return ok

    def _seed_ik_from_geometry(
        self, arm: str, target: np.ndarray, positions: np.ndarray, arm_indices: List[int]
    ) -> np.ndarray:
        """粗瞄准：肩肘伸向目标，腕部预置为顶抓（夹爪朝下）。"""
        base = self._get_link_world_position(self._arm_base_path(arm))
        if base is None or len(arm_indices) < 2:
            return positions
        dx = float(target[0] - base[0])
        dy = float(target[1] - base[1])
        dz = float(target[2] - base[2])
        horiz = max(1e-4, float(np.hypot(dx, dy)))
        yaw = float(np.arctan2(dy, dx))
        if len(arm_indices) >= 1:
            positions[arm_indices[0]] = yaw
        if len(arm_indices) >= 2:
            positions[arm_indices[1]] = float(np.clip(np.arctan2(dz, horiz), -0.35, 1.05))
        if len(arm_indices) >= 3:
            positions[arm_indices[2]] = float(np.clip(horiz * 0.38, 0.15, 0.88))
        if len(arm_indices) >= 4:
            positions[arm_indices[3]] = WRIST_DOWN_J4_RAD
        if len(arm_indices) >= 5:
            positions[arm_indices[4]] = WRIST_DOWN_J5_RAD
        if len(arm_indices) >= 6:
            positions[arm_indices[5]] = WRIST_DOWN_J6_RAD
        return positions

    def _apply_joint_targets(self, all_pos: np.ndarray, arm_indices: List[int]) -> None:
        """仅 PD 目标驱动（apply_action），禁止 set_joint_positions 瞬移（会穿模）。"""
        if not self._articulation or not arm_indices:
            return
        try:
            from isaacsim.core.utils.types import ArticulationAction

            idx_arr = np.array(arm_indices, dtype=np.int32)
            targets = np.array([float(all_pos[i]) for i in arm_indices], dtype=np.float32)
            self._articulation.apply_action(
                ArticulationAction(joint_positions=targets, joint_indices=idx_arr)
            )
        except Exception as exc:
            if not self._drive_warned:
                self._drive_warned = True
                print(f"NovaRobotRuntime: apply_action failed: {exc}")

    def _clamp_joint_step(
        self,
        current: np.ndarray,
        desired: np.ndarray,
        arm_indices: List[int],
        max_step: float,
    ) -> np.ndarray:
        out = current.copy()
        for idx in arm_indices:
            delta = float(np.clip(desired[idx] - current[idx], -max_step, max_step))
            out[idx] = current[idx] + delta
        return out

    def _nudge_from_ee_error(
        self,
        positions: np.ndarray,
        arm_indices: List[int],
        ee_pos: np.ndarray,
        target: np.ndarray,
    ) -> np.ndarray:
        """根据末端误差迭代修正关节（每帧小步，朝目标收敛）。"""
        err = target - ee_pos
        dist = float(np.linalg.norm(err))
        if dist < 1e-6:
            return positions.copy()
        gain = min(1.35, 0.25 + 0.55 * dist)
        out = positions.copy()
        if len(arm_indices) >= 1:
            out[arm_indices[0]] += float(np.clip(gain * (0.42 * err[0] + 0.30 * err[1]), -0.09, 0.09))
        if len(arm_indices) >= 2:
            out[arm_indices[1]] += float(np.clip(gain * 0.50 * err[2], -0.09, 0.09))
        if len(arm_indices) >= 3:
            reach = float(np.hypot(err[0], err[1]))
            out[arm_indices[2]] += float(np.clip(gain * 0.22 * (reach - 0.22), -0.07, 0.07))
        if len(arm_indices) >= 4:
            out[arm_indices[3]] += float(np.clip(gain * (-0.18 * err[2]), -0.06, 0.06))
        if len(arm_indices) >= 5:
            out[arm_indices[4]] += float(np.clip(gain * 0.12 * err[0], -0.05, 0.05))
        if len(arm_indices) >= 6:
            out[arm_indices[5]] += float(np.clip(gain * 0.10 * err[1], -0.05, 0.05))
        return out

    def _drive_arm_joints(self, positions: np.ndarray, arm_indices: List[int]) -> None:
        if not self._articulation or not arm_indices:
            return
        all_pos = self._full_joint_positions()
        if all_pos is None:
            if not self._drive_warned:
                self._drive_warned = True
                print("NovaRobotRuntime: WARN get_joint_positions returned None")
            return
        before = all_pos.copy()
        for idx in arm_indices:
            all_pos[idx] = float(positions[idx])
        self._apply_joint_targets(all_pos, arm_indices)
        if not self._drive_warned and np.max(np.abs(all_pos - before)) > 1e-5:
            print(
                f"NovaRobotRuntime: PD targets {arm_indices[:4]} "
                f"-> {[f'{all_pos[i]:.2f}' for i in arm_indices[:4]]}"
            )
            self._drive_warned = True

    def _apply_joint_positions(self, positions: np.ndarray, indices: Sequence[int]) -> None:
        self._drive_arm_joints(positions, list(indices))

    def set_gripper(self, arm: str, open_gripper: bool) -> None:
        if not self._articulation:
            return
        if arm == "arm2":
            targets = GRIPPER2_OPEN if open_gripper else GRIPPER2_CLOSED
            indices = self._gripper2_indices
        else:
            targets = GRIPPER1_OPEN if open_gripper else GRIPPER1_CLOSED
            indices = self._gripper1_indices
        if not indices:
            return
        cur = self._full_joint_positions()
        if cur is None:
            return
        for i, idx in enumerate(indices):
            if i < len(targets):
                cur[idx] = float(targets[i])
        self._apply_joint_targets(cur, list(indices))

    async def hold_gripper_async(self, arm: str, open_gripper: bool, frames: int = 12) -> None:
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(max(1, frames)):
            self.set_gripper(arm, open_gripper=open_gripper)
            await app.next_update_async()

    def _get_gripper_center_world(self, arm: str) -> Optional[np.ndarray]:
        """两指中点作为抓取末端（比腕关节 J*_6 更贴近真实夹取点）。"""
        fingers = self._gripper_finger_paths.get(arm) or []
        pts = []
        for path in fingers:
            p = self._get_link_world_position(path)
            if p is not None:
                pts.append(p)
        if pts:
            return np.mean(np.stack(pts, axis=0), axis=0)
        ee_path = self._ee_paths.get(arm)
        if ee_path:
            return self._get_link_world_position(ee_path)
        return None

    def _get_tracking_position(self, arm: str, use_gripper_center: bool) -> Optional[np.ndarray]:
        if use_gripper_center:
            return self._get_gripper_center_world(arm)
        ee_path = self._ee_paths.get(arm)
        if not ee_path:
            return None
        return self._get_link_world_position(ee_path)

    async def _move_arm_heuristic_async(
        self,
        arm: str,
        target_position: Sequence[float],
        target_quat_xyzw: Optional[Sequence[float]] = None,
        max_steps: int = 280,
        position_tol: float = 0.045,
        use_gripper_center: bool = True,
    ) -> bool:
        """分帧 PD 驱动末端朝目标移动（默认跟踪夹爪中心，顶抓姿态）。"""
        import omni.kit.app

        if not self._articulation:
            return False
        arm_indices = self._arm2_indices if arm == "arm2" else self._arm1_indices
        if not arm_indices:
            print(f"NovaRobotRuntime: move_arm_toward missing arm={arm} indices")
            return False

        target = np.array(target_position, dtype=np.float64)
        app = omni.kit.app.get_app()
        ee0 = self._get_tracking_position(arm, use_gripper_center)
        ee_label = "gripper" if use_gripper_center else "wrist"
        if ee0 is not None:
            dist0 = float(np.linalg.norm(target - ee0))
            print(
                f"NovaRobotRuntime: {arm} move start {ee_label}=({ee0[0]:.3f},{ee0[1]:.3f},{ee0[2]:.3f}) "
                f"target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) dist={dist0:.3f}m "
                f"(top-down PD)"
            )
        _ = target_quat_xyzw

        seeded = False
        for step in range(max_steps):
            ee_pos = self._get_tracking_position(arm, use_gripper_center)
            if ee_pos is None:
                print(f"NovaRobotRuntime: {arm} ee invalid @ step {step}")
                return False
            err = target - ee_pos
            dist = float(np.linalg.norm(err))
            if dist < position_tol:
                print(f"NovaRobotRuntime: {arm} reached target in {step} steps (dist={dist:.4f})")
                return True
            if step % 40 == 0:
                print(
                    f"NovaRobotRuntime: {arm} step={step} {ee_label}=({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}) "
                    f"dist={dist:.3f}m"
                )

            current = self._full_joint_positions()
            if current is None:
                return False
            if not seeded:
                geom = self._seed_ik_from_geometry(arm, target, current.copy(), arm_indices)
                toward = self._clamp_joint_step(current, geom, arm_indices, MAX_JOINT_STEP_RAD * 1.8)
                seeded = True
            else:
                toward = current.copy()
            nudged = self._nudge_from_ee_error(toward, arm_indices, ee_pos, target)
            next_pos = self._clamp_joint_step(current, nudged, arm_indices, MAX_JOINT_STEP_RAD)
            self._apply_joint_targets(next_pos, arm_indices)
            await app.next_update_async()

        ee_final = self._get_tracking_position(arm, use_gripper_center)
        if ee_final is None:
            return False
        ok = float(np.linalg.norm(target - ee_final)) < position_tol * 2.5
        if not ok:
            dist = float(np.linalg.norm(target - ee_final))
            print(f"NovaRobotRuntime: {arm} IK stopped short dist={dist:.3f}m target={target.tolist()}")
        return ok

    async def move_arm_toward_async(
        self,
        arm: str,
        target_position: Sequence[float],
        target_quat_xyzw: Optional[Sequence[float]] = None,
        max_steps: int = 280,
        position_tol: float = 0.045,
        use_gripper_center: bool = True,
    ) -> bool:
        """兼容旧调用：委托 ``move_arm_to_pose_async``。"""
        _ = use_gripper_center
        return await self.move_arm_to_pose_async(
            arm,
            target_position,
            target_quat_xyzw,
            use_orientation=target_quat_xyzw is not None,
            top_down=target_quat_xyzw is None,
            max_steps=max_steps,
            position_tol=position_tol,
        )

    def move_arm_toward(
        self,
        arm: str,
        target_position: Sequence[float],
        target_quat_xyzw: Optional[Sequence[float]] = None,
        steps: int = 50,
        position_tol: float = 0.025,
    ) -> bool:
        """同步版（兼容）；抓取流程请用 move_arm_toward_async。"""
        if not self._articulation:
            return False
        arm_indices = self._arm2_indices if arm == "arm2" else self._arm1_indices
        ee_path = self._ee_paths.get(arm)
        if not arm_indices or not ee_path:
            return False

        target = np.array(target_position, dtype=np.float64)
        seeded = False
        for step in range(steps):
            ee_pos = self._get_link_world_position(ee_path)
            if ee_pos is None:
                return False
            if float(np.linalg.norm(target - ee_pos)) < position_tol:
                return True
            current = self._full_joint_positions()
            if current is None:
                return False
            if not seeded:
                geom = self._seed_ik_from_geometry(arm, target, current.copy(), arm_indices)
                toward = self._clamp_joint_step(current, geom, arm_indices, MAX_JOINT_STEP_RAD * 1.8)
                seeded = True
            else:
                toward = current.copy()
            nudged = self._nudge_from_ee_error(toward, arm_indices, ee_pos, target)
            next_pos = self._clamp_joint_step(current, nudged, arm_indices, MAX_JOINT_STEP_RAD)
            self._apply_joint_targets(next_pos, arm_indices)
        ee_final = self._get_link_world_position(ee_path)
        if ee_final is None:
            return False
        return float(np.linalg.norm(target - ee_final)) < position_tol * 2.5

    def _get_link_world_position(self, link_path: str) -> Optional[np.ndarray]:
        try:
            from pxr import UsdGeom
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return None
            prim = stage.GetPrimAtPath(link_path)
            if not prim or not prim.IsValid():
                return None
            mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            return np.array(mat.ExtractTranslation(), dtype=np.float64)
        except Exception:
            return None

    def _find_link_prim_path(self, link_name: str) -> Optional[str]:
        try:
            import omni.usd
            from pxr import UsdPhysics

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return None
            direct = f"{ROBOT_PRIM_PATH}/{link_name}"
            prim = stage.GetPrimAtPath(direct)
            if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return direct
            for prim in stage.Traverse():
                path = prim.GetPath().pathString
                if not path.startswith(ROBOT_PRIM_PATH):
                    continue
                if prim.GetName() != link_name:
                    continue
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    return path
            prim = stage.GetPrimAtPath(direct)
            if prim and prim.IsValid():
                return direct
            return None
        except Exception:
            return None
