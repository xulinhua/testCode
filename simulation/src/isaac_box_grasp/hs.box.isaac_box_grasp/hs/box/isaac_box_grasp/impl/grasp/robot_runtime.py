# -*- coding: utf-8 -*-
"""机器人运行时：关节诊断、差速轮、左臂启发式 IK、夹爪。"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ...defaults import (
    DEFAULT_WHEEL_BASE_M,
    DEFAULT_WHEEL_RADIUS_M,
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    LEFT_ARM_JOINTS,
    LEFT_EE_LINK_NAME,
    LEFT_FINGER_JOINTS,
    LEFT_WHEEL_JOINT,
    RIGHT_WHEEL_JOINT,
    ROBOT_PRIM_PATH,
)


class RobotRuntime:
    def __init__(
        self,
        robot_prim_path: str = ROBOT_PRIM_PATH,
        wheel_radius: float = DEFAULT_WHEEL_RADIUS_M,
        wheel_base: float = DEFAULT_WHEEL_BASE_M,
    ):
        self.robot_prim_path = robot_prim_path
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self._articulation = None
        self._diff_controller = None
        self._dof_name_to_index: Dict[str, int] = {}
        self._arm_dof_indices: List[int] = []
        self._finger_dof_indices: List[int] = []
        self._left_wheel_idx: Optional[int] = None
        self._right_wheel_idx: Optional[int] = None
        self._ee_prim_path: Optional[str] = None
        self._pending_wheel_vel: Tuple[float, float] = (0.0, 0.0)

    @property
    def is_ready(self) -> bool:
        return self._articulation is not None

    def initialize(self) -> bool:
        try:
            from isaacsim.core.prims import SingleArticulation
            from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
                DifferentialController,
            )

            self._articulation = SingleArticulation(self.robot_prim_path)
            self._articulation.initialize()
            if not self._articulation.is_physics_handle_valid():
                print(f"RobotRuntime: articulation invalid at {self.robot_prim_path}")
                self._articulation = None
                return False

            names = list(self._articulation.dof_names)
            self._dof_name_to_index = {n: i for i, n in enumerate(names)}
            self._arm_dof_indices = [
                self._dof_name_to_index[n] for n in LEFT_ARM_JOINTS if n in self._dof_name_to_index
            ]
            self._finger_dof_indices = [
                self._dof_name_to_index[n] for n in LEFT_FINGER_JOINTS if n in self._dof_name_to_index
            ]
            self._left_wheel_idx = self._dof_name_to_index.get(LEFT_WHEEL_JOINT)
            self._right_wheel_idx = self._dof_name_to_index.get(RIGHT_WHEEL_JOINT)
            self._ee_prim_path = self._find_link_prim_path(LEFT_EE_LINK_NAME)

            self._diff_controller = DifferentialController(
                name="box_grasp_diff",
                wheel_radius=self.wheel_radius,
                wheel_base=self.wheel_base,
            )
            print(f"RobotRuntime: ready dofs={len(names)} arm={len(self._arm_dof_indices)} ee={self._ee_prim_path}")
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"RobotRuntime.initialize failed: {exc}")
            self._articulation = None
            return False

    def shutdown(self) -> None:
        self._articulation = None
        self._diff_controller = None
        self._dof_name_to_index.clear()
        self._arm_dof_indices.clear()
        self._finger_dof_indices.clear()
        self._pending_wheel_vel = (0.0, 0.0)

    def log_diagnostics(self) -> None:
        if not self._articulation:
            print("RobotRuntime: (not initialized)")
            return
        names = list(self._articulation.dof_names)
        print(f"RobotRuntime diagnostics @ {self.robot_prim_path}")
        print(f"  dof count={len(names)}")
        for label, joint_list in (
            ("left_arm", LEFT_ARM_JOINTS),
            ("left_finger", LEFT_FINGER_JOINTS),
            ("wheels", (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT)),
        ):
            found = [n for n in joint_list if n in self._dof_name_to_index]
            missing = [n for n in joint_list if n not in self._dof_name_to_index]
            print(f"  {label}: found={found} missing={missing}")
        print(f"  ee_prim={self._ee_prim_path}")

    def set_cmd_vel(self, linear: float, angular: float) -> None:
        self._pending_wheel_vel = (float(linear), float(angular))

    def apply_wheel_velocities(self) -> None:
        if not self._articulation or self._diff_controller is None:
            return
        if self._left_wheel_idx is None or self._right_wheel_idx is None:
            return
        lin, ang = self._pending_wheel_vel
        action = self._diff_controller.forward(command=np.array([lin, ang]))
        vels = action.joint_velocities
        if vels is None or len(vels) < 2:
            return
        self._articulation.set_joint_velocity_targets(
            velocities=np.array([float(vels[0]), float(vels[1])], dtype=np.float32),
            joint_indices=np.array([self._left_wheel_idx, self._right_wheel_idx], dtype=np.int32),
        )

    def set_gripper(self, open_gripper: bool) -> None:
        if not self._articulation or not self._finger_dof_indices:
            return
        targets = GRIPPER_OPEN if open_gripper else GRIPPER_CLOSED
        positions = self._articulation.get_joint_positions()
        if positions is None:
            return
        positions = np.array(positions, dtype=np.float64)
        for i, idx in enumerate(self._finger_dof_indices):
            if i < len(targets):
                positions[idx] = targets[i]
        self._articulation.set_joint_positions(positions)

    def move_left_arm_toward(
        self,
        target_position: Sequence[float],
        target_quat_xyzw: Optional[Sequence[float]] = None,
        steps: int = 40,
        position_tol: float = 0.02,
    ) -> bool:
        if not self._articulation or not self._arm_dof_indices or not self._ee_prim_path:
            return False
        target = np.array(target_position, dtype=np.float64)
        for step in range(steps):
            ee_pos = self._get_link_world_position()
            if ee_pos is None:
                return False
            err = target - ee_pos
            if float(np.linalg.norm(err)) < position_tol:
                return True
            positions = self._articulation.get_joint_positions()
            if positions is None:
                return False
            positions = np.array(positions, dtype=np.float64)
            step_scale = 0.15 * max(0.3, 1.0 - step / float(steps))
            for idx in self._arm_dof_indices:
                delta = 1e-3
                base = float(positions[idx])
                positions[idx] = base + delta
                self._articulation.set_joint_positions(positions)
                pos_plus = self._get_link_world_position()
                positions[idx] = base - delta
                self._articulation.set_joint_positions(positions)
                pos_minus = self._get_link_world_position()
                positions[idx] = base
                if pos_plus is None or pos_minus is None:
                    continue
                grad = (pos_plus - pos_minus) / (2.0 * delta)
                positions[idx] = base + step_scale * float(np.dot(err, grad))
            self._articulation.set_joint_positions(positions)
        return float(np.linalg.norm(target - (self._get_link_world_position() or target))) < position_tol * 2

    def _get_link_world_position(self) -> Optional[np.ndarray]:
        if not self._ee_prim_path:
            return None
        try:
            from pxr import UsdGeom
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return None
            prim = stage.GetPrimAtPath(self._ee_prim_path)
            if not prim or not prim.IsValid():
                return None
            mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            return np.array(mat.ExtractTranslation(), dtype=np.float64)
        except Exception:
            return None

    def _find_link_prim_path(self, link_name: str) -> Optional[str]:
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return None
            root = stage.GetPrimAtPath(self.robot_prim_path)
            if not root:
                return None
            for prim in stage.Traverse():
                path = prim.GetPath().pathString
                if not path.startswith(self.robot_prim_path):
                    continue
                if prim.GetName() == link_name:
                    return path
            return None
        except Exception:
            return None
