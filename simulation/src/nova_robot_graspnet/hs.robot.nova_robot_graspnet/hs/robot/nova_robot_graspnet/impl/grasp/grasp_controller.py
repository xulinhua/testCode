# -*- coding: utf-8 -*-
"""双臂抓取控制器：盒心顶抓 / 外部夹爪 6D 位姿，URDF 数值 IK。"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import numpy as np

from ...global_variables import (
    ARM1_BASE_LINK,
    ARM1_BASE_PATH,
    ARM2_BASE_LINK,
    ARM2_BASE_PATH,
    BASE_LINK_PATH,
    BOX_LINK_PATH,
    BOX_POSE_FRAME,
    BOX_SIZE_Z,
    DEFAULT_ARM1_RESET_XYZRPY,
    DEFAULT_ARM2_RESET_XYZRPY,
)
from ..pose_utils import (
    Pose6D,
    get_box_world_pose,
    get_prim_world_matrix,
    matrix_to_translation_quat,
    read_box_pose_in_frame,
    transform_pose_between_frames,
    xyzrpy_tuple_to_pose6d,
)
from .interfaces import GraspResult, Transform6D
from .robot_runtime import NovaRobotRuntime


class NovaGraspController:
    """Scheme A 抓取执行器（URDF IK + PD 跟踪）。"""

    def __init__(self):
        self._runtime: Optional[NovaRobotRuntime] = None
        self._arm_mode = "auto"
        self._last_result: Optional[GraspResult] = None
        self._reset_before_motion = True
        self._arm1_reset_pose = xyzrpy_tuple_to_pose6d(DEFAULT_ARM1_RESET_XYZRPY)
        self._arm2_reset_pose = xyzrpy_tuple_to_pose6d(DEFAULT_ARM2_RESET_XYZRPY)

    def bind_runtime(self, runtime: Optional[NovaRobotRuntime]) -> None:
        self._runtime = runtime

    def set_arm_mode(self, mode: str) -> None:
        self._arm_mode = mode if mode in ("auto", "arm1", "arm2") else "auto"

    def set_reset_config(
        self,
        arm1_pose: Pose6D,
        arm2_pose: Pose6D,
        *,
        enabled: bool = True,
    ) -> None:
        self._arm1_reset_pose = arm1_pose
        self._arm2_reset_pose = arm2_pose
        self._reset_before_motion = bool(enabled)

    async def _prepare_arm_motion(self, active_arm: str) -> bool:
        """抓取 / 移动前：双臂复位，非工作臂 X 向避让。"""
        rt = self._runtime
        if rt is None or not rt.is_ready:
            return False
        if self._reset_before_motion:
            ok_reset = await rt.reset_arms_to_poses_async(
                self._arm1_reset_pose, self._arm2_reset_pose
            )
            if not ok_reset:
                print("NovaGraspController: WARN reset incomplete — continuing")
            await rt.hold_gripper_async("arm1", open_gripper=True, frames=4)
            await rt.hold_gripper_async("arm2", open_gripper=True, frames=4)
        ok_park = await rt.park_passive_arm_async(active_arm)
        if not ok_park:
            print(f"NovaGraspController: WARN passive arm park incomplete (active={active_arm})")
        return True

    @property
    def last_result(self) -> Optional[GraspResult]:
        return self._last_result

    def execute_grasp(self, pose: Transform6D) -> GraspResult:
        """同步入口（仅排队诊断）；实际运动请用 execute_grasp_async。"""
        import asyncio

        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                asyncio.ensure_future(self.execute_grasp_async(pose))
                return GraspResult(ok=True, message="grasp scheduled (async)")
        except RuntimeError:
            pass
        result = GraspResult(ok=False, message="Use execute_grasp_async while simulation is running")
        self._last_result = result
        return result

    @staticmethod
    def _arm_base_path(arm: str) -> str:
        return ARM2_BASE_PATH if arm == "arm2" else ARM1_BASE_PATH

    @staticmethod
    def _arm_base_frame(arm: str) -> str:
        return ARM2_BASE_LINK if arm == "arm2" else ARM1_BASE_LINK

    def pose_from_stage_box(self) -> Optional[Transform6D]:
        """从 Stage 读取 grasp_box 刚体根位姿（与 TF grasp_box 帧一致）。"""
        mat = get_prim_world_matrix(BOX_LINK_PATH)
        if mat is None:
            return None
        t, q = matrix_to_translation_quat(mat)
        return Transform6D(
            translation=list(t),
            rotation_xyzw=list(q),
            target_frame="world",
            pose_role="box_center",
        )

    def pose_from_published_box_gt(self) -> Optional[Transform6D]:
        """与 Pub box once /box_pose 同源：base_link 下盒心 + link 旋转。"""
        data = read_box_pose_in_frame(BOX_LINK_PATH, BASE_LINK_PATH)
        if not data:
            return None
        return Transform6D(
            translation=list(data["position"]),
            rotation_xyzw=list(data["orientation_xyzw"]),
            target_frame=BOX_POSE_FRAME,
            pose_role="gripper_pose",
        )

    def _log_pose_validation(self, target_world: Tuple[float, float, float]) -> None:
        """对比抓取目标与 Stage 盒位姿，便于判断是位姿问题还是 IK 问题。"""
        link_mat = get_prim_world_matrix(BOX_LINK_PATH)
        if link_mat is not None:
            lt, _ = matrix_to_translation_quat(link_mat)
            dx = float(target_world[0] - lt[0])
            dy = float(target_world[1] - lt[1])
            dz = float(target_world[2] - lt[2])
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            print(
                f"NovaGraspController: pose check grasp_box=({lt[0]:.3f},{lt[1]:.3f},{lt[2]:.3f}) "
                f"target=({target_world[0]:.3f},{target_world[1]:.3f},{target_world[2]:.3f}) "
                f"delta={dist:.3f}m"
            )
        box_pose = get_box_world_pose()
        if box_pose is None:
            return
        bt, _ = box_pose
        print(
            f"NovaGraspController: pose check visual_bbox_center="
            f"({bt[0]:.3f},{bt[1]:.3f},{bt[2]:.3f})"
        )

    def _to_world_pose(
        self, pose: Transform6D
    ) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
        frame = (pose.target_frame or BOX_POSE_FRAME).strip("/")
        t = pose.translation
        q = pose.rotation_xyzw
        if frame in ("world", ""):
            return (tuple(t), tuple(q))
        if frame == BOX_POSE_FRAME.strip("/"):
            mat_base = get_prim_world_matrix(BASE_LINK_PATH)
            if mat_base is None:
                return None
            from pxr import Gf

            pose_m = Gf.Matrix4d(1.0)
            pose_m.SetRotateOnly(Gf.Quatd(q[3], Gf.Vec3d(*q[:3])))
            pose_m.SetTranslateOnly(Gf.Vec3d(*t))
            # USD 行向量：p_world = p_local * pose_m * mat_base
            mat_world = pose_m * mat_base
            return matrix_to_translation_quat(mat_world)
        if frame in (ARM1_BASE_LINK, ARM2_BASE_LINK):
            arm_path = ARM1_BASE_PATH if frame == ARM1_BASE_LINK else ARM2_BASE_PATH
            return transform_pose_between_frames(tuple(t), tuple(q), arm_path, "/World")
        return transform_pose_between_frames(tuple(t), tuple(q), f"/World/{frame}", "/World")

    @staticmethod
    def _plan_top_down_from_box_center(
        box_center: Tuple[float, float, float],
        box_half_z: float = BOX_SIZE_Z * 0.5,
    ) -> Tuple[List[float], List[float], List[float]]:
        """盒心 → 顶抓路点（夹爪 TCP 高度）。"""
        cx, cy, cz = box_center
        return (
            [cx, cy, cz + box_half_z + 0.14],
            [cx, cy, cz],
            [cx, cy, cz + box_half_z + 0.10],
        )

    @staticmethod
    def _plan_gripper_pose_waypoints(
        gripper_world: Tuple[float, float, float],
        gripper_quat: Tuple[float, float, float, float],
    ) -> Tuple[List[float], List[float], List[float], Tuple[float, float, float, float]]:
        """外部夹爪 6D → 预位 / 抓取 / 抬起（保留姿态）。"""
        gx, gy, gz = gripper_world
        pre = [gx, gy, gz + 0.12]
        grasp = [gx, gy, gz]
        lift = [gx, gy, gz + 0.10]
        return pre, grasp, lift, gripper_quat

    @staticmethod
    def _is_identity_quat_xyzw(
        quat: Tuple[float, float, float, float], tol: float = 1e-3
    ) -> bool:
        """单位四元数（或近单位）视为仅位置测试，不做 6D 姿态跟踪。"""
        x, y, z, w = [float(v) for v in quat]
        n = (x * x + y * y + z * z + w * w) ** 0.5
        if n < 1e-8:
            return True
        x, y, z, w = x / n, y / n, z / n, w / n
        return abs(x) < tol and abs(y) < tol and abs(z) < tol and abs(abs(w) - 1.0) < tol

    @staticmethod
    def _plan_gripper_position_only_waypoints(
        gripper_world: Tuple[float, float, float],
    ) -> Tuple[List[float], List[float], List[float]]:
        """外部位置 + 顶抓姿态（忽略发布侧四元数）。"""
        gx, gy, gz = gripper_world
        return (
            [gx, gy, gz + 0.12],
            [gx, gy, gz],
            [gx, gy, gz + 0.10],
        )

    def _plan_motion_waypoints(
        self, pose: Transform6D, target_t: Tuple[float, float, float], target_q: Tuple[float, float, float, float]
    ) -> Tuple[List[float], List[float], List[float], Optional[Tuple[float, float, float, float]], bool, str]:
        """根据 pose 角色规划 pre / grasp / lift 路点。"""
        use_orientation = False
        grasp_quat = None
        role = "gripper_pose" if pose.is_gripper_pose() else "box_center"
        if pose.is_gripper_pose():
            if self._is_identity_quat_xyzw(target_q):
                pre, ee_grasp, lift = self._plan_gripper_position_only_waypoints(target_t)
                mode = "POSITION-ONLY"
            else:
                pre, ee_grasp, lift, grasp_quat = self._plan_gripper_pose_waypoints(target_t, target_q)
                use_orientation = True
                mode = "6D"
        else:
            pre, ee_grasp, lift = self._plan_top_down_from_box_center(target_t)
            mode = "top-down"
        return pre, ee_grasp, lift, grasp_quat, use_orientation, f"{role}/{mode}"

    async def move_to_pose_async(self, pose: Transform6D, *, final_approach: bool = False) -> GraspResult:
        """仅移动到目标位姿；默认停在预位点（盒顶上方），不闭合夹爪。"""
        import omni.timeline

        if not omni.timeline.get_timeline_interface().is_playing():
            result = GraspResult(ok=False, message="Timeline not playing — press Play (▶) first")
            self._last_result = result
            print(f"NovaGraspController: {result.message}")
            return result

        rt = self._runtime
        if rt is None or not rt.is_ready:
            result = GraspResult(ok=False, message="Robot articulation not ready (Load + Play first)")
            self._last_result = result
            print(f"NovaGraspController: {result.message} runtime={rt}")
            return result

        role = "gripper_pose" if pose.is_gripper_pose() else "box_center"
        print(
            f"NovaGraspController: move-only start role={role} arm_mode={self._arm_mode} "
            f"frame={pose.target_frame} t={pose.translation} q={pose.rotation_xyzw}"
        )

        world_pose = self._to_world_pose(pose)
        if world_pose is None:
            result = GraspResult(
                ok=False, message=f"Failed to transform pose to world (frame={pose.target_frame})"
            )
            self._last_result = result
            print(f"NovaGraspController: {result.message}")
            return result

        target_t, target_q = world_pose
        self._log_pose_validation(target_t)

        arm = rt.choose_arm(target_t, self._arm_mode)
        print(f"NovaGraspController: selected {arm} (arm_mode={self._arm_mode})")
        await self._prepare_arm_motion(arm)
        arm_local = transform_pose_between_frames(target_t, target_q, "/World", self._arm_base_path(arm))
        if arm_local:
            at, _ = arm_local
            print(
                f"NovaGraspController: {arm} target world=({target_t[0]:.3f},{target_t[1]:.3f},{target_t[2]:.3f}) "
                f"{self._arm_base_frame(arm)}=({at[0]:.3f},{at[1]:.3f},{at[2]:.3f})"
            )

        pre, ee_grasp, _lift, grasp_quat, use_orientation, plan_mode = self._plan_motion_waypoints(
            pose, target_t, target_q
        )
        print(f"NovaGraspController: {arm} move-only plan={plan_mode} pre={pre} target={ee_grasp}")
        if rt is not None:
            rt.log_alignment(arm, "plan_pre", np.array(pre, dtype=np.float64))

        rt.set_gripper(arm, open_gripper=True)
        rt._drive_warned = False
        await rt.hold_gripper_async(arm, open_gripper=True, frames=6)

        ok_pre = await rt.move_arm_to_pose_async(
            arm, pre, grasp_quat, use_orientation=use_orientation, top_down=not use_orientation
        )
        print(f"NovaGraspController: {arm} move-only pre ok={ok_pre}")
        ok_app = ok_pre
        if ok_pre and final_approach:
            ok_app = await rt.move_arm_to_pose_async(
                arm, ee_grasp, grasp_quat, use_orientation=use_orientation, top_down=not use_orientation
            )
            print(f"NovaGraspController: {arm} move-only approach ok={ok_app} (gripper stays open)")
        else:
            grip = rt._get_gripper_center_world(arm)
            if grip is not None:
                d_pre = float(np.linalg.norm(np.array(pre, dtype=np.float64) - grip))
                print(
                    f"NovaGraspController: {arm} move-only stopped at pre "
                    f"gripper=({grip[0]:.3f},{grip[1]:.3f},{grip[2]:.3f}) dist_to_pre={d_pre:.3f}m"
                )

        msg = (
            f"{arm} move-only plan={plan_mode} pre={ok_pre} approach={ok_app if final_approach else 'skipped'} "
            f"t={target_t}"
        )
        print(f"NovaGraspController: {msg}")
        result = GraspResult(ok=bool(ok_app), message=msg, arm=arm)
        self._last_result = result
        return result

    async def execute_grasp_async(self, pose: Transform6D) -> GraspResult:
        import omni.timeline

        if not omni.timeline.get_timeline_interface().is_playing():
            result = GraspResult(ok=False, message="Timeline not playing — press Play (▶) first")
            self._last_result = result
            print(f"NovaGraspController: {result.message}")
            return result

        rt = self._runtime
        if rt is None or not rt.is_ready:
            result = GraspResult(ok=False, message="Robot articulation not ready (Load + Play first)")
            self._last_result = result
            print(f"NovaGraspController: {result.message} runtime={rt}")
            return result

        role = "gripper_pose" if pose.is_gripper_pose() else "box_center"
        print(
            f"NovaGraspController: grasp start role={role} arm_mode={self._arm_mode} "
            f"frame={pose.target_frame} t={pose.translation} q={pose.rotation_xyzw}"
        )

        world_pose = self._to_world_pose(pose)
        if world_pose is None:
            result = GraspResult(
                ok=False, message=f"Failed to transform pose to world (frame={pose.target_frame})"
            )
            self._last_result = result
            print(f"NovaGraspController: {result.message}")
            return result

        target_t, target_q = world_pose
        self._log_pose_validation(target_t)

        arm = rt.choose_arm(target_t, self._arm_mode)
        print(f"NovaGraspController: selected {arm} (arm_mode={self._arm_mode})")
        await self._prepare_arm_motion(arm)
        arm_local = transform_pose_between_frames(target_t, target_q, "/World", self._arm_base_path(arm))
        if arm_local:
            at, _ = arm_local
            print(
                f"NovaGraspController: {arm} target world=({target_t[0]:.3f},{target_t[1]:.3f},{target_t[2]:.3f}) "
                f"{self._arm_base_frame(arm)}=({at[0]:.3f},{at[1]:.3f},{at[2]:.3f})"
            )

        rt.set_gripper(arm, open_gripper=True)
        rt._drive_warned = False
        await rt.hold_gripper_async(arm, open_gripper=True, frames=6)

        pre, ee_grasp, lift, grasp_quat, use_orientation, plan_mode = self._plan_motion_waypoints(
            pose, target_t, target_q
        )
        print(f"NovaGraspController: {arm} grasp plan={plan_mode} pre={pre} grasp={ee_grasp}")

        ok_pre = await rt.move_arm_to_pose_async(
            arm, pre, grasp_quat, use_orientation=use_orientation, top_down=not use_orientation
        )
        print(f"NovaGraspController: {arm} pre-grasp ok={ok_pre}")
        ok_app = (
            await rt.move_arm_to_pose_async(
                arm, ee_grasp, grasp_quat, use_orientation=use_orientation, top_down=not use_orientation
            )
            if ok_pre
            else False
        )
        print(f"NovaGraspController: {arm} final approach ok={ok_app}")
        if ok_app:
            rt.set_gripper(arm, open_gripper=False)
            await rt.hold_gripper_async(arm, open_gripper=False, frames=8)
            await rt.move_arm_to_pose_async(
                arm, lift, grasp_quat, use_orientation=use_orientation, top_down=not use_orientation
            )

        msg = f"{arm} role={role} pre={ok_pre} approach={ok_app} t={target_t}"
        print(f"NovaGraspController: {msg}")
        result = GraspResult(ok=bool(ok_app), message=msg, arm=arm)
        self._last_result = result
        return result
