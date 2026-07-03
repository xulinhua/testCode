# -*- coding: utf-8 -*-
"""左臂抓取：启发式 IK + 夹爪 + debug 坐标系。"""

from __future__ import annotations

from typing import Optional

from ...defaults import LEFT_EE_LINK_NAME
from ..interfaces import GraspResult, IGraspExecutor, Transform6D
from .robot_runtime import RobotRuntime


class IsaacGraspController(IGraspExecutor):
    def __init__(self):
        self._last_result: Optional[GraspResult] = None
        self._robot_runtime: Optional[RobotRuntime] = None

    def bind_robot_runtime(self, runtime: Optional[RobotRuntime]) -> None:
        self._robot_runtime = runtime

    @property
    def last_result(self) -> Optional[GraspResult]:
        return self._last_result

    def execute_grasp(self, pose_world: Transform6D) -> GraspResult:
        self._show_debug_frame(pose_world)
        rt = self._robot_runtime
        if rt is None or not rt.is_ready:
            msg = "Grasp skipped: robot articulation not ready (Load robot first)"
            print(f"IsaacGraspController: {msg}")
            result = GraspResult(ok=False, message=msg)
            self._last_result = result
            return result

        target_t = pose_world.translation
        target_q = pose_world.rotation_xyzw

        rt.set_gripper(open_gripper=True)
        pre = [target_t[0], target_t[1], target_t[2] + 0.08]
        ok_pre = rt.move_left_arm_toward(pre, target_q)
        ok_app = rt.move_left_arm_toward(target_t, target_q) if ok_pre else False
        if ok_app:
            rt.set_gripper(open_gripper=False)
            lift = [target_t[0], target_t[1], target_t[2] + 0.06]
            rt.move_left_arm_toward(lift, target_q)

        msg = (
            f"Left arm grasp ({LEFT_EE_LINK_NAME}): pre={ok_pre} approach={ok_app} "
            f"t={target_t} q={target_q}"
        )
        print(f"IsaacGraspController: {msg}")
        result = GraspResult(ok=bool(ok_app), message=msg)
        self._last_result = result
        return result

    def _show_debug_frame(self, pose_world: Transform6D) -> None:
        try:
            from pxr import Gf, Sdf, UsdGeom
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return
            path = "/World/grasp_target_debug"
            xform = UsdGeom.Xform.Define(stage, Sdf.Path(path))
            xform.ClearXformOpOrder()
            t = pose_world.translation
            q = pose_world.rotation_xyzw
            xform.AddTranslateOp().Set(Gf.Vec3d(t[0], t[1], t[2]))
            xform.AddOrientOp().Set(
                Gf.Quatd(q[3], Gf.Vec3d(q[0], q[1], q[2]))
            )
        except Exception as exc:
            print(f"IsaacGraspController debug frame: {exc}")
