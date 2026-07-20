# -*- coding: utf-8 -*-
"""数据集采集：UI 范围随机盒子位姿 → PhysX 稳定 → Replicator 写 RGB/Depth/GT。"""

from __future__ import annotations

import json
import os
import random
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Dict, List, Optional

from ...global_variables import (
    BASE_LINK_PATH,
    BOX_LINK_PATH,
    BOX_POSE_FRAME,
    COLLECT_PITCH_RANGE,
    COLLECT_ROLL_RANGE,
    COLLECT_TX_RANGE,
    COLLECT_TY_RANGE,
    COLLECT_TZ_RANGE,
    COLLECT_YAW_RANGE,
    DEFAULT_RENDER_SUBFRAMES,
    DEFAULT_SETTLE_STEPS,
    ROBOT_PRIM_PATH,
)
from ..kit_extensions import ensure_replicator_enabled
from ..pose_utils import Pose6D, get_box_world_pose, read_box_pose_in_frame

if TYPE_CHECKING:
    from ..topic_config import SessionTopicConfig


@dataclass
class BoxPoseRange:
    """盒子 6D 随机采样范围（世界系平移 + 欧拉角度）。"""

    tx: tuple[float, float] = COLLECT_TX_RANGE
    ty: tuple[float, float] = COLLECT_TY_RANGE
    tz: tuple[float, float] = COLLECT_TZ_RANGE
    roll: tuple[float, float] = COLLECT_ROLL_RANGE
    pitch: tuple[float, float] = COLLECT_PITCH_RANGE
    yaw: tuple[float, float] = COLLECT_YAW_RANGE

    def sample(self) -> Pose6D:
        def _r(lo_hi: tuple[float, float]) -> float:
            lo, hi = lo_hi
            if abs(hi - lo) < 1e-9:
                return float(lo)
            return random.uniform(lo, hi)

        return Pose6D(
            translation=(_r(self.tx), _r(self.ty), _r(self.tz)),
            rotation_deg=(_r(self.roll), _r(self.pitch), _r(self.yaw)),
        )

    def as_dict(self) -> dict:
        return {
            "tx": list(self.tx),
            "ty": list(self.ty),
            "tz": list(self.tz),
            "roll": list(self.roll),
            "pitch": list(self.pitch),
            "yaw": list(self.yaw),
        }


def _pose6d_dict(pose: Pose6D) -> dict:
    return {
        "translation": list(pose.translation),
        "rotation_deg": list(pose.rotation_deg),
    }


@dataclass
class CollectProgress:
    """采集进度回调数据。"""

    index: int = 0
    total: int = 0
    message: str = ""


class DataCollector:
    """管理一次批量采集会话。"""

    def __init__(self):
        self._is_collecting = False
        self._render_products: List = []
        self._camera_keys: List[str] = []

    @property
    def is_collecting(self) -> bool:
        return self._is_collecting

    async def collect(
        self,
        *,
        output_root: str,
        num_samples: int,
        pose_range: BoxPoseRange,
        topic_config: "SessionTopicConfig",
        prepare_sample: Callable[[Pose6D], None],
        hold_sample: Optional[Callable[[], None]] = None,
        settle_steps: int = DEFAULT_SETTLE_STEPS,
        render_subframes: int = DEFAULT_RENDER_SUBFRAMES,
        on_progress: Optional[Callable[[CollectProgress], None]] = None,
    ) -> Optional[str]:
        """执行完整采集，返回本次 run 目录绝对路径。"""
        if self._is_collecting:
            print("DataCollector: already collecting")
            return None
        if num_samples < 1:
            print("DataCollector: invalid num_samples")
            return None
        if not ensure_replicator_enabled():
            print("DataCollector: Replicator not available")
            return None

        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("DataCollector: no stage")
            return None

        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            timeline.play()

        run_name = time.strftime("%Y_%m_%d_%H_%M_%S")
        run_dir = os.path.join(output_root, run_name)
        os.makedirs(run_dir, exist_ok=True)

        self._is_collecting = True
        manifest_samples: List[dict] = []
        try:
            self._setup_render_products(topic_config)
            if not self._render_products:
                print("DataCollector: no camera render products")
                return None

            for i in range(num_samples):
                if on_progress:
                    on_progress(
                        CollectProgress(i + 1, num_samples, f"sample {i + 1}/{num_samples}")
                    )

                pose = pose_range.sample()
                print(
                    "DataCollector: sample "
                    f"{i + 1}/{num_samples} "
                    f"t=({pose.translation[0]:.3f},{pose.translation[1]:.3f},{pose.translation[2]:.3f}) "
                    f"rpy=({pose.rotation_deg[0]:.1f},{pose.rotation_deg[1]:.1f},{pose.rotation_deg[2]:.1f})"
                )
                prepare_sample(pose)
                await self._wait_render_settled(settle_steps, stage)
                if hold_sample:
                    hold_sample()

                frame_dir = os.path.join(run_dir, f"frame_{i:06d}")
                os.makedirs(frame_dir, exist_ok=True)
                sample_meta = await self._capture_frame(
                    frame_dir,
                    index=i,
                    topic_config=topic_config,
                    render_subframes=render_subframes,
                    sampled_pose=pose,
                )
                manifest_samples.append(sample_meta)
                print(f"DataCollector: saved {frame_dir}")
                if on_progress:
                    on_progress(
                        CollectProgress(
                            i + 1,
                            num_samples,
                            f"saved {i + 1}/{num_samples}",
                        )
                    )

            manifest = {
                "version": 1,
                "capture_timestamp": run_name,
                "box_pose_frame": BOX_POSE_FRAME,
                "num_samples": num_samples,
                "settle_steps": settle_steps,
                "pose_range": pose_range.as_dict(),
                "box_pose_mode": "kinematic_pinned",
                "cameras": self._camera_keys,
                "samples": manifest_samples,
            }
            manifest_path = os.path.join(run_dir, "manifest.json")
            with open(manifest_path, "w", encoding="utf-8") as fp:
                json.dump(manifest, fp, indent=2)
            print(f"DataCollector: complete -> {run_dir}")
            return run_dir
        finally:
            self._teardown_render_products()
            self._is_collecting = False

    def _setup_render_products(self, topic_config: "SessionTopicConfig") -> None:
        import omni.replicator.core as rep

        self._teardown_render_products()
        for cam in topic_config.cameras:
            if not cam.camera_prim_path:
                continue
            rp = rep.create.render_product(cam.camera_prim_path, (int(cam.width), int(cam.height)))
            self._render_products.append(rp)
            self._camera_keys.append(cam.key)

    def _teardown_render_products(self) -> None:
        self._render_products.clear()
        self._camera_keys.clear()

    async def _wait_render_settled(self, steps: int, stage) -> None:
        """采集用：盒子已 kinematic 钉扎，仅等待渲染帧稳定。"""
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(max(1, steps)):
            await app.next_update_async()

    async def _wait_physics_settled(self, steps: int, stage) -> None:
        """物理释放场景：先跑满最小步数，再检测盒子位姿连续稳定后采集。"""
        import omni.kit.app

        app = omni.kit.app.get_app()
        for _ in range(max(1, steps)):
            await app.next_update_async()

        pos_tol_m = 0.0008
        ang_tol_deg = 0.35
        stable_needed = 10
        stable_count = 0
        last: Optional[tuple] = None
        for _ in range(180):
            await app.next_update_async()
            world = get_box_world_pose(stage)
            if world is None:
                stable_count = 0
                last = None
                continue
            pos, quat = world
            sample = (pos, quat)
            if last is not None:
                dp = sum((a - b) ** 2 for a, b in zip(pos, last[0])) ** 0.5
                dot = abs(
                    quat[0] * last[1][0]
                    + quat[1] * last[1][1]
                    + quat[2] * last[1][2]
                    + quat[3] * last[1][3]
                )
                dot = min(1.0, max(-1.0, dot))
                import math

                dang = math.degrees(math.acos(dot)) * 2.0
                if dp <= pos_tol_m and dang <= ang_tol_deg:
                    stable_count += 1
                    if stable_count >= stable_needed:
                        return
                else:
                    stable_count = 0
            last = sample

    async def _capture_frame(
        self,
        frame_dir: str,
        *,
        index: int,
        topic_config: "SessionTopicConfig",
        render_subframes: int,
        sampled_pose: Optional[Pose6D] = None,
    ) -> dict:
        import omni.replicator.core as rep
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(
            output_dir=frame_dir,
            rgb=True,
            distance_to_image_plane=True,
            camera_params=True,
            use_common_output_dir=True,
        )
        writer.attach(self._render_products)
        await rep.orchestrator.step_async(
            rt_subframes=render_subframes,
            delta_time=None,
            pause_timeline=False,
        )
        writer.detach()

        box_base = read_box_pose_in_frame(BOX_LINK_PATH, BASE_LINK_PATH, stage)
        box_pose_path = os.path.join(frame_dir, "box_pose.json")
        with open(box_pose_path, "w", encoding="utf-8") as fp:
            json.dump(
                {
                    "frame_id": BOX_POSE_FRAME,
                    "box_pose_base_link": box_base,
                },
                fp,
                indent=2,
            )

        cam_poses: Dict[str, dict] = {}
        cam_infos: Dict[str, dict] = {}
        for cam in topic_config.cameras:
            if cam.key not in self._camera_keys:
                continue
            cam_link = self._camera_link_path(cam.key)
            if cam_link:
                rel = read_box_pose_in_frame(BOX_LINK_PATH, cam_link, stage)
                if rel:
                    cam_poses[cam.key] = rel
            intr = self._read_camera_intrinsics(cam.camera_prim_path, cam.width, cam.height, stage)
            if intr:
                cam_infos[cam.key] = intr
                info_path = os.path.join(frame_dir, f"{cam.key}_camera_info.json")
                with open(info_path, "w", encoding="utf-8") as fp:
                    json.dump(intr, fp, indent=2)

        joints = self._read_joint_positions(stage)
        if joints:
            with open(os.path.join(frame_dir, "joint_states.json"), "w", encoding="utf-8") as fp:
                json.dump(joints, fp, indent=2)

        return {
            "index": index,
            "folder": os.path.basename(frame_dir),
            "sampled_pose_world": _pose6d_dict(sampled_pose) if sampled_pose else None,
            "box_pose_base_link": box_base,
            "box_pose_cam": cam_poses,
            "camera_info": cam_infos,
        }

    @staticmethod
    def _camera_link_path(cam_key: str) -> Optional[str]:
        from ...global_variables import CAMERA_DEFS, GEMINI335_PRIM_PATH

        for spec in CAMERA_DEFS:
            if spec["key"] != cam_key:
                continue
            abs_path = spec.get("prim_path")
            if abs_path:
                # Gemini335：用传感器根作为外参 link
                if abs_path.startswith(GEMINI335_PRIM_PATH):
                    return GEMINI335_PRIM_PATH
                return abs_path
            suffix = spec["prim_suffix"].split("/RSD455")[0]
            return f"{ROBOT_PRIM_PATH}/{suffix}"
        return None

    @staticmethod
    def _read_camera_intrinsics(prim_path: str, width: int, height: int, stage) -> Optional[dict]:
        if not prim_path:
            return None
        from pxr import UsdGeom

        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None
        cam = UsdGeom.Camera(prim)
        fl = float(cam.GetFocalLengthAttr().Get() or 24.0)
        ha = float(cam.GetHorizontalApertureAttr().Get() or 20.955)
        va = float(cam.GetVerticalApertureAttr().Get() or 15.955)
        w, h = int(width), int(height)
        fx = fl / ha * w
        fy = fl / va * h
        cx, cy = w * 0.5, h * 0.5
        return {
            "width": w,
            "height": h,
            "frame_id": prim_path.split("/")[-3] if "cam" in prim_path else "",
            "K": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            "focal_length_mm": fl,
            "horizontal_aperture_mm": ha,
            "vertical_aperture_mm": va,
        }

    @staticmethod
    def _read_joint_positions(stage) -> Optional[dict]:
        try:
            from ...global_variables import ROBOT_ROOT_JOINT_PATH
            from pxr import UsdPhysics

            prim = stage.GetPrimAtPath(ROBOT_ROOT_JOINT_PATH)
            if not prim or not prim.IsValid():
                return None
            joint_names: List[str] = []
            positions: List[float] = []
            for child in prim.GetChildren():
                if child.IsA(UsdPhysics.Joint):
                    name = child.GetName()
                    joint_names.append(name)
                    positions.append(0.0)
            return {"names": joint_names, "positions": positions}
        except Exception:
            return None
