# -*- coding: utf-8 -*-
"""Load 场景（可选）、机器人 USD、扩展相机。"""

from __future__ import annotations

from typing import Optional, Tuple

import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, Sdf, UsdGeom, UsdLux

from ..defaults import DEFAULT_ROBOT_USD, ROBOT_PRIM_PATH, SCENE_PRIM_PATH
from ..paths import resolve_asset_file
from .calib.intrinsics_provider import DEFAULT_HORIZONTAL_FOV_DEG, usd_camera_attrs_from_fov
from .grasp.robot_loader import UsdRobotLoader
from .grasp.robot_runtime import RobotRuntime

CAMERA_PRIM_PATH = "/World/box_grasp_camera"


class SceneLoader:
    def __init__(self, scenes_dir: str, robots_dir: str):
        self._scenes_dir = scenes_dir
        self._robots_dir = robots_dir
        self.world: Optional[World] = None
        self.camera_prim_path = CAMERA_PRIM_PATH
        self.robot_prim_path = ROBOT_PRIM_PATH
        self.scene_prim_path = SCENE_PRIM_PATH
        self.robot_loader = UsdRobotLoader(robots_dir, ROBOT_PRIM_PATH)
        self.robot_runtime = RobotRuntime(ROBOT_PRIM_PATH)
        self._loaded = False
        self._scene_loaded = False
        self._scene_usd_path: Optional[str] = None

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def scene_loaded(self) -> bool:
        return self._scene_loaded

    def load(
        self,
        scene_usd: str = "",
        robot_usd: str = DEFAULT_ROBOT_USD,
        camera_position=(0.0, 0.0, 1.2),
        camera_resolution=(640, 480),
        horizontal_fov_deg: float = DEFAULT_HORIZONTAL_FOV_DEG,
    ) -> bool:
        try:
            stage = get_current_stage()
            if stage is None:
                print("SceneLoader: no USD stage")
                return False

            if self.world is None:
                self.world = World(physics_dt=1.0 / 60.0)

            self._scene_loaded = self._load_scene_optional(stage, scene_usd)
            robot_ok = self.robot_loader.load_robot(robot_usd or DEFAULT_ROBOT_USD, self.robot_prim_path)
            if not robot_ok:
                print("SceneLoader: robot USD load failed")
                self._loaded = False
                return False

            self._ensure_dome_light(stage)
            self._create_or_update_camera(
                stage, camera_position, camera_resolution, horizontal_fov_deg
            )

            self.world.reset()
            self._stop_timeline()

            if not self.robot_runtime.initialize():
                print("SceneLoader: robot articulation init failed (check prim path)")

            self.robot_runtime.log_diagnostics()
            self._loaded = True

            if self._scene_loaded:
                print("SceneLoader: scene + robot + camera ready — Press Play to stream")
            else:
                print(
                    "SceneLoader: robot + camera ready (no scene USD yet; "
                    f"place file in {self._scenes_dir} when ready) — Press Play to stream"
                )
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SceneLoader.load failed: {exc}")
            self._loaded = False
            return False

    def unload(self) -> None:
        self.robot_runtime.shutdown()
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                if self._scene_loaded and stage.GetPrimAtPath(self.scene_prim_path):
                    stage.RemovePrim(self.scene_prim_path)
                self.robot_loader.unload_robot()
                if stage.GetPrimAtPath(self.camera_prim_path):
                    stage.RemovePrim(self.camera_prim_path)
        except Exception as exc:
            print(f"SceneLoader.unload: {exc}")
        self._loaded = False
        self._scene_loaded = False
        self._scene_usd_path = None

    def _load_scene_optional(self, stage, scene_usd: str) -> bool:
        path = resolve_asset_file(scene_usd, self._scenes_dir)
        if not path:
            if scene_usd and str(scene_usd).strip():
                print(
                    f"SceneLoader: scene USD not found: {scene_usd!r} "
                    f"(expected under {self._scenes_dir}) — skipping scene"
                )
            return False
        try:
            add_reference_to_stage(usd_path=path, prim_path=self.scene_prim_path)
            self._scene_usd_path = path
            print(f"SceneLoader: referenced scene {path} -> {self.scene_prim_path}")
            return True
        except Exception as exc:
            print(f"SceneLoader: scene reference failed: {exc}")
            return False

    def _ensure_dome_light(self, stage) -> None:
        light_path = "/World/DomeLight"
        if stage.GetPrimAtPath(light_path):
            return
        light = UsdLux.DomeLight.Define(stage, Sdf.Path(light_path))
        light.CreateIntensityAttr(800.0)

    def _create_or_update_camera(self, stage, position, resolution, horizontal_fov_deg) -> None:
        width, height = resolution
        focal_mm, horiz_ap_mm, vert_ap_mm = usd_camera_attrs_from_fov(
            width, height, horizontal_fov_deg
        )
        cam_path = Sdf.Path(self.camera_prim_path)
        if stage.GetPrimAtPath(cam_path):
            cam = UsdGeom.Camera(stage.GetPrimAtPath(cam_path))
        else:
            cam = UsdGeom.Camera.Define(stage, cam_path)

        xform = UsdGeom.Xformable(cam.GetPrim())
        xform.ClearXformOpOrder()
        translate = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        translate.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        cam.CreateHorizontalApertureAttr(horiz_ap_mm)
        cam.CreateVerticalApertureAttr(vert_ap_mm)
        cam.CreateFocalLengthAttr(focal_mm)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000.0))
        print(
            f"SceneLoader: camera {self.camera_prim_path} {width}x{height} "
            f"fov_h={horizontal_fov_deg:.1f}deg at {position}"
        )

    @staticmethod
    def _stop_timeline() -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().stop()
