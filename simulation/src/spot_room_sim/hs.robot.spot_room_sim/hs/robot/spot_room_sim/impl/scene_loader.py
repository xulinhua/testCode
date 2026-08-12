# -*- coding: utf-8 -*-
"""加载本地 Simple Room + Spot（扩展工作流：异步 World 初始化）。"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux

from ..global_variables import (
    CAMERA_LOCAL_RPY_DEG,
    CAMERA_LOCAL_XYZ,
    CAMERA_PRIM_NAME,
    CAMERA_PRIM_PATH_FALLBACK,
    CLUTTER_ROOT_PATH,
    DEFAULT_HORIZONTAL_FOV_DEG,
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_IMAGE_WIDTH,
    GRAPH_ROOT,
    MAX_PHYSICS_STEPS_PER_FRAME,
    PHYSICS_DT,
    RENDERING_DT,
    ROOM_PRIM_PATH,
    SPOT_BODY_CANDIDATES,
    SPOT_PRIM_PATH,
    SPOT_SPAWN_ORI,
    SPOT_SPAWN_POS,
    VIEWPORT_EYE,
    VIEWPORT_TARGET,
)
from ..paths import (
    assets_ready,
    get_extension_root,
    local_simple_room_usd,
    local_spot_usd,
    simple_room_textures_are_lite,
)
from .kit_extensions import ensure_core_api_enabled
from .room_clutter import remove_room_clutter, spawn_room_clutter
from .spot_runtime import SpotRuntime


def _rpy_deg_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    r = math.radians(roll)
    p = math.radians(pitch)
    y = math.radians(yaw)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _rpy_rad_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    return _rpy_deg_to_quat_xyzw(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# Gazebo / cam_mgr_ros：camera_link → optical = rpy(-π/2, 0, -π/2)
OPTICAL_FROM_LINK_QUAT_XYZW = _rpy_rad_to_quat_xyzw(-math.pi / 2.0, 0.0, -math.pi / 2.0)
def _usd_camera_attrs_from_fov(width: int, height: int, hfov_deg: float) -> Tuple[float, float, float]:
    horiz_ap = 20.955
    vert_ap = horiz_ap * float(height) / max(float(width), 1.0)
    focal = 0.5 * horiz_ap / math.tan(math.radians(hfov_deg) * 0.5)
    return focal, horiz_ap, vert_ap


class SceneLoader:
    def __init__(self, ext_root: Optional[str] = None):
        self._ext_root = ext_root or get_extension_root()
        self.world = None
        self.spot_runtime = SpotRuntime(SPOT_PRIM_PATH, ext_root=self._ext_root)
        self.camera_prim_path = CAMERA_PRIM_PATH_FALLBACK
        self.body_prim_path = SPOT_PRIM_PATH
        self.cam_xyz_base = CAMERA_LOCAL_XYZ
        # USD 相机 prim 姿态（仅用于摆放渲染相机）；ROS cam_0_link 用机体式（Z 上）
        self.cam_quat_xyzw_base = _rpy_deg_to_quat_xyzw(*CAMERA_LOCAL_RPY_DEG)
        # base_link→cam_0_link：前视安装，与 base 同轴向（X 前 Y 左 Z 上）
        self.cam_link_quat_xyzw_base = (0.0, 0.0, 0.0, 1.0)
        self.cam_optical_from_link_quat_xyzw = OPTICAL_FROM_LINK_QUAT_XYZW
        self._loaded = False

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    def load(self, *args, **kwargs) -> bool:
        """同步入口已废弃：扩展必须走 load_async。"""
        print("SceneLoader.load: use load_async() in extension workflow")
        return False

    async def load_async(
        self,
        image_size: Tuple[int, int] = (DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT),
        horizontal_fov_deg: float = DEFAULT_HORIZONTAL_FOV_DEG,
        spawn_pos: Tuple[float, float, float] = SPOT_SPAWN_POS,
    ) -> bool:
        """官方扩展 Load 流程：clear World → 摆资产 → initialize_simulation_context_async → reset_async → pause。"""
        t0 = time.perf_counter()
        try:
            if not ensure_core_api_enabled():
                print("SceneLoader: isaacsim.core.api unavailable")
                return False

            import omni.kit.app
            from isaacsim.core.api import World
            from isaacsim.core.simulation_manager import SimulationManager
            from isaacsim.core.utils.stage import update_stage_async

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                print("SceneLoader: no USD stage")
                return False

            room_usd = local_simple_room_usd(self._ext_root)
            if not room_usd or not assets_ready(self._ext_root):
                print(
                    "SceneLoader: local assets missing/incomplete. "
                    "Run: python3 scripts/download_assets.py && python3 scripts/make_lite_textures.py"
                )
                return False

            if not simple_room_textures_are_lite(self._ext_root):
                print(
                    "SceneLoader: WARNING textures still hires (~100MB). "
                    "Run: python3 scripts/make_lite_textures.py"
                )

            self.unload()

            # 清掉旧 World，避免半初始化单例（日志里 _physx_interface=None）
            prev = World.instance()
            if prev is not None:
                try:
                    prev.clear_all_callbacks()
                except Exception:
                    pass
                try:
                    prev.clear_instance()
                except Exception:
                    pass
            self._clear_stale_sim_views()

            # Unload 后 Spot 偶发删不掉 → 二次 Load 在 define_prim(/World/Spot) 崩掉，
            # 只留下空房间（Status IDLE），狗和杂物都不会生成。
            for path in (SPOT_PRIM_PATH, ROOM_PRIM_PATH, CLUTTER_ROOT_PATH, GRAPH_ROOT):
                self._force_remove_prim(path)
            await update_stage_async()
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()
            for path in (SPOT_PRIM_PATH, ROOM_PRIM_PATH, CLUTTER_ROOT_PATH):
                if self._prim_exists(path):
                    print(f"SceneLoader: leftover still present after purge: {path}")
                    self._force_remove_prim(path)
            await update_stage_async()
            await omni.kit.app.get_app().next_update_async()

            self.world = World(
                stage_units_in_meters=1.0,
                physics_dt=PHYSICS_DT,
                rendering_dt=RENDERING_DT,
            )
            try:
                SimulationManager.set_physics_dt(PHYSICS_DT)
            except Exception as exc:
                print(f"SceneLoader: set_physics_dt skipped: {exc}")
            self._apply_runtime_performance_settings()

            t_room = time.perf_counter()
            room_prim = self._ensure_xform_prim(ROOM_PRIM_PATH)
            try:
                room_prim.GetReferences().ClearReferences()
            except Exception:
                pass
            room_prim.GetReferences().AddReference(room_usd)
            print(f"SceneLoader: Simple Room (local) -> {ROOM_PRIM_PATH}")
            print(f"  {room_usd}  ({time.perf_counter() - t_room:.2f}s)")

            self._disable_heavy_room_prims(stage)

            # 先只挂 Spot USD（不创建 SingleArticulation），避免踩到已 invalidate 的 SimulationView
            t_spot = time.perf_counter()
            self._place_spot_usd(spawn_pos)
            print(f"SceneLoader: Spot USD placed ({time.perf_counter() - t_spot:.2f}s)")

            self._ensure_dome_light(stage)
            self.body_prim_path = self._resolve_body_prim(stage)
            self.camera_prim_path = self._create_front_camera(
                stage, self.body_prim_path, image_size, horizontal_fov_deg
            )

            print("SceneLoader: initialize_simulation_context_async ...")
            await self.world.initialize_simulation_context_async()
            await omni.kit.app.get_app().next_update_async()

            # 物理上下文就绪后再包 Articulation / 加杂物
            # （此时若 SimulationView 为 None，Articulation 不会误用已 invalidate 的旧 view）
            t_wrap = time.perf_counter()
            self.spot_runtime.spawn(position=spawn_pos, orientation_wxyz=SPOT_SPAWN_ORI)
            robot = self.spot_runtime.robot
            if robot is not None:
                try:
                    self.world.scene.add(robot)
                    print("SceneLoader: Spot articulation added to World.scene")
                except Exception as exc:
                    print(f"SceneLoader: world.scene.add(Spot) skipped: {exc}")
            print(f"SceneLoader: Spot wrap ({time.perf_counter() - t_wrap:.2f}s)")

            try:
                spawn_room_clutter(self.world)
            except Exception as exc:
                import traceback

                traceback.print_exc()
                print(f"SceneLoader: room clutter failed: {exc}")

            t_reset = time.perf_counter()
            print("SceneLoader: world.reset_async ...")
            await self.world.reset_async()
            await update_stage_async()
            try:
                await self.world.pause_async()
            except Exception:
                self._stop_timeline()
            print(f"SceneLoader: world.reset_async ({time.perf_counter() - t_reset:.2f}s)")

            self.spot_runtime.mark_needs_init()
            self._frame_viewport_in_room(spawn_pos)

            self._loaded = True
            print(
                f"SceneLoader ready in {time.perf_counter() - t0:.2f}s: "
                f"Spot={SPOT_PRIM_PATH} body={self.body_prim_path} cam={self.camera_prim_path}"
            )
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SceneLoader.load_async failed: {exc}")
            self._loaded = False
            return False

    def unload(self) -> None:
        self._stop_timeline()
        # 先从 World 摘掉 robot，再 shutdown（shutdown 会清空 robot 引用）
        if self.world is not None:
            try:
                robot = self.spot_runtime.robot
                if robot is not None:
                    try:
                        self.world.scene.remove_object(robot.name)
                    except Exception:
                        pass
            except Exception:
                pass
            try:
                self.world.clear_all_callbacks()
            except Exception:
                pass

        self.spot_runtime.shutdown()

        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                remove_room_clutter(stage)
                # 删 Spot 根即可（含 front_cam）；勿先删子 prim 再删父，易残留
                for path in (SPOT_PRIM_PATH, ROOM_PRIM_PATH, CLUTTER_ROOT_PATH, GRAPH_ROOT):
                    self._force_remove_prim(path)
        except Exception as exc:
            print(f"SceneLoader.unload prims: {exc}")

        # 清 World 单例，否则二次 Load 易踩半失效 PhysX / is_homogeneous
        try:
            from isaacsim.core.api import World

            if self.world is not None or World.instance() is not None:
                try:
                    inst = self.world if self.world is not None else World.instance()
                    if inst is not None:
                        try:
                            inst.clear_all_callbacks()
                        except Exception:
                            pass
                except Exception:
                    pass
                try:
                    World.clear_instance()
                except Exception:
                    pass
        except Exception as exc:
            print(f"SceneLoader.unload World: {exc}")

        self._clear_stale_sim_views()
        self.world = None
        self._loaded = False
        self.camera_prim_path = CAMERA_PRIM_PATH_FALLBACK
        self.body_prim_path = SPOT_PRIM_PATH
        print("SceneLoader: unloaded (World cleared)")

    @staticmethod
    def _prim_exists(path: str) -> bool:
        if not path:
            return False
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return False
            prim = stage.GetPrimAtPath(path)
            return bool(prim and prim.IsValid())
        except Exception:
            return False

    @staticmethod
    def _force_remove_prim(path: str) -> None:
        """尽量删掉 prim（DeletePrimsCommand 优先）；失败只打日志。"""
        if not path:
            return
        try:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return
            prim = stage.GetPrimAtPath(path)
            if not (prim and prim.IsValid()):
                return
            try:
                from omni.usd.commands import DeletePrimsCommand

                DeletePrimsCommand([path]).do()
                print(f"SceneLoader: deleted {path}")
                return
            except Exception as exc:
                print(f"SceneLoader: DeletePrimsCommand({path}) failed: {exc}")
            try:
                stage.RemovePrim(path)
                print(f"SceneLoader: RemovePrim {path}")
            except Exception as exc:
                print(f"SceneLoader: RemovePrim({path}) failed: {exc}")
        except Exception as exc:
            print(f"SceneLoader: force_remove({path}) failed: {exc}")

    @staticmethod
    def _ensure_xform_prim(path: str):
        """已存在则复用，避免 define_prim 二次 Load 抛 'prim already exists'。"""
        from isaacsim.core.utils.prims import define_prim

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(path) if stage is not None else None
        if prim and prim.IsValid():
            print(f"SceneLoader: reuse existing prim {path}")
            return prim
        return define_prim(path, "Xform")

    @staticmethod
    def _clear_stale_sim_views() -> None:
        """丢掉已 invalidate 但仍非 None 的 SimulationView，避免 SingleArticulation 构造时崩。"""
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            for attr in ("_physics_sim_view", "_physics_sim_view__warp"):
                view = getattr(SimulationManager, attr, None)
                if view is None:
                    continue
                try:
                    view.invalidate()
                except Exception:
                    pass
                try:
                    setattr(SimulationManager, attr, None)
                except Exception:
                    pass
            try:
                SimulationManager._simulation_view_created = False
            except Exception:
                pass
            print("SceneLoader: cleared stale SimulationManager physics views")
        except Exception as exc:
            print(f"SceneLoader: clear sim views skipped: {exc}")

    def _place_spot_usd(self, spawn_pos: Tuple[float, float, float]) -> None:
        """仅引用 Spot USD 并写出生位姿，不创建 Articulation 包装。"""
        from pxr import Gf, UsdGeom

        usd = local_spot_usd(self._ext_root)
        if not usd:
            raise FileNotFoundError("Local Spot USD missing")
        # 二次 Load 时若 /World/Spot 残留，define_prim 会直接抛错；改为复用
        prim = self._ensure_xform_prim(SPOT_PRIM_PATH)
        try:
            prim.GetReferences().ClearReferences()
        except Exception:
            pass
        prim.GetReferences().AddReference(usd)
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(float(spawn_pos[0]), float(spawn_pos[1]), float(spawn_pos[2]))
        )
        qw, qx, qy, qz = [float(v) for v in SPOT_SPAWN_ORI]
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))
        print(f"SceneLoader: Spot USD -> {SPOT_PRIM_PATH} ({usd})")

    def _disable_heavy_room_prims(self, stage) -> None:
        from pxr import Usd

        try:
            room = stage.GetPrimAtPath(ROOM_PRIM_PATH)
            if not room or not room.IsValid():
                return
            for prim in Usd.PrimRange(room):
                if "sky" not in prim.GetName().lower():
                    continue
                try:
                    prim.SetActive(False)
                    print(f"SceneLoader: deactivated heavy prim {prim.GetPath()}")
                except Exception:
                    pass
        except Exception as exc:
            print(f"SceneLoader: skip deactivate sky: {exc}")

    def _resolve_body_prim(self, stage) -> str:
        for name in SPOT_BODY_CANDIDATES:
            path = f"{SPOT_PRIM_PATH}/{name}"
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                print(f"SceneLoader: Spot body prim = {path}")
                return path
        print(f"SceneLoader: Spot body not found under candidates, use {SPOT_PRIM_PATH}")
        return SPOT_PRIM_PATH

    def _create_front_camera(
        self,
        stage,
        parent_path: str,
        image_size: Tuple[int, int],
        horizontal_fov_deg: float,
    ) -> str:
        width, height = int(image_size[0]), int(image_size[1])
        cam_path = f"{parent_path}/{CAMERA_PRIM_NAME}"
        focal, horiz_ap, vert_ap = _usd_camera_attrs_from_fov(width, height, horizontal_fov_deg)

        if stage.GetPrimAtPath(cam_path):
            cam = UsdGeom.Camera(stage.GetPrimAtPath(cam_path))
        else:
            cam = UsdGeom.Camera.Define(stage, Sdf.Path(cam_path))

        qx, qy, qz, qw = self.cam_quat_xyzw_base
        xform = UsdGeom.Xformable(cam.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(float(CAMERA_LOCAL_XYZ[0]), float(CAMERA_LOCAL_XYZ[1]), float(CAMERA_LOCAL_XYZ[2]))
        )
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))

        cam.CreateHorizontalApertureAttr(horiz_ap)
        cam.CreateVerticalApertureAttr(vert_ap)
        cam.CreateFocalLengthAttr(focal)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 30.0))
        print(
            f"SceneLoader: front camera {cam_path} {width}x{height} "
            f"fov_h={horizontal_fov_deg:.1f} local_xyz={CAMERA_LOCAL_XYZ} "
            f"usd_rpy_deg={CAMERA_LOCAL_RPY_DEG} | 朝向=机器狗前方(+X)，画面上=+Z"
        )
        return cam_path

    @staticmethod
    def _apply_runtime_performance_settings() -> None:
        """Avoid physics catch-up spiral and cut RTX cost while Spot runs at 500Hz."""
        try:
            import carb

            s = carb.settings.get_settings()
            # Cap physics substeps per rendered frame (spiral-of-death guard)
            for key, val in (
                ("/app/player/maxTimeStepsPerFrame", int(MAX_PHYSICS_STEPS_PER_FRAME)),
                ("/app/player/useFixedTimeStepping", False),
                ("/app/runLoops/main/rateLimitEnabled", True),
                ("/app/runLoops/main/rateLimitFrequency", 30),
                ("/rtx/ecoMode/enabled", True),
                ("/rtx/reflections/enabled", False),
                ("/rtx/indirectDiffuse/enabled", False),
                ("/rtx/ambientOcclusion/enabled", False),
            ):
                try:
                    s.set(key, val)
                except Exception:
                    pass
            print(
                f"SceneLoader: perf caps maxPhysicsSteps/frame={MAX_PHYSICS_STEPS_PER_FRAME} "
                f"render~{1.0 / RENDERING_DT:.0f}Hz ecoMode=on"
            )
        except Exception as exc:
            print(f"SceneLoader: perf settings skipped: {exc}")

    @staticmethod
    def _ensure_dome_light(stage) -> None:
        light_path = "/World/DomeLight"
        if stage.GetPrimAtPath(light_path):
            try:
                light = UsdLux.DomeLight(stage.GetPrimAtPath(light_path))
                light.CreateIntensityAttr(1200.0)
            except Exception:
                pass
            return
        light = UsdLux.DomeLight.Define(stage, Sdf.Path(light_path))
        light.CreateIntensityAttr(1200.0)

    @staticmethod
    def _frame_viewport_in_room(spawn_pos: Tuple[float, float, float]) -> None:
        """强制主视口用 Perspective 看向 Spot。

        CreateRenderProduct / ROS 相机启动后，视口常被切到 front_cam，
        看起来像「机器狗和障碍物都没了」（其实是狗眼视角）。
        """
        persp = "/OmniverseKit_Persp"
        eye = list(VIEWPORT_EYE)
        target = [
            float(spawn_pos[0]),
            float(spawn_pos[1]),
            max(0.35, float(spawn_pos[2]) * 0.65),
        ]
        try:
            from omni.kit.viewport.utility import get_active_viewport

            vp = get_active_viewport()
            if vp is not None:
                try:
                    vp.camera_path = persp
                except Exception:
                    pass
        except Exception as exc:
            print(f"SceneLoader: force Perspective skipped: {exc}")

        try:
            from isaacsim.core.utils.viewports import set_camera_view
            import numpy as np

            set_camera_view(
                eye=np.asarray(eye, dtype=float),
                target=np.asarray(target, dtype=float),
                camera_prim_path=persp,
            )
            print(f"SceneLoader: viewport {persp} eye={eye} target={target}")
            return
        except Exception as exc:
            print(f"SceneLoader: set_camera_view failed ({exc}), try ViewportManager")
        try:
            from isaacsim.core.rendering_manager import ViewportManager

            cam = ViewportManager.get_camera()
            ViewportManager.set_camera_view(camera=cam, eye=eye, target=target)
            print(f"SceneLoader: ViewportManager eye={eye} target={target}")
        except Exception as exc:
            print(f"SceneLoader: viewport framing failed: {exc}")

    @staticmethod
    def _stop_timeline() -> None:
        import omni.timeline

        omni.timeline.get_timeline_interface().stop()
