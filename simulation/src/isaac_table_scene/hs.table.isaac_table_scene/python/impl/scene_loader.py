# -*- coding: utf-8 -*-
"""加载参考项目 mz 桌子、桌上 cassette 扫描模型、桌边相机与灯光。"""

from __future__ import annotations

import math
import os
from typing import Optional, Tuple

import omni.usd
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

from ..cassette_assets import (
    center_height_in_link,
    geometry_centering_offset,
    resolve_cassette_paths,
)
from ..global_variables import (
    CAMERA_LINK_PATH,
    CAMERA_OPTICAL_PATH,
    CAMERA_PRIM_PATH,
    CUBOID_GEOM_PATH,
    CUBOID_LINK_PATH,
    CUBOID_PRIM_PATH,
    DEFAULT_CAMERA_EDGE_Y_RATIO,
    DEFAULT_CAMERA_HEIGHT_Z,
    DEFAULT_TABLE_TOP_Z,
    OPTICAL_FRAME_RPY_DEG,
    ORBBEC_G335_DEPTH_HFOV_DEG,
    ORBBEC_G335_DEPTH_HEIGHT,
    ORBBEC_G335_DEPTH_VFOV_DEG,
    ORBBEC_G335_DEPTH_WIDTH,
    TABLE_PRIM_PATH,
)
from ..paths import resolve_mz_dir
from .cassette_mesh_builder import load_cassette_mesh_from_obj
from .camera_config import usd_camera_attrs_from_hv_fov
from .pose_utils import Pose6D, camera_pose_look_at_cuboid, set_pose
from .usd_assets import (
    apply_table_texture_session_absolute,
    register_mz_texture_search_path,
    repair_mz_usd_on_disk,
    repair_table_reference_layers,
    verify_mz_texture_files,
)


class SceneLoader:
    def __init__(self, raw_data_dir: str, ext_root: str):
        self._raw_data_dir = raw_data_dir
        self._ext_root = ext_root
        self.world: Optional[World] = None
        self.table_prim_path = TABLE_PRIM_PATH
        self.cuboid_link_path = CUBOID_LINK_PATH
        self.cuboid_prim_path = CUBOID_PRIM_PATH
        self.camera_link_path = CAMERA_LINK_PATH
        self.camera_prim_path = CAMERA_PRIM_PATH
        self.table_top_z = DEFAULT_TABLE_TOP_Z
        self._table_center_xy = (0.0, 0.0)
        self._table_half_extent_y = 0.5
        self._last_cuboid_pose: Optional[Pose6D] = None
        self._last_camera_pose: Optional[Pose6D] = None
        self._loaded = False

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def last_camera_pose(self) -> Optional[Pose6D]:
        return self._last_camera_pose

    @property
    def last_cuboid_pose(self) -> Optional[Pose6D]:
        return self._last_cuboid_pose

    def load(
        self,
        cuboid_pose: Pose6D = None,
        camera_pose: Pose6D = None,
        camera_resolution=(ORBBEC_G335_DEPTH_WIDTH, ORBBEC_G335_DEPTH_HEIGHT),
        horizontal_fov_deg: float = ORBBEC_G335_DEPTH_HFOV_DEG,
        vertical_fov_deg: float = ORBBEC_G335_DEPTH_VFOV_DEG,
        **kwargs,
    ) -> bool:
        _ = kwargs.get("cassette_scale")
        _ = kwargs.get("cuboid_size")
        try:
            stage = get_current_stage()
            if stage is None:
                print("SceneLoader: no USD stage")
                return False

            if self.world is None:
                self.world = World(physics_dt=1.0 / 60.0)

            self._prepare_stage(stage)
            self.world.scene.add_default_ground_plane()
            self._ensure_lights(stage)

            if not self._load_table_usd(stage):
                print("SceneLoader: mz.usd table load failed")
                return False

            self.table_top_z = self._estimate_table_top_z(stage)
            cx, cy, half_y = self._estimate_table_footprint(stage)
            self._table_center_xy = (cx, cy)
            self._table_half_extent_y = half_y

            cuboid_pose = cuboid_pose or self._default_cassette_pose()
            camera_pose = camera_pose or self._default_camera_pose(cuboid_pose)

            self._last_cuboid_pose = cuboid_pose
            self._last_camera_pose = camera_pose

            self._create_cassette(stage, cuboid_pose)
            self._create_camera(
                stage,
                camera_pose,
                camera_resolution,
                horizontal_fov_deg,
                vertical_fov_deg,
            )

            self.world.reset()
            self._stop_timeline()
            self._loaded = True
            print(
                f"SceneLoader: mz table (top z≈{self.table_top_z:.3f}) + cassette + camera ready"
            )
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SceneLoader.load failed: {exc}")
            self._loaded = False
            return False

    def unload(self) -> None:
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                for path in (
                    self.table_prim_path,
                    self.cuboid_link_path,
                    "/World/cuboid",
                    self.camera_link_path,
                    "/World/DomeLight",
                    "/World/DefaultLight",
                ):
                    if stage.GetPrimAtPath(path):
                        stage.RemovePrim(path)
        except Exception as exc:
            print(f"SceneLoader.unload: {exc}")
        self._loaded = False
        self._last_cuboid_pose = None
        self._last_camera_pose = None

    def _estimate_table_footprint(self, stage) -> Tuple[float, float, float]:
        """返回桌面中心 (cx, cy) 与 +Y 方向半宽。"""
        prim = stage.GetPrimAtPath(self.table_prim_path)
        if not prim or not prim.IsValid():
            return 0.0, 0.0, 0.5
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bound = cache.ComputeWorldBound(prim)
        rmin = bound.GetRange().GetMin()
        rmax = bound.GetRange().GetMax()
        cx = (float(rmin[0]) + float(rmax[0])) * 0.5
        cy = (float(rmin[1]) + float(rmax[1])) * 0.5
        half_y = max((float(rmax[1]) - float(rmin[1])) * 0.5, 0.2)
        return cx, cy, half_y

    def _default_cassette_pose(self) -> Pose6D:
        cx, cy = self._table_center_xy
        z = self.table_top_z + 0.002
        return Pose6D((cx, cy, z), (0.0, 0.0, 0.0))

    def _cassette_look_at_center(self, link_pose: Pose6D) -> Tuple[float, float, float]:
        tx, ty, tz = link_pose.translation
        return (tx, ty, tz + center_height_in_link())

    def _default_camera_pose(self, cuboid_pose: Pose6D) -> Pose6D:
        """桌面 +Y 边缘、高度 1.8m，视线对准 cassette 几何中心。"""
        target = self._cassette_look_at_center(cuboid_pose)
        cx, cy_center, _ = cuboid_pose.translation
        edge_y = cy_center + self._table_half_extent_y * DEFAULT_CAMERA_EDGE_Y_RATIO
        eye = (cx, edge_y, DEFAULT_CAMERA_HEIGHT_Z)
        return camera_pose_look_at_cuboid(eye, target, OPTICAL_FRAME_RPY_DEG)

    def _prepare_stage(self, stage) -> None:
        if self.world is not None:
            try:
                if self.world.is_playing():
                    self.world.stop()
                self.world.clear()
            except Exception:
                pass

        world_prim = stage.GetPrimAtPath("/World")
        if world_prim and world_prim.IsValid():
            for child in list(world_prim.GetChildren()):
                name = child.GetName()
                if name in ("GroundPlane", "groundPlane", "defaultGroundPlane"):
                    continue
                try:
                    stage.RemovePrim(child.GetPath())
                except Exception as exc:
                    print(f"Remove {child.GetPath()}: {exc}")

    def _load_table_usd(self, stage) -> bool:
        mz_dir = resolve_mz_dir(self._raw_data_dir)
        if not mz_dir:
            print("SceneLoader: mz.usd not found (check data/raw_data/mz symlink)")
            return False

        table_usd = os.path.join(mz_dir, "mz.usd")
        prim_path = self.table_prim_path
        try:
            register_mz_texture_search_path(mz_dir)
            verify_mz_texture_files(mz_dir)
            repair_mz_usd_on_disk(mz_dir)

            add_reference_to_stage(usd_path=os.path.abspath(table_usd), prim_path=prim_path)
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return False

            xform = UsdGeom.Xformable(prim)
            xform.AddScaleOp().Set(Gf.Vec3d(2.5, 2.5, 2.5))
            set_pose(prim_path, Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), stage)

            if not UsdPhysics.RigidBodyAPI(prim):
                UsdPhysics.RigidBodyAPI.Apply(prim)
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(10.0)
            self._apply_collision_recursive(prim)

            repair_table_reference_layers(stage, prim_path, mz_dir)
            apply_table_texture_session_absolute(stage, prim_path, mz_dir)
            print(f"SceneLoader: loaded mz table from {table_usd}")
            return True
        except Exception as exc:
            print(f"SceneLoader: table USD failed: {exc}")
            return False

    def _estimate_table_top_z(self, stage) -> float:
        prim = stage.GetPrimAtPath(self.table_prim_path)
        if not prim or not prim.IsValid():
            return DEFAULT_TABLE_TOP_Z
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bound = cache.ComputeWorldBound(prim)
        zmax = bound.GetRange().GetMax()[2]
        if zmax > 0.01:
            return float(zmax)
        return DEFAULT_TABLE_TOP_Z

    def _create_cassette(self, stage, link_pose: Pose6D) -> None:
        for legacy in (CUBOID_LINK_PATH, "/World/cuboid"):
            if stage.GetPrimAtPath(legacy):
                stage.RemovePrim(legacy)

        paths = resolve_cassette_paths(self._ext_root)
        if not os.path.isfile(paths.obj_path):
            raise FileNotFoundError(f"cassette OBJ missing: {paths.obj_path}")

        UsdGeom.Xform.Define(stage, Sdf.Path(CUBOID_LINK_PATH))
        set_pose(CUBOID_LINK_PATH, link_pose, stage)

        geom_path = Sdf.Path(CUBOID_GEOM_PATH)
        UsdGeom.Xform.Define(stage, geom_path)
        geom_xf = UsdGeom.Xformable(stage.GetPrimAtPath(geom_path))
        geom_xf.ClearXformOpOrder()
        ox, oy, oz = geometry_centering_offset()
        geom_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(ox, oy, oz))

        mesh_path = f"{CUBOID_GEOM_PATH}/mesh"
        load_cassette_mesh_from_obj(stage, mesh_path, paths)

        link_prim = stage.GetPrimAtPath(CUBOID_LINK_PATH)
        rb = UsdPhysics.RigidBodyAPI.Apply(link_prim)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(False)
        mass_api = UsdPhysics.MassAPI.Apply(link_prim)
        mass_api.CreateMassAttr().Set(0.15)

        for child in Usd.PrimRange(link_prim):
            if child.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(child)
                mesh_col = UsdPhysics.MeshCollisionAPI.Apply(child)
                mesh_col.GetApproximationAttr().Set("convexHull")

        print(f"SceneLoader: cuboid_link (cassette) on table at {link_pose.translation}")

    def _create_camera(
        self,
        stage,
        camera_pose: Pose6D,
        resolution: Tuple[int, int],
        horizontal_fov_deg: float,
        vertical_fov_deg: float,
    ) -> None:
        width, height = resolution
        focal_mm, horiz_ap_mm, vert_ap_mm = usd_camera_attrs_from_hv_fov(
            horizontal_fov_deg, vertical_fov_deg
        )
        if stage.GetPrimAtPath(CAMERA_LINK_PATH):
            stage.RemovePrim(CAMERA_LINK_PATH)

        UsdGeom.Xform.Define(stage, Sdf.Path(CAMERA_LINK_PATH))
        set_pose(CAMERA_LINK_PATH, camera_pose, stage)

        optical = UsdGeom.Xform.Define(stage, Sdf.Path(CAMERA_OPTICAL_PATH))
        optical_xf = UsdGeom.Xformable(optical)
        optical_xf.ClearXformOpOrder()
        optical_xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat).Set(
            Gf.Vec3f(*OPTICAL_FRAME_RPY_DEG)
        )

        cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAMERA_PRIM_PATH))
        cam.CreateHorizontalApertureAttr(horiz_ap_mm)
        cam.CreateVerticalApertureAttr(vert_ap_mm)
        cam.CreateFocalLengthAttr(focal_mm)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
        print(
            f"SceneLoader: camera_link + optical + sensor {width}x{height} "
            f"at {camera_pose.translation} rot={camera_pose.rotation_deg}"
        )

    @staticmethod
    def _apply_collision_recursive(parent_prim) -> int:
        count = 0
        for child_prim in Usd.PrimRange(parent_prim):
            if child_prim.IsA(UsdGeom.Mesh) or child_prim.IsA(UsdGeom.Cube):
                UsdPhysics.CollisionAPI.Apply(child_prim)
                mesh_col = UsdPhysics.MeshCollisionAPI.Apply(child_prim)
                mesh_col.GetApproximationAttr().Set("convexHull")
                count += 1
        return count

    @staticmethod
    def _ensure_lights(stage) -> None:
        dome_path = "/World/DomeLight"
        if not stage.GetPrimAtPath(dome_path):
            dome = UsdLux.DomeLight.Define(stage, Sdf.Path(dome_path))
            dome.CreateIntensityAttr(900.0)

        light_path = "/World/DefaultLight"
        if stage.GetPrimAtPath(light_path):
            return
        light = UsdLux.DistantLight.Define(stage, Sdf.Path(light_path))
        light.CreateIntensityAttr(3000)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.9))
        rot_z = Gf.Rotation(Gf.Vec3d.ZAxis(), 45)
        rot_y = Gf.Rotation(Gf.Vec3d.YAxis(), -30)
        combined = rot_y * rot_z
        mat = Gf.Matrix4d(1.0)
        mat.SetRotate(combined)
        rot_mat3 = mat.ExtractRotationMatrix()
        sy = math.sqrt(rot_mat3[0][0] ** 2 + rot_mat3[1][0] ** 2)
        if sy > 1e-6:
            ex = math.degrees(math.atan2(rot_mat3[2][1], rot_mat3[2][2]))
            ey = math.degrees(math.atan2(-rot_mat3[2][0], sy))
            ez = math.degrees(math.atan2(rot_mat3[1][0], rot_mat3[0][0]))
        else:
            ex = math.degrees(math.atan2(-rot_mat3[1][2], rot_mat3[1][1]))
            ey = math.degrees(math.atan2(-rot_mat3[2][0], sy))
            ez = 0.0
        xform = UsdGeom.Xformable(light.GetPrim())
        xform.AddRotateXYZOp().Set(Gf.Vec3f(ex, ey, ez))

    @staticmethod
    def _stop_timeline() -> None:
        import omni.timeline

        tl = omni.timeline.get_timeline_interface()
        tl.stop()
        tl.set_current_time(0)
