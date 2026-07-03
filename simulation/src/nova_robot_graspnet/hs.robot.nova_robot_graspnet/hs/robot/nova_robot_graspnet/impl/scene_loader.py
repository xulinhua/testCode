# -*- coding: utf-8 -*-
"""GraspNet 场景加载：不锈钢桌、Nova 双臂、抓取盒、相机物理修复。

Load 流程（顺序不可随意调整）：
  1. 清空 /World 旧内容 → 地面 + 灯光
  2. 引用 nova_robot USD → 挂载到桌面 → 清理 ArUco / ActionGraph
  3. 加载 RSD455 payload 并剥离相机刚体（避免 PhysX 嵌套错误）
  4. 创建 6 腿桌子 + OBJ/USD 抓取盒
  5. Timeline 保持 Stop（关节零位写 USD；PhysX reset 延至 Play 后）
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Dict, Optional

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

if TYPE_CHECKING:
    from isaacsim.core.api.world import World

try:
    from pxr import PhysxSchema
except ImportError:
    PhysxSchema = None

from ..global_variables import (
    ARM_J1_X,
    ARM_J2_X,
    BOX_COLLISION_PATH,
    BOX_LINK_PATH,
    BOX_MASS_KG,
    BOX_SIZE_X,
    BOX_SIZE_Y,
    BOX_SIZE_Z,
    BOX_VISUAL_PATH,
    CAMERA_DEFS,
    PLATFORM_DEPTH_BELOW_BASE,
    ROBOT_FOOTPRINT_CENTER_XY,
    ROBOT_MOUNT_FINE_XY,
    ROBOT_MOUNT_FINE_Z,
    ROBOT_PRIM_PATH,
    TABLE_CENTER,
    TABLE_LEG_HEIGHT,
    TABLE_LEG_INSET,
    TABLE_LEG_SIZE,
    TABLE_LENGTH,
    TABLE_PRIM_PATH,
    TABLE_TOP_THICKNESS,
    TABLE_TOP_Z,
    TABLE_WIDTH,
)
from ..paths import load_box_meta, resolve_robot_usd
from .box_visual_loader import apply_box_textures, load_box_visual
from .pose_utils import Pose6D, set_pose


class SceneLoader:
    """在 USD Stage 上构建 GraspNet 仿真场景。

    Attributes:
        world: Isaac World 实例（物理步长 1/60 s）。
        robot_prim_path: 机器人根路径，默认 ``/World/nova_robot``。
        table_prim_path: 桌子根路径。
        box_link_path: 抓取盒刚体根路径（TF / 位姿写入此 prim）。
        table_top_z: 桌面顶面世界坐标 z（米）。
        camera_prim_paths: Load 后解析到的三路相机 prim 路径 ``{cam0|cam1|cam2: path}``。
    """

    def __init__(self, robot_dir: str, box_dir: str):
        """初始化资源目录。

        Args:
            robot_dir: ``data/robot``，含 ``nova_robot_prepared.usda``。
            box_dir: ``data/box``，含 ``grasp_box.usda`` 与 ``grasp_box_meta.json``。
        """
        self._robot_dir = robot_dir
        self._box_dir = box_dir
        self.world: Optional["World"] = None
        self.robot_prim_path = ROBOT_PRIM_PATH
        self.table_prim_path = TABLE_PRIM_PATH
        self.box_link_path = BOX_LINK_PATH
        self.table_top_z = TABLE_TOP_Z
        self._camera_prim_paths: Dict[str, str] = {}
        self._loaded = False
        self._physics_initialized = False
        self._mount_xy: tuple[float, float] = (0.0, 0.0)
        self._arm_center_xy: tuple[float, float] = (0.0, 0.0)
        self._mount_z: float = TABLE_TOP_Z + PLATFORM_DEPTH_BELOW_BASE

    @property
    def mount_xy(self) -> tuple[float, float]:
        """Load 后 base_link 原点在桌面的 xy（原点在 J1 侧基座，非台面中心）。"""
        return self._mount_xy

    @property
    def arm_center_xy(self) -> tuple[float, float]:
        """Load 后双臂 J1/J2 中点在世界系 xy。"""
        return self._arm_center_xy

    @property
    def default_box_center(self) -> tuple[float, float, float]:
        """Load 后抓取盒默认世界系中心（双臂中点、机器人平台顶面）。"""
        return (
            self._arm_center_xy[0],
            self._arm_center_xy[1],
            self._mount_z + BOX_SIZE_Z * 0.5,
        )

    @property
    def is_loaded(self) -> bool:
        """场景是否已成功 Load（与 Timeline Play 无关）。"""
        return self._loaded

    @property
    def camera_prim_paths(self) -> Dict[str, str]:
        """返回相机路径副本，供 ROS 出流配置使用。"""
        return dict(self._camera_prim_paths)

    def load(self, box_pose: Optional[Pose6D] = None) -> bool:
        """加载完整场景到当前 Stage。

        Args:
            box_pose: 抓取盒世界系 6D 位姿；``None`` 时使用 Load 后实测双臂中心上方。

        Returns:
            成功为 ``True``；任一步骤失败则回滚 ``_loaded`` 并返回 ``False``。
        """
        try:
            from isaacsim.core.api.world import World
            from isaacsim.core.utils.stage import get_current_stage

            from .kit_extensions import ensure_core_api_enabled

            if not ensure_core_api_enabled():
                print("SceneLoader: isaacsim.core.api not available")
                return False

            stage = get_current_stage()
            if stage is None:
                print("SceneLoader: no USD stage")
                return False

            if self.world is None:
                self.world = World(physics_dt=1.0 / 60.0)

            self._prepare_stage(stage)
            self.world.scene.add_default_ground_plane()
            self._ensure_lights(stage)

            if not self._load_robot(stage):
                return False

            self._mount_robot_on_table(stage)
            self._remove_aruco_boards(stage)
            self._ensure_camera_payloads_loaded(stage)
            self._fix_sensor_mount_physics(stage)

            self._create_table(stage)
            if box_pose is None:
                box_pose = Pose6D(self.default_box_center, (0.0, 0.0, 0.0))
            self._create_box(stage, box_pose)

            self._resolve_camera_paths(stage)

            self._reset_robot_joints_zero_usd(stage)
            self._stop_timeline()
            self._loaded = True
            print("SceneLoader: table + nova_robot + box ready — Press Play to stream ROS")
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SceneLoader.load failed: {exc}")
            self._loaded = False
            return False

    def unload(self) -> None:
        """移除机器人、桌子、抓取盒 prim，并清空相机路径缓存。"""
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                for path in (
                    self.robot_prim_path,
                    self.table_prim_path,
                    self.box_link_path,
                ):
                    if stage.GetPrimAtPath(path):
                        stage.RemovePrim(path)
        except Exception as exc:
            print(f"SceneLoader.unload: {exc}")
        self._camera_prim_paths.clear()
        self._loaded = False
        self._physics_initialized = False
        self.world = None

    def _prepare_stage(self, stage) -> None:
        """确保存在 ``/World``，并删除除地面外的旧子节点（避免重复 Load 叠加）。"""
        world = stage.GetPrimAtPath("/World")
        if not world or not world.IsValid():
            UsdGeom.Xform.Define(stage, "/World")
        for child in list(stage.GetPrimAtPath("/World").GetChildren()):
            if child.GetName() not in ("GroundPlane", "groundPlane", "defaultGroundPlane"):
                stage.RemovePrim(child.GetPath())

    def _ensure_lights(self, stage) -> None:
        """添加 Dome + Distant 光（若尚未存在）。"""
        if stage.GetPrimAtPath("/World/DomeLight"):
            return
        dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        dome.CreateIntensityAttr(800.0)
        distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
        distant.CreateIntensityAttr(2500.0)
        xform = UsdGeom.Xformable(distant)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 45.0, 0.0))

    def _load_robot(self, stage) -> bool:
        """引用 ``nova_robot_prepared.usda``（或回退 ``nova_robot.usda``）到 ``ROBOT_PRIM_PATH``。"""
        from isaacsim.core.utils.stage import add_reference_to_stage

        usd_path = resolve_robot_usd(self._robot_dir)
        if not usd_path:
            print(f"SceneLoader: nova_robot.usda not found under {self._robot_dir}")
            return False
        add_reference_to_stage(usd_path=usd_path, prim_path=self.robot_prim_path)
        self._strip_embedded_action_graph(stage)
        try:
            stage.Load(Sdf.Path(self.robot_prim_path), Usd.LoadWithDescendants)
        except Exception as exc:
            print(f"SceneLoader: robot stage.Load: {exc}")
        robot = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot or not robot.IsValid():
            print(f"SceneLoader: robot prim missing at {self.robot_prim_path}")
            return False
        try:
            robot.Load()
        except Exception as exc:
            print(f"SceneLoader: robot prim.Load: {exc}")
        self._finalize_robot_visuals(stage)
        print(f"SceneLoader: loaded robot from {usd_path}")
        return True

    @staticmethod
    def _hide_visual_only(prim) -> int:
        """仅隐藏渲染，不改 purpose（保留 PhysX 碰撞）。"""
        count = 0
        for node in Usd.PrimRange(prim):
            if not node.IsA(UsdGeom.Imageable):
                continue
            UsdGeom.Imageable(node).MakeInvisible()
            count += 1
        return count

    @staticmethod
    def _hide_imageable_subtree(prim) -> int:
        """将 prim 及其子树设为 guide + invisible。"""
        count = 0
        for node in Usd.PrimRange(prim):
            if not node.IsA(UsdGeom.Imageable):
                continue
            imageable = UsdGeom.Imageable(node)
            imageable.MakeInvisible()
            imageable.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
            count += 1
        return count

    def _finalize_robot_visuals(self, stage) -> None:
        """显示 link 渲染 mesh，隐藏碰撞代理（collisions + base_link 龙门架 Cube）。"""
        robot = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot or not robot.IsValid():
            return

        hidden = 0
        shown = 0

        for prim in Usd.PrimRange(robot):
            if prim.GetName() == "collisions":
                hidden += self._hide_visual_only(prim)

        base_visuals = stage.GetPrimAtPath(f"{self.robot_prim_path}/base_link/visuals")
        if base_visuals and base_visuals.IsValid():
            for prim in Usd.PrimRange(base_visuals):
                if prim.GetName().startswith("mesh_") or prim.IsA(UsdGeom.Cube):
                    hidden += self._hide_imageable_subtree(prim)

        for prim in Usd.PrimRange(robot):
            if prim.GetName() != "visuals":
                continue
            if prim.GetParent() and prim.GetParent().GetName() == "base_link":
                continue
            if prim.IsA(UsdGeom.Imageable):
                UsdGeom.Imageable(prim).MakeVisible()
                shown += 1
            for desc in Usd.PrimRange(prim):
                if not desc.IsA(UsdGeom.Imageable):
                    continue
                if desc.GetName().startswith("mesh_") or desc.IsA(UsdGeom.Cube):
                    continue
                vis_attr = UsdGeom.Imageable(desc).GetVisibilityAttr()
                if vis_attr and vis_attr.Get() == UsdGeom.Tokens.invisible:
                    UsdGeom.Imageable(desc).MakeVisible()
                    shown += 1

        base_mesh = stage.GetPrimAtPath(
            f"{self.robot_prim_path}/base_link/visuals/base_link"
        )
        if base_mesh and base_mesh.IsValid() and base_mesh.IsA(UsdGeom.Imageable):
            UsdGeom.Imageable(base_mesh).MakeVisible()
            shown += 1

        if hidden or shown:
            print(
                f"SceneLoader: robot visuals — shown {shown} prim(s), "
                f"hid {hidden} collision proxy prim(s)"
            )

    def _measure_support_bottom_world(self, stage) -> Optional[float]:
        """机器人当前位姿下，平台支撑面最低点世界坐标 z。"""
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        for path in (
            f"{self.robot_prim_path}/base_link/visuals/base_link",
            f"{self.robot_prim_path}/base_link/visuals",
            f"{self.robot_prim_path}/base_link",
        ):
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            bound = cache.ComputeWorldBound(prim)
            box_range = bound.GetRange()
            if not box_range.IsEmpty():
                return float(box_range.GetMin()[2])
        return None

    def _mount_robot_on_table(self, stage) -> None:
        """把台面几何中心对齐桌心，平台底面贴桌面（可 ROBOT_MOUNT_FINE_Z 微调）。

        base_link 原点是 URDF 定义的 J1 平台顶面角点，不是整台中心。
        z 方向：先临时放到 z=0，量平台 mesh 世界包围盒底面，再对齐到 TABLE_TOP_Z。
        """
        footprint_cx, footprint_cy = self._measure_footprint_center_xy(stage)
        mount_x = TABLE_CENTER[0] - footprint_cx + ROBOT_MOUNT_FINE_XY[0]
        mount_y = TABLE_CENTER[1] - footprint_cy + ROBOT_MOUNT_FINE_XY[1]

        target_bottom_z = TABLE_TOP_Z + ROBOT_MOUNT_FINE_Z
        set_pose(
            self.robot_prim_path,
            Pose6D((mount_x, mount_y, 0.0), (0.0, 0.0, 0.0)),
            stage,
        )
        support_bottom = self._measure_support_bottom_world(stage)
        if support_bottom is not None:
            mount_z = target_bottom_z - support_bottom
        else:
            depth = self._measure_platform_depth_below_base(stage)
            mount_z = TABLE_TOP_Z + depth + ROBOT_MOUNT_FINE_Z
            support_bottom = mount_z - depth

        self._mount_xy = (mount_x, mount_y)
        self._mount_z = mount_z
        self._arm_center_xy = (
            mount_x + (ARM_J1_X + ARM_J2_X) * 0.5,
            mount_y,
        )
        set_pose(
            self.robot_prim_path,
            Pose6D((mount_x, mount_y, mount_z), (0.0, 0.0, 0.0)),
            stage,
        )
        final_bottom = self._measure_support_bottom_world(stage)
        print(
            f"SceneLoader: robot mounted at ({mount_x:.3f}, {mount_y:.3f}, {mount_z:.3f}), "
            f"platform bottom z={final_bottom:.3f} (target {target_bottom_z:.3f}), "
            f"arm midpoint=({self._arm_center_xy[0]:.3f}, {self._arm_center_xy[1]:.3f})"
        )

    def _measure_footprint_center_xy(self, stage) -> tuple[float, float]:
        """base_link 本地系下整台台面 mesh 包围盒中心（用于对齐桌心）。"""
        base_path = f"{self.robot_prim_path}/base_link"
        base_prim = stage.GetPrimAtPath(base_path)
        if not base_prim or not base_prim.IsValid():
            return ROBOT_FOOTPRINT_CENTER_XY
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bound = cache.ComputeLocalBound(base_prim)
        box_range = bound.GetRange()
        if box_range.IsEmpty():
            return ROBOT_FOOTPRINT_CENTER_XY
        mn = box_range.GetMin()
        mx = box_range.GetMax()
        return (
            (float(mn[0]) + float(mx[0])) * 0.5,
            (float(mn[1]) + float(mx[1])) * 0.5,
        )

    def _measure_platform_depth_below_base(self, stage) -> float:
        """测量 base_link 上表面 (z=0) 到网格底面的距离。"""
        base_path = f"{self.robot_prim_path}/base_link"
        base_prim = stage.GetPrimAtPath(base_path)
        if not base_prim or not base_prim.IsValid():
            return PLATFORM_DEPTH_BELOW_BASE
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bound = cache.ComputeLocalBound(base_prim)
        box_range = bound.GetRange()
        if box_range.IsEmpty():
            return PLATFORM_DEPTH_BELOW_BASE
        min_z = float(box_range.GetMin()[2])
        if min_z >= 0.0:
            return PLATFORM_DEPTH_BELOW_BASE
        return -min_z

    def _remove_aruco_boards(self, stage) -> None:
        """移除标定板几何、mesh 实例及 aruco_mat 材质（贴图路径在拷贝 USD 中无效）。"""
        robot = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot or not robot.IsValid():
            return
        paths = []
        for prim in Usd.PrimRange(robot):
            name = prim.GetName().lower()
            path_str = str(prim.GetPath()).lower()
            if "aruco" in name or "/aruco" in path_str:
                paths.append(prim.GetPath())
        paths.sort(key=lambda p: len(str(p).split("/")), reverse=True)
        removed = 0
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                stage.RemovePrim(path)
                removed += 1
        if removed:
            print(f"SceneLoader: removed {removed} aruco-related prim(s) (boards + materials)")

    def _strip_embedded_action_graph(self, stage) -> None:
        """删除拷贝 USD 内嵌的 OmniGraph（与本扩展 ROS 图冲突）。"""
        ag_path = f"{self.robot_prim_path}/ActionGraph"
        prim = stage.GetPrimAtPath(ag_path)
        if prim and prim.IsValid():
            stage.RemovePrim(ag_path)
            print(f"SceneLoader: removed embedded ActionGraph at {ag_path}")

    def _ensure_camera_payloads_loaded(self, stage) -> None:
        """RSD455 为 payload：同步 Load，不在 Load 回调里 pump app.update（会触发 Vulkan 崩溃）。"""
        mount_suffixes = (
            "base_link/cam0",
            "J1_6/cam1",
            "J2_6/cam2",
        )
        for suffix in mount_suffixes:
            mount_path = f"{self.robot_prim_path}/{suffix}"
            mount = stage.GetPrimAtPath(mount_path)
            if not mount or not mount.IsValid():
                print(f"SceneLoader: camera mount not found: {mount_path}")
                continue
            try:
                stage.Load(Sdf.Path(mount_path), Usd.LoadWithDescendants)
            except Exception as exc:
                print(f"SceneLoader: stage.Load {mount_path}: {exc}")
                try:
                    mount.Load()
                except Exception as exc2:
                    print(f"SceneLoader: prim.Load {mount_path}: {exc2}")

        missing = []
        for suffix in mount_suffixes:
            rsd_path = f"{self.robot_prim_path}/{suffix}/RSD455"
            rsd = stage.GetPrimAtPath(rsd_path)
            if not rsd or not rsd.IsValid():
                missing.append(suffix)
        if missing:
            print(f"SceneLoader: WARN RSD455 not yet present for {missing} (need network for payload)")
        else:
            print("SceneLoader: RSD455 payloads present")

    def _strip_prim_physics(self, prim) -> int:
        """递归剥离 prim 上的刚体/碰撞 API，返回移除的 API 数量。"""
        count = 0
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            count += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
            count += 1
        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI)
            count += 1
        if PhysxSchema is not None:
            for api in (
                PhysxSchema.PhysxRigidBodyAPI,
                PhysxSchema.PhysxCollisionAPI,
                PhysxSchema.PhysxConvexHullCollisionAPI,
                PhysxSchema.PhysxTriangleMeshCollisionAPI,
            ):
                if prim.HasAPI(api):
                    prim.RemoveAPI(api)
                    count += 1
        attr = prim.GetAttribute("physics:collisionEnabled")
        if attr and attr.IsValid():
            attr.Set(False)
        rb_attr = prim.GetAttribute("physics:rigidBodyEnabled")
        if rb_attr and rb_attr.IsValid():
            rb_attr.Set(False)
        return count

    def _fix_sensor_mount_physics(self, stage) -> None:
        """RSD455 payload 自带碰撞/刚体，作为 link 子节点会触发 PhysX 嵌套刚体错误。"""
        mount_suffixes = (
            "base_link/cam0",
            "J1_6/cam1",
            "J2_6/cam2",
        )
        fixed = 0
        for suffix in mount_suffixes:
            mount_path = f"{self.robot_prim_path}/{suffix}"
            mount = stage.GetPrimAtPath(mount_path)
            if not mount or not mount.IsValid():
                print(f"SceneLoader: camera mount not found: {mount_path}")
                continue

            # 原 nova_isaac_sim/2.usda 无 resetXformStack；须保持相机随父 link 运动
            xformable = UsdGeom.Xformable(mount)
            if xformable:
                xformable.SetResetXformStack(False)
            reset_attr = mount.GetAttribute("xformOp:resetXformStack")
            if reset_attr and reset_attr.IsValid():
                reset_attr.Set(False)

            for prim in Usd.PrimRange(mount):
                fixed += self._strip_prim_physics(prim)

            print(f"SceneLoader: fixed camera mount physics at {mount_path}")

        if fixed:
            print(f"SceneLoader: stripped {fixed} physics API(s) on camera mounts")

    def fix_camera_physics(self, stage) -> None:
        """公开入口：Play 前可再次调用，确保相机 mount 无嵌套刚体。"""
        self._fix_sensor_mount_physics(stage)

    def _create_table(self, stage) -> None:
        """创建不锈钢桌：顶板 + 四角腿 + 长边中间两条腿（共 6 腿）， kinematic 刚体。"""
        table_xform = UsdGeom.Xform.Define(stage, self.table_prim_path)
        set_pose(
            self.table_prim_path,
            Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            stage,
        )

        material = self._create_stainless_material(
            stage, f"{self.table_prim_path}/StainlessSteel"
        )

        # 桌面中心高度：顶面 z=TABLE_TOP_Z，板厚 TABLE_TOP_THICKNESS
        top_z = TABLE_TOP_Z - TABLE_TOP_THICKNESS * 0.5
        self._add_table_part(
            stage,
            f"{self.table_prim_path}/top",
            (0.0, 0.0, top_z),
            (TABLE_LENGTH, TABLE_WIDTH, TABLE_TOP_THICKNESS),
            material,
        )

        half_l = TABLE_LENGTH * 0.5 - TABLE_LEG_INSET
        half_w = TABLE_WIDTH * 0.5 - TABLE_LEG_INSET
        leg_z = TABLE_LEG_HEIGHT * 0.5
        # 腿位：四角 + 长边 (Y±) 中点各一条
        leg_positions = (
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, half_w),
            (-half_l, -half_w),
            (0.0, half_w),
            (0.0, -half_w),
        )
        for index, (lx, ly) in enumerate(leg_positions):
            self._add_table_part(
                stage,
                f"{self.table_prim_path}/leg_{index}",
                (lx, ly, leg_z),
                (TABLE_LEG_SIZE, TABLE_LEG_SIZE, TABLE_LEG_HEIGHT),
                material,
            )

        rb = UsdPhysics.RigidBodyAPI.Apply(table_xform.GetPrim())
        rb.CreateKinematicEnabledAttr(True)
        self.table_top_z = TABLE_TOP_Z
        print(
            f"SceneLoader: table {TABLE_LENGTH}×{TABLE_WIDTH} m, "
            f"top z={TABLE_TOP_Z:.3f}, 6 legs"
        )

    def _create_stainless_material(self, stage, mat_path: str):
        """创建不锈钢 PreviewSurface 材质并返回 Material prim。"""
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.92)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.28)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.78, 0.78, 0.8)
        )
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    def _add_table_part(
        self,
        stage,
        prim_path: str,
        center: tuple[float, float, float],
        size: tuple[float, float, float],
        material,
    ) -> None:
        """添加桌子的一个立方体部件（translate + scale 定义尺寸与位置）。"""
        mesh = UsdGeom.Cube.Define(stage, prim_path)
        mesh.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(mesh)
        xform.AddTranslateOp().Set(Gf.Vec3d(*center))
        xform.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)

    def _box_collision_size(self) -> tuple[float, float, float]:
        """从 ``grasp_box_meta.json`` 读取包围盒尺寸，供简化碰撞体使用。"""
        meta = load_box_meta(self._box_dir)
        if meta and "size_m" in meta:
            size = meta["size_m"]
            return (
                float(size.get("x", BOX_SIZE_X)),
                float(size.get("y", BOX_SIZE_Y)),
                float(size.get("z", BOX_SIZE_Z)),
            )
        return (float(BOX_SIZE_X), float(BOX_SIZE_Y), float(BOX_SIZE_Z))

    def _create_box(self, stage, pose: Pose6D) -> None:
        """创建抓取盒：USD/OBJ 视觉 + 轴对齐盒碰撞 + 动态刚体。"""
        link = UsdGeom.Xform.Define(stage, self.box_link_path)
        set_pose(self.box_link_path, pose, stage)
        UsdGeom.Xform.Define(stage, BOX_VISUAL_PATH)

        # 视觉 mesh 在 deferred_post_load 中构建（OBJ 解析约 20–40s，避免阻塞 Load 按钮）
        sx, sy, sz = self._box_collision_size()
        collision = UsdGeom.Cube.Define(stage, BOX_COLLISION_PATH)
        collision.CreateSizeAttr(1.0)
        UsdGeom.Xformable(collision).AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
        try:
            UsdGeom.Imageable(collision).CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
        except Exception:
            pass

        rb = UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(False)
        mass = UsdPhysics.MassAPI.Apply(link.GetPrim())
        mass.CreateMassAttr(float(BOX_MASS_KG))
        print(f"SceneLoader: box collision size ({sx:.3f}, {sy:.3f}, {sz:.3f}) m")

    def build_box_visual_if_needed(self, stage) -> None:
        """延迟构建带贴图的盒子 mesh（Load 后异步调用）。"""
        mesh_path = f"{BOX_VISUAL_PATH}/mesh"
        prim = stage.GetPrimAtPath(mesh_path)
        if prim and prim.IsValid() and prim.IsA(UsdGeom.Mesh):
            return
        if load_box_visual(stage, self._box_dir):
            return
        print("SceneLoader: WARN box textured mesh unavailable")

    def fix_box_visual_material(self, stage=None) -> None:
        """Load 后延迟调用：等 payload 就绪后再次应用贴图。"""
        if stage is None:
            stage = omni.usd.get_context().get_stage()
        if stage and self._loaded:
            apply_box_textures(stage, self._box_dir)

    def _resolve_camera_paths(self, stage) -> None:
        """按 ``CAMERA_DEFS`` 解析 cam0/cam1/cam2 的 ``Camera_Pseudo_Depth`` prim 路径。"""
        self._camera_prim_paths.clear()
        for spec in CAMERA_DEFS:
            key = spec["key"]
            path = f"{self.robot_prim_path}/{spec['prim_suffix']}"
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                self._camera_prim_paths[key] = path
                print(f"SceneLoader: camera {key} -> {path}")
            else:
                print(f"SceneLoader: WARN camera prim not found: {path}")

    def invalidate_physics(self) -> None:
        """Timeline Stop / Unload 后清除标记，下次 Play 重新 soft reset。"""
        self._physics_initialized = False

    def ensure_physics_on_play(self) -> bool:
        """Play 期间做一次 ``world.reset(soft=True)``，避免 Stop 态 reset 导致 tensor view 失效。"""
        if not self._loaded or self.world is None or self._physics_initialized:
            return self._physics_initialized
        import omni.timeline

        if not omni.timeline.get_timeline_interface().is_playing():
            return False
        try:
            self.world.reset(soft=True)
            self._physics_initialized = True
            print("SceneLoader: world.reset(soft=True) on Play")
            return True
        except Exception as exc:
            print(f"SceneLoader: world.reset(soft=True) warning: {exc}")
            return False

    def _reset_robot_joints_zero_usd(self, stage) -> None:
        """Load 时通过 USD 属性置关节零位（不调用 Articulation tensor API）。"""
        robot = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot or not robot.IsValid():
            return
        count = 0
        for prim in Usd.PrimRange(robot):
            if prim.IsA(UsdPhysics.RevoluteJoint):
                for attr_name in (
                    "state:angular:physics:position",
                    "state:angular:physics:velocity",
                    "drive:angular:physics:targetPosition",
                    "drive:angular:physics:targetVelocity",
                ):
                    attr = prim.GetAttribute(attr_name)
                    if attr and attr.IsValid():
                        attr.Set(0.0)
                count += 1
            elif prim.IsA(UsdPhysics.PrismaticJoint):
                for attr_name in (
                    "state:linear:physics:position",
                    "state:linear:physics:velocity",
                    "drive:linear:physics:targetPosition",
                    "drive:linear:physics:targetVelocity",
                ):
                    attr = prim.GetAttribute(attr_name)
                    if attr and attr.IsValid():
                        attr.Set(0.0)
                count += 1
        if count:
            print(f"SceneLoader: zeroed {count} joint(s) via USD")

    def prepare_sim_on_play(self) -> None:
        """兼容旧调用：Play 时注册 PhysX（由 Session 延迟触发）。"""
        self.ensure_physics_on_play()

    @staticmethod
    def _stop_timeline() -> None:
        """Load 结束时强制 Stop，避免未 Play 时物理/ROS 图误触发。"""
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            timeline.stop()
