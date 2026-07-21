# -*- coding: utf-8 -*-
"""GraspNet 场景加载：不锈钢桌、Nova 双臂、抓取盒、相机物理修复。

优先加载 ``data/scenes/nova_graspnet_scene.usda``（采集对齐的总场景）。
无总场景时回退到分步拼装：

  1. 清空 /World 旧内容 → 深色程序化地面
  2. 引用 nova_robot USD → 挂载到桌面 → 清理 ArUco / ActionGraph
  3. 加载本地 RSD455 payload 并剥离相机刚体
  4. 创建 6 腿桌子 + OBJ/USD 抓取盒
  5. Timeline 保持 Stop（PhysX reset 延至 Play 后）
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING, Dict, Optional

import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

if TYPE_CHECKING:
    from isaacsim.core.api.world import World

try:
    from pxr import PhysxSchema
except ImportError:
    PhysxSchema = None

from ..global_variables import (
    ARM1_BASE_LINK,
    ARM1_GRIPPER_FINGERS,
    ARM2_BASE_LINK,
    ARM2_GRIPPER_FINGERS,
    ARM_DRIVE_SPECS,
    ARM_J1_X,
    ARM_J2_X,
    ARM_MAX_JOINT_VELOCITY_DEG_S,
    ARM_WRIST_MAX_JOINT_VELOCITY_DEG_S,
    BOX_COLLISION_GEO_ROOT,
    BOX_COLLISION_PATH,
    BOX_LINK_PATH,
    BOX_MASS_KG,
    BOX_SIZE_X,
    BOX_SIZE_Y,
    BOX_SIZE_Z,
    BOX_VISUAL_PATH,
    CAMERA_DEFS,
    CAPTURE_LIGHTS_PATH,
    FLAT_GRID_PATH,
    GEMINI335_PRIM_PATH,
    GEMINI335_RGB_CAMERA_PATH,
    GRASP_DYNAMIC_FRICTION,
    GRASP_PHYSICS_MATERIAL_PATH,
    GRASP_RESTITUTION,
    GRASP_STATIC_FRICTION,
    GRIPPER_DRIVE_DAMPING,
    GRIPPER_DRIVE_MAX_FORCE,
    GRIPPER_DRIVE_STIFFNESS,
    GROUND_PLANE_COLOR,
    GROUND_PLANE_PRIM_PATH,
    PLATFORM_DEPTH_BELOW_BASE,
    ROBOT_FOOTPRINT_CENTER_XY,
    ROBOT_MOUNT_FINE_XY,
    ROBOT_MOUNT_FINE_Z,
    ROBOT_PRIM_PATH,
    ROBOT_ROOT_JOINT_PATH,
    SCENE_ROBOT_MOUNT_XYZ,
    TABLE_CENTER,
    TABLE_LEG_HEIGHT,
    TABLE_LEG_INSET,
    TABLE_LEG_SIZE,
    TABLE_LENGTH,
    TABLE_PRIM_PATH,
    TABLE_TOP_THICKNESS,
    DEFAULT_BOX_CENTER,
    DEFAULT_BOX_POSE_RPY,
    DEFAULT_JOINT_ANGLES_DEG,
    TABLE_TOP_Z,
    TABLE_WIDTH,
    WORKSPACE_COLLISION_FILL_NAME,
    WORKSPACE_COLLISION_INSET_M,
    WORKSPACE_BASE_LINK_MESH_TOP_Z,
)
from ..mesh_bounds import base_link_stl_z_bounds
from ..paths import load_box_meta, resolve_robot_usd, resolve_scene_usd
from .box_physics import (
    configure_box_usd_dynamic,
    lift_box_clear_of_surface,
    release_box_for_gravity,
)
from .box_mesh_builder import ensure_box_collision, has_box_collision, remove_legacy_box_collision_cube, setup_box_collision
from .box_visual_loader import _box_has_visible_mesh, apply_box_textures, load_box_visual
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
        self._data_dir = os.path.dirname(os.path.abspath(robot_dir))
        self.world: Optional["World"] = None
        self.robot_prim_path = ROBOT_PRIM_PATH
        self.table_prim_path = TABLE_PRIM_PATH
        self.box_link_path = BOX_LINK_PATH
        self.table_top_z = TABLE_TOP_Z
        self._camera_prim_paths: Dict[str, str] = {}
        self._loaded = False
        self._physics_initialized = False
        self._box_rigid = None
        self._table_rigid = None
        self._robot_articulation = None
        self._mount_xy: tuple[float, float] = (0.0, 0.0)
        self._arm_center_xy: tuple[float, float] = (0.0, 0.0)
        self._mount_z: float = TABLE_TOP_Z + PLATFORM_DEPTH_BELOW_BASE
        self._workspace_surface_z_world: float = (
            TABLE_TOP_Z + WORKSPACE_BASE_LINK_MESH_TOP_Z
        )
        self._box_visual_ready = False
        self._master_scene_path: Optional[str] = None
        self._using_master_scene = False

    @property
    def mount_xy(self) -> tuple[float, float]:
        """Load 后 base_link 原点在桌面的 xy（原点在 J1 侧基座，非台面中心）。"""
        return self._mount_xy

    @property
    def arm_center_xy(self) -> tuple[float, float]:
        """Load 后双臂 J1/J2 中点在世界系 xy。"""
        return self._arm_center_xy

    @property
    def workspace_surface_z_world(self) -> float:
        """Load 后 base_link.stl 顶面在世界系 z（与 workspace_fill 顶齐）。"""
        return self._workspace_surface_z_world

    @property
    def default_box_center(self) -> tuple[float, float, float]:
        """抓取盒默认世界系中心（UI / 场景初始值）。"""
        return DEFAULT_BOX_CENTER

    @property
    def robot_articulation(self):
        return self._robot_articulation

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
            box_pose: 抓取盒世界系 6D 位姿；``None`` 时使用默认盒心。

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

            master = resolve_scene_usd(self._data_dir)
            if master:
                return self._load_master_scene(stage, master, box_pose)
            print("SceneLoader: master scene missing — falling back to procedural load")
            return self._load_procedural_scene(stage, box_pose)
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SceneLoader.load failed: {exc}")
            self._loaded = False
            self._using_master_scene = False
            return False

    def _load_master_scene(
        self, stage, scene_usd: str, box_pose: Optional[Pose6D]
    ) -> bool:
        """加载 ``nova_graspnet_scene.usda``（相机/灯光/环境已按采集场景对齐）。"""
        self._detach_master_scene_sublayer(stage)
        self._prepare_stage(stage)
        self._clear_root_scene_prims(stage)

        root = stage.GetRootLayer()
        abs_scene = os.path.abspath(scene_usd)
        if abs_scene not in list(root.subLayerPaths):
            root.subLayerPaths.insert(0, abs_scene)
        self._master_scene_path = abs_scene
        self._using_master_scene = True

        robot = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot or not robot.IsValid():
            print(f"SceneLoader: master scene missing robot at {self.robot_prim_path}")
            self._detach_master_scene_sublayer(stage)
            self._using_master_scene = False
            return False

        mx, my, mz = SCENE_ROBOT_MOUNT_XYZ
        self._mount_xy = (float(mx), float(my))
        self._mount_z = float(mz)
        self._arm_center_xy = (
            float(mx) + 0.5 * (ARM_J1_X + ARM_J2_X),
            float(my),
        )
        self._workspace_surface_z_world = float(mz) + WORKSPACE_BASE_LINK_MESH_TOP_Z

        self._strip_embedded_action_graph(stage)
        self._remove_aruco_boards(stage)
        self._ensure_camera_payloads_loaded(stage)
        self._fix_sensor_mount_physics(stage)
        self._boost_arm_drives(stage)
        self._boost_gripper_drives(stage)
        try:
            self._ensure_base_platform_collision_fill(stage)
        except Exception as exc:
            print(f"SceneLoader: workspace collision fill skipped: {exc}")

        if box_pose is not None:
            set_pose(self.box_link_path, box_pose, stage)
            print(
                f"SceneLoader: override grasp_box pose "
                f"t={box_pose.translation} rpy={box_pose.rotation_deg}"
            )

        # Ensure collision / visual + texture (master scene mesh may lack MaterialBindingAPI)
        try:
            ensure_box_collision(stage, self._box_dir)
        except Exception as exc:
            print(f"SceneLoader: box collision ensure: {exc}")
        if not _box_has_visible_mesh(stage, self.box_link_path):
            load_box_visual(stage, self._box_dir)
        else:
            apply_box_textures(stage, self._box_dir)
        self._apply_grasp_friction(stage)

        self._fix_flatgrid_texture_paths(stage)
        self._ensure_gemini335_visual(stage)
        # cam0 绑 Gemini335 Stream_rgb，须在引用就绪后再解析
        self._resolve_camera_paths(stage)
        self._sync_cam0_tf_to_gemini_rgb(stage)
        self._apply_default_robot_joints_usd(stage)
        self._stop_timeline()
        self._loaded = True
        print(
            f"SceneLoader: master scene ready ({os.path.basename(abs_scene)}) — "
            "cam/light/env capture-aligned; Press Play to stream ROS"
        )
        return True

    def _ensure_gemini335_visual(self, stage) -> None:
        """确保 /World/Gemini335 存在并引用 Orbbec USD（采集位姿；供 cam0 出流）。"""
        gemini_usd = os.path.join(self._data_dir, "sensors", "orbbec_gemini_335.usd")
        if not os.path.isfile(gemini_usd):
            print(
                "SceneLoader: Gemini335 asset missing — "
                f"place orbbec_gemini_335.usd under {os.path.join(self._data_dir, 'sensors')}"
            )
            return

        # 与 bake_graspnet_scene / 采集场景一致的机位
        gemini_t = (0.0, -0.31703293039677527, 1.6606134042644396)
        gemini_q = (
            -1.6081226496766364e-16,
            1.0460184128151074e-16,
            0.9659258262890684,
            0.2588190451025208,
        )

        prim = stage.GetPrimAtPath(GEMINI335_PRIM_PATH)
        if not prim or not prim.IsValid():
            xform = UsdGeom.Xform.Define(stage, GEMINI335_PRIM_PATH)
            xf = UsdGeom.Xformable(xform)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*gemini_t))
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Quatd(gemini_q[0], Gf.Vec3d(*gemini_q[1:]))
            )
            xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
            prim = xform.GetPrim()
            print(f"SceneLoader: created {GEMINI335_PRIM_PATH} at capture pose")

        # Already has real meshes from Orbbec asset?
        has_mesh = False
        for child in Usd.PrimRange(prim):
            if child.IsA(UsdGeom.Mesh):
                pts = UsdGeom.Mesh(child).GetPointsAttr().Get()
                if pts and len(pts) > 0:
                    has_mesh = True
                    break
        if has_mesh:
            print("SceneLoader: Gemini335 visual OK (mesh present)")
            return

        # Remove stand-in Camera, add asset reference
        cam = stage.GetPrimAtPath(f"{GEMINI335_PRIM_PATH}/Camera")
        if cam and cam.IsValid():
            stage.RemovePrim(cam.GetPath())
        try:
            refs = prim.GetReferences()
            refs.ClearReferences()
            refs.AddReference(os.path.abspath(gemini_usd))
            try:
                stage.Load(Sdf.Path(GEMINI335_PRIM_PATH), Usd.LoadWithDescendants)
            except Exception:
                pass
            print(f"SceneLoader: Gemini335 referenced {gemini_usd}")
        except Exception as exc:
            print(f"SceneLoader: Gemini335 reference failed: {exc}")

    def _fix_flatgrid_texture_paths(self, stage) -> None:
        """把 Grid 材质相对贴图改成绝对路径，避免 Fabric/Hydra 丢 layer 锚点。"""
        from pxr import Sdf

        tex_dir = os.path.join(self._data_dir, "env", "Materials", "Textures")
        mapping = {
            "inputs:diffuse_texture": "Wireframe_blue.png",
            "inputs:emissive_color_texture": "WireframeBlur_basecolor.png",
            "inputs:emissive_mask_texture": "WireframeBlur_blue.png",
        }
        missing = [
            name for name in mapping.values() if not os.path.isfile(os.path.join(tex_dir, name))
        ]
        if missing:
            print(
                f"SceneLoader: WARN FlatGrid textures missing under {tex_dir}: {missing} "
                "(run scripts/download_environment.py)"
            )
            return

        fixed = 0
        for prim in stage.Traverse():
            if prim.GetName() != "Shader":
                continue
            path = str(prim.GetPath())
            if "theGrid" not in path and "FlatGrid" not in path:
                continue
            for attr_name, filename in mapping.items():
                attr = prim.GetAttribute(attr_name)
                if not attr or not attr.IsValid():
                    continue
                abs_tex = os.path.abspath(os.path.join(tex_dir, filename))
                try:
                    attr.Set(Sdf.AssetPath(abs_tex))
                    fixed += 1
                except Exception as exc:
                    print(f"SceneLoader: set {path}.{attr_name} failed: {exc}")
        if fixed:
            print(f"SceneLoader: FlatGrid textures -> absolute paths (x{fixed})")

    def _load_procedural_scene(self, stage, box_pose: Optional[Pose6D]) -> bool:
        """旧路径：运行时拼装桌/机器人/盒（无总场景 USDA 时使用）。"""
        self._using_master_scene = False
        self._master_scene_path = None
        self._prepare_stage(stage)
        self._add_local_ground_plane(stage)
        self._create_table(stage)

        if not self._load_robot(stage):
            return False

        self._mount_robot_on_table(stage)
        try:
            self._ensure_base_platform_collision_fill(stage)
        except Exception as exc:
            print(f"SceneLoader: workspace collision fill skipped: {exc}")
        self._align_cam0_to_gantry_beam(stage)
        self._remove_aruco_boards(stage)
        self._ensure_camera_payloads_loaded(stage)
        self._fix_sensor_mount_physics(stage)

        if box_pose is None:
            box_pose = Pose6D(DEFAULT_BOX_CENTER, DEFAULT_BOX_POSE_RPY)
        self._create_box(stage, box_pose)
        self._apply_grasp_friction(stage)

        self._ensure_gemini335_visual(stage)
        self._resolve_camera_paths(stage)
        self._sync_cam0_tf_to_gemini_rgb(stage)

        self._apply_default_robot_joints_usd(stage)
        self._stop_timeline()
        self._loaded = True
        print("SceneLoader: table + nova_robot + box ready — Press Play to stream ROS")
        return True

    def _clear_root_scene_prims(self, stage) -> None:
        """清除总场景相关的根级 prim（Environment / FlatGrid / CaptureLights）。"""
        for path in (
            "/Environment",
            FLAT_GRID_PATH,
            CAPTURE_LIGHTS_PATH,
            GEMINI335_PRIM_PATH,
            GROUND_PLANE_PRIM_PATH,
            self.table_prim_path,
            self.robot_prim_path,
            self.box_link_path,
        ):
            if stage.GetPrimAtPath(path) and stage.GetPrimAtPath(path).IsValid():
                try:
                    stage.RemovePrim(path)
                except Exception:
                    pass

    def _detach_master_scene_sublayer(self, stage) -> None:
        """从 RootLayer 移除总场景 sublayer。"""
        try:
            root = stage.GetRootLayer()
            paths = list(root.subLayerPaths)
            keep = []
            for p in paths:
                base = os.path.basename(str(p))
                if "nova_graspnet_scene" in base:
                    continue
                if self._master_scene_path and os.path.abspath(str(p)) == os.path.abspath(
                    self._master_scene_path
                ):
                    continue
                keep.append(p)
            if keep != paths:
                root.subLayerPaths = keep
        except Exception as exc:
            print(f"SceneLoader: detach master sublayer: {exc}")
        self._master_scene_path = None

    def unload(self) -> None:
        """移除机器人、桌子、抓取盒 prim，并清空相机路径缓存。"""
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                self._detach_master_scene_sublayer(stage)
                for path in (
                    self.robot_prim_path,
                    self.table_prim_path,
                    self.box_link_path,
                    GEMINI335_PRIM_PATH,
                    GROUND_PLANE_PRIM_PATH,
                    FLAT_GRID_PATH,
                    CAPTURE_LIGHTS_PATH,
                    "/Environment",
                ):
                    if stage.GetPrimAtPath(path):
                        stage.RemovePrim(path)
        except Exception as exc:
            print(f"SceneLoader.unload: {exc}")
        self._camera_prim_paths.clear()
        self._loaded = False
        self._using_master_scene = False
        self._physics_initialized = False
        self._box_visual_ready = False
        self._box_rigid = None
        self._table_rigid = None
        self._robot_articulation = None
        self.world = None

    def _register_table_physics(self) -> bool:
        """桌子注册为 kinematic 刚体，否则盒子会穿过桌面落到地面。"""
        if self._table_rigid is not None:
            return True
        import omni.timeline

        if not omni.timeline.get_timeline_interface().is_playing():
            return False
        try:
            from isaacsim.core.prims import SingleRigidPrim

            self._table_rigid = SingleRigidPrim(
                prim_path=self.table_prim_path, name="graspnet_table"
            )
            self._table_rigid.initialize()
            print(f"SceneLoader: table -> PhysX kinematic ({self.table_prim_path})")
            return True
        except Exception as exc:
            print(f"SceneLoader: table PhysX register failed: {exc}")
            self._table_rigid = None
            return False

    def _prepare_stage(self, stage) -> None:
        """确保存在 ``/World``，并删除除地面外的旧子节点（避免重复 Load 叠加）。"""
        world = stage.GetPrimAtPath("/World")
        if not world or not world.IsValid():
            UsdGeom.Xform.Define(stage, "/World")
        for child in list(stage.GetPrimAtPath("/World").GetChildren()):
            stage.RemovePrim(child.GetPath())

    def _add_local_ground_plane(self, stage) -> None:
        """深色程序化地面（Isaac GroundPlane + PreviewSurface），不访问 Nucleus。"""
        prim_path = GROUND_PLANE_PRIM_PATH
        prim = stage.GetPrimAtPath(prim_path)
        if prim and prim.IsValid():
            return

        import numpy as np

        from isaacsim.core.api.objects import GroundPlane

        GroundPlane(
            prim_path=prim_path,
            z_position=0.0,
            color=np.array(GROUND_PLANE_COLOR, dtype=np.float32),
        )
        print(
            f"SceneLoader: dark ground plane rgb={GROUND_PLANE_COLOR}"
        )

    def apply_default_light_rig(self) -> None:
        """灯光：总场景已含采集对齐球灯时跳过；否则用视口 Default。"""
        if self._using_master_scene:
            print("SceneLoader: keep capture-aligned lights from master scene")
            return
        try:
            import omni.kit.commands

            omni.kit.commands.execute(
                "SetLightingMenuModeCommand",
                lighting_mode="Default",
            )
            print("SceneLoader: viewport lighting rig = Default")
        except Exception as exc:
            print(f"SceneLoader: Default light rig failed: {exc}")

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
        self._boost_arm_drives(stage)
        self._boost_gripper_drives(stage)
        print(f"SceneLoader: loaded robot from {usd_path}")
        return True

    def _set_joint_drive_attrs(self, prim, values: dict) -> None:
        for attr_name, value in values.items():
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid():
                attr.Set(float(value))
            else:
                prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float).Set(float(value))

    def _boost_arm_drives(self, stage) -> None:
        """加强双臂 J*_1~6 的 USD angular drive（垂直升降/姿态跟踪）。

        失败只打日志，不抛异常——避免打断 Load / Play / ROS 出流。
        """
        try:
            robot = stage.GetPrimAtPath(self.robot_prim_path)
            gravity_disabled = 0
            if robot and robot.IsValid():
                for body in Usd.PrimRange(robot):
                    rigid_api = UsdPhysics.RigidBodyAPI(body)
                    if not rigid_api:
                        continue
                    attr = body.GetAttribute("physxRigidBody:disableGravity")
                    if attr and attr.IsValid():
                        attr.Set(True)
                    else:
                        body.CreateAttribute(
                            "physxRigidBody:disableGravity", Sdf.ValueTypeNames.Bool
                        ).Set(True)
                    gravity_disabled += 1

            # TGS：提高 drive/constraint 收敛；velocity iteration=0 是官方常用稳定设置。
            root_joint = stage.GetPrimAtPath(f"{self.robot_prim_path}/root_joint")
            if root_joint and root_joint.IsValid():
                root_joint.CreateAttribute(
                    "physxArticulation:solverPositionIterationCount",
                    Sdf.ValueTypeNames.Int,
                ).Set(32)
                root_joint.CreateAttribute(
                    "physxArticulation:solverVelocityIterationCount",
                    Sdf.ValueTypeNames.Int,
                ).Set(0)

            tuned = 0
            for name, stiffness, damping, max_force in ARM_DRIVE_SPECS:
                path = f"{self.robot_prim_path}/joints/{name}"
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid():
                    continue
                self._set_joint_drive_attrs(
                    prim,
                    {
                        "drive:angular:physics:stiffness": float(stiffness),
                        "drive:angular:physics:damping": float(damping),
                        "drive:angular:physics:maxForce": float(max_force),
                    },
                )
                # 降低关节摩擦，减轻开环 PD 静差（原先 jointFriction=1）
                fr_attr = prim.GetAttribute("physxJoint:jointFriction")
                if fr_attr and fr_attr.IsValid():
                    fr_attr.Set(0.05)
                else:
                    prim.CreateAttribute(
                        "physxJoint:jointFriction", Sdf.ValueTypeNames.Float
                    ).Set(0.05)
                # 腕部 J5/J6 需要更快跟姿态；上限见 global_variables。
                max_velocity_deg_s = (
                    float(ARM_WRIST_MAX_JOINT_VELOCITY_DEG_S)
                    if "_5_joint" in name or "_6_joint" in name
                    else float(ARM_MAX_JOINT_VELOCITY_DEG_S)
                )
                vel_attr = prim.GetAttribute("physxJoint:maxJointVelocity")
                if vel_attr and vel_attr.IsValid():
                    vel_attr.Set(max_velocity_deg_s)
                else:
                    prim.CreateAttribute(
                        "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float
                    ).Set(max_velocity_deg_s)
                tuned += 1
            if tuned:
                print(
                    f"SceneLoader: boosted arm USD drives x{tuned} "
                    f"(gravity compensated links={gravity_disabled}; "
                    f"maxVel J1-4={ARM_MAX_JOINT_VELOCITY_DEG_S:.0f}deg/s "
                    f"J5-6={ARM_WRIST_MAX_JOINT_VELOCITY_DEG_S:.0f}deg/s; "
                    "TGS pos/vel iterations=32/0)"
                )
        except Exception as exc:
            print(f"SceneLoader: WARN _boost_arm_drives skipped: {exc}")

    def _apply_arm_drive_gains_articulation(self) -> None:
        """Play 后经 Articulation API 写入臂关节 PD + maxEffort（运行时生效）。"""
        art = self._robot_articulation
        if art is None:
            self._apply_gripper_drive_gains_articulation()
            return
        try:
            import numpy as np

            gains = {n: (stiff, damp) for n, stiff, damp, _mf in ARM_DRIVE_SPECS}
            max_forces = {n: mf for n, _s, _d, mf in ARM_DRIVE_SPECS}
            names = list(art.dof_names or [])
            if not names:
                self._apply_gripper_drive_gains_articulation()
                return
            name_to_i = {n: i for i, n in enumerate(names)}
            joint_names = [n for n in gains if n in name_to_i]
            if joint_names:
                kps = np.array([[gains[n][0] for n in joint_names]], dtype=np.float32)
                kds = np.array([[gains[n][1] for n in joint_names]], dtype=np.float32)
                efforts = np.array(
                    [[max_forces[n] for n in joint_names]], dtype=np.float32
                )
                view = art
                if hasattr(art, "_articulation_view"):
                    view = art._articulation_view
                gains_ok = False
                if hasattr(view, "set_gains"):
                    view.set_gains(kps=kps, kds=kds, joint_names=joint_names)
                    gains_ok = True
                elif hasattr(art, "set_gains"):
                    art.set_gains(kps=kps, kds=kds, joint_names=joint_names)
                    gains_ok = True
                if gains_ok:
                    try:
                        if hasattr(view, "set_max_efforts"):
                            view.set_max_efforts(
                                efforts, joint_indices=None, joint_names=joint_names
                            )
                        elif hasattr(art, "set_max_efforts"):
                            art.set_max_efforts(efforts, joint_names=joint_names)
                    except Exception as exc_mf:
                        print(
                            f"SceneLoader: WARN arm set_max_efforts skipped: {exc_mf}"
                        )
                    print(
                        f"SceneLoader: articulation arm gains for {len(joint_names)} joints "
                        f"(J*_1 kp={gains['J1_1_joint'][0]}; "
                        f"wrist kp={gains['J1_5_joint'][0]})"
                    )
                else:
                    print("SceneLoader: articulation set_gains API missing")
        except Exception as exc:
            print(f"SceneLoader: articulation set_gains skipped: {exc}")
        self._apply_gripper_drive_gains_articulation()

    def _apply_gripper_drive_gains_articulation(self) -> None:
        """Play 后写入夹爪 PD + maxEffort，避免 soft reset 回到 USD 弱驱动导致夹不住。"""
        art = self._robot_articulation
        if art is None:
            return
        try:
            import numpy as np
        except Exception:
            return
        joint_names = [
            "J1_7_joint",
            "J1_8_joint",
            "J2_7_joint",
            "J2_8_joint",
        ]
        try:
            names = list(art.dof_names or [])
            if not names:
                return
            name_to_i = {n: i for i, n in enumerate(names)}
            present = [n for n in joint_names if n in name_to_i]
            if not present:
                return
            kps = np.array([[GRIPPER_DRIVE_STIFFNESS] * len(present)], dtype=np.float32)
            kds = np.array([[GRIPPER_DRIVE_DAMPING] * len(present)], dtype=np.float32)
            efforts = np.array([[GRIPPER_DRIVE_MAX_FORCE] * len(present)], dtype=np.float32)
            view = art
            if hasattr(art, "_articulation_view"):
                view = art._articulation_view
            if hasattr(view, "set_gains"):
                view.set_gains(kps=kps, kds=kds, joint_names=present)
            elif hasattr(art, "set_gains"):
                art.set_gains(kps=kps, kds=kds, joint_names=present)
            if hasattr(view, "set_max_efforts"):
                view.set_max_efforts(efforts, joint_indices=None, joint_names=present)
            elif hasattr(art, "set_max_efforts"):
                art.set_max_efforts(efforts, joint_names=present)
            print(
                f"SceneLoader: gripper articulation gains {present} "
                f"kp={GRIPPER_DRIVE_STIFFNESS} kd={GRIPPER_DRIVE_DAMPING} "
                f"maxEffort={GRIPPER_DRIVE_MAX_FORCE}"
            )
        except Exception as exc:
            print(f"SceneLoader: gripper articulation gains skipped: {exc}")

    def _boost_gripper_drives(self, stage) -> None:
        """夹爪 prismatic：提高刚度/阻尼/maxForce，保证夹紧力够用。"""
        joint_names = (
            "J1_7_joint",
            "J1_8_joint",
            "J2_7_joint",
            "J2_8_joint",
        )
        stiffness = float(GRIPPER_DRIVE_STIFFNESS)
        damping = float(GRIPPER_DRIVE_DAMPING)
        max_force = float(GRIPPER_DRIVE_MAX_FORCE)
        tuned = 0
        for name in joint_names:
            path = f"{self.robot_prim_path}/joints/{name}"
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                continue
            self._set_joint_drive_attrs(
                prim,
                {
                    "drive:linear:physics:stiffness": stiffness,
                    "drive:linear:physics:damping": damping,
                    "drive:linear:physics:maxForce": max_force,
                },
            )
            tuned += 1
        if tuned:
            print(
                f"SceneLoader: boosted gripper drives x{tuned} "
                f"(stiffness={stiffness}, damping={damping}, maxForce={max_force})"
            )

    def _ensure_grasp_friction_material(self, stage):
        """创建/更新高摩擦 Physics Material（夹爪与盒子共用）。"""
        mat_path = GRASP_PHYSICS_MATERIAL_PATH
        parent = "/World/PhysicsMaterials"
        if not stage.GetPrimAtPath(parent).IsValid():
            UsdGeom.Scope.Define(stage, parent)
        mat = UsdShade.Material.Define(stage, mat_path)
        prim = mat.GetPrim()
        api = UsdPhysics.MaterialAPI.Apply(prim)
        api.CreateStaticFrictionAttr(float(GRASP_STATIC_FRICTION))
        api.CreateDynamicFrictionAttr(float(GRASP_DYNAMIC_FRICTION))
        api.CreateRestitutionAttr(float(GRASP_RESTITUTION))
        if PhysxSchema is not None:
            px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
            # 取两侧较大摩擦，避免桌面低摩擦把有效 μ 拉低
            try:
                px.CreateFrictionCombineModeAttr("max")
            except Exception:
                prim.CreateAttribute(
                    "physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token
                ).Set("max")
            try:
                px.CreateRestitutionCombineModeAttr("min")
            except Exception:
                prim.CreateAttribute(
                    "physxMaterial:restitutionCombineMode", Sdf.ValueTypeNames.Token
                ).Set("min")
            try:
                px.CreateImprovePatchFrictionAttr(True)
            except Exception:
                pass
        return mat

    @staticmethod
    def _bind_physics_material(prim, material) -> bool:
        """只绑 materialPurpose=physics，绝不写默认 purpose（会盖掉视觉贴图）。"""
        if not prim or not prim.IsValid() or material is None:
            return False
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            return False
        bind_api = UsdShade.MaterialBindingAPI.Apply(prim)
        bind_api.Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        if PhysxSchema is not None:
            try:
                px_col = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                # 已有非法默认值时 Create 可能不覆盖，必须显式 Set
                c_attr = px_col.GetContactOffsetAttr()
                r_attr = px_col.GetRestOffsetAttr()
                if c_attr and c_attr.IsValid():
                    c_attr.Set(0.002)
                else:
                    px_col.CreateContactOffsetAttr(0.002)
                if r_attr and r_attr.IsValid():
                    r_attr.Set(0.0)
                else:
                    px_col.CreateRestOffsetAttr(0.0)
            except Exception:
                pass
        return True

    def _apply_grasp_friction(self, stage) -> None:
        """给盒子/夹爪的碰撞体绑高摩擦材质（不影响 visual 贴图）。"""
        material = self._ensure_grasp_friction_material(stage)
        bound = 0

        box_root = stage.GetPrimAtPath(BOX_COLLISION_GEO_ROOT)
        if box_root and box_root.IsValid():
            for prim in Usd.PrimRange(box_root):
                if self._bind_physics_material(prim, material):
                    bound += 1

        box_link = stage.GetPrimAtPath(self.box_link_path)
        if box_link and box_link.IsValid():
            # 历史版本曾把摩擦材质绑到刚体根（默认 purpose + strongerThanDescendants），会盖掉贴图
            try:
                if box_link.HasAPI(UsdShade.MaterialBindingAPI):
                    UsdShade.MaterialBindingAPI(box_link).UnbindAllBindings()
            except Exception:
                pass
            if PhysxSchema is not None:
                try:
                    px_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(box_link)
                    px_rb.CreateSolverPositionIterationCountAttr(16)
                    px_rb.CreateSolverVelocityIterationCountAttr(4)
                    px_rb.CreateSleepThresholdAttr(0.0)
                except Exception:
                    pass
            mass = UsdPhysics.MassAPI.Apply(box_link)
            mass.CreateMassAttr(float(BOX_MASS_KG))

        for finger in ARM1_GRIPPER_FINGERS + ARM2_GRIPPER_FINGERS:
            finger_path = f"{self.robot_prim_path}/{finger}"
            finger_prim = stage.GetPrimAtPath(finger_path)
            if not finger_prim or not finger_prim.IsValid():
                continue
            for prim in Usd.PrimRange(finger_prim):
                if self._bind_physics_material(prim, material):
                    bound += 1

        # 摩擦绑定后强制恢复盒子视觉贴图（防止历史错误默认 Bind 残留）
        try:
            apply_box_textures(stage, self._box_dir)
        except Exception as exc:
            print(f"SceneLoader: re-apply box textures after friction: {exc}")

        print(
            f"SceneLoader: grasp friction applied bindings={bound} "
            f"μs={GRASP_STATIC_FRICTION} μd={GRASP_DYNAMIC_FRICTION} "
            f"box_mass={BOX_MASS_KG}kg combine=max "
            f"gripper_maxForce={GRIPPER_DRIVE_MAX_FORCE}"
        )

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

    @staticmethod
    def _range_in_parent_frame(stage, prim_path: str, parent_path: str):
        """把 prim 世界包围盒变换到 parent 本地系，返回 (min_xyz, max_xyz) 或 None。"""
        prim = stage.GetPrimAtPath(prim_path)
        parent = stage.GetPrimAtPath(parent_path)
        if not prim or not prim.IsValid() or not parent or not parent.IsValid():
            return None

        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        world_range = cache.ComputeWorldBound(prim).GetRange()
        if world_range.IsEmpty():
            return None

        parent_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        parent_inv = parent_world.GetInverse()
        mn = world_range.GetMin()
        mx = world_range.GetMax()
        corners = (
            Gf.Vec3d(mn[0], mn[1], mn[2]),
            Gf.Vec3d(mx[0], mn[1], mn[2]),
            Gf.Vec3d(mn[0], mx[1], mn[2]),
            Gf.Vec3d(mx[0], mx[1], mn[2]),
            Gf.Vec3d(mn[0], mn[1], mx[2]),
            Gf.Vec3d(mx[0], mn[1], mx[2]),
            Gf.Vec3d(mn[0], mx[1], mx[2]),
            Gf.Vec3d(mx[0], mx[1], mx[2]),
        )
        local = [parent_inv.Transform(corner) for corner in corners]
        xs = [float(p[0]) for p in local]
        ys = [float(p[1]) for p in local]
        zs = [float(p[2]) for p in local]
        return (
            (min(xs), min(ys), min(zs)),
            (max(xs), max(ys), max(zs)),
        )

    def _measure_base_link_mesh_z_bounds(self) -> tuple[float, float]:
        """URDF → base_link.stl 在 base_link 下的 (floor_z, top_z)。"""
        try:
            floor_z, top_z = base_link_stl_z_bounds(self._robot_dir)
            return float(floor_z), float(top_z)
        except Exception as exc:
            print(f"SceneLoader: base_link.stl bounds fallback ({exc})")
            return -PLATFORM_DEPTH_BELOW_BASE, WORKSPACE_BASE_LINK_MESH_TOP_Z

    def _sync_workspace_surface_z_from_fill(self, stage) -> None:
        """用 workspace_fill 世界包围盒顶面更新落物面高度（比 mount+STL 更准确）。"""
        fill_path = (
            f"{self.robot_prim_path}/base_link/{WORKSPACE_COLLISION_FILL_NAME}"
        )
        prim = stage.GetPrimAtPath(fill_path)
        if not prim or not prim.IsValid():
            return
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        rng = cache.ComputeWorldBound(prim).GetRange()
        if rng.IsEmpty():
            return
        self._workspace_surface_z_world = float(rng.GetMax()[2])

    def _ensure_base_platform_collision_fill(self, stage) -> None:
        """J1/J2 底座之间补立方体碰撞：顶面齐 base_link.stl 顶（整件 base_link 高度）。"""
        base_path = f"{self.robot_prim_path}/base_link"
        # collisions 是 instanceable 引用，不能在其下 Define；挂在 base_link 直系子节点。
        fill_path = f"{base_path}/{WORKSPACE_COLLISION_FILL_NAME}"
        existing = stage.GetPrimAtPath(fill_path)
        if existing and existing.IsValid():
            stage.RemovePrim(fill_path)

        base_prim = stage.GetPrimAtPath(base_path)
        if not base_prim or not base_prim.IsValid():
            return

        floor_z, surface_z = self._measure_base_link_mesh_z_bounds()
        if surface_z <= floor_z + 1e-4:
            floor_z = -PLATFORM_DEPTH_BELOW_BASE
            surface_z = WORKSPACE_BASE_LINK_MESH_TOP_Z

        self._workspace_surface_z_world = self._mount_z + surface_z

        j1_range = self._range_in_parent_frame(
            stage, f"{self.robot_prim_path}/{ARM1_BASE_LINK}/collisions", base_path
        )
        j2_range = self._range_in_parent_frame(
            stage, f"{self.robot_prim_path}/{ARM2_BASE_LINK}/collisions", base_path
        )
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        base_range = cache.ComputeLocalBound(base_prim).GetRange()

        inset = WORKSPACE_COLLISION_INSET_M
        if j1_range and j2_range and not base_range.IsEmpty():
            j1_lo, j1_hi = j1_range
            j2_lo, j2_hi = j2_range
            base_lo = base_range.GetMin()
            base_hi = base_range.GetMax()
            x0 = float(j1_hi[0]) + inset
            x1 = float(j2_lo[0]) - inset
            y0 = float(base_lo[1]) + inset
            y1 = float(base_hi[1]) - inset
        else:
            x0 = float(ARM_J1_X) + 0.16 + inset
            x1 = float(ARM_J2_X) - 0.16 - inset
            y0 = -0.40
            y1 = 0.20

        if x1 <= x0 or y1 <= y0:
            print("SceneLoader: WARN workspace collision fill skipped (invalid gap)")
            return

        sx = x1 - x0
        sy = y1 - y0
        height = surface_z - floor_z
        cx = (x0 + x1) * 0.5
        cy = (y0 + y1) * 0.5
        cz = (surface_z + floor_z) * 0.5

        cube = UsdGeom.Cube.Define(stage, Sdf.Path(fill_path))
        cube.CreateSizeAttr(1.0)
        cube.CreatePurposeAttr().Set(UsdGeom.Tokens.default_)
        imageable = UsdGeom.Imageable(cube)
        imageable.MakeVisible()
        xform = UsdGeom.Xformable(cube)
        xform.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
        xform.AddScaleOp().Set(Gf.Vec3f(sx, sy, float(height)))
        coll = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        coll.CreateCollisionEnabledAttr(True)
        if PhysxSchema is not None:
            PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())

        material = self._create_workspace_fill_material(
            stage, f"{fill_path}_material"
        )
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

        self._sync_workspace_surface_z_from_fill(stage)

        print(
            f"SceneLoader: workspace collision fill @ {fill_path} "
            f"(visible+collision) size=({sx:.3f},{sy:.3f},{height:.3f}) "
            f"center=({cx:.3f},{cy:.3f},{cz:.3f}) "
            f"base_link.stl z=[{floor_z:.3f},{surface_z:.3f}] "
            f"world_top={self._workspace_surface_z_world:.3f}"
        )

    def _align_cam0_to_gantry_beam(self, stage) -> None:
        """cam0 应挂在龙门横梁连接件正下方；USD 原 translate 偏了约 2cm。"""
        base_path = f"{self.robot_prim_path}/base_link"
        connector_path = (
            f"{base_path}/gantry_left_column/gantry_beam/gantry_connector"
        )
        cam_path = f"{base_path}/cam0"
        connector = stage.GetPrimAtPath(connector_path)
        cam = stage.GetPrimAtPath(cam_path)
        if not connector or not connector.IsValid() or not cam or not cam.IsValid():
            print("SceneLoader: cam0/gantry_connector missing — skip cam0 align")
            return

        rng = self._range_in_parent_frame(stage, connector_path, base_path)
        if not rng:
            print("SceneLoader: gantry_connector bbox missing — skip cam0 align")
            return

        lo, hi = rng
        cx = (lo[0] + hi[0]) * 0.5
        cy = (lo[1] + hi[1]) * 0.5

        cam_xform = UsdGeom.Xformable(cam)
        current_z = 1.009
        translate_op = None
        for op in cam_xform.GetOrderedXformOps():
            if op.IsInverseOp():
                continue
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
                val = op.Get()
                if val is not None:
                    current_z = float(val[2])
                break
        if translate_op is None:
            translate_op = cam_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(cx, cy, current_z))
        print(
            f"SceneLoader: cam0 aligned to gantry beam center "
            f"xy=({cx:.3f},{cy:.3f}), z={current_z:.3f}"
        )

    def sync_cam0_tf_to_gemini_rgb(self, stage=None) -> None:
        """对外接口：把 TF ``cam0`` 对齐到 Gemini 出流等效外参（不改画面）。"""
        if stage is None:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
        if stage is not None:
            self._sync_cam0_tf_to_gemini_rgb(stage)

    def _sync_cam0_tf_to_gemini_rgb(self, stage) -> None:
        """不移动 Gemini（画面不变），只改 ``base_link/cam0`` 姿态。

        Gemini335 Orient X≈-150° ⇒ 离竖直约 30°。原先 cam0 TF 为 15°（俯仰 75°），
        与画面不一致会导致 optical→base 约 20cm 偏差。

        注意：不要用 Stream_rgb 的 USD 矩阵盲目换算，也不要用
        ``Gf.Rotation(Rz)*Ry`` 连乘——后者实际得到 RPY(60,0,90)/forward=(0,1,0)，
        抓取 Z≈1.45m（Δ≈1m）。这里显式写入 RPY(0,60°,90°) 对应四元数。
        """
        import math

        base_path = f"{self.robot_prim_path}/base_link"
        cam_path = f"{base_path}/cam0"
        cam_prim = stage.GetPrimAtPath(cam_path)
        if not cam_prim or not cam_prim.IsValid():
            print("SceneLoader: cam0 TF sync skipped — base_link/cam0 missing")
            return

        # Keep existing / gantry-aligned translation.
        translation = Gf.Vec3d(0.53, -0.499, 1.009)
        for op in UsdGeom.Xformable(cam_prim).GetOrderedXformOps():
            if (not op.IsInverseOp()) and op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                val = op.Get()
                if val is not None:
                    translation = Gf.Vec3d(val)
                break

        # Prefer Gemini root world XY/Z mapped into base_link (position only).
        gemini_prim = stage.GetPrimAtPath(GEMINI335_PRIM_PATH)
        base_prim = stage.GetPrimAtPath(base_path)
        if (
            gemini_prim and gemini_prim.IsValid()
            and base_prim and base_prim.IsValid()
        ):
            gemini_world = UsdGeom.Xformable(gemini_prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            base_world = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            gemini_in_base = base_world.GetInverse().Transform(
                gemini_world.ExtractTranslation()
            )
            if abs(float(gemini_in_base[2])) > 0.1:
                translation = Gf.Vec3d(gemini_in_base)

        # camera_link RPY(0, 60°, 90°): forward≈(0, 0.5, -0.866), tilt≈30° from vertical.
        # DO NOT use Gf.Rotation(Rz)*Ry — that composition yields RPY(60,0,90) /
        # forward=(0,1,0) and grasp Z≈1.45m (ΔZ≈1m vs box). Write quat explicitly.
        cr, sr = 1.0, 0.0  # half roll 0
        cp = math.cos(math.radians(30.0))  # half pitch 60
        sp = math.sin(math.radians(30.0))
        cy = math.cos(math.radians(45.0))  # half yaw 90
        sy = math.sin(math.radians(45.0))
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        rotation = Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz))
        rotation.Normalize()

        rot_m = Gf.Matrix4d()
        rot_m.SetRotate(Gf.Rotation(rotation))
        forward = rot_m.TransformDir(Gf.Vec3d(1, 0, 0))
        fx, fy, fz = float(forward[0]), float(forward[1]), float(forward[2])
        norm = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
        tilt_from_vertical_deg = math.degrees(
            math.acos(min(1.0, max(-1.0, -fz / norm)))
        )
        if abs(tilt_from_vertical_deg - 30.0) > 5.0 or fy < 0.3:
            raise RuntimeError(
                f"cam0 TF quat wrong: tilt={tilt_from_vertical_deg:.1f}° "
                f"forward=({fx:.3f},{fy:.3f},{fz:.3f}); expected ~30° / (0,0.5,-0.87)"
            )

        cam_xform = UsdGeom.Xformable(cam_prim)
        cam_xform.ClearXformOpOrder()
        cam_xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(translation)
        cam_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(rotation)
        print(
            f"SceneLoader: cam0 TF set to Gemini-equivalent 30° mount (image unchanged) "
            f"t=({float(translation[0]):.3f},{float(translation[1]):.3f},{float(translation[2]):.3f}) "
            f"RPY≈(0,60,90) tilt_from_vertical≈{tilt_from_vertical_deg:.1f}° "
            f"forward=({fx:.3f},{fy:.3f},{fz:.3f})"
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
        """RSD455 为 payload（``data/robot/sensors/rsd455.usd``）：同步 Load 子树。"""
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
            print(
                f"SceneLoader: WARN RSD455 not yet present for {missing} "
                f"(run scripts/download_rsd455.py && prepare_robot_usd.py)"
            )
        else:
            print("SceneLoader: RSD455 local payloads loaded")

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
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(True)
        if PhysxSchema is not None:
            px_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(table_xform.GetPrim())
            px_rb.CreateDisableGravityAttr(True)
        self.table_top_z = TABLE_TOP_Z
        print(
            f"SceneLoader: table {TABLE_LENGTH}×{TABLE_WIDTH} m, "
            f"top z={TABLE_TOP_Z:.3f}, 6 legs"
        )

    def _create_stainless_material(self, stage, mat_path: str):
        """桌腿/桌面用不锈钢 PreviewSurface。"""
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

    def _create_workspace_fill_material(self, stage, mat_path: str):
        """中间工作台补块（workspace_fill）米白哑光。"""
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.88)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.94, 0.91, 0.86)
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
        coll = UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        coll.CreateCollisionEnabledAttr(True)
        if PhysxSchema is not None:
            PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
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

    def _create_box_visual_placeholder(
        self, stage, sx: float, sy: float, sz: float
    ) -> None:
        """Load 后真实 mesh 就绪前，用带材质的立方体占位（碰撞体 purpose=guide 不可见）。"""
        path = f"{BOX_VISUAL_PATH}/placeholder"
        if stage.GetPrimAtPath(path).IsValid():
            return
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
        mat_path = f"{self.box_link_path}/Looks/placeholder_material"
        material = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.82, 0.62, 0.38)
        )
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

    def _create_box(self, stage, pose: Pose6D) -> None:
        """创建抓取盒：视觉 mesh 加载后在其上挂 convexHull 碰撞（不用简化立方体）。"""
        link = UsdGeom.Xform.Define(stage, self.box_link_path)
        set_pose(self.box_link_path, pose, stage)
        UsdGeom.Xform.Define(stage, BOX_VISUAL_PATH)
        remove_legacy_box_collision_cube(stage)

        sx, sy, sz = self._box_collision_size()
        self._create_box_visual_placeholder(stage, sx, sy, sz)

        rb = UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(False)
        if PhysxSchema is not None:
            px_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(link.GetPrim())
            px_rb.CreateSleepThresholdAttr(0.0)
            px_rb.CreateStabilizationThresholdAttr(0.0)
            px_rb.CreateDisableGravityAttr(False)
        mass = UsdPhysics.MassAPI.Apply(link.GetPrim())
        mass.CreateMassAttr(float(BOX_MASS_KG))
        # Load 时立即挂碰撞（含 meta 回退盒），避免 Play 后无碰撞穿桌
        if not ensure_box_collision(stage, self._box_dir):
            print("SceneLoader: WARN box collision not ready at load")
        lift_box_clear_of_surface(
            stage,
            surface_z_world=self._workspace_surface_z_world,
            log=False,
        )
        print(
            f"SceneLoader: grasp_box at ({pose.translation[0]:.3f}, "
            f"{pose.translation[1]:.3f}, {pose.translation[2]:.3f}), "
            f"mesh collision pending (visual load)"
        )
        if not load_box_visual(stage, self._box_dir):
            print("SceneLoader: box placeholder visible until deferred mesh build")

    def build_box_visual_if_needed(self, stage) -> None:
        """延迟构建带贴图的盒子 mesh（Load 后异步调用，仅执行一次）。"""
        if self._box_visual_ready:
            return
        mesh_path = f"{BOX_VISUAL_PATH}/mesh"
        if _box_has_visible_mesh(stage, mesh_path):
            remove_legacy_box_collision_cube(stage)
            if not has_box_collision(stage):
                try:
                    setup_box_collision(stage, box_dir=self._box_dir)
                except Exception as exc:
                    print(f"SceneLoader: setup_box_collision failed: {exc}")
            apply_box_textures(stage, self._box_dir)
            self._box_visual_ready = True
            return
        if load_box_visual(stage, self._box_dir):
            self._box_visual_ready = True
            return
        print("SceneLoader: WARN box textured mesh unavailable (placeholder cube remains visible)")

    def fix_box_visual_material(self, stage=None) -> None:
        """Load 后延迟调用：等 payload 就绪后再次应用贴图。"""
        if stage is None:
            stage = omni.usd.get_context().get_stage()
        if stage and self._loaded:
            apply_box_textures(stage, self._box_dir)

    def _resolve_camera_paths(self, stage) -> None:
        """按 ``CAMERA_DEFS`` 解析 cam0/cam1/cam2 的 Camera prim 路径。"""
        self._camera_prim_paths.clear()
        for spec in CAMERA_DEFS:
            key = spec["key"]
            abs_path = spec.get("prim_path")
            if abs_path:
                path = abs_path
            else:
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
        self._box_rigid = None
        self._table_rigid = None
        self._robot_articulation = None

    def register_physics_prims(self, *, kinematic_box: bool = False) -> bool:
        """Play 后配置盒子 USD 刚体并初始化机器人 articulation（不用 SingleRigidPrim，避免 wakeUp 报错）。"""
        if not self._loaded or self.world is None:
            return False
        import omni.timeline

        if not omni.timeline.get_timeline_interface().is_playing():
            return False

        configure_box_usd_dynamic(kinematic=kinematic_box)
        self._enable_robot_self_collision()
        print(
            f"SceneLoader: grasp_box USD dynamic @ {self.box_link_path} "
            f"(kinematic={kinematic_box})"
        )

        from isaacsim.core.prims import SingleArticulation

        def _art_ready(art) -> bool:
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
            try:
                return len(list(art.dof_names or [])) > 0
            except Exception:
                return False

        if self._robot_articulation is None:
            for path in (ROBOT_ROOT_JOINT_PATH, ROBOT_PRIM_PATH):
                try:
                    art = SingleArticulation(prim_path=path, name="nova_robot_grasp")
                    art.initialize()
                    if _art_ready(art):
                        names = list(art.dof_names or [])
                        self._robot_articulation = art
                        print(
                            f"SceneLoader: robot articulation @ {path} "
                            f"(dofs={len(names)})"
                        )
                        break
                except Exception as exc:
                    print(f"SceneLoader: articulation init @ {path}: {exc}")
        elif hasattr(self._robot_articulation, "initialize"):
            try:
                self._robot_articulation.initialize()
            except Exception:
                pass

        return self._robot_articulation is not None

    @staticmethod
    def _enable_robot_self_collision() -> None:
        """关闭 articulation 自碰撞。

        机器人碰撞体由凸包近似，J*_1 臂根与 base/platform 在关节附近存在包络重叠。
        全局开启 self-collision 会产生持续约束，把肩部 yaw 卡住，表现为 J1 目标与
        实际相差数度，而 J2~J6 基本到位。环境/桌面碰撞仍保持启用。
        """
        try:
            import omni.usd
            from pxr import PhysxSchema

            stage = omni.usd.get_context().get_stage()
            if not stage:
                return
            prim = stage.GetPrimAtPath(ROBOT_ROOT_JOINT_PATH)
            if not prim or not prim.IsValid():
                return
            api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            api.CreateEnabledSelfCollisionsAttr(False)
            print("SceneLoader: robot self-collision OFF (avoid convex-hull joint lock)")
        except Exception as exc:
            print(f"SceneLoader: self-collision configure skipped: {exc}")

    def step_physics(self) -> None:
        """已弃用：物理由 Timeline PhysX 步进；world.step 会把刚体拉回 USD xform。"""
        return

    def ensure_physics_on_play(
        self,
        *,
        release_box: bool = True,
        kinematic_box: Optional[bool] = None,
    ) -> bool:
        """Play 期间注册 articulation 后 soft reset（顺序：先注册，再 reset）。"""
        if not self._loaded or self.world is None or self._physics_initialized:
            return self._physics_initialized
        import omni.timeline

        if not omni.timeline.get_timeline_interface().is_playing():
            return False
        kin = bool(kinematic_box) if kinematic_box is not None else (not release_box)
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                self.build_box_visual_if_needed(stage)
                if not ensure_box_collision(stage, self._box_dir):
                    print("SceneLoader: WARN box has no collision — skip gravity release")
                    return False
            self._register_table_physics()
            self.register_physics_prims(kinematic_box=kin)
            if stage:
                self._boost_arm_drives(stage)
                self._boost_gripper_drives(stage)
                self._apply_grasp_friction(stage)
                self._apply_default_robot_joints_usd(stage)
            self.world.reset(soft=True)
            if stage:
                self._boost_arm_drives(stage)
                self._boost_gripper_drives(stage)
                self._apply_grasp_friction(stage)
                self._apply_default_robot_joints_usd(stage)
            self._apply_default_robot_joints_articulation()
            self._apply_arm_drive_gains_articulation()  # 内含夹爪 articulation gains
            if release_box:
                release_box_for_gravity(
                    log=True, surface_z_world=self._workspace_surface_z_world
                )
            self._physics_initialized = True
            print(
                "SceneLoader: physics ready "
                f"(table + box kinematic={kin}, release_box={release_box})"
            )
            return True
        except Exception as exc:
            print(f"SceneLoader: ensure_physics_on_play warning: {exc}")
            return False

    def _apply_default_robot_joints_usd(self, stage) -> None:
        """Load/Play：按 DEFAULT_JOINT_ANGLES_DEG 写 USD 关节（度）；未列出的为 0。"""
        robot = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot or not robot.IsValid():
            return
        count = 0
        applied: dict[str, float] = {}
        for prim in Usd.PrimRange(robot):
            name = prim.GetName()
            if prim.IsA(UsdPhysics.RevoluteJoint):
                angle = float(DEFAULT_JOINT_ANGLES_DEG.get(name, 0.0))
                for attr_name in (
                    "state:angular:physics:position",
                    "state:angular:physics:velocity",
                    "drive:angular:physics:targetPosition",
                    "drive:angular:physics:targetVelocity",
                ):
                    attr = prim.GetAttribute(attr_name)
                    if attr and attr.IsValid():
                        attr.Set(0.0 if "velocity" in attr_name else angle)
                count += 1
                if abs(angle) > 1e-6:
                    applied[name] = angle
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
            extra = ", ".join(f"{k}={v:g}°" for k, v in applied.items()) or "all zero"
            print(f"SceneLoader: arm base yaw (whole-arm swing) ({count}): {extra}")

    def _apply_default_robot_joints_articulation(self) -> None:
        """PhysX 就绪后按名称写入底座偏航（弧度）；兼容 dof 名带/不带 _joint。"""
        art = self._robot_articulation
        if art is None:
            return
        try:
            import math

            import numpy as np

            names = list(art.dof_names or [])
            if not names:
                return
            pos = art.get_joint_positions()
            if pos is None:
                pos = np.zeros(len(names), dtype=np.float64)
            else:
                pos = np.asarray(pos, dtype=np.float64).copy()
            touched = []
            for i, name in enumerate(names):
                deg = DEFAULT_JOINT_ANGLES_DEG.get(name)
                if deg is None and name.endswith("_joint"):
                    deg = DEFAULT_JOINT_ANGLES_DEG.get(name)
                if deg is None and not name.endswith("_joint"):
                    deg = DEFAULT_JOINT_ANGLES_DEG.get(f"{name}_joint")
                if deg is None:
                    continue
                pos[i] = math.radians(float(deg))
                touched.append(f"{name}={deg:g}°")
            if not touched:
                print(
                    f"SceneLoader: WARN arm yaw dofs not found in {names[:8]}..."
                )
                return
            if hasattr(art, "set_joint_positions"):
                art.set_joint_positions(pos)
            if hasattr(art, "set_joint_position_targets"):
                art.set_joint_position_targets(pos)
            print("SceneLoader: articulation arm swing: " + ", ".join(touched))
        except Exception as exc:
            print(f"SceneLoader: articulation arm swing skipped: {exc}")

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
