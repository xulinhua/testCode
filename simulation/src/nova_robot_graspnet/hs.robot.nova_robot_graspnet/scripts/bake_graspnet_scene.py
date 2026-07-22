#!/usr/bin/env python3
"""烘焙采集对齐的总场景 ``data/scenes/nova_graspnet_scene.usda``。

对齐 dobot_graspent_datacapture 的相机 / 灯光 / 环境配置：
  - FlatGrid（本地 default_environment）+ 四角 SphereLight
  - DistantLight 隐藏
  - cam0 RSD455 隐藏（避免挡主视角）；三路 RSD455 去刚体
  - Gemini335 机位（本地 Camera 占位；有 Orbbec USD 时自动引用）
  - nova_robot / 桌子相对路径引用（grasp_box 由插件按 UI 运行时创建）

用法（Isaac python 或带 pxr 的环境）::

    ./python.sh scripts/bake_graspnet_scene.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _ext_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _rel(from_dir: Path, to_path: Path) -> str:
    return os.path.relpath(to_path.resolve(), from_dir.resolve()).replace("\\", "/")


def bake(out_path: Path | None = None) -> Path:
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade
    except ImportError as exc:
        raise SystemExit(
            "bake_graspnet_scene requires pxr (run with Isaac Sim python.sh)\n"
            f"ImportError: {exc}"
        )

    root = _ext_root()
    scenes_dir = root / "data" / "scenes"
    scenes_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_path or (scenes_dir / "nova_graspnet_scene.usda")

    robot_usd = root / "data" / "robot" / "nova_robot_prepared.usda"
    if not robot_usd.is_file():
        robot_usd = root / "data" / "robot" / "nova_robot.usda"
    env_usd = root / "data" / "env" / "default_environment.usd"
    gemini_local = root / "data" / "sensors" / "orbbec_gemini_335.usd"

    if not robot_usd.is_file():
        raise SystemExit(f"robot USD missing: {robot_usd}")
    if not env_usd.is_file():
        raise SystemExit(f"env USD missing: {env_usd} (run scripts/download_environment.py)")

    # Capture-aligned constants (from dobot_graspent_datacapture.usd)
    robot_mount = (-0.5299997329711914, 0.18000000715255737, 0.6400001108646393)
    gemini_t = (0.0, -0.31703293039677527, 1.6606134042644396)
    gemini_q = (-1.6081226496766364e-16, 1.0460184128151074e-16, 0.9659258262890684, 0.2588190451025208)
    distant_q = (0.6532814824381883, 0.2705980500730985, 0.27059805007309845, 0.6532814824381882)
    sphere_positions = (
        (0.0, 0.0, 2.0),
        (-1.5, 0.0, 2.0),
        (0.0, 1.5, 2.0),
        (1.5, 0.0, 2.0),
        (0.0, -1.5, 2.0),
    )
    sphere_orient = (0.5000000000000001, 0.5, 0.49999999999999994, 0.5)  # w,x,y,z for Gf.Quatd

    if out_path.is_file():
        out_path.unlink()

    stage = Usd.Stage.CreateNew(str(out_path))
    stage.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # ---- FlatGrid environment (local Isaac grid) ----
    flat = stage.DefinePrim("/FlatGrid", "Xform")
    flat.GetReferences().AddReference(_rel(scenes_dir, env_usd))

    # ---- Environment distant light (hidden, capture-aligned) ----
    env = UsdGeom.Xform.Define(stage, "/Environment")
    dlight = UsdLux.DistantLight.Define(stage, "/Environment/defaultLight")
    dlight.CreateIntensityAttr(3000.0)
    dlight.CreateAngleAttr(1.0)
    dlight.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    UsdGeom.Imageable(dlight).MakeInvisible()
    dx = UsdGeom.Xformable(dlight)
    dx.ClearXformOpOrder()
    dx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
    dx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(distant_q[0], Gf.Vec3d(*distant_q[1:]))
    )
    dx.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))

    # ---- Capture sphere lights (intensity 10000 @ z=2; own prims, avoid FlatGrid clash) ----
    UsdGeom.Xform.Define(stage, "/CaptureLights")
    names = ("SphereLight", "SphereLight_01", "SphereLight_02", "SphereLight_03", "SphereLight_04")
    for i, (name, pos) in enumerate(zip(names, sphere_positions)):
        slight = UsdLux.SphereLight.Define(stage, f"/CaptureLights/{name}")
        slight.CreateIntensityAttr(10000.0)
        slight.CreateRadiusAttr(0.25)
        slight.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        slight.CreateColorTemperatureAttr(5000.0 if i else 5500.0)
        sx = UsdGeom.Xformable(slight)
        sx.ClearXformOpOrder()
        sx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        sx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(sphere_orient[0], Gf.Vec3d(*sphere_orient[1:]))
        )
        sx.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))
        UsdGeom.Imageable(slight).MakeVisible()

    # Mute any SphereLight brought in by FlatGrid env reference
    for mute_path in ("/FlatGrid/SphereLight", "/FlatGrid/World/SphereLight"):
        mute = stage.GetPrimAtPath(mute_path)
        if mute and mute.IsValid():
            UsdGeom.Imageable(mute).MakeInvisible()
            intens = mute.GetAttribute("inputs:intensity")
            if intens:
                intens.Set(0.0)

    # ---- Ground plane under World (keep plugin path) ----
    # Prefer env-provided grid; still create a thin collision plane for physics fallback.
    ground = UsdGeom.Xform.Define(stage, "/World/defaultGroundPlane")
    plane = UsdGeom.Mesh.Define(stage, "/World/defaultGroundPlane/geom")
    # unit square on XY at z=0, large scale via xform
    plane.CreatePointsAttr(
        [(-1, -1, 0), (1, -1, 0), (1, 1, 0), (-1, 1, 0)]
    )
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    UsdGeom.Xformable(plane).AddScaleOp().Set(Gf.Vec3f(50.0, 50.0, 1.0))
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

    # ---- Table (same geometry as capture / plugin) ----
    table = UsdGeom.Xform.Define(stage, "/World/graspnet_table")
    UsdGeom.Xformable(table).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    UsdGeom.Xformable(table).AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))
    rb = UsdPhysics.RigidBodyAPI.Apply(table.GetPrim())
    rb.CreateRigidBodyEnabledAttr(True)
    rb.CreateKinematicEnabledAttr(True)

    mat = UsdShade.Material.Define(stage, "/World/graspnet_table/StainlessSteel")
    shader = UsdShade.Shader.Define(stage, "/World/graspnet_table/StainlessSteel/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.92)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.28)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.78, 0.78, 0.8)
    )
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    def add_cube(path: str, center, size):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)

    add_cube("/World/graspnet_table/top", (0, 0, 0.575), (2.0, 1.0, 0.05))
    legs = (
        (0.88, 0.38),
        (0.88, -0.38),
        (-0.88, 0.38),
        (-0.88, -0.38),
        (0.0, 0.38),
        (0.0, -0.38),
    )
    for i, (lx, ly) in enumerate(legs):
        add_cube(f"/World/graspnet_table/leg_{i}", (lx, ly, 0.275), (0.08, 0.08, 0.55))

    # ---- Robot ----
    robot = stage.DefinePrim("/World/nova_robot", "Xform")
    robot.GetReferences().AddReference(_rel(scenes_dir, robot_usd))
    rxf = UsdGeom.Xformable(robot)
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*robot_mount))
    rxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, Gf.Vec3d(0, 0, 0)))
    rxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))

    # 双臂底座同向 -90°（整臂绕 J*_1 转开，避免挡盒子且左右朝向一致）
    for jname, yaw_deg in (("J1_1_joint", -90.0), ("J2_1_joint", -90.0)):
        jp = stage.OverridePrim(f"/World/nova_robot/joints/{jname}")
        jp.CreateAttribute(
            "drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float
        ).Set(float(yaw_deg))
        jp.CreateAttribute(
            "state:angular:physics:position", Sdf.ValueTypeNames.Float
        ).Set(float(yaw_deg))

    # Camera overs: 三路 RSD455 全部隐藏 mesh（防重载后腕部相机掉桌上挡 Gemini）；
    # Camera 传感器 prim 仍可用；抓取主视角用 Gemini335。
    for cam_suffix in (
        "base_link/cam0/RSD455",
        "J1_6/cam1/RSD455",
        "J2_6/cam2/RSD455",
    ):
        path = f"/World/nova_robot/{cam_suffix}"
        prim = stage.OverridePrim(path)
        prim.CreateAttribute("physics:rigidBodyEnabled", Sdf.ValueTypeNames.Bool).Set(False)
        UsdGeom.Imageable(prim).MakeInvisible()
        for part in (
            "Visual/Case_front",
            "Visual/Glass",
            "Visual/Case_back",
            "Visual/USB_C",
            "Visual/Mount",
            "Visual/Camera_module",
            "Visual/Front_mask",
            "Visual/camera_mask",
        ):
            vp = stage.OverridePrim(f"{path}/{part}")
            vp.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)

    # grasp_box 不写入 master：由插件按 UI（Paper box / Cassette）运行时创建。

    # ---- Gemini335 capture camera (pose-aligned; local Camera always present) ----
    gemini = UsdGeom.Xform.Define(stage, "/World/Gemini335")
    gxf = UsdGeom.Xformable(gemini)
    gxf.ClearXformOpOrder()
    gxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*gemini_t))
    # Capture scene Quatd stream is (w, x, y, z)
    gxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(gemini_q[0], Gf.Vec3d(gemini_q[1], gemini_q[2], gemini_q[3]))
    )
    gxf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1, 1, 1))

    if gemini_local.is_file():
        gemini.GetPrim().GetReferences().AddReference(_rel(scenes_dir, gemini_local))
        print(f"Gemini335: local asset {_rel(scenes_dir, gemini_local)}")
    else:
        # Stand-in optical camera so the pose is usable without Nucleus
        cam = UsdGeom.Camera.Define(stage, "/World/Gemini335/Camera")
        cam.CreateFocalLengthAttr(1.93)
        cam.CreateHorizontalApertureAttr(3.896)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000000))
        print(
            "Gemini335: no local Orbbec USD — authored stand-in Camera at capture pose "
            "(place orbbec_gemini_335.usd under data/sensors/ and re-bake to swap)"
        )

    stage.GetRootLayer().Export(str(out_path))
    print(f"Wrote master scene: {out_path}")
    print(f"  robot={_rel(scenes_dir, robot_usd)}")
    print(f"  env={_rel(scenes_dir, env_usd)}")
    print("  grasp_box=runtime (plugin UI)")
    print(f"  robot_mount={robot_mount}")
    print(f"  lights=Distant(hidden)+5xSphere@10000, Gemini335 + RSD455 overs")
    return out_path


def main() -> int:
    out = None
    if len(sys.argv) > 1:
        out = Path(sys.argv[1])
    bake(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
