# -*- coding: utf-8 -*-
"""Simulation scene: visual ground/table, boards, camera, lights (no physics)."""

from __future__ import annotations

import math
import os
from typing import Optional, Tuple

import omni.usd
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

from ..global_variables import (
    BOARD_CENTER_XY,
    BOARD_PATH,
    CAMERA_LINK_PATH,
    CAMERA_OPTICAL_PATH,
    CAMERA_PRIM_PATH,
    DEFAULT_HFOV_DEG,
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_IMAGE_WIDTH,
    GROUND_PATH,
    LIGHT_PATH,
    OPTICAL_FRAME_RPY_DEG,
    ROOT_PRIM,
    TABLE_PATH,
    TABLE_SIZE_XY,
    TABLE_THICKNESS,
    TABLE_TOP_Z,
)
from .aruco_markers import clamp_marker_id, marker_bits, normalize_dictionary_name
from .board_factory import BoardSpec
from .camera_config import usd_camera_attrs_from_fov
from .pose_utils import look_at_matrix, set_translate_rotate, set_world_matrix

BOARD_THICKNESS = 0.008


class SceneLoader:
    def __init__(self, texture_dir: str):
        self._texture_dir = os.path.abspath(texture_dir)
        os.makedirs(self._texture_dir, exist_ok=True)
        self._loaded = False
        self._board_target = (
            BOARD_CENTER_XY[0],
            BOARD_CENTER_XY[1],
            TABLE_TOP_Z + BOARD_THICKNESS,
        )
        self._last_board_type = ""
        self._texture_path = ""

    @property
    def is_loaded(self) -> bool:
        return self._loaded

    @property
    def board_target(self) -> Tuple[float, float, float]:
        return self._board_target

    @property
    def camera_prim_path(self) -> str:
        return CAMERA_PRIM_PATH

    def load(
        self,
        board_spec: BoardSpec,
        camera_resolution=(DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT),
        horizontal_fov_deg: float = DEFAULT_HFOV_DEG,
        initial_eye: Optional[Tuple[float, float, float]] = None,
    ) -> bool:
        try:
            stage = get_current_stage()
            if stage is None:
                print("SceneLoader: no USD stage")
                return False

            self._prepare_stage(stage)
            # Visual-only ground (no PhysX / collision / Isaac World)
            self._create_visual_ground(stage)
            self._ensure_lights(stage)
            self._create_table(stage)
            self._create_or_update_board(stage, board_spec)

            if initial_eye is not None:
                eye = initial_eye
            elif board_spec.is_trihedral:
                eye = (
                    self._board_target[0] + 0.12,
                    self._board_target[1] - 0.12,
                    self._board_target[2] + 0.95,
                )
            else:
                eye = (
                    self._board_target[0] + 0.55,
                    self._board_target[1] - 0.35,
                    self._board_target[2] + 0.55,
                )
            self._create_camera(stage, eye, camera_resolution, horizontal_fov_deg)
            self._stop_timeline()
            self._focus_viewport_on_board()
            self._loaded = True
            print(
                f"SceneLoader: board={board_spec.board_type} "
                f"size={board_spec.physical_size_xy} "
                f"target={self._board_target} tex={self._texture_path} "
                f"camera={CAMERA_PRIM_PATH} (no physics/collision)"
            )
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"SceneLoader.load failed: {exc}")
            self._loaded = False
            return False

    def switch_board(self, board_spec: BoardSpec) -> bool:
        if not self._loaded:
            return False
        stage = get_current_stage()
        if stage is None:
            return False
        try:
            self._create_or_update_board(stage, board_spec)
            # Skip viewport refocus on Apply — saves time on large board rebuilds.
            print(f"SceneLoader: switched board → {board_spec.board_type} tex={self._texture_path}")
            return True
        except Exception as exc:
            print(f"SceneLoader.switch_board failed: {exc}")
            return False

    def set_camera_look_at(
        self,
        eye: Tuple[float, float, float],
        lookat: Tuple[float, float, float] | None = None,
        roll_deg: float = 0.0,
    ) -> None:
        stage = get_current_stage()
        if stage is None or not self._loaded:
            return
        target = lookat if lookat is not None else self._board_target
        m = look_at_matrix(eye, target, roll_deg=roll_deg)
        set_world_matrix(stage, CAMERA_OPTICAL_PATH, m)

    def unload(self) -> None:
        try:
            stage = omni.usd.get_context().get_stage()
            if stage:
                prim = stage.GetPrimAtPath(ROOT_PRIM)
                if prim and prim.IsValid():
                    from omni.usd.commands import DeletePrimsCommand

                    DeletePrimsCommand([ROOT_PRIM]).do()
        except Exception as exc:
            print(f"SceneLoader.unload: {exc}")
        self._loaded = False

    def _prepare_stage(self, stage) -> None:
        if not stage.GetPrimAtPath("/World"):
            UsdGeom.Xform.Define(stage, "/World")
        root = stage.GetPrimAtPath(ROOT_PRIM)
        if root and root.IsValid():
            from omni.usd.commands import DeletePrimsCommand

            DeletePrimsCommand([ROOT_PRIM]).do()
        UsdGeom.Xform.Define(stage, ROOT_PRIM)

    def _ensure_lights(self, stage) -> None:
        # Keep lighting soft so black board cells stay black (not washed to gray).
        dome_i, key_i, fill_i = 600.0, 900.0, 4500.0
        try:
            if not stage.GetPrimAtPath(LIGHT_PATH):
                light = UsdLux.DomeLight.Define(stage, LIGHT_PATH)
                light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
            else:
                light = UsdLux.DomeLight(stage.GetPrimAtPath(LIGHT_PATH))
            light.CreateIntensityAttr(dome_i)

            key_path = f"{ROOT_PRIM}/KeyLight"
            if not stage.GetPrimAtPath(key_path):
                dist = UsdLux.DistantLight.Define(stage, key_path)
                dist.CreateAngleAttr(3.0)
                set_translate_rotate(stage, key_path, (0, 0, 0), (-55.0, 25.0, 0.0))
            else:
                dist = UsdLux.DistantLight(stage.GetPrimAtPath(key_path))
            dist.CreateIntensityAttr(key_i)

            fill_path = f"{ROOT_PRIM}/FillLight"
            if not stage.GetPrimAtPath(fill_path):
                sphere = UsdLux.SphereLight.Define(stage, fill_path)
                sphere.CreateRadiusAttr(0.05)
                sphere.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
                xform = UsdGeom.Xformable(sphere.GetPrim())
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(
                    Gf.Vec3d(BOARD_CENTER_XY[0], BOARD_CENTER_XY[1], TABLE_TOP_Z + 1.2)
                )
            else:
                sphere = UsdLux.SphereLight(stage.GetPrimAtPath(fill_path))
            sphere.CreateIntensityAttr(fill_i)
        except Exception as exc:
            print(f"SceneLoader: lights warning (continue): {exc}")

    def _create_visual_ground(self, stage) -> None:
        """Large flat ground for visuals only — no RigidBody / CollisionAPI."""
        if stage.GetPrimAtPath(GROUND_PATH):
            from omni.usd.commands import DeletePrimsCommand

            DeletePrimsCommand([GROUND_PATH]).do()
        ground = UsdGeom.Mesh.Define(stage, GROUND_PATH)
        # 20m x 20m quad on z=0
        half = 10.0
        ground.CreatePointsAttr(
            [
                Gf.Vec3f(-half, -half, 0.0),
                Gf.Vec3f(half, -half, 0.0),
                Gf.Vec3f(half, half, 0.0),
                Gf.Vec3f(-half, half, 0.0),
            ]
        )
        ground.CreateFaceVertexCountsAttr([4])
        ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        ground.CreateNormalsAttr(
            [Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1), Gf.Vec3f(0, 0, 1)]
        )
        ground.CreateDoubleSidedAttr(False)
        self._apply_color(stage, GROUND_PATH, (0.22, 0.24, 0.26))

    def _create_table(self, stage) -> None:
        table = UsdGeom.Cube.Define(stage, TABLE_PATH)
        table.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(table.GetPrim())
        xform.ClearXformOpOrder()
        sx, sy = TABLE_SIZE_XY
        xform.AddTranslateOp().Set(
            Gf.Vec3d(
                BOARD_CENTER_XY[0],
                BOARD_CENTER_XY[1],
                TABLE_TOP_Z - TABLE_THICKNESS * 0.5,
            )
        )
        xform.AddScaleOp().Set(Gf.Vec3f(sx, sy, TABLE_THICKNESS))
        self._apply_color(stage, TABLE_PATH, (0.55, 0.48, 0.38))

    def _create_or_update_board(self, stage, spec: BoardSpec) -> None:
        """Build board with solid geometry (no texture dependency)."""
        w, h = spec.physical_size_xy
        z_top = TABLE_TOP_Z + BOARD_THICKNESS

        if stage.GetPrimAtPath(BOARD_PATH):
            from omni.usd.commands import DeletePrimsCommand

            DeletePrimsCommand([BOARD_PATH]).do()
        UsdGeom.Xform.Define(stage, BOARD_PATH)
        self._ensure_shared_bw_materials(stage)

        t = spec.board_type
        if spec.is_trihedral:
            self._board_target = self._create_trihedral_board(stage, spec, z_top)
        elif t == "circles_symmetric":
            self._board_target = self._create_circles_board(stage, spec, z_top, asymmetric=False)
        elif t == "circles_asymmetric":
            self._board_target = self._create_circles_board(stage, spec, z_top, asymmetric=True)
        elif t == "aruco":
            self._board_target = self._create_aruco_single(stage, spec, z_top)
        elif t == "aruco_grid":
            self._board_target = self._create_aruco_grid(stage, spec, z_top)
        elif t == "charuco":
            self._board_target = self._create_charuco_board(stage, spec, z_top)
        else:
            # chessboard (default)
            self._board_target = self._create_flat_chess(stage, spec, z_top)

        self._texture_path = ""
        self._last_board_type = t
        print(
            f"SceneLoader: geometry board type={t} size=({w:.3f},{h:.3f}) "
            f"target={self._board_target}"
        )

    def _add_cube(
        self,
        stage,
        path: str,
        center: Tuple[float, float, float],
        scale: Tuple[float, float, float],
        rgb: Tuple[float, float, float],
    ) -> None:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*center))
        xform.AddScaleOp().Set(Gf.Vec3f(*scale))
        self._apply_color(stage, path, rgb)

    def _uv_to_world(
        self,
        origin: Tuple[float, float, float],
        axis: str,
        u: float,
        v: float,
        n: float = 0.0,
    ) -> Gf.Vec3f:
        ox, oy, oz = origin
        if axis == "xy":
            return Gf.Vec3f(ox + u, oy + v, oz + n)
        if axis == "xz":
            return Gf.Vec3f(ox + u, oy + n, oz + v)
        return Gf.Vec3f(ox + n, oy + u, oz + v)

    def _add_flat_quad(
        self,
        stage,
        path: str,
        origin: Tuple[float, float, float],
        axis: str,
        u0: float,
        v0: float,
        u1: float,
        v1: float,
        rgb: Tuple[float, float, float],
        n: float = 0.0,
    ) -> None:
        """Axis-aligned planar quad in UV (exact edges, for calibration patterns)."""
        mesh = UsdGeom.Mesh.Define(stage, path)
        p00 = self._uv_to_world(origin, axis, u0, v0, n)
        p10 = self._uv_to_world(origin, axis, u1, v0, n)
        p11 = self._uv_to_world(origin, axis, u1, v1, n)
        p01 = self._uv_to_world(origin, axis, u0, v1, n)
        mesh.CreatePointsAttr([p00, p10, p11, p01])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateDoubleSidedAttr(True)
        # Face normal
        if axis == "xy":
            nrm = Gf.Vec3f(0, 0, 1)
        elif axis == "xz":
            nrm = Gf.Vec3f(0, 1, 0)
        else:
            nrm = Gf.Vec3f(1, 0, 0)
        mesh.CreateNormalsAttr([nrm, nrm, nrm, nrm])
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        self._apply_color(stage, path, rgb)

    def _add_flat_disc(
        self,
        stage,
        path: str,
        center: Tuple[float, float, float],
        radius: float,
        axis: str = "xy",
        rgb: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        segments: int = 128,
        n: float = 0.0,
    ) -> None:
        """High-segment planar disc (not UsdGeom.Cylinder — that tessellates to ~8 sides)."""
        segments = max(32, int(segments))
        mesh = UsdGeom.Mesh.Define(stage, path)
        c0 = self._uv_to_world(center, axis, 0.0, 0.0, n)
        pts = [c0]
        for i in range(segments):
            ang = 2.0 * math.pi * i / segments
            u = radius * math.cos(ang)
            v = radius * math.sin(ang)
            pts.append(self._uv_to_world(center, axis, u, v, n))
        mesh.CreatePointsAttr(pts)
        counts = [3] * segments
        indices = []
        for i in range(segments):
            i1 = i + 1
            i2 = 1 + ((i + 1) % segments)
            indices.extend([0, i1, i2])
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateDoubleSidedAttr(True)
        if axis == "xy":
            nrm = Gf.Vec3f(0, 0, 1)
        elif axis == "xz":
            nrm = Gf.Vec3f(0, 1, 0)
        else:
            nrm = Gf.Vec3f(1, 0, 0)
        mesh.CreateNormalsAttr([nrm] * len(pts))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        self._apply_color(stage, path, rgb)

    def _add_backing(
        self,
        stage,
        center_xy: Tuple[float, float],
        size_xy: Tuple[float, float],
        z_center: float,
        thickness: float,
    ) -> None:
        path = f"{BOARD_PATH}/Backing"
        self._add_cube(
            stage,
            path,
            (center_xy[0], center_xy[1], z_center),
            (size_xy[0] + 0.012, size_xy[1] + 0.012, thickness),
            (1.0, 1.0, 1.0),
        )

    def _create_flat_chess(self, stage, spec: BoardSpec, z_top: float) -> Tuple[float, float, float]:
        cols, rows = spec.face_squares()
        s = spec.square_length_m
        w, h = cols * s, rows * s
        cx, cy = BOARD_CENTER_XY
        self._add_backing(stage, (cx, cy), (w, h), TABLE_TOP_Z + BOARD_THICKNESS * 0.5, BOARD_THICKNESS)
        root = f"{BOARD_PATH}/Squares"
        UsdGeom.Xform.Define(stage, root)
        origin = (cx - w * 0.5, cy - h * 0.5, z_top + 0.0004)
        for r in range(rows):
            for c in range(cols):
                rgb = (1.0, 1.0, 1.0) if (r + c) % 2 == 0 else (0.0, 0.0, 0.0)
                self._add_flat_quad(
                    stage,
                    f"{root}/sq_{r}_{c}",
                    origin,
                    "xy",
                    c * s,
                    r * s,
                    (c + 1) * s,
                    (r + 1) * s,
                    rgb,
                )
        return (cx, cy, z_top)

    def _create_charuco_board(self, stage, spec: BoardSpec, z_top: float) -> Tuple[float, float, float]:
        """OpenCV CharucoBoard: squares_x/y = cell count, (0,0) black, markers on white."""
        cols, rows = int(spec.squares_x), int(spec.squares_y)
        s = spec.square_length_m
        w, h = cols * s, rows * s
        cx, cy = BOARD_CENTER_XY
        self._add_backing(stage, (cx, cy), (w, h), TABLE_TOP_Z + BOARD_THICKNESS * 0.5, BOARD_THICKNESS)
        origin = (cx - w * 0.5, cy - h * 0.5, z_top + 0.0004)
        sq_root = f"{BOARD_PATH}/Squares"
        UsdGeom.Xform.Define(stage, sq_root)
        for r in range(rows):
            for c in range(cols):
                rgb = (0.0, 0.0, 0.0) if (r + c) % 2 == 0 else (1.0, 1.0, 1.0)
                self._add_flat_quad(
                    stage,
                    f"{sq_root}/sq_{r}_{c}",
                    origin,
                    "xy",
                    c * s,
                    r * s,
                    (c + 1) * s,
                    (r + 1) * s,
                    rgb,
                )
        marker_frac = min(0.95, max(0.4, spec.marker_length_m / max(s, 1e-6)))
        mk_root = f"{BOARD_PATH}/Markers"
        UsdGeom.Xform.Define(stage, mk_root)
        dictionary = normalize_dictionary_name(getattr(spec, "dictionary", "") or "")
        mid = clamp_marker_id(dictionary, int(getattr(spec, "marker_id", 0) or 0))
        for r in range(rows):
            for c in range(cols):
                if (r + c) % 2 == 0:
                    continue  # black square — OpenCV puts markers on white
                cu = (c + 0.5) * s
                cv = (r + 0.5) * s
                center = (origin[0] + cu, origin[1] + cv, origin[2])
                self._add_aruco_marker_geom(
                    stage,
                    f"{mk_root}/m_{r}_{c}",
                    center,
                    s * marker_frac,
                    mid,
                    axis="xy",
                    n_lift=0.00005,
                    dictionary=dictionary,
                )
                mid += 1
        return (cx, cy, z_top)

    def _create_circles_board(
        self, stage, spec: BoardSpec, z_top: float, asymmetric: bool
    ) -> Tuple[float, float, float]:
        w, h = spec.physical_size_xy
        cx, cy = BOARD_CENTER_XY
        self._add_backing(stage, (cx, cy), (w, h), TABLE_TOP_Z + BOARD_THICKNESS * 0.5, BOARD_THICKNESS)
        # Pure white top plane so circle edges are clean
        plane_z = z_top + 0.0003
        self._add_flat_quad(
            stage,
            f"{BOARD_PATH}/WhiteFace",
            (cx - w * 0.5, cy - h * 0.5, plane_z),
            "xy",
            0.0,
            0.0,
            w,
            h,
            (1.0, 1.0, 1.0),
        )
        root = f"{BOARD_PATH}/Circles"
        UsdGeom.Xform.Define(stage, root)
        s = spec.square_length_m
        sx, sy = spec.squares_x, spec.squares_y
        margin = s * 0.5
        radius = s * (0.22 if asymmetric else 0.28)
        for r in range(sy):
            for c in range(sx):
                if asymmetric:
                    x = cx - w * 0.5 + margin + c * s + s * 0.5
                    y = cy - h * 0.5 + margin + (r * 0.5 + 0.5) * s
                    if r % 2 == 1:
                        x += s * 0.5
                else:
                    x = cx - (sx * s) * 0.5 + s * 0.5 + c * s
                    y = cy - (sy * s) * 0.5 + s * 0.5 + r * s
                self._add_flat_disc(
                    stage,
                    f"{root}/c_{r}_{c}",
                    (x, y, plane_z),
                    radius,
                    axis="xy",
                    rgb=(0.0, 0.0, 0.0),
                    segments=128,
                    n=0.00025,
                )
        return (cx, cy, z_top)

    def _create_aruco_single(
        self, stage, spec: BoardSpec, z_top: float
    ) -> Tuple[float, float, float]:
        """Single ArUco marker on a white plate (dictionary + marker_id from spec)."""
        w, h = spec.physical_size_xy
        cx, cy = BOARD_CENTER_XY
        self._add_backing(
            stage, (cx, cy), (w, h), TABLE_TOP_Z + BOARD_THICKNESS * 0.5, BOARD_THICKNESS
        )
        plane_z = z_top + 0.0003
        self._add_flat_quad(
            stage,
            f"{BOARD_PATH}/WhiteFace",
            (cx - w * 0.5, cy - h * 0.5, plane_z),
            "xy",
            0.0,
            0.0,
            w,
            h,
            (1.0, 1.0, 1.0),
        )
        L = max(float(spec.marker_length_m), 0.01)
        dictionary = normalize_dictionary_name(getattr(spec, "dictionary", "") or "")
        mid = clamp_marker_id(dictionary, int(getattr(spec, "marker_id", 0) or 0))
        root = f"{BOARD_PATH}/ArucoSingle"
        UsdGeom.Xform.Define(stage, root)
        self._add_aruco_marker_geom(
            stage,
            f"{root}/m0",
            (cx, cy, plane_z),
            L,
            mid,
            axis="xy",
            n_lift=0.00025,
            dictionary=dictionary,
        )
        print(
            f"SceneLoader: single ArUco id={mid} size={L:.4f}m "
            f"{dictionary} plate=({w:.3f},{h:.3f})"
        )
        return (cx, cy, z_top)

    def _create_aruco_grid(self, stage, spec: BoardSpec, z_top: float) -> Tuple[float, float, float]:
        w, h = spec.physical_size_xy
        cx, cy = BOARD_CENTER_XY
        self._add_backing(stage, (cx, cy), (w, h), TABLE_TOP_Z + BOARD_THICKNESS * 0.5, BOARD_THICKNESS)
        plane_z = z_top + 0.0003
        self._add_flat_quad(
            stage,
            f"{BOARD_PATH}/WhiteFace",
            (cx - w * 0.5, cy - h * 0.5, plane_z),
            "xy",
            0.0,
            0.0,
            w,
            h,
            (1.0, 1.0, 1.0),
        )
        root = f"{BOARD_PATH}/Aruco"
        UsdGeom.Xform.Define(stage, root)
        s = spec.square_length_m
        sx, sy = spec.squares_x, spec.squares_y
        margin = s * 0.5
        dictionary = normalize_dictionary_name(getattr(spec, "dictionary", "") or "")
        mid = clamp_marker_id(dictionary, int(getattr(spec, "marker_id", 0) or 0))
        for r in range(sy):
            for c in range(sx):
                x = cx - w * 0.5 + margin + c * s + s * 0.5
                y = cy - h * 0.5 + margin + r * s + s * 0.5
                self._add_aruco_marker_geom(
                    stage,
                    f"{root}/m_{r}_{c}",
                    (x, y, plane_z),
                    s * 0.90,
                    mid,
                    axis="xy",
                    n_lift=0.00025,
                    dictionary=dictionary,
                )
                mid += 1
        return (cx, cy, z_top)

    def _add_aruco_marker_geom(
        self,
        stage,
        root: str,
        center: Tuple[float, float, float],
        size: float,
        marker_id: int,
        axis: str = "xy",
        n_lift: float = 0.00025,
        flip_u: bool = False,
        dictionary: str = "DICT_4X4_250",
    ) -> None:
        """ArUco marker as coplanar cells (border + bits, same n).

        Stacking a black plate under raised white bits causes parallax: detectMarkers
        boxes drift off the printed marker at oblique views. All cells share n_lift.

        Corner order must match CharucoBoard / generateImageMarker:
        bit row 0 at **low v** (board +Y / image +y). Putting row 0 at high v
        rotates every marker 180° → centers still match, but the four corners
        map wrong and interpolateCornersCharuco / PnP blow up.
        """
        UsdGeom.Xform.Define(stage, root)
        bits, n_bits = marker_bits(dictionary, int(marker_id))
        grid = n_bits + 2
        cell = size / float(grid)
        half = size * 0.5
        if axis == "xy":
            origin = (center[0] - half, center[1] - half, center[2])
        elif axis == "xz":
            origin = (center[0] - half, center[1], center[2] - half)
        else:
            origin = (center[0], center[1] - half, center[2] - half)

        for r in range(grid):
            for c in range(grid):
                if r == 0 or r == grid - 1 or c == 0 or c == grid - 1:
                    white = False
                else:
                    if flip_u:
                        white = bool(bits[r - 1][n_bits - 1 - (c - 1)])
                    else:
                        white = bool(bits[r - 1][c - 1])
                u0 = cell * float(c)
                u1 = u0 + cell
                # r=0 → low v (OpenCV marker top / bit row 0)
                v0 = cell * float(r)
                self._add_flat_quad(
                    stage,
                    f"{root}/c_{r}_{c}",
                    origin,
                    axis,
                    u0,
                    v0,
                    u1,
                    v0 + cell,
                    (1.0, 1.0, 1.0) if white else (0.0, 0.0, 0.0),
                    n=n_lift,
                )

    def _create_trihedral_board(
        self, stage, spec: BoardSpec, z_top: float
    ) -> Tuple[float, float, float]:
        """Three orthogonal **square** faces with white borders; opening → +Z.

        Local build: apex at origin, faces along +X/+Y/+Z.
        Each face is a square plate: pattern is n×n outer cells (n = max(sx,sy)+1)
        inset by one-square white margin on all sides (helps OpenCV detection).
        """
        _ = z_top
        pattern = spec.face_pattern
        if pattern not in ("chess", "charuco", "aruco"):
            pattern = "chess"

        s = float(spec.square_length_m)
        # Force square grid: use the larger of squares_x / squares_y (inner corners)
        n_inner = max(int(spec.squares_x), int(spec.squares_y), 3)
        if pattern == "aruco":
            cols = rows = n_inner
            margin = s * 1.0
        else:
            cols = rows = n_inner + 1  # outer black/white cells
            margin = s * 1.0  # ≈ one square of white border
        face_side = cols * s + 2.0 * margin

        faces_root = f"{BOARD_PATH}/Trihedral"
        UsdGeom.Xform.Define(stage, faces_root)
        thick = 0.004
        mid_base = {"xy": 0, "xz": 0, "yz": 0}  # identical boards; suite splits by geometry
        local_origin = (0.0, 0.0, 0.0)
        dictionary = normalize_dictionary_name(getattr(spec, "dictionary", "") or "")
        first_id = clamp_marker_id(dictionary, int(getattr(spec, "marker_id", 0) or 0))

        for axis, _mid0 in mid_base.items():
            self._paint_trihedral_face(
                stage,
                f"{faces_root}/Face{axis.upper()}",
                origin=local_origin,
                axis=axis,
                cols=cols,
                rows=rows,
                square=s,
                thickness=thick,
                pattern=pattern,
                margin=margin,
                face_side=face_side,
                marker_id0=first_id,
                marker_length_m=spec.marker_length_m,
                dictionary=dictionary,
            )

        # Tip the corner so the concave center opens upward (+Z).
        rot = Gf.Rotation(Gf.Vec3d(1.0, 1.0, 1.0), Gf.Vec3d(0.0, 0.0, 1.0))

        def xform_pt(p: Tuple[float, float, float]) -> Gf.Vec3d:
            return rot.TransformDir(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))

        fw = fh = face_side
        local_pts = [
            (0.0, 0.0, 0.0),
            (fw, 0.0, 0.0),
            (0.0, fh, 0.0),
            (fw, fh, 0.0),
            (fw, 0.0, fh),
            (0.0, 0.0, fh),
            (0.0, fw, fh),
            (0.0, fw, 0.0),
        ]
        world_pts = [xform_pt(p) for p in local_pts]
        min_z = min(p[2] for p in world_pts)
        cx = sum(p[0] for p in world_pts) / len(world_pts)
        cy = sum(p[1] for p in world_pts) / len(world_pts)
        tx = BOARD_CENTER_XY[0] - cx
        ty = BOARD_CENTER_XY[1] - cy
        tz = TABLE_TOP_Z + 0.004 - min_z

        xf = UsdGeom.Xformable(stage.GetPrimAtPath(faces_root))
        xf.ClearXformOpOrder()
        M = Gf.Matrix4d(1.0)
        M.SetRotate(rot)
        M.SetTranslateOnly(Gf.Vec3d(tx, ty, tz))
        xf.AddTransformOp().Set(M)

        apex = Gf.Vec3d(tx, ty, tz)
        opening_h = face_side * 0.35
        target = (float(apex[0]), float(apex[1]), float(apex[2]) + opening_h)
        extra = ""
        if pattern in ("charuco", "aruco"):
            extra = (
                f" {pattern} cells={cols}x{rows} {dictionary} "
                f"first_id={first_id}; identical IDs on all faces"
            )
        print(
            f"SceneLoader: trihedral square faces n_inner={n_inner} "
            f"side={face_side:.3f}m margin={margin:.3f}m apex="
            f"({apex[0]:.3f},{apex[1]:.3f},{apex[2]:.3f}){extra}"
        )
        return target

    def _paint_trihedral_face(
        self,
        stage,
        root: str,
        origin: Tuple[float, float, float],
        axis: str,
        cols: int,
        rows: int,
        square: float,
        thickness: float,
        pattern: str,
        margin: float,
        face_side: float,
        marker_id0: int,
        marker_length_m: float,
        dictionary: str = "DICT_4X4_250",
    ) -> None:
        """Square face plate with solid B/W cells + ArUco geometry (detect-correct)."""
        UsdGeom.Xform.Define(stage, root)
        ox, oy, oz = origin
        fw = fh = float(face_side)
        n_pat = 0.00035
        n_white = 0.00025

        # White backing (square plate)
        if axis == "xy":
            self._add_cube(
                stage,
                f"{root}/back",
                (ox + fw * 0.5, oy + fh * 0.5, oz - thickness * 0.5),
                (fw + 0.004, fh + 0.004, thickness),
                (1.0, 1.0, 1.0),
            )
            face_origin = (ox, oy, oz)
        elif axis == "xz":
            self._add_cube(
                stage,
                f"{root}/back",
                (ox + fw * 0.5, oy - thickness * 0.5, oz + fh * 0.5),
                (fw + 0.004, thickness, fh + 0.004),
                (1.0, 1.0, 1.0),
            )
            face_origin = (ox, oy, oz)
        else:
            self._add_cube(
                stage,
                f"{root}/back",
                (ox - thickness * 0.5, oy + fw * 0.5, oz + fh * 0.5),
                (thickness, fw + 0.004, fh + 0.004),
                (1.0, 1.0, 1.0),
            )
            face_origin = (ox, oy, oz)

        self._add_flat_quad(
            stage,
            f"{root}/white_face",
            face_origin,
            axis,
            0.0,
            0.0,
            fw,
            fh,
            (1.0, 1.0, 1.0),
            n=n_white,
        )

        if pattern in ("chess", "charuco"):
            origin_black = pattern == "charuco"
            for r in range(rows):
                for c in range(cols):
                    even = (r + c) % 2 == 0
                    black = even if origin_black else (not even)
                    rgb = (0.0, 0.0, 0.0) if black else (1.0, 1.0, 1.0)
                    self._add_flat_quad(
                        stage,
                        f"{root}/sq_{r}_{c}",
                        face_origin,
                        axis,
                        margin + c * square,
                        margin + r * square,
                        margin + (c + 1) * square,
                        margin + (r + 1) * square,
                        rgb,
                        n=n_pat,
                    )
            if pattern == "charuco":
                mid = marker_id0
                marker_frac = min(0.95, max(0.4, marker_length_m / max(square, 1e-6)))
                # Mirror side-face layout in U; bits stay OpenCV-correct
                flip_u = axis in ("xz", "yz")
                for r in range(rows):
                    for c in range(cols):
                        if (r + c) % 2 == 0:
                            continue
                        cu = margin + (c + 0.5) * square
                        cv = margin + (r + 0.5) * square
                        if axis == "xy":
                            ctr = (face_origin[0] + cu, face_origin[1] + cv, face_origin[2])
                        elif axis == "xz":
                            ctr = (face_origin[0] + cu, face_origin[1], face_origin[2] + cv)
                        else:
                            ctr = (face_origin[0], face_origin[1] + cu, face_origin[2] + cv)
                        self._add_aruco_marker_geom(
                            stage,
                            f"{root}/m_{r}_{c}",
                            ctr,
                            square * marker_frac,
                            mid,
                            axis=axis,
                            n_lift=n_pat + 0.00003,
                            flip_u=flip_u,
                            dictionary=dictionary,
                        )
                        mid += 1
        else:
            mid = marker_id0
            flip_u = axis in ("xz", "yz")
            for r in range(rows):
                for c in range(cols):
                    cu = margin + (c + 0.5) * square
                    cv = margin + (r + 0.5) * square
                    if axis == "xy":
                        ctr = (face_origin[0] + cu, face_origin[1] + cv, face_origin[2])
                    elif axis == "xz":
                        ctr = (face_origin[0] + cu, face_origin[1], face_origin[2] + cv)
                    else:
                        ctr = (face_origin[0], face_origin[1] + cu, face_origin[2] + cv)
                    self._add_aruco_marker_geom(
                        stage,
                        f"{root}/m_{r}_{c}",
                        ctr,
                        square * 0.90,
                        mid,
                        axis=axis,
                        n_lift=n_pat + 0.00003,
                        flip_u=flip_u,
                        dictionary=dictionary,
                    )
                    mid += 1

    def _ensure_shared_bw_materials(self, stage) -> None:
        """Two shared PreviewSurface mats under the board — cuts Apply material prim spam."""
        looks = f"{BOARD_PATH}/SharedLooks"
        if not stage.GetPrimAtPath(looks):
            UsdGeom.Xform.Define(stage, looks)
        for name, rgb in (("Black", (0.0, 0.0, 0.0)), ("White", (1.0, 1.0, 1.0))):
            mat_path = f"{looks}/{name}"
            if stage.GetPrimAtPath(mat_path):
                continue
            mat = UsdShade.Material.Define(stage, mat_path)
            shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            shader.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.0)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    def _apply_color(self, stage, prim_path: str, rgb: Tuple[float, float, float]) -> None:
        # Prefer shared B/W materials when painting calibration cells
        if abs(rgb[0]) + abs(rgb[1]) + abs(rgb[2]) < 0.05:
            mat_path = f"{BOARD_PATH}/SharedLooks/Black"
        elif rgb[0] > 0.95 and rgb[1] > 0.95 and rgb[2] > 0.95:
            mat_path = f"{BOARD_PATH}/SharedLooks/White"
        else:
            mat_path = None
        if mat_path and stage.GetPrimAtPath(mat_path):
            UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(prim_path)).Bind(
                UsdShade.Material(stage.GetPrimAtPath(mat_path))
            )
            return
        mat_path = f"{prim_path}/Looks/Material"
        if stage.GetPrimAtPath(f"{prim_path}/Looks"):
            from omni.usd.commands import DeletePrimsCommand

            DeletePrimsCommand([f"{prim_path}/Looks"]).do()
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(prim_path)).Bind(mat)

    def _create_camera(
        self,
        stage,
        eye: Tuple[float, float, float],
        resolution: tuple,
        hfov_deg: float,
    ) -> None:
        for path in (CAMERA_LINK_PATH, CAMERA_OPTICAL_PATH):
            if not stage.GetPrimAtPath(path):
                UsdGeom.Xform.Define(stage, path)

        cam = UsdGeom.Camera.Define(stage, CAMERA_PRIM_PATH)
        width, height = resolution
        focal, hap, vap = usd_camera_attrs_from_fov(width, height, hfov_deg)
        cam.CreateFocalLengthAttr(float(focal))
        cam.CreateHorizontalApertureAttr(float(hap))
        cam.CreateVerticalApertureAttr(float(vap))
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))

        set_translate_rotate(stage, CAMERA_LINK_PATH, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        _ = OPTICAL_FRAME_RPY_DEG
        set_world_matrix(stage, CAMERA_OPTICAL_PATH, look_at_matrix(eye, self._board_target))

    def _focus_viewport_on_board(self) -> None:
        """加载后把透视视口对准标定板，避免只看到空地面。"""
        try:
            import omni.kit.viewport.utility as vp_utils

            viewport = vp_utils.get_active_viewport()
            if viewport is None:
                return
            # 选中标定板根节点并 Frame
            try:
                import omni.usd

                ctx = omni.usd.get_context()
                ctx.get_selection().set_selected_prim_paths([BOARD_PATH], True)
            except Exception:
                pass
            try:
                from omni.kit.viewport.utility.camera_state import ViewportCameraState

                cam_state = ViewportCameraState(viewport=viewport)
                # 斜上方看向板中心
                eye = Gf.Vec3d(
                    self._board_target[0] + 0.6,
                    self._board_target[1] - 0.5,
                    self._board_target[2] + 0.7,
                )
                tgt = Gf.Vec3d(*self._board_target)
                cam_state.set_position_world(eye, True)
                cam_state.set_target_world(tgt, True)
            except Exception as exc:
                print(f"SceneLoader: viewport camera frame fallback ({exc})")
                # 旧 API：直接 frame selection
                try:
                    import omni.kit.commands

                    omni.kit.commands.execute("FramePrimsCommand", prim_paths=[BOARD_PATH])
                except Exception:
                    pass
        except Exception as exc:
            print(f"SceneLoader: focus viewport skipped: {exc}")

    def _stop_timeline(self) -> None:
        try:
            import omni.timeline

            omni.timeline.get_timeline_interface().stop()
        except Exception:
            pass
