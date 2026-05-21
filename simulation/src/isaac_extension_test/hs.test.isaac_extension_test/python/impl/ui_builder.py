# -*- coding: utf-8 -*-
# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
import re
import shutil
import random
import numpy as np
import asyncio
import time
import json
from datetime import datetime


import omni.timeline
import omni.ui as ui
import omni.replicator.core as rep
from isaacsim.core.api.objects.cuboid import FixedCuboid
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
# from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton

from isaacsim.gui.components.ui_utils import btn_builder, str_builder, cb_builder, combo_cb_scrolling_frame_builder, multi_cb_builder, combo_cb_dropdown_builder, multi_dropdown_builder
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style, dropdown_builder, combo_cb_str_builder   
from isaacsim.storage.native import get_assets_root_path
from omni.usd import StageEventType
from pxr import Sdf, UsdLux, UsdGeom,Gf, UsdPhysics, PhysxSchema, Usd

from omni.isaac.core.utils.semantics import add_update_semantics, get_semantics
# from omni.isaac.core.utils.semantics import add_labels
from ..global_variables import class_names, num_classes, meshOptions
from ..paths import get_extension_paths
from .camera_config import compute_intrinsics_k, write_camera_params_json
from .lx_writer import LxWriter
from .usd_assets import (
    apply_table_texture_session_absolute,
    notify_stage_assets_changed,
    register_mz_texture_search_path,
    repair_mz_usd_on_disk,
    repair_table_reference_layers,
    verify_mz_texture_files,
)
from .util import generate_camera_pose
# from .layouts import CategorySelector

TABLE_PRIM_PATH = "/World/table_mz"

SPAWN_FIELD_STYLE_ENABLED = {
    "background_color": 0xFF2B2B2B,
    "color": 0xFFDDDDDD,
}
SPAWN_FIELD_STYLE_DISABLED = {
    "background_color": 0xFF5C5C5C,
    "color": 0xFF909090,
}
SPAWN_LABEL_STYLE_ENABLED = {"color": 0xFFDDDDDD}
SPAWN_LABEL_STYLE_DISABLED = {"color": 0xFF909090}

def _timestamp_ms() -> str:
    """Folder/dialog timestamp with millisecond precision: YYYY_MM_DD_HH_MM_SS_mmm."""
    now = datetime.now()
    return now.strftime("%Y_%m_%d_%H_%M_%S") + f"_{now.microsecond // 1000:03d}"


class UIBuilder:
    """
    插件主要业务类。

    负责三件事：
    1. 构建左侧插件 UI，读取物体数量、生成范围、相机采集参数等输入。
    2. 在 Stage 中创建基础场景：地面、灯光、相机、桌子、随机物体和物理碰撞。
    3. 调用 Replicator + LxWriter 逐帧采集 RGB、深度、语义分割和点云数据。

    extension.py 只负责生命周期和事件转发，本类才是项目的核心逻辑。
    """

    def __init__(self, ext_path: str):
        self._ext_root, self._data_dir, self._raw_data_dir = get_extension_paths(ext_path)
        if not os.path.isdir(self._raw_data_dir):
            print(f"⚠️ 未找到资源目录: {self._raw_data_dir}（Load 将无法加载 USD）")

        # Frames 是可折叠 UI 区块，Isaac 示例模板中常用来做分组面板。
        self.frames = []
        # 使用 UIElementWrapper 创建的控件需要在 cleanup() 中释放，避免热重载后残留回调。
        self.wrapped_ui_elements = []

        # Timeline 控制仿真播放/暂停/停止；物体下落和 Replicator 逐帧采集都依赖时间推进。
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

        self.stage = omni.usd.get_context().get_stage()
        # UI 控件索引表：后续通过 key 读取用户输入值或控制按钮状态。
        self._buttons = {}
        self._line_edit = {}
        # 场景对象句柄。load_world() 创建，collect_data() 使用。
        self.camera = None
        self.render_product = None
        self.ground = None
        self.light = None
        # self.object 保存 /World/obj_i 这些容器 Prim 路径，用于采集前记录物体位姿。
        self.object = []
        # work_dir 指向扩展根目录；采集输出写入 work_dir/data_log/<时间戳>/。
        self.work_dir = self._ext_root
        self.camera_name = None
        # 类别配置来自 global_variables.py。_classes_select 会随复选框变化。
        self._num_classes = num_classes
        self._class_names = class_names.copy()
        self._classes_select = class_names.copy()
        # 每类物体的生成表格行：checkbox、数量、XY 分布范围。
        self._spawn_rows = {}
        # self.world = None
        # World 是 Isaac Core 的场景/物理容器，这里指定 60Hz 物理步长。
        self.world = World(physics_dt=1.0 / 60.0)
        # Replicator Writer。采集前 attach，采集完成后 detach，避免重复写帧。
        self.writter = None
        self._is_collecting = False
        self._physics_initialized = False
        self._camera_resolution = None
        self._capture_frame_poses = []
        self._info_dialog = None
        self._info_dialog_sub = None
        self._pending_info_dialog = None

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """菜单打开后的回调。

        当前没有额外逻辑，保留这个函数是为了兼容 extension.py 的通用模板。
        """
        pass

    def on_timeline_event(self, event):
        """Timeline 播放/暂停/停止回调。

        Args:
            event (omni.timeline.TimelineEventType): 时间轴事件。
        """
        pass
        # if event.type == int(omni.timeline.TimelineEventType.STOP):
        #     # When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
        #     # For complete robustness, the user should resolve those edge cases here
        #     # In general, for extensions based off this template, there is no value to having the user click the play/stop
        #     # button instead of using the Load/Reset/Run buttons provided.
        #     self._scenario_state_btn.reset()
        #     self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """每个物理步的回调。

        目前采集流程没有逐物理步控制，函数保留为后续实现夹爪、机械臂或稳定检测的扩展点。

        Args:
            step (float): 当前物理步长。
        """
        pass

    def on_stage_event(self, event):
        """Stage 事件回调。

        Args:
            event (omni.usd.StageEventType): Stage 打开、关闭等事件。
        """
        pass
        
        # if event.type == int(StageEventType.OPENED):
        #     # If the user opens a new stage, the extension should completely reset
        #     self._reset_extension()

    def cleanup(self):
        """
        Stage 关闭、窗口隐藏或扩展热载时调用。

        通过 Isaac UI wrapper 创建的控件内部可能注册了回调，这里统一 cleanup。
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()
        self._close_info_dialog()

    def _stop_timeline_after_load(self):
        """停止仿真并将时间轴回到 0 帧（接近手动点「复位」）。"""
        try:
            if self.world.is_playing():
                self.world.stop()
            self._timeline.stop()
            self._timeline.set_current_time(0)
        except Exception as exc:
            print(f"timeline stop at t=0: {exc}")

    def _prepare_stage_for_load(self):
        """第二次 Load 前清空 /World 与 World 注册，避免旧物体/地面残留。"""
        try:
            if self.world.is_playing():
                self.world.stop()
            self._timeline.stop()
            self.world.clear()
        except Exception as exc:
            print(f"prepare stage: {exc}")

        self.stage = omni.usd.get_context().get_stage()
        if self.stage:
            world_prim = self.stage.GetPrimAtPath("/World")
            if world_prim and world_prim.IsValid():
                for child in list(world_prim.GetChildren()):
                    try:
                        self.stage.RemovePrim(child.GetPath())
                    except Exception as exc:
                        print(f"Remove {child.GetPath()}: {exc}")

        self.object = []
        self.ground = None
        self.light = None
        self.camera = None
        self.render_product = None
        self._camera_resolution = None
        self._physics_initialized = False

    def _finalize_load_after_spawn(self, spawn_count: int, spawn_detail: str):
        """Reset physics immediately after Load, matching the reference extension."""
        try:
            self.world.reset()
            self._physics_initialized = True
        except Exception as exc:
            print(f"world.reset after load: {exc}")
        self._stop_timeline_after_load()

        summary = (
            f"Spawned {spawn_count} object(s).\n"
            f"{spawn_detail}\n\n"
            "Timeline is at frame 0.\n"
            "Press Play to drop objects, then Start capture.\n"
            "If still invisible, click timeline Reset once."
        )
        print(f"Load complete. {spawn_count} object(s).")
        self._show_info_dialog("Load complete", summary)

    def cleanup_for_load(self):
        """Load 场景前清理采集/弹窗状态，不销毁主面板 UI 控件。"""
        self._close_info_dialog()
        if self.writter:
            try:
                self.writter.detach()
            except Exception:
                pass
            self.writter = None
        self._physics_initialized = False

    def _ensure_physics_initialized(self):
        """在 Start 采集前做一次 soft reset，避免 Load 阶段触发 play/GPU 崩溃。"""
        if self._physics_initialized:
            return
        self.world.reset(soft=True)
        self._physics_initialized = True
        self._stop_timeline_after_load()

    @staticmethod
    def _sanitize_prim_name(name: str) -> str:
        token = re.sub(r"[^A-Za-z0-9_]", "_", name)
        if token and token[0].isdigit():
            token = f"_{token}"
        return token or "asset"

    def _spawn_prim_path(self, class_name: str, class_index: int, usd_stem: str) -> str:
        """e.g. /World/bwb_00_bwb — class, per-class index, asset file stem."""
        cls = self._sanitize_prim_name(class_name)
        asset = self._sanitize_prim_name(usd_stem)
        return f"/World/{cls}_{class_index:02d}_{asset}"

    def _sync_classes_select_from_table(self):
        """根据表格勾选状态更新 _classes_select。"""
        self._classes_select = [
            name
            for name in self._class_names
            if self._spawn_rows.get(name, {}).get("checkbox")
            and self._spawn_rows[name]["checkbox"].model.get_value_as_bool()
        ]
        print(f"Selected classes: {self._classes_select}")

    def _set_spawn_row_fields_enabled(self, category_name: str, enabled: bool):
        row = self._spawn_rows.get(category_name)
        if not row:
            return
        field_style = SPAWN_FIELD_STYLE_ENABLED if enabled else SPAWN_FIELD_STYLE_DISABLED
        label_style = SPAWN_LABEL_STYLE_ENABLED if enabled else SPAWN_LABEL_STYLE_DISABLED
        for key in ("count", "x_min", "x_max", "y_min", "y_max"):
            field = row.get(key)
            if field is not None:
                field.enabled = enabled
                field.style = field_style
        class_label = row.get("class_label")
        if class_label is not None:
            class_label.style = label_style

    def _on_spawn_row_checkbox_changed(self, category_name: str, enabled: bool):
        self._set_spawn_row_fields_enabled(category_name, enabled)
        self._sync_classes_select_from_table()

    def _build_object_spawn_table(self, ui_style):
        """物体类别表格：勾选、数量、每类独立 XY 分布范围。"""
        self._spawn_rows = {}
        col_widths = [24, 72, 44, 56, 56, 56, 56]
        headers = ["", "Class", "Qty", "X min", "X max", "Y min", "Y max"]

        with ui.VStack(style=ui_style, spacing=4, height=0):
            with ui.HStack(height=20):
                for title, width in zip(headers, col_widths):
                    ui.Label(title, width=width, height=0)

            for name in self._class_names:
                self._spawn_rows[name] = {}
                with ui.HStack(height=24):
                    cb = ui.CheckBox(width=col_widths[0])
                    cb.model.set_value(True)
                    cb.model.add_value_changed_fn(
                        lambda model, cat=name: self._on_spawn_row_checkbox_changed(
                            cat, model.get_value_as_bool()
                        )
                    )
                    self._spawn_rows[name]["checkbox"] = cb

                    class_label = ui.Label(name, width=col_widths[1], height=0)
                    class_label.style = SPAWN_LABEL_STYLE_ENABLED
                    self._spawn_rows[name]["class_label"] = class_label

                    count_f = ui.StringField(width=col_widths[2], style=SPAWN_FIELD_STYLE_ENABLED)
                    count_f.model.set_value("2")
                    self._spawn_rows[name]["count"] = count_f

                    for key, width, default in (
                        ("x_min", col_widths[3], "-0.3"),
                        ("x_max", col_widths[4], "0.3"),
                        ("y_min", col_widths[5], "-0.3"),
                        ("y_max", col_widths[6], "0.3"),
                    ):
                        field = ui.StringField(width=width, style=SPAWN_FIELD_STYLE_ENABLED)
                        field.model.set_value(default)
                        self._spawn_rows[name][key] = field

                self._set_spawn_row_fields_enabled(name, True)

        self._sync_classes_select_from_table()

    def _read_string_field(self, field) -> str:
        return field.model.get_value_as_string().strip()

    def _read_spawn_field_float(self, field, default: float) -> float:
        try:
            return float(self._read_string_field(field))
        except ValueError:
            return default

    def _read_spawn_field_int(self, field, default: int) -> int:
        try:
            return int(float(self._read_string_field(field)))
        except ValueError:
            return default

    def _add_string_field(self, ui_style, key: str, label: str, default: str, tooltip: str = ""):
        but_dict = {
            "label": label,
            "type": "stringfield",
            "default_val": default,
            "tooltip": tooltip or label,
            "on_clicked_fn": None,
            "use_folder_picker": False,
            "read_only": False,
        }
        self._line_edit[key] = str_builder(**but_dict)

    def _add_string_fields(self, ui_style, fields):
        """fields: list of (key, label, default) or (key, label, default, tooltip)."""
        with ui.VStack(style=ui_style, spacing=5, height=0):
            for item in fields:
                if len(item) == 4:
                    key, label, default, tooltip = item
                else:
                    key, label, default = item
                    tooltip = label
                self._add_string_field(ui_style, key, label, default, tooltip)

    def _read_spawn_plans_from_ui(self):
        """
        读取表格中已勾选类别的生成计划。

        Returns:
            list[dict]: 每项含 name, count, x_min, x_max, y_min, y_max
        """
        plans = []
        for name in self._class_names:
            row = self._spawn_rows.get(name)
            if not row or not row["checkbox"].model.get_value_as_bool():
                continue
            count = self._read_spawn_field_int(row["count"], 0)
            if count <= 0:
                continue
            plans.append(
                {
                    "name": name,
                    "count": count,
                    "x_min": self._read_spawn_field_float(row["x_min"], -0.3),
                    "x_max": self._read_spawn_field_float(row["x_max"], 0.3),
                    "y_min": self._read_spawn_field_float(row["y_min"], -0.3),
                    "y_max": self._read_spawn_field_float(row["y_max"], 0.3),
                }
            )
        return plans

    def build_ui(self):
        """
        构建插件 UI。

        这个函数在窗口每次重新打开时都会执行，因此只负责创建控件和绑定回调；
        真实场景对象不要在这里创建，避免打开/关闭窗口造成重复加载。
        """
        try:
            self._build_ui_impl()
        except Exception as exc:
            import traceback

            traceback.print_exc()
            with ui.VStack(spacing=5, height=0):
                ui.Label(f"UI build failed: {exc}", height=0)

    def _build_ui_impl(self):
        ui_style = get_style()

        scene_frame = CollapsableFrame("Scene Parameters", collapsed=False)
        with scene_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                ui.Label(
                    "Per-class spawn (uncheck a row to disable its fields):",
                    height=0,
                )
                self._build_object_spawn_table(ui_style)

        camera_frame = CollapsableFrame("Camera Parameters", collapsed=False)
        with camera_frame:
            self._add_string_fields(
                ui_style,
                [
                    ("image_width", "Image width", "640"),
                    ("image_height", "Image height", "480"),
                    ("focal_length", "Focal length (mm)", "24"),
                    ("horiz_aperture", "Horiz aperture (mm)", "20.955"),
                    ("vert_aperture", "Vert aperture (mm)", "15.2908"),
                    ("cam_init_x", "Init cam X", "0"),
                    ("cam_init_y", "Init cam Y", "0"),
                    ("cam_init_z", "Init cam Z", "1"),
                    ("camera_fps", "Camera FPS", "30"),
                    ("render_subframes", "Render subframes", "8"),
                ],
            )

        capture_frame = CollapsableFrame("Capture Parameters", collapsed=True)
        with capture_frame:
            self._add_string_fields(
                ui_style,
                [
                    ("num_of_shots", "Number of shots", "20"),
                    ("cylinder_radius", "Cylinder radius", "0.5"),
                    ("cylinder_height", "Cylinder height", "4"),
                    ("obj_x", "Look-at X", "0"),
                    ("obj_y", "Look-at Y", "0"),
                    ("upper_cylinder_height", "Upper cylinder height", "0.5"),
                    ("obj_radius", "Obj radius", "0.25"),
                ],
            )

        actions_frame = CollapsableFrame("Actions", collapsed=False)
        with actions_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._buttons["load_scene"] = btn_builder(
                    label="load scene",
                    type="button",
                    text="Load",
                    tooltip="Load table, objects, lights and camera",
                    on_clicked_fn=self.load_world,
                )
                self._buttons["collect_data"] = btn_builder(
                    label="collect data",
                    type="button",
                    text="Start",
                    tooltip="Start data collection",
                    on_clicked_fn=self.collection_on_button_click,
                )
                self._buttons["clear_data_log"] = btn_builder(
                    label="clear data_log",
                    type="button",
                    text="Clear data_log",
                    tooltip="Delete all captures under data_log/",
                    on_clicked_fn=self._clear_data_log_impl,
                )

        self.frames = [scene_frame, camera_frame, capture_frame, actions_frame]

        # run_scenario_frame = CollapsableFrame("Run Scenario")

        # with run_scenario_frame:
        #     with ui.VStack(style=get_style(), spacing=5, height=0):
        #         self._scenario_state_btn = StateButton(
        #             "Run Scenario",
        #             "RUN",
        #             "STOP",
        #             on_a_click_fn=self._on_run_scenario_a_text,
        #             on_b_click_fn=self._on_run_scenario_b_text,
        #             physics_callback_fn=self._update_scenario,
        #         )
        #         self._scenario_state_btn.enabled = False
        #         self.wrapped_ui_elements.append(self._scenario_state_btn)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        """模板初始化入口。当前项目没有使用示例机械臂场景，所以保持空实现。"""
        pass
    #     self._articulation = None
    #     self._cuboid = None
    #     self._scenario = ExampleScenario()

    def clear_scene(self):
        """
        清除当前 Stage 中的所有 Prim（除了根节点 /）。
        注意：不会删除引用的资产，只删除当前 Layer 中的 Prim。
        """
        # 获取所有顶级子 Prim（/World, /Cams, /Render 等）
        root_prim = self.stage.GetPseudoRoot()
        children = list(root_prim.GetChildren())  # 转为列表，避免迭代时修改问题

        for child in children:
            # 获取 Prim 的路径
            path = child.GetPath()

            # ⚠️ 可选：跳过某些关键 Prim（如 PhysicsScene, Render Settings）
            # 如果你不确定，可以清空所有
            # 这里我们删除所有，add_default_ground_plane 会重建必要的
            try:
                self.stage.RemovePrim(path)
                print(f"🗑️ Removed: {path}")
            except Exception as e:
                print(f"Failed to remove {path}: {e}")

    def create_new_world(self):
        """
        删除当前场景，创建一个全新的 Stage 和 World。
        """

        # 1. ✅ 清理旧的 World（如果存在）
        if self.world is not None:
            if self.world.is_playing():
                self.world.stop()  # 停止仿真
            self.world.clear()  # 清理内部状态
            self.world = None  # 解引用

        # 1. 获取当前 USD Context
        usd_context = omni.usd.get_context()

        # 2. 关闭当前 Stage（这会清空一切）
        usd_context.new_stage()  # ⚡ 关键：new_stage() 会创建一个空的 Stage
        print("✅ Cleared everything and created a new empty stage.")

        # 3. 创建新的 World 实例（它会绑定到新的 Stage）
        # self.world = World(stage_units_in_meters=1.0)  # 可自定义参数
        self.world = World(physics_dt=1.0 / 60.0)
        # self.world.initialize()
        print("✅ New World created on fresh stage.")

    def load_world(self):
        """
        加载并初始化采集场景。

        流程：
        1. 创建/复用地面、灯光、相机和 render_product。
        2. 从 data/raw_data/mz/mz.usd 加载桌子模型。
        3. 根据 UI 选择的类别，从 data/raw_data/<类别>/ 中随机挑选 USD 模型。
        4. 给桌子和物体添加语义标签、刚体、质量和碰撞近似。
        5. reset World，让物理系统接管物体下落。
        """
        self.cleanup_for_load()
        self._prepare_stage_for_load()

        # 默认地面来自 Isaac Core；加语义标签后，分割和点云语义里能识别为 ground。
        self.ground = self.world.scene.add_default_ground_plane()
        # ground_usd_path = get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd"
        # self.ground = rep.create.from_usd(ground_usd_path, semantics=[('class', 'ground')])
        # self.ground = rep.create_from_usd(ground_usd_path)
        # 追加标签（不覆盖已有）
        # add_labels(self.ground, ["ground"], "class", overwrite=True)
        add_update_semantics(self.ground.prim, "ground", "select_classes")
        # _ = get_semantics(self.ground.prim)
        self.stage = omni.usd.get_context().get_stage()

        # 每次 Load 在清空 /World 后重建灯光。
        light_path = Sdf.Path("/World/DefaultLight")
        if not self.stage.GetPrimAtPath(light_path):
            self.light = UsdLux.DistantLight.Define(self.stage, light_path)
            self.light.CreateIntensityAttr(3000)
            self.light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.9))
            self.light.CreateAngleAttr(0.53)
            rot_z = Gf.Rotation(Gf.Vec3d.ZAxis(), 45)
            rot_y = Gf.Rotation(Gf.Vec3d.YAxis(), -30)
            xform = self.light.AddRotateXYZOp()

            combined_rot = rot_y * rot_z
            mat = Gf.Matrix4d(1.0)
            mat.SetRotate(combined_rot)
            rot_mat3 = mat.ExtractRotationMatrix()

            import math
            m = rot_mat3
            epsilon = 1e-6
            sy = math.sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])

            if sy > epsilon:
                euler_x_rad = math.atan2(m[2][1], m[2][2])
                euler_y_rad = math.atan2(-m[2][0], sy)
                euler_z_rad = math.atan2(m[1][0], m[0][0])
            else:
                euler_x_rad = math.atan2(-m[1][2], m[1][1])
                euler_y_rad = math.atan2(-m[2][0], sy)
                euler_z_rad = 0.0

            euler_angles_deg = Gf.Vec3f(
                math.degrees(euler_x_rad),
                math.degrees(euler_y_rad),
                math.degrees(euler_z_rad),
            )
            xform.Set(euler_angles_deg)
            print(f"成功设置灯光旋转 (XYZ degrees): {euler_angles_deg}")

        # Replicator 相机在 Start 采集时再创建，避免 Load 阶段与 Viewport 抢 GPU。

        # 生成桌子。路径使用相对当前 Python 文件的位置，避免写死某台机器的绝对路径。
        prim_path = TABLE_PRIM_PATH
        mz_dir = os.path.join(self._raw_data_dir, "mz")
        table_usd_path = os.path.join(mz_dir, "mz.usd")
        if not os.path.isfile(table_usd_path):
            print(f"Missing table USD: {table_usd_path}")
            print("Ensure data/raw_data/mz/mz.usd exists in this extension.")
            return

        register_mz_texture_search_path(mz_dir)
        verify_mz_texture_files(mz_dir)
        n_repaired = repair_mz_usd_on_disk(mz_dir)
        print(f"Table USD repair (disk): {n_repaired} update(s)")

        added_obj = add_reference_to_stage(
            usd_path=os.path.abspath(table_usd_path),
            prim_path=prim_path,
        )
        add_update_semantics(added_obj, "mz", "select_classes")
        self.set_obj_position(prim_path, (0, 0, 0))

        n_ref = repair_table_reference_layers(self.stage, prim_path, mz_dir)
        n_sess = apply_table_texture_session_absolute(self.stage, prim_path, mz_dir)
        print(
            f"Table textures: reference_layer={n_ref}, "
            f"session_absolute={n_sess}"
        )
        # 1. 获取物体的 Prim
        obj_prim = self.stage.GetPrimAtPath(prim_path)
        if not obj_prim:
            print(f"❌ 错误：无法找到 Prim {prim_path}，跳过物理属性添加。")
        xform = UsdGeom.Xform(obj_prim)
        xform.AddScaleOp().Set(Gf.Vec3d(2.5, 2.5, 2.5))
        # xform.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))

        # --- 给桌子添加刚体、质量和碰撞属性 ---
        #  应用 Physics Rigid Body API
        # 检查是否已存在
        if not UsdPhysics.RigidBodyAPI(obj_prim):
            applied_rb_api = UsdPhysics.RigidBodyAPI.Apply(obj_prim)

        #应用 Mass API 并设置质量
        # 检查是否已存在 MassAPI
        mass_api = UsdPhysics.MassAPI(obj_prim)
        if not mass_api:
            mass_api = UsdPhysics.MassAPI.Apply(obj_prim)

        if mass_api:
            # 检查质量属性是否存在，如果不存在则创建并设置
            mass_attr = mass_api.GetMassAttr()
            if not mass_attr:
                mass_attr = mass_api.CreateMassAttr()
            # 设置质量值 (例如 1.0 kg)
            mass_attr.Set(10)

        # 递归给桌子内部所有 Mesh 子节点添加碰撞；否则物体会穿过桌面。
        num_collisions = self.apply_collision_to_mesh_children(obj_prim)

        self._sync_classes_select_from_table()
        spawn_plans = self._read_spawn_plans_from_ui()
        if not spawn_plans:
            print("No spawn plan: enable at least one class with Qty > 0.")
            return 1

        data_folder = self._raw_data_dir

        self.work_dir = self._ext_root
        os.makedirs(self.work_dir, exist_ok=True)

        data_log_path = os.path.join(self.work_dir, "data_log")
        if not os.path.exists(data_log_path):
            os.mkdir(data_log_path)

        usd_paths_by_class = {}
        for plan in spawn_plans:
            class_path = os.path.join(data_folder, plan["name"])
            if not os.path.isdir(class_path):
                print(f"Warning: class folder missing, skip {class_path}")
                continue
            paths = [
                os.path.join(class_path, f)
                for f in os.listdir(class_path)
                if f.lower().endswith((".usd", ".usda", ".usdc"))
            ]
            if paths:
                usd_paths_by_class[plan["name"]] = paths
                print(f"Class {plan['name']}: {len(paths)} USD(s), spawn {plan['count']}")
            else:
                print(f"Warning: no USD in {class_path}")

        if not usd_paths_by_class:
            print("Error: no USD models found for enabled classes.")
            return

        total_spawn = sum(
            plan["count"] for plan in spawn_plans if plan["name"] in usd_paths_by_class
        )
        print(f"Spawning {total_spawn} object(s) from spawn table")

        spawn_lines = []
        class_spawn_index = {}
        for plan in spawn_plans:
            class_name = plan["name"]
            class_usds = usd_paths_by_class.get(class_name)
            if not class_usds:
                continue

            x_lo = min(plan["x_min"], plan["x_max"])
            x_hi = max(plan["x_min"], plan["x_max"])
            y_lo = min(plan["y_min"], plan["y_max"])
            y_hi = max(plan["y_min"], plan["y_max"])

            for _ in range(plan["count"]):
                obj_path = random.choice(class_usds)
                usd_stem = os.path.basename(obj_path).rsplit(".", 1)[0]
                class_idx = class_spawn_index.get(class_name, 0)
                prim_path = self._spawn_prim_path(class_name, class_idx, usd_stem)
                class_spawn_index[class_name] = class_idx + 1

                added_obj = add_reference_to_stage(usd_path=obj_path, prim_path=prim_path)
                line = f"{prim_path} <- {class_name}/{os.path.basename(obj_path)}"
                print(f"Spawned {line}")
                spawn_lines.append(line)

                semantic_label = usd_stem
                if semantic_label in self._classes_select or class_name in self._classes_select:
                    add_update_semantics(added_obj, semantic_label, "select_classes")
                else:
                    add_update_semantics(added_obj, class_name, "select_classes")

                self.object.append(prim_path)

                x = random.uniform(x_lo, x_hi)
                y = random.uniform(y_lo, y_hi)
                z = 2
                self.set_obj_position(prim_path, (x, y, z))
                print(f"Placed {prim_path} at ({x:.3f}, {y:.3f}, {z:.3f})")

                obj_prim = self.stage.GetPrimAtPath(prim_path)
                if not obj_prim:
                    print(f"Error: Prim not found {prim_path}")
                    continue

                if not UsdPhysics.RigidBodyAPI(obj_prim):
                    UsdPhysics.RigidBodyAPI.Apply(obj_prim)

                mass_api = UsdPhysics.MassAPI(obj_prim)
                if not mass_api:
                    mass_api = UsdPhysics.MassAPI.Apply(obj_prim)

                if mass_api:
                    mass_attr = mass_api.GetMassAttr()
                    if not mass_attr:
                        mass_attr = mass_api.CreateMassAttr()
                    mass_attr.Set(0.1)

                num_collisions = self.apply_collision_to_mesh_children(obj_prim)
                if num_collisions == 0:
                    print(f"No collision meshes under {prim_path}")

        detail = "\n".join(spawn_lines[:12])
        if len(spawn_lines) > 12:
            detail += f"\n... and {len(spawn_lines) - 12} more"
        self._finalize_load_after_spawn(len(self.object), detail)

    # =====================================================
    # 3. 为所有子级 Mesh 添加 MeshCollisionAPI
    # =====================================================
    @staticmethod
    def apply_collision_to_mesh_children(parent_prim):
        """
        给 parent_prim 及其所有几何子节点添加碰撞。

        USD 模型通常是一个 Xform 容器，真正的 Mesh 在子级；只给顶层 Prim 加碰撞不一定生效。
        当前碰撞近似使用 convexHull，速度和稳定性比较适合数据采集场景。

        Args:
            parent_prim: 要递归处理的 USD Prim。

        Returns:
            int: 成功添加碰撞的几何体数量
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            print("❌ 无法获取 Stage")
            return 0

        applied_count = 0

        # 递归遍历所有后代
        for child_prim in Usd.PrimRange(parent_prim):
            # 检查是否是支持的几何类型
            if not (child_prim.IsA(UsdGeom.Mesh) or
                    child_prim.IsA(UsdGeom.Cube) or
                    child_prim.IsA(UsdGeom.Sphere) or
                    child_prim.IsA(UsdGeom.Cylinder) or
                    child_prim.IsA(UsdGeom.Capsule)):
                continue

            # ✅ 检查是否是 Mesh 并有顶点数据（仅对 Mesh 有意义）
            if child_prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(child_prim)
                points = mesh.GetPointsAttr().Get()
                if not points or len(points) == 0:
                    print(f"⚠️  {child_prim.GetPath()} 是 Mesh 但无顶点数据，跳过")
                    # continue

            #
            # UsdPhysics.CollisionAPI.Apply(child_prim)
            # child_prim.GetAttribute("physxCollision:approximationShape").Set("convexHull")
            # # UsdPhysics.MeshCollisionAPI.Apply(child_prim)
            # print(f"🔧 为 {child_prim.GetPath()} 应用 CollisionAPI")

            # 1. 应用 MeshCollisionAPI
            # meshcollsion_api 需要先collision_api后再设置
            collisionAPI = UsdPhysics.CollisionAPI.Apply(child_prim)
            meshcollsion_api = UsdPhysics.MeshCollisionAPI.Apply(child_prim)
            meshcollsion_api.GetApproximationAttr().Set(meshOptions[2])
            # print(f"✅ Collision added to {child_prim.GetPath()} with Convex Hull")

            applied_count += 1
        return applied_count

    @staticmethod
    def setup_xform(prim_path):
        """清空指定 Prim 的旧 transform op，并创建一个新的 translate op。"""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xform(prim)
        # 清除所有现有操作
        xform.ClearXformOpOrder()
        # 添加一个平移操作
        translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        return translate_op

    def set_obj_position(self, prim_path, position):
        """设置 Prim 的世界平移位置。position 为 (x, y, z)。"""
        translate_op = self.setup_xform(prim_path)
        translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
    # def _load_scene(self):
    #     """
    #     This function is called when the user clicks the Reset Button.
    #     It clears the current stage and creates a new one with a light and a ground plane.
    #     """
    #     # ✅ 使用 World 来统一管理 stage
    #     world = World.instance()
    #     if world is None:
    #         # world = World(stage_units_in_meters=1.0)
    #         world = World()
    #
    #     # # 🔁 reset 会自动创建新 stage
    #     # world.reset()
    #     # print("New USD stage created")
    #
    #     # # ✅ 在 reset 之后添加所有内容
    #     self._add_light_to_stage()
    #     self._add_ground_plane_to_stage()

    def _read_ui_float(self, key: str, default: float) -> float:
        if key not in self._line_edit:
            return default
        try:
            return self._line_edit[key].get_value_as_float()
        except Exception:
            try:
                return float(self._line_edit[key].get_value_as_string())
            except Exception:
                return default

    def _read_ui_int(self, key: str, default: int) -> int:
        if key not in self._line_edit:
            return default
        try:
            return self._line_edit[key].get_value_as_int()
        except Exception:
            try:
                return int(float(self._line_edit[key].get_value_as_string()))
            except Exception:
                return default

    def get_camera_settings_from_ui(self):
        """从 UI 读取相机配置，并计算内参矩阵 K。"""
        width = max(1, self._read_ui_int("image_width", 640))
        height = max(1, self._read_ui_int("image_height", 480))
        focal_length_mm = self._read_ui_float("focal_length", 24.0)
        horiz_aperture_mm = self._read_ui_float("horiz_aperture", 20.955)
        vert_aperture_mm = self._read_ui_float("vert_aperture", 15.2908)
        init_pos = [
            self._read_ui_float("cam_init_x", 0.0),
            self._read_ui_float("cam_init_y", 0.0),
            self._read_ui_float("cam_init_z", 1.0),
        ]
        camera_fps = self._read_ui_float("camera_fps", 30.0)
        render_subframes = max(1, self._read_ui_int("render_subframes", 8))

        k = compute_intrinsics_k(
            width, height, focal_length_mm, horiz_aperture_mm, vert_aperture_mm
        )
        return {
            "resolution": {"width": width, "height": height},
            "focal_length_mm": focal_length_mm,
            "horiz_aperture_mm": horiz_aperture_mm,
            "vert_aperture_mm": vert_aperture_mm,
            "initial_position": init_pos,
            "camera_fps": camera_fps,
            "render_subframes": render_subframes,
            "intrinsics_K": k,
        }

    def _get_sampling_params_from_ui(self):
        return {
            "cylinder_radius": self._read_ui_float("cylinder_radius", 0.5),
            "cylinder_height": self._read_ui_float("cylinder_height", 4.0),
            "obj_x": self._read_ui_float("obj_x", 0.0),
            "obj_y": self._read_ui_float("obj_y", 0.0),
            "upper_cylinder_height": self._read_ui_float("upper_cylinder_height", 0.5),
            "obj_radius": self._read_ui_float("obj_radius", 0.25),
        }

    def _ensure_replicator_camera_from_ui(self):
        """按 UI 创建或重建 Replicator 相机与 render_product。"""
        settings = self.get_camera_settings_from_ui()
        width = settings["resolution"]["width"]
        height = settings["resolution"]["height"]
        resolution = (width, height)
        init_pos = tuple(settings["initial_position"])

        if self.writter:
            try:
                self.writter.detach()
            except Exception:
                pass

        if self.camera is not None and self._camera_resolution != resolution:
            self.camera = None
            self.render_product = None

        if self.camera is None:
            self.camera = rep.create.camera(position=init_pos)
            self.render_product = rep.create.render_product(self.camera, resolution)
            self._camera_resolution = resolution
            print(f"✅ Replicator camera {resolution}, init position {init_pos}")
        else:
            with self.camera:
                rep.modify.pose(position=init_pos)

        return settings

    @staticmethod
    def get_camera_intrinsics(prim_path=None, camera_settings=None):
        """返回内参矩阵 K；优先使用 camera_settings，否则为 640x480 默认。"""
        if camera_settings:
            return camera_settings["intrinsics_K"]
        return compute_intrinsics_k(640, 480, 24.0, 20.955, 15.2908)

    def _hide_info_dialog(self):
        dialog = getattr(self, "_info_dialog", None)
        if dialog is None:
            return
        try:
            dialog.visible = False
        except Exception:
            pass

    def _close_info_dialog(self):
        """关闭弹窗；destroy 延后到下一帧，避免 omni.ui 在事件里 destroy 报错。"""
        self._pending_info_dialog = None
        self._defer_info_dialog_teardown(create_next=False)

    def _defer_info_dialog_teardown(self, create_next: bool = False):
        self._hide_info_dialog()

        def _on_update(_event):
            sub = getattr(self, "_info_dialog_sub", None)
            if sub is not None:
                try:
                    sub.unsubscribe()
                except Exception:
                    pass
                self._info_dialog_sub = None

            dialog = getattr(self, "_info_dialog", None)
            if dialog is not None:
                try:
                    dialog.destroy()
                except Exception:
                    pass
            self._info_dialog = None

            pending = getattr(self, "_pending_info_dialog", None)
            if create_next and pending is not None:
                self._pending_info_dialog = None
                self._build_info_dialog(*pending)

        try:
            old_sub = getattr(self, "_info_dialog_sub", None)
            if old_sub is not None:
                old_sub.unsubscribe()
            stream = omni.kit.app.get_app().get_update_event_stream()
            self._info_dialog_sub = stream.create_subscription_to_pop(_on_update)
        except Exception:
            self._info_dialog = None
            if create_next and self._pending_info_dialog is not None:
                title, message = self._pending_info_dialog
                self._pending_info_dialog = None
                self._build_info_dialog(title, message)

    def _build_info_dialog(self, title: str, message: str):
        print(f"{title}\n{message}")

        line_count = max(1, message.count("\n") + 1)
        win_height = min(420, 130 + line_count * 22)

        self._info_dialog = ui.Window(
            "HsTestInfoDialog",
            width=460,
            height=win_height,
            visible=True,
        )

        def on_ok():
            self._defer_info_dialog_teardown(create_next=False)

        with self._info_dialog.frame:
            with ui.VStack(spacing=10, height=0):
                ui.Label(title, height=0)
                ui.Label(message, word_wrap=True, height=0)
                ui.Button("OK", clicked_fn=on_ok, height=28)

    def _show_info_dialog(self, title: str, message: str):
        """Show a simple English dialog after clear-data_log and similar actions."""
        self._pending_info_dialog = (title, message)
        self._defer_info_dialog_teardown(create_next=True)

    def _clear_data_log_impl(self):
        """清空扩展根目录下 data_log/ 内的全部采集结果。"""
        root = self.work_dir or self._ext_root
        data_log_path = os.path.join(root, "data_log")

        if not os.path.isdir(data_log_path):
            os.makedirs(data_log_path, exist_ok=True)
            self._show_info_dialog(
                "Clear data_log",
                "data_log did not exist. Created an empty directory.",
            )
            return

        removed = []
        errors = []
        for name in os.listdir(data_log_path):
            path = os.path.join(data_log_path, name)
            try:
                if os.path.isdir(path):
                    shutil.rmtree(path)
                else:
                    os.remove(path)
                removed.append(name)
            except OSError as exc:
                errors.append(f"{name}: {exc}")

        if not removed and not errors:
            self._show_info_dialog("Clear data_log", "data_log is already empty. Nothing to delete.")
            return

        lines = [f"Deleted {len(removed)} item(s)."]
        preview = removed[:15]
        if preview:
            lines.append("Removed:")
            lines.extend(f"  - {n}" for n in preview)
        if len(removed) > 15:
            lines.append(f"  ... ({len(removed)} total)")
        if errors:
            lines.append(f"Failed {len(errors)} item(s):")
            lines.extend(f"  - {e}" for e in errors[:5])

        self._show_info_dialog("Clear data_log", "\n".join(lines))
        print(f"✅ data_log cleared: {len(removed)} removed, {len(errors)} errors")

    def collector_folder_prep(self, folder_name):
        """创建一次采集任务的输出目录：data_log/<folder_name>/。"""
        try:
            print(self.work_dir + "/data_log/{}".format(folder_name))
        except:
            pass
        os.mkdir(self.work_dir + "/data_log/{}".format(folder_name))
        abs_folder_path = self.work_dir + "/data_log/{}".format(folder_name)
        return abs_folder_path

    def _pick_item_pose_prim(self, prim):
        """Return the model Prim used for pose export, with safe fallbacks."""
        children = [child for child in prim.GetChildren() if child and child.IsValid()]
        if not children:
            return prim

        prim_name = prim.GetName()
        class_hint = prim_name.split("_", 1)[0] if prim_name else ""
        name_hints = [class_hint, *self._classes_select]
        name_hints = [name.lower() for name in name_hints if name]

        for child in children:
            child_name = child.GetName().lower()
            if any(hint in child_name for hint in name_hints):
                return child

        for child in children:
            try:
                if UsdGeom.Xformable(child):
                    return child
            except Exception:
                pass

        return prim

    def get_current_item_info(self):
        """
        读取当前场景中每个随机物体的世界位姿。

        返回的数据会写入 info.json。这里记录的是 obj_i 下实际模型子 Prim 的世界变换，
        不是外层 /World/obj_i 容器的局部 transform，因此更接近可见几何体的真实位姿。
        """
        item_data = []
        # physics = omni.physx.get_physx_scene_query_interface()
        # timeline = omni.timeline.get_timeline_interface()
        # rigid_body_api = omni.physx.get_rigid_body_interface()
        # physx = omni.physx.get_physx_scene_query_interface()
        # if hasattr(physx, 'get_rigid_body_position') and hasattr(physx, 'get_rigid_body_rotation'):
        #     position = physx.get_rigid_body_position(prim_path)
        #     rotation = physx.get_rigid_body_rotation(prim_path)  # 返回carb.Float4, 顺序为(x, y, z, w)
        #     # 转换为(w, x, y, z)
        #     quat = (rotation.w, rotation.x, rotation.y, rotation.z)
        #     return list(position), list(quat)
        for prim_path in self.object:
            prim = self.stage.GetPrimAtPath(prim_path)
            if not prim:
                continue
            prim_name = prim.GetName()
            # print(prim.GetAllChildrenNames())
            # rigid_body = UsdPhysics.RigidBodyAPI(prim)
            time = Usd.TimeCode.Default()
            # position = physx.get_rigid_body_position(prim_path)
            # rotation = physx.get_rigid_body_rotation(prim_path)
            # transform = rigid_body.get_rigid_pose()
            # pose = rigid_body_api.get_rigid_body_pose(rigid_body)
            # print(dir(rigid_body))
            # print(dir(prim))
            # print(prim.GetAllChildrenNames())

            # 模型导入后通常结构为 /World/<class>_xx_<asset>/<模型名>。
            # 子节点命名不统一时，回退到第一个可变换子 Prim 或外层容器，避免采集中断。
            current_prim = self._pick_item_pose_prim(prim)
            # position = rigid_body.GetTargetPositionAttr().Get(time)
            # orientation = rigid_body.GetTargetRotationAttr().Get(time)

            xform = UsdGeom.Xformable(current_prim)

            world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)

            translation: Gf.Vec3d = world_transform.ExtractTranslation()
            rotation: Gf.Rotation = world_transform.ExtractRotation()
            quat: Gf.Quatd = rotation.GetQuat()
            quat_numpy = np.array([quat.GetReal(), *quat.GetImaginary()])
            # rr = {"name": prim.GetAllChildrenNames()[0], "translation": np.array(translation).tolist(), "quat": np.array(quat_numpy).tolist()}
            rr = {
                "name": current_prim.GetName(),
                "prim_path": str(prim_path),
                "pose_prim_path": str(current_prim.GetPath()),
                "class": prim_name.split("_")[0] if prim_name else current_prim.GetName(),
                "translation": np.array(translation).tolist(),
                "quat": np.array(quat_numpy).tolist(),
            }
            # print(rr)
            item_data.append(rr)
        return item_data

    # ✅ 2. 在 async 函数中
    async def generate_frames(self, shots_count, cylinder, camera_settings):
        """
        逐帧随机生成相机位姿并推进 Replicator。

        每一帧都会调用 generate_camera_pose()，在给定圆柱采样区域内生成相机位置，
        并让相机朝向 obj_x/obj_y 附近的目标区域。
        """
        rt_subframes = camera_settings.get("render_subframes", 8)
        self._capture_frame_poses = []

        for i in range(shots_count):
            print(f'---------------------------------------- Frame {i}')

            with rep.trigger.on_frame():
                with self.camera:
                    pose = generate_camera_pose(
                        cylinder_radius=cylinder["cylinder_radius"],
                        cylinder_height=cylinder["cylinder_height"],
                        obj_xy=np.array([cylinder["obj_x"], cylinder["obj_y"]]),
                        upper_cylinder_height=cylinder["upper_cylinder_height"],
                        obj_radius=cylinder["obj_radius"],
                    )
                    rot = pose["euler_angles"].copy()
                    rot[1] -= 90
                    rep.modify.pose(position=pose["camera_position"], rotation=rot)

                    self._capture_frame_poses.append(
                        {
                            "frame_id": i,
                            "camera_position": pose["camera_position"].tolist(),
                            "target_point": pose["target_point"].tolist(),
                            "quaternion_xyzw": pose["quaternion"].tolist(),
                            "euler_zyx_deg": pose["euler_angles"].tolist(),
                            "euler_applied_zyx_deg": rot.tolist(),
                        }
                    )

            await rep.orchestrator.step_async(
                rt_subframes=rt_subframes,
                delta_time=None,
                pause_timeline=False,
            )

    async def collect_data(self):
        """
        执行一次完整数据采集。

        输出目录：
            data_log/<当前时间>/

        输出内容：
            camera_params.json: 相机配置与每帧外参
            info.json: 相机内参和物体位姿
            rgb_*.png: RGB 图像
            distance_to_camera_*.npy: 深度
            semantic_segmentation_*.png/json: 语义分割及标签映射
            pointcloud_*.npy / lx_data_*.pcd: 点云及 PCD 文件
        """

        if (not len(self.object)):
            print("No object found!")
            return 1
        if self.work_dir is None:
            print("work dir not define, init scene first")
            return 1
        self.stage = omni.usd.get_context().get_stage()
        shots_count = self._read_ui_int("num_of_shots", 20)
        if shots_count < 1:
            print("Invalid num_of_shots")
            return 1

        self._ensure_physics_initialized()
        self._stop_timeline_after_load()

        camera_settings = self._ensure_replicator_camera_from_ui()
        if self.camera is None or self.render_product is None:
            print("Replicator camera not ready. Please Load scene first.")
            return 1

        cylinder = self._get_sampling_params_from_ui()

        current_info = self.get_current_item_info()
        t_start = time.time()
        start_ts = _timestamp_ms()
        folder_name = start_ts
        abs_folder_path = self.collector_folder_prep(folder_name)
        print(f"Collection started at {start_ts} -> {abs_folder_path}")

        camera_params_doc = {
            "capture_timestamp": folder_name,
            **camera_settings,
            "pose_sampling": cylinder,
            "num_shots": shots_count,
        }

        info = {
            "camera_info": camera_settings["intrinsics_K"],
            "objects": current_info,
        }
        with open(os.path.join(abs_folder_path, "info.json"), "w", encoding="utf-8") as fp:
            json.dump(info, fp, indent=4)

        if self.writter:
            self.writter.detach()

        self.writter = rep.WriterRegistry.get("LxWriter")
        self.writter.initialize(
            output_dir=abs_folder_path,
            pointcloud=True,
            rgb=True,
            distance_to_camera=True,
            semantic_segmentation=True,
            semantic_types=["select_classes"],
        )
        self.writter.attach([self.render_product])

        await self.generate_frames(shots_count, cylinder, camera_settings)

        cam_path = write_camera_params_json(abs_folder_path, camera_params_doc, self._capture_frame_poses)
        print(f"Camera params saved: {cam_path}")

        self.writter.detach()

        elapsed_s = time.time() - t_start
        end_ts = _timestamp_ms()
        rel_out = f"data_log/{folder_name}"
        summary = (
            f"Output folder:\n  {rel_out}\n\n"
            f"Start time:  {start_ts}\n"
            f"End time:    {end_ts}\n"
            f"Duration:    {elapsed_s:.1f} s\n"
            f"Frames:      {shots_count}"
        )
        print(
            f"Collection complete. start={start_ts} end={end_ts} "
            f"elapsed={elapsed_s:.1f}s frames={shots_count} path={abs_folder_path}"
        )
        self._show_info_dialog("Collection complete", summary)

    def collection_on_button_click(self):
        if getattr(self, "_is_collecting", False):
            print("Collection already in progress.")
            return

        self._is_collecting = True
        asyncio.ensure_future(self._run_collection())

    async def _run_collection(self):
        try:
            await self.collect_data()
        finally:
            self._is_collecting = False

    def _setup_scene(self):
        """
        这个函数是由Load Button的setup_scene_fn回调函数调用的。
        在用户点击Load Button时，会创建一个新的World实例，然后调用这个函数。
        用户现在可以将他们的资产加载到Stage上，并将它们添加到World场景中。
        在这个例子中，显式地加载了一个新的Stage，并重新加载了所有资产。
        如果用户依赖于热重载功能，并且不希望在每次加载资产时都重新加载，
        他们可以在这里进行检查，以查看他们所需的资产是否已经在Stage上，
        并避免重新加载任何内容，除非它们已经存在。
        
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.

        In this example, a new stage is loaded explicitly, and all assets are reloaded.
        If the user is relying on hot-reloading and does not want to reload assets every time,
        they may perform a check here to see if their desired assets are already on the stage,
        and avoid loading anything if they are.  In this case, the user would still need to add
        their assets to the World (which has low overhead).  See commented code section in this function.
        """
        # 清理原有场景并新建带光线和地面的基础场景
        # self._reset_scene()
        
        # self._load_scene()
        # print("Loading Scene")
        # Load the UR10e
        self.world.scene.add_default_ground_plane()

        robot_prim_path = "/ur10e"
        
        obj_path = ""
        
        # path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        path_to_robot_usd = os.path.join(self._raw_data_dir, "sqm", "sqm.usd")

        # Do not reload assets when hot reloading.  This should only be done while extension is under development.
        # if not is_prim_path_valid(robot_prim_path):
        #     create_new_stage()
        #     add_reference_to_stage(path_to_robot_usd, robot_prim_path)
        # else:
        #     print("Robot already on Stage")

        # 强制清除原有场景
        # create_new_stage()
        # self._add_light_to_stage()

        add_reference_to_stage(path_to_robot_usd, robot_prim_path)

        obj_path = path_to_robot_usd
        prim_path = robot_prim_path

        # 添加到场景
        added_obj = add_reference_to_stage(usd_path=obj_path, prim_path=prim_path)
        # 将添加的对象存入列表 (如果 self.object 是用来存储这些引用的)
        # self.object.append(added_obj)
        # 或者直接使用 prim_path 操作，通常更可靠
        # self.object.append(prim_path)  # 存储路径可能比存储 add_reference_to_stage 的返回值更通用

        # --- 添加刚体和碰撞属性 ---
        # 1. 获取物体的 Prim
        obj_prim = self.stage.GetPrimAtPath(prim_path)
        if not obj_prim:
            print(f"❌ 错误：无法找到 Prim {prim_path}，跳过物理属性添加。")

        # 2. 应用 Physics Rigid Body API
        # 检查是否已存在
        if not UsdPhysics.RigidBodyAPI(obj_prim):
            applied_rb_api = UsdPhysics.RigidBodyAPI.Apply(obj_prim)
            if applied_rb_api:
                print(f"  ✅ 已应用 Physics RigidBody API 到 {prim_path}")
            else:
                print(f"  ⚠️ 无法为 {prim_path} 应用 Physics RigidBody API。")
                # 可以选择跳过这个物体
                # continue
        else:
            print(f"  ℹ️  {prim_path} 已存在 Physics RigidBody API。")

        # 3. (可选) 应用 Mass API 并设置质量
        # 检查是否已存在 MassAPI
        mass_api = UsdPhysics.MassAPI(obj_prim)
        if not mass_api:
            mass_api = UsdPhysics.MassAPI.Apply(obj_prim)

        if mass_api:
            # 检查质量属性是否存在，如果不存在则创建并设置
            mass_attr = mass_api.GetMassAttr()
            if not mass_attr:
                mass_attr = mass_api.CreateMassAttr()
            # 设置质量值 (例如 1.0 kg)
            mass_attr.Set(1.0)
            print(f"  ✅ 已设置 {prim_path} 的质量为 1.0 kg")
        else:
            print(f"  ⚠️ 无法为 {prim_path} 应用或获取 Mass API。")

        # 4. 应用 Physics Collision API (使用 MeshCollisionAPI)
        # 检查是否已存在任何碰撞API

        applied_collision_api = UsdPhysics.MeshCollisionAPI.Apply(obj_prim)
        if applied_collision_api:
            print(f"  ✅ 已应用 Mesh Collision API 到 {prim_path}")
        else:
            print(f"  ⚠️ 无法为 {prim_path} 应用 Mesh Collision API。")




    # def _setup_scenario(self):
    #     """
    #     This function is attached to the Load Button as the setup_post_load_fn callback.
    #     The user may assume that their assets have been loaded by their setup_scene_fn callback, that
    #     their objects are properly initialized, and that the timeline is paused on timestep 0.

    #     In this example, a scenario is initialized which will move each robot joint one at a time in a loop while moving the
    #     provided prim in a circle around the robot.
    #     """
    #     self._reset_scenario()

    #     # UI management
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = True
    #     self._reset_btn.enabled = True

    # def _reset_scenario(self):
    #     self._scenario.teardown_scenario()
    #     self._scenario.setup_scenario(self._articulation, self._cuboid)

    # def _on_post_reset_btn(self):
    #     """
    #     This function is attached to the Reset Button as the post_reset_fn callback.
    #     The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

    #     They may also assume that objects that were added to the World.Scene have been moved to their default positions.
    #     I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
    #     """
    #     self._reset_scenario()

    #     # UI management
    #     self._scenario_state_btn.reset()
    #     self._scenario_state_btn.enabled = True

    # def _update_scenario(self, step: float):
    #     """This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the physics_callback_fn argument.
    #     This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
    #     When the b_text "STOP" is pressed, the physics callback is removed.

    #     Args:
    #         step (float): The dt of the current physics step
    #     """
    #     self._scenario.update_scenario(step)

    # def _on_run_scenario_a_text(self):
    #     """
    #     This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the on_a_click_fn argument.
    #     It is called when the StateButton is clicked while saying a_text "RUN".

    #     This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
    #     the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
    #     through the left-hand UI toolbar.
    #     """
    #     self._timeline.play()

    # def _on_run_scenario_b_text(self):
    #     """
    #     This function is attached to the Run Scenario StateButton.
    #     This function was passed in as the on_b_click_fn argument.
    #     It is called when the StateButton is clicked while saying a_text "STOP"

    #     Pausing the timeline on b_text is not strictly necessary for this example to run.
    #     Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
    #     the robot will stop getting new commands and the cube will stop updating without needing to
    #     pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
    #     forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
    #     this example prettier, but if curious, the user should observe what happens when this line is removed.
    #     """
    #     self._timeline.pause()

    # def _reset_extension(self):
    #     """This is called when the user opens a new stage from self.on_stage_event().
    #     All state should be reset.
    #     """
    #     self._on_init()
    #     self._reset_ui()

    # def _reset_ui(self):
        # self._scenario_state_btn.reset()
        # self._scenario_state_btn.enabled = False
        # self._reset_btn.enabled = False
        
    

    
