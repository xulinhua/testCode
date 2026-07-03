# -*- coding: utf-8 -*-
"""料盒识别抓取扩展 UI 与业务编排。"""

from __future__ import annotations

import os

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import btn_builder, get_style, str_builder

from ..defaults import DEFAULT_ROBOT_USD, DEFAULT_SCENE_USD
from ..paths import get_extension_paths
from .calib.calib_exporter import export_calibration
from .calib.intrinsics_provider import DEFAULT_HORIZONTAL_FOV_DEG, StageIntrinsicsProvider
from .scene_loader import SceneLoader
from .session_controller import SessionController
from .topic_config import STREAM_TOGGLE_SPECS, TOPIC_FIELD_SPECS, TopicConfig, topic_config_from_ui


class UIBuilder:
    def __init__(self, ext_path: str):
        self._ext_root, self._data_dir, self._scenes_dir, self._robots_dir, self._calib_dir = (
            get_extension_paths(ext_path)
        )
        os.makedirs(self._scenes_dir, exist_ok=True)
        os.makedirs(self._robots_dir, exist_ok=True)
        os.makedirs(self._calib_dir, exist_ok=True)

        self._line_edit = {}
        self._topic_fields = {}
        self._stream_toggles = {}
        self._buttons = {}
        self._status_label = None
        self._calib_info_label = None
        self._calib_sample_label = None
        self.frames = []

        self._session = SessionController(
            scene_loader=SceneLoader(self._scenes_dir, self._robots_dir),
            on_status=self._set_status,
        )

    def cleanup(self):
        self._session.stop_all()

    def on_timeline_event(self, event):
        self._session.on_timeline_event(event)
        self._refresh_calib_info()

    def on_physics_step(self, step):
        self._session.on_physics_step(step)

    def on_app_update(self):
        self._session.on_update()

    def on_stage_event(self, _event):
        pass

    def build_ui(self):
        ui_style = get_style()

        scene_frame = CollapsableFrame("Scene", collapsed=False)
        with scene_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                ui.Label(
                    "Scene USD optional (put under data/scenes/). Robot USD under data/robots/.",
                    height=0,
                    word_wrap=True,
                )
                self._add_field(
                    ui_style,
                    "scene_usd",
                    "Scene USD (optional)",
                    DEFAULT_SCENE_USD,
                )
                self._add_field(
                    ui_style,
                    "robot_usd",
                    "Robot USD",
                    DEFAULT_ROBOT_USD,
                )
                self._add_field(ui_style, "cam_x", "Camera X", "0")
                self._add_field(ui_style, "cam_y", "Camera Y", "0")
                self._add_field(ui_style, "cam_z", "Camera Z", "1.2")
                self._add_field(ui_style, "image_width", "Image width", "640")
                self._add_field(ui_style, "image_height", "Image height", "480")
                self._add_field(
                    ui_style,
                    "horizontal_fov_deg",
                    "Horizontal FOV (deg)",
                    f"{DEFAULT_HORIZONTAL_FOV_DEG:.1f}",
                )

        topic_frame = CollapsableFrame("ROS Topics", collapsed=False)
        with topic_frame:
            self._build_topic_fields(ui_style)

        stream_frame = CollapsableFrame("Stream Options", collapsed=False)
        with stream_frame:
            self._build_stream_toggles(ui_style)

        calib_frame = CollapsableFrame("Calibration", collapsed=False)
        with calib_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._calib_info_label = ui.Label("Intrinsics/extrinsics: (Load first)", height=0)
                self._calib_sample_label = ui.Label("Samples: 0", height=0)
                self._buttons["calib_start"] = btn_builder(
                    label="calib_start",
                    type="button",
                    text="Start sampling",
                    tooltip="Begin hand-eye sample collection",
                    on_clicked_fn=self._on_calib_start,
                )
                self._buttons["calib_capture"] = btn_builder(
                    label="calib_capture",
                    type="button",
                    text="Capture sample",
                    tooltip="Record current Stage T_world_cam",
                    on_clicked_fn=self._on_calib_capture,
                )
                self._buttons["calib_finish"] = btn_builder(
                    label="calib_finish",
                    type="button",
                    text="Finish & apply",
                    tooltip="Average samples and set extrinsics override",
                    on_clicked_fn=self._on_calib_finish,
                )
                self._buttons["export_calib"] = btn_builder(
                    label="export_calib",
                    type="button",
                    text="Export intrinsics & extrinsics",
                    tooltip="Write JSON to calib_output/",
                    on_clicked_fn=self._on_export_calib,
                )

        actions_frame = CollapsableFrame("Actions", collapsed=False)
        with actions_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._status_label = ui.Label("Status: IDLE", height=0)
                self._buttons["load"] = btn_builder(
                    label="load",
                    type="button",
                    text="Load",
                    tooltip="Load scene (if set), robot USD, and camera",
                    on_clicked_fn=self._on_load,
                )
                self._buttons["start_grasp"] = btn_builder(
                    label="start_grasp",
                    type="button",
                    text="Start",
                    tooltip="Subscribe grasp PoseStamped (world), left arm",
                    on_clicked_fn=self._on_start_grasp,
                )
                self._buttons["stop_grasp"] = btn_builder(
                    label="stop_grasp",
                    type="button",
                    text="Stop grasp",
                    tooltip="Unsubscribe grasp pose topic",
                    on_clicked_fn=self._on_stop_grasp,
                )

        self.frames = [scene_frame, topic_frame, stream_frame, calib_frame, actions_frame]
        self._apply_session_config()

    def _add_field(self, ui_style, key, label, default):
        but_dict = {
            "label": label,
            "type": "stringfield",
            "default_val": default,
            "tooltip": label,
            "on_clicked_fn": None,
            "use_folder_picker": False,
            "read_only": False,
        }
        self._line_edit[key] = str_builder(**but_dict)

    def _build_topic_fields(self, ui_style):
        self._topic_fields = {}
        with ui.VStack(style=ui_style, spacing=4, height=0):
            for key, label in TOPIC_FIELD_SPECS:
                with ui.HStack(height=24):
                    ui.Label(label, width=220, height=0)
                    field = ui.StringField(height=0)
                    field.model.set_value(getattr(TopicConfig(), key))
                    self._topic_fields[key] = field

    def _build_stream_toggles(self, ui_style):
        self._stream_toggles = {}
        defaults = TopicConfig()
        with ui.VStack(style=ui_style, spacing=4, height=0):
            for key, label in STREAM_TOGGLE_SPECS:
                with ui.HStack(height=24):
                    cb = ui.CheckBox(width=24)
                    cb.model.set_value(getattr(defaults, key))
                    ui.Label(label, height=0)
                    self._stream_toggles[key] = cb

    def _read_field(self, key: str, default: str = "") -> str:
        widget = self._line_edit.get(key)
        if widget is None:
            return default
        try:
            return widget.get_value_as_string().strip()
        except Exception:
            return default

    def _read_float(self, key: str, default: float) -> float:
        try:
            return float(self._read_field(key, str(default)))
        except ValueError:
            return default

    def _read_int(self, key: str, default: int) -> int:
        try:
            return int(float(self._read_field(key, str(default))))
        except ValueError:
            return default

    def _collect_topic_config(self):
        fields = {}
        for key, field in self._topic_fields.items():
            fields[key] = field.model.get_value_as_string()
        toggles = {}
        for key, cb in self._stream_toggles.items():
            toggles[key] = cb.model.get_value_as_bool()
        return topic_config_from_ui(fields, toggles)

    def _intrinsics_provider(self) -> StageIntrinsicsProvider:
        return StageIntrinsicsProvider(
            width=self._read_int("image_width", 640),
            height=self._read_int("image_height", 480),
            horizontal_fov_deg=self._read_float("horizontal_fov_deg", DEFAULT_HORIZONTAL_FOV_DEG),
        )

    def _apply_session_config(self):
        cfg = self._collect_topic_config()
        intr = self._intrinsics_provider()
        from .scene_loader import CAMERA_PRIM_PATH

        self._session.configure(
            topic_config=cfg,
            intrinsics=intr,
            camera_prim_path=CAMERA_PRIM_PATH,
            resolution=(intr.width, intr.height),
        )

    def _on_load(self):
        self._apply_session_config()
        pos = (
            self._read_float("cam_x", 0.0),
            self._read_float("cam_y", 0.0),
            self._read_float("cam_z", 1.2),
        )
        ok = self._session.load_scene(
            scene_usd=self._read_field("scene_usd", DEFAULT_SCENE_USD),
            robot_usd=self._read_field("robot_usd", DEFAULT_ROBOT_USD),
            camera_position=pos,
        )
        self._refresh_calib_info()
        if ok:
            print("Load OK — press Timeline Play to stream ROS topics and enable cmd_vel")

    def _on_start_grasp(self):
        self._apply_session_config()
        self._session.start_grasp_mode()

    def _on_stop_grasp(self):
        self._session.stop_grasp_mode()

    def _on_calib_start(self):
        self._session.calib_session.start_sampling()
        self._refresh_calib_info()

    def _on_calib_capture(self):
        self._session.calib_session.capture_sample()
        self._refresh_calib_info()

    def _on_calib_finish(self):
        self._session.calib_session.finish_and_apply()
        self._refresh_calib_info()

    def _on_export_calib(self):
        intr = self._intrinsics_provider()
        result = self._session.calib_session.finish_and_export(self._calib_dir)
        if not result.ok:
            result = export_calibration(
                self._calib_dir,
                intr.get_intrinsics(),
                self._session.extrinsics_provider.get_T_world_camera(),
                calibrated=self._session.extrinsics_provider.is_calibrated,
            )
        if result.ok:
            self._refresh_calib_info()
            print(result.message)

    def _refresh_calib_info(self):
        if self._calib_info_label is None:
            return
        intr = self._intrinsics_provider().get_camera_info_dict()
        extr = self._session.extrinsics_provider.get_T_world_camera()
        src = "calibrated" if self._session.extrinsics_provider.is_calibrated else "stage default"
        self._calib_info_label.text = (
            f"K fx={intr['fx']:.1f} fy={intr['fy']:.1f} "
            f"cx={intr['cx']:.1f} cy={intr['cy']:.1f} "
            f"fov_h={intr['horizontal_fov_deg']:.1f}deg\n"
            f"T_world_cam ({src}): t={extr.translation} q={extr.rotation_xyzw}"
        )
        if self._calib_sample_label is not None:
            n = self._session.calib_session.sample_count
            active = self._session.calib_session.is_active
            self._calib_sample_label.text = f"Samples: {n}" + (" (sampling)" if active else "")

    def _set_status(self, text: str):
        if self._status_label is not None:
            self._status_label.text = f"Status: {text}"
        print(f"Status: {text}")
