# -*- coding: utf-8 -*-
"""桌面场景扩展 UI：桌子、长方体、相机 6D 位姿与 ROS 出流。"""

from __future__ import annotations

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import btn_builder, get_style

from ..global_variables import (
    DEFAULT_CAMERA_HEIGHT_Z,
    DEFAULT_CAMERA_ROTATION_DEG,
    DEFAULT_TABLE_TOP_Z,
    ORBBEC_G335_DEPTH_HFOV_DEG,
    ORBBEC_G335_DEPTH_HEIGHT,
    ORBBEC_G335_DEPTH_VFOV_DEG,
    ORBBEC_G335_DEPTH_WIDTH,
)
from ..paths import get_extension_paths
from .pose_utils import Pose6D, parse_pose_from_fields
from .scene_loader import SceneLoader
from .session_controller import SessionController
from .topic_config import STREAM_TOGGLE_SPECS, TOPIC_FIELD_SPECS, topic_config_from_ui
from .ui_numeric import (
    add_float_param,
    add_int_param,
    read_entry_as_float,
    read_entry_as_int,
    set_entry_value,
)


class UIBuilder:
    def __init__(self, ext_path: str):
        self._ext_root, self._data_dir, self._raw_data_dir = get_extension_paths(ext_path)
        self._line_edit = {}
        self._topic_fields = {}
        self._stream_toggles = {}
        self._live_toggles = {}
        self._buttons = {}
        self._status_label = None
        self.frames = []

        self._session = SessionController(
            scene_loader=SceneLoader(self._raw_data_dir, self._ext_root),
            on_status=self._set_status,
        )

    def cleanup(self):
        self._session.stop_streaming_only()

    def shutdown(self):
        self._session.stop_all()

    def on_timeline_event(self, event):
        self._session.on_timeline_event(event)

    def on_physics_step(self, _step):
        pass

    def on_app_update(self):
        self._session.on_app_update()

    def on_stage_event(self, _event):
        pass

    def build_ui(self):
        ui_style = get_style()

        scene_frame = CollapsableFrame("Scene", collapsed=False)
        with scene_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                ui.Label(
                    "Uses mz.usd table + cassette/ scan mesh (20260518.obj) on table top.",
                    height=0,
                    word_wrap=True,
                )
                ui.Label("Hover a value and scroll wheel to adjust.", height=0)
                ui.Label("Cassette scan mesh is loaded 1:1 (meters, no scale).", height=0, word_wrap=True)
                cb = self._on_param_changed
                self._line_edit["image_width"] = add_int_param(
                    "Image width (Orbbec G335 depth)",
                    ORBBEC_G335_DEPTH_WIDTH,
                    min_val=64,
                    max_val=4096,
                    on_changed=cb,
                )
                self._line_edit["image_height"] = add_int_param(
                    "Image height (Orbbec G335 depth)",
                    ORBBEC_G335_DEPTH_HEIGHT,
                    min_val=64,
                    max_val=4096,
                    on_changed=cb,
                )
                self._line_edit["horizontal_fov_deg"] = add_float_param(
                    "Horizontal FOV (deg, depth 90°)",
                    ORBBEC_G335_DEPTH_HFOV_DEG,
                    step=1.0,
                    fmt="%.1f",
                    min_val=10.0,
                    max_val=170.0,
                    on_changed=cb,
                )

        camera_pose_frame = CollapsableFrame("Camera 6D Pose (map)", collapsed=False)
        with camera_pose_frame:
            self._build_pose_fields(
                ui_style,
                "cam",
                (
                    "0",
                    "0.52",
                    f"{DEFAULT_CAMERA_HEIGHT_Z:.2f}",
                    f"{DEFAULT_CAMERA_ROTATION_DEG[0]:.1f}",
                    f"{DEFAULT_CAMERA_ROTATION_DEG[1]:.1f}",
                    f"{DEFAULT_CAMERA_ROTATION_DEG[2]:.1f}",
                ),
            )

        object_pose_frame = CollapsableFrame("Cassette 6D Pose (map)", collapsed=False)
        with object_pose_frame:
            self._build_pose_fields(
                ui_style,
                "obj",
                ("0", "0", f"{DEFAULT_TABLE_TOP_Z + 0.002:.3f}", "0", "0", "0"),
            )

        runtime_frame = CollapsableFrame("Runtime Pose Control", collapsed=False)
        with runtime_frame:
            with ui.VStack(style=ui_style, spacing=4, height=0):
                ui.Label(
                    "During Play: enable Live apply to drive Stage from fields below.",
                    height=0,
                    word_wrap=True,
                )
                self._live_toggles["live_camera"] = self._add_checkbox_row(
                    ui_style, "Live apply camera pose", False
                )
                self._live_toggles["live_object"] = self._add_checkbox_row(
                    ui_style, "Live apply cuboid pose", True
                )
                self._live_toggles["kinematic_object"] = self._add_checkbox_row(
                    ui_style, "Cuboid kinematic (ignore physics while posing)", False
                )
                self._buttons["apply_poses"] = btn_builder(
                    label="apply_poses",
                    type="button",
                    text="Apply poses now",
                    tooltip="Apply camera & cuboid 6D pose from fields to Stage",
                    on_clicked_fn=self._on_apply_poses,
                )
                self._buttons["sync_poses"] = btn_builder(
                    label="sync_poses",
                    type="button",
                    text="Sync from Stage",
                    tooltip="Read current Stage poses back into UI fields",
                    on_clicked_fn=self._on_sync_poses,
                )

        topic_frame = CollapsableFrame("ROS Topics", collapsed=False)
        with topic_frame:
            self._build_topic_fields(ui_style)

        stream_frame = CollapsableFrame("Stream Options", collapsed=False)
        with stream_frame:
            self._build_stream_toggles(ui_style)

        actions_frame = CollapsableFrame("Actions", collapsed=False)
        with actions_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._status_label = ui.Label("Status: IDLE", height=0)
                self._buttons["start_stream"] = btn_builder(
                    label="start_stream",
                    type="button",
                    text="Start ROS stream",
                    tooltip="Start publishing without waiting for Timeline Play event",
                    on_clicked_fn=self._on_start_stream,
                )
                self._buttons["stop_stream"] = btn_builder(
                    label="stop_stream",
                    type="button",
                    text="Stop ROS stream",
                    tooltip="Stop ROS camera/TF graphs",
                    on_clicked_fn=self._on_stop_stream,
                )
                self._buttons["load"] = btn_builder(
                    label="load",
                    type="button",
                    text="Load scene",
                    tooltip="Create table, cuboid, camera; then Press Play",
                    on_clicked_fn=self._on_load,
                )
                self._buttons["unload"] = btn_builder(
                    label="unload",
                    type="button",
                    text="Unload",
                    tooltip="Stop streaming and remove scene prims",
                    on_clicked_fn=self._on_unload,
                )

        self.frames = [
            scene_frame,
            camera_pose_frame,
            object_pose_frame,
            runtime_frame,
            topic_frame,
            stream_frame,
            actions_frame,
        ]
        self._apply_session_config()

    def _on_param_changed(self):
        self._apply_session_config()
        if not self._session.scene_loader.is_loaded:
            return
        self._session.apply_poses_from_ui(
            force_camera=self._read_checkbox(self._live_toggles, "live_camera", True),
            force_object=self._read_checkbox(self._live_toggles, "live_object", True),
        )

    def _build_pose_fields(self, ui_style, prefix: str, defaults):
        pos_labels = (("tx", "X (m)"), ("ty", "Y (m)"), ("tz", "Z (m)"))
        rot_labels = (("roll", "Roll (deg)"), ("pitch", "Pitch (deg)"), ("yaw", "Yaw (deg)"))
        cb = self._on_param_changed
        with ui.VStack(style=ui_style, spacing=4, height=0):
            for (suffix, label), default in zip(pos_labels, defaults[:3]):
                key = f"{prefix}_{suffix}"
                self._line_edit[key] = add_float_param(
                    label, float(default), step=0.01, fmt="%.4f", on_changed=cb
                )
            for (suffix, label), default in zip(rot_labels, defaults[3:]):
                key = f"{prefix}_{suffix}"
                self._line_edit[key] = add_float_param(
                    label, float(default), step=0.5, fmt="%.2f", on_changed=cb
                )

    @staticmethod
    def _add_checkbox_row(ui_style, label: str, default: bool):
        with ui.HStack(height=24):
            cb = ui.CheckBox(width=24)
            cb.model.set_value(default)
            ui.Label(label, height=0)
            return cb

    def _build_topic_fields(self, ui_style):
        self._topic_fields = {}
        from .topic_config import TopicConfig

        defaults = TopicConfig()
        with ui.VStack(style=ui_style, spacing=4, height=0):
            for key, label in TOPIC_FIELD_SPECS:
                with ui.HStack(height=24):
                    ui.Label(label, width=220, height=0)
                    field = ui.StringField(height=0)
                    field.model.set_value(getattr(defaults, key))
                    self._topic_fields[key] = field

    def _build_stream_toggles(self, ui_style):
        self._stream_toggles = {}
        from .topic_config import TopicConfig

        defaults = TopicConfig()
        with ui.VStack(style=ui_style, spacing=4, height=0):
            for key, label in STREAM_TOGGLE_SPECS:
                with ui.HStack(height=24):
                    cb = ui.CheckBox(width=24)
                    cb.model.set_value(getattr(defaults, key))
                    ui.Label(label, height=0)
                    self._stream_toggles[key] = cb

    def _read_field(self, key: str, default: str = "") -> str:
        entry = self._line_edit.get(key)
        if entry is None:
            return default
        try:
            return str(read_entry_as_float(entry, float(default or 0)))
        except (TypeError, ValueError):
            return default

    def _read_float(self, key: str, default: float) -> float:
        entry = self._line_edit.get(key)
        if entry is None:
            return default
        return read_entry_as_float(entry, default)

    def _read_int(self, key: str, default: int) -> int:
        entry = self._line_edit.get(key)
        if entry is None:
            return default
        return read_entry_as_int(entry, default)

    def _read_checkbox(self, toggles, key: str, default: bool = False) -> bool:
        cb = toggles.get(key)
        if cb is None:
            return default
        try:
            return cb.model.get_value_as_bool()
        except Exception:
            return default

    def _pose_reader(self) -> dict:
        keys = (
            "cam_tx", "cam_ty", "cam_tz", "cam_roll", "cam_pitch", "cam_yaw",
            "obj_tx", "obj_ty", "obj_tz", "obj_roll", "obj_pitch", "obj_yaw",
        )
        return {k: self._read_field(k) for k in keys}

    def _apply_session_config(self):
        topic_cfg = topic_config_from_ui(
            {k: f.model.get_value_as_string() for k, f in self._topic_fields.items()},
            {k: cb.model.get_value_as_bool() for k, cb in self._stream_toggles.items()},
        )
        width = max(1, self._read_int("image_width", 640))
        height = max(1, self._read_int("image_height", 480))
        self._session.configure(topic_cfg, (width, height), self._pose_reader)
        self._session.set_live_apply(
            self._read_checkbox(self._live_toggles, "live_camera", True),
            self._read_checkbox(self._live_toggles, "live_object", True),
            self._read_checkbox(self._live_toggles, "kinematic_object", False),
        )

    def _set_status(self, text: str):
        if self._status_label:
            self._status_label.text = f"Status: {text}"
        print(f"Status: {text}")

    def _on_load(self):
        self._apply_session_config()
        fov = self._read_float("horizontal_fov_deg", ORBBEC_G335_DEPTH_HFOV_DEG)
        width = max(1, self._read_int("image_width", ORBBEC_G335_DEPTH_WIDTH))
        height = max(1, self._read_int("image_height", ORBBEC_G335_DEPTH_HEIGHT))

        self._session.load_scene(
            {
                "camera_resolution": (width, height),
                "horizontal_fov_deg": fov,
                "vertical_fov_deg": ORBBEC_G335_DEPTH_VFOV_DEG,
            }
        )
        loader = self._session.scene_loader
        poses = self._session.sync_poses_to_ui()
        if loader.last_camera_pose is not None:
            cp = loader.last_camera_pose
            poses.update(
                {
                    "cam_tx": f"{cp.translation[0]:.4f}",
                    "cam_ty": f"{cp.translation[1]:.4f}",
                    "cam_tz": f"{cp.translation[2]:.4f}",
                    "cam_roll": f"{cp.rotation_deg[0]:.2f}",
                    "cam_pitch": f"{cp.rotation_deg[1]:.2f}",
                    "cam_yaw": f"{cp.rotation_deg[2]:.2f}",
                }
            )
        for key, val in poses.items():
            widget = self._line_edit.get(key)
            if widget is not None:
                set_entry_value(widget, val)
        # 相机位姿已在 Load 时写好，勿用错误欧拉角覆盖
        self._session.apply_poses_from_ui(force_object=True, force_camera=False)

    def _on_start_stream(self):
        self._apply_session_config()
        import omni.timeline

        tl = omni.timeline.get_timeline_interface()
        if not tl.is_playing():
            tl.play()
        self._session.start_streaming_manual()

    def _on_stop_stream(self):
        self._session.stop_streaming_only()

    def _on_unload(self):
        self._session.stop_all()

    def _on_apply_poses(self):
        self._apply_session_config()
        self._session.apply_poses_from_ui(force_camera=True, force_object=True)

    def _on_sync_poses(self):
        values = self._session.sync_poses_to_ui()
        for key, val in values.items():
            model = self._line_edit.get(key)
            if model is not None:
                set_entry_value(model, val)
