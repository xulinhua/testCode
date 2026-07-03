# -*- coding: utf-8 -*-
"""Nova GraspNet 扩展 UI。"""

from __future__ import annotations

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import btn_builder, get_style

from ..global_variables import (
    DEFAULT_BOX_CENTER,
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_WIDTH,
)
from ..paths import get_extension_paths
from .pose_utils import Pose6D, parse_pose_from_fields
from .topic_config import (
    CAMERA_STREAM_TOGGLE_SPECS,
    CAMERA_TOPIC_FIELD_SPECS,
    ROBOT_STREAM_TOGGLE_SPECS,
    ROBOT_TOPIC_FIELD_SPECS,
    SessionTopicConfig,
    topic_config_from_ui,
)
from .ui_numeric import add_int_param, read_entry_as_float, read_entry_as_int, set_entry_value


class UIBuilder:
    """GraspNet 扩展主 UI：场景 Load、盒子位姿、三路相机与 ROS 配置。"""

    def __init__(self, ext_path: str):
        """解析扩展路径，创建 ``SessionController`` 与 ``SceneLoader``。"""
        self._ext_root, self._data_dir, self._robot_dir, self._box_dir = get_extension_paths(ext_path)
        self._line_edit = {}
        self._camera_topic_fields = {}
        self._camera_stream_toggles = {}
        self._camera_resolution = {}
        self._robot_topic_fields = {}
        self._robot_stream_toggles = {}
        self._live_toggles = {}
        self._buttons = {}
        self._status_label = None
        self.frames = []
        self._session = None

    @property
    def session(self):
        """首次访问时再 import SceneLoader / SessionController（避免面板打开前加载 core.api）。"""
        if self._session is None:
            from .scene_loader import SceneLoader
            from .session_controller import SessionController

            self._session = SessionController(
                scene_loader=SceneLoader(self._robot_dir, self._box_dir),
                on_status=self._set_status,
            )
        return self._session

    def cleanup(self):
        """窗口隐藏时停止 ROS 出流（保留场景）。"""
        if self._session is not None:
            self._session.stop_streaming_only()

    def shutdown(self):
        """扩展关闭时 Unload 场景并停止所有图。"""
        if self._session is not None:
            self._session.stop_all()

    def on_timeline_event(self, event):
        """转发 Timeline Play/Stop 到 SessionController。"""
        if self._session is not None:
            self._session.on_timeline_event(event)

    def on_physics_step(self, _step):
        """PhysX 步进回调（当前未使用，预留）。"""

    def on_app_update(self):
        """每帧：live apply 时写盒子位姿。"""
        if self._session is not None:
            self._session.on_app_update()

    def on_stage_event(self, _event):
        """Stage 开关事件（当前未使用）。"""

    def build_ui(self):
        """构建所有 CollapsableFrame 面板与按钮回调。"""
        ui_style = get_style()
        defaults = SessionTopicConfig.default()

        box_pose_frame = CollapsableFrame("Box 6D Pose (world)", collapsed=False)
        with box_pose_frame:
            self._build_pose_fields(
                ui_style,
                "obj",
                (
                    f"{DEFAULT_BOX_CENTER[0]:.3f}",
                    f"{DEFAULT_BOX_CENTER[1]:.3f}",
                    f"{DEFAULT_BOX_CENTER[2]:.3f}",
                    "0",
                    "0",
                    "0",
                ),
            )

        runtime_frame = CollapsableFrame("Runtime Pose Control", collapsed=False)
        with runtime_frame:
            with ui.VStack(style=ui_style, spacing=4, height=0):
                self._live_toggles["live_object"] = self._add_checkbox_row(
                    ui_style, "Live apply box pose during Play", False
                )
                self._live_toggles["kinematic_object"] = self._add_checkbox_row(
                    ui_style, "Box kinematic while posing (disable gravity)", False
                )
                self._buttons["physics_grasp"] = btn_builder(
                    label="physics_grasp",
                    type="button",
                    text="Enable physics grasp (gravity ON)",
                    tooltip="Uncheck Live apply + kinematic; box will fall under gravity",
                    on_clicked_fn=self._on_enable_physics_grasp,
                )
                self._buttons["apply_poses"] = btn_builder(
                    label="apply_poses",
                    type="button",
                    text="Apply box pose now",
                    tooltip="Apply box 6D pose from fields to Stage",
                    on_clicked_fn=self._on_apply_poses,
                )
                self._buttons["sync_poses"] = btn_builder(
                    label="sync_poses",
                    type="button",
                    text="Sync from Stage",
                    tooltip="Read current box pose back into UI fields",
                    on_clicked_fn=self._on_sync_poses,
                )

        camera_frames = []
        for cam in defaults.cameras:
            cam_frame = CollapsableFrame(cam.label, collapsed=True)
            with cam_frame:
                self._build_camera_panel(ui_style, cam.key, cam)
            camera_frames.append(cam_frame)

        robot_frame = CollapsableFrame("Robot ROS", collapsed=True)
        with robot_frame:
            self._build_robot_panel(ui_style, defaults.robot)

        actions_frame = CollapsableFrame("Actions", collapsed=False)
        with actions_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._status_label = ui.Label("Status: IDLE", height=0)
                self._buttons["start_stream"] = btn_builder(
                    label="start_stream",
                    type="button",
                    text="Start ROS stream",
                    tooltip="Play simulation and publish cameras + joint_states + TF",
                    on_clicked_fn=self._on_start_stream,
                )
                self._buttons["stop_stream"] = btn_builder(
                    label="stop_stream",
                    type="button",
                    text="Stop ROS stream",
                    tooltip="Stop publishing and stop simulation (same as Timeline Stop)",
                    on_clicked_fn=self._on_stop_stream,
                )
                self._buttons["load"] = btn_builder(
                    label="load",
                    type="button",
                    text="Load scene",
                    tooltip="Load table + Nova robot + box; then Press Play",
                    on_clicked_fn=self._on_load,
                )
                self._buttons["unload"] = btn_builder(
                    label="unload",
                    type="button",
                    text="Unload",
                    tooltip="Stop streaming and remove scene",
                    on_clicked_fn=self._on_unload,
                )

        self.frames = [box_pose_frame, runtime_frame, *camera_frames, robot_frame, actions_frame]
        self._apply_session_config()

    def _build_camera_panel(self, ui_style, key: str, cam_defaults):
        cb = self._on_param_changed
        self._camera_topic_fields[key] = {}
        self._camera_stream_toggles[key] = {}
        self._camera_resolution[key] = {}

        with ui.VStack(style=ui_style, spacing=4, height=0):
            self._camera_resolution[key]["width"] = add_int_param(
                "Width (px)",
                DEFAULT_CAMERA_WIDTH,
                min_val=64,
                max_val=4096,
                on_changed=cb,
            )
            self._camera_resolution[key]["height"] = add_int_param(
                "Height (px)",
                DEFAULT_CAMERA_HEIGHT,
                min_val=64,
                max_val=4096,
                on_changed=cb,
            )
            for fk, label in CAMERA_TOPIC_FIELD_SPECS:
                with ui.HStack(height=24):
                    ui.Label(label, width=220, height=0)
                    field = ui.StringField(height=0)
                    field.model.set_value(getattr(cam_defaults, fk))
                    self._camera_topic_fields[key][fk] = field
            for tk, label in CAMERA_STREAM_TOGGLE_SPECS:
                with ui.HStack(height=24):
                    cb_w = ui.CheckBox(width=24)
                    cb_w.model.set_value(getattr(cam_defaults, tk))
                    ui.Label(label, height=0)
                    self._camera_stream_toggles[key][tk] = cb_w

    def _build_robot_panel(self, ui_style, robot_defaults):
        with ui.VStack(style=ui_style, spacing=4, height=0):
            for fk, label in ROBOT_TOPIC_FIELD_SPECS:
                with ui.HStack(height=24):
                    ui.Label(label, width=220, height=0)
                    field = ui.StringField(height=0)
                    field.model.set_value(getattr(robot_defaults, fk))
                    self._robot_topic_fields[fk] = field
            for tk, label in ROBOT_STREAM_TOGGLE_SPECS:
                with ui.HStack(height=24):
                    cb = ui.CheckBox(width=24)
                    cb.model.set_value(getattr(robot_defaults, tk))
                    ui.Label(label, height=0)
                    self._robot_stream_toggles[tk] = cb

    def _build_pose_fields(self, ui_style, prefix: str, defaults):
        from .ui_numeric import add_float_param

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

    def _on_param_changed(self):
        self._apply_session_config()
        if self.session.scene_loader.is_loaded:
            self.session.apply_box_pose_from_ui(
                force=self._read_checkbox(self._live_toggles, "live_object", False)
            )

    def _apply_session_config(self):
        """从 UI 控件读取话题/分辨率/勾选，调用 ``SessionController.configure``。"""
        default_cams = {c.key: c for c in SessionTopicConfig.default().cameras}
        camera_fields = {}
        camera_toggles = {}
        camera_res = {}
        for key in self._camera_topic_fields:
            cam_defaults = default_cams.get(key)
            camera_fields[key] = {
                fk: self._read_string_field(self._camera_topic_fields[key].get(fk))
                for fk, _ in CAMERA_TOPIC_FIELD_SPECS
            }
            camera_toggles[key] = {
                tk: self._read_checkbox(
                    self._camera_stream_toggles.get(key, {}),
                    tk,
                    getattr(cam_defaults, tk, True) if cam_defaults else True,
                )
                for tk, _ in CAMERA_STREAM_TOGGLE_SPECS
            }
            camera_res[key] = {
                "width": read_entry_as_int(
                    self._camera_resolution.get(key, {}).get("width"), DEFAULT_CAMERA_WIDTH
                ),
                "height": read_entry_as_int(
                    self._camera_resolution.get(key, {}).get("height"), DEFAULT_CAMERA_HEIGHT
                ),
            }
        robot_fields = {
            fk: self._read_string_field(self._robot_topic_fields.get(fk))
            for fk, _ in ROBOT_TOPIC_FIELD_SPECS
        }
        robot_toggles = {
            tk: self._read_checkbox(self._robot_stream_toggles, tk, True)
            for tk, _ in ROBOT_STREAM_TOGGLE_SPECS
        }
        cfg = topic_config_from_ui(
            camera_fields,
            camera_toggles,
            camera_res,
            robot_fields,
            robot_toggles,
            self.session.scene_loader.camera_prim_paths,
        )
        self.session.configure(cfg, self._read_pose_fields)
        self.session.set_live_apply(
            obj=self._read_checkbox(self._live_toggles, "live_object", False),
            kinematic_object=self._read_checkbox(self._live_toggles, "kinematic_object", False),
        )

    def _read_pose_fields(self) -> dict:
        result = {}
        for key in self._line_edit:
            entry = self._line_edit[key]
            result[key] = str(read_entry_as_float(entry, 0.0))
        return result

    @staticmethod
    def _read_string_field(field, default: str = "") -> str:
        if field is None:
            return default
        try:
            return str(field.model.get_value_as_string()).strip()
        except Exception:
            return default

    @staticmethod
    def _read_checkbox(toggles: dict, key: str, default: bool) -> bool:
        cb = toggles.get(key)
        if cb is None:
            return default
        try:
            return bool(cb.model.get_value_as_bool())
        except Exception:
            return default

    def _default_box_pose(self) -> Pose6D:
        return parse_pose_from_fields(
            self._read_pose_fields().get("obj_tx", f"{DEFAULT_BOX_CENTER[0]:.3f}"),
            self._read_pose_fields().get("obj_ty", f"{DEFAULT_BOX_CENTER[1]:.3f}"),
            self._read_pose_fields().get("obj_tz", f"{DEFAULT_BOX_CENTER[2]:.3f}"),
            self._read_pose_fields().get("obj_roll", "0"),
            self._read_pose_fields().get("obj_pitch", "0"),
            self._read_pose_fields().get("obj_yaw", "0"),
            Pose6D(DEFAULT_BOX_CENTER, (0.0, 0.0, 0.0)),
        )

    def _on_load(self):
        """Load scene 按钮：应用配置 → Load → Sync 位姿回 UI。"""
        self._apply_session_config()
        ok = self.session.load_scene(box_pose=self._default_box_pose())
        if ok:
            synced = self.session.sync_poses_to_ui()
            for k, v in synced.items():
                if k in self._line_edit:
                    set_entry_value(self._line_edit[k], v)
            self._apply_session_config()

    def _on_unload(self):
        self.session.stop_all()
        self._set_status(self.session.status_text())

    def _on_start_stream(self):
        self._apply_session_config()
        self.session.start_streaming_manual()
        self._set_status(self.session.status_text())

    def _on_stop_stream(self):
        self.session.stop_streaming_manual()
        self._set_status(self.session.status_text())

    def _on_apply_poses(self):
        self._apply_session_config()
        self.session.apply_box_pose_from_ui(force=True)

    def _on_enable_physics_grasp(self):
        """一键关闭摆位模式，启用重力。"""
        for key in ("live_object", "kinematic_object"):
            cb = self._live_toggles.get(key)
            if cb is not None:
                cb.model.set_value(False)
        self._apply_session_config()
        self.session.enable_physics_grasp()
        self.session.apply_box_pose_from_ui(force=True)
        self._set_status(self.session.status_text())

    def _on_sync_poses(self):
        synced = self.session.sync_poses_to_ui()
        for k, v in synced.items():
            if k in self._line_edit:
                set_entry_value(self._line_edit[k], v)

    def _set_status(self, text: str):
        if self._status_label:
            self._status_label.text = f"Status: {text}"
