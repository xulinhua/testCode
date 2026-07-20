# -*- coding: utf-8 -*-
"""Nova GraspNet 扩展 UI。"""

from __future__ import annotations

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style

from ..global_variables import (
    BOX_POSE_TOPIC,
    DATA_LOG_DIRNAME,
    DEFAULT_BOX_CENTER,
    DEFAULT_BOX_POSE_RPY,
    DEFAULT_CAMERA_HEIGHT,
    DEFAULT_CAMERA_WIDTH,
    DEFAULT_COLLECT_SAMPLES,
    DEFAULT_SETTLE_STEPS,
    COLLECT_PITCH_RANGE,
    COLLECT_ROLL_RANGE,
    COLLECT_TX_RANGE,
    COLLECT_TY_RANGE,
    COLLECT_TZ_RANGE,
    COLLECT_YAW_RANGE,
)
from ..paths import get_extension_paths
from .data_collect import BoxPoseRange
from .pose_utils import Pose6D, parse_pose_from_fields
from .topic_config import (
    CAMERA_STREAM_TOGGLE_SPECS,
    CAMERA_TOPIC_FIELD_SPECS,
    ROBOT_STREAM_TOGGLE_SPECS,
    ROBOT_TOPIC_FIELD_SPECS,
    SessionTopicConfig,
    topic_config_from_ui,
)
from .ui_numeric import (
    COMPACT_COL_W,
    DUAL_COL_W,
    LABEL_FRAC,
    ROW_HEIGHT,
    TOPIC_LABEL_W,
    VALUE_FRAC,
    add_int_param,
    read_entry_as_float,
    read_entry_as_int,
    set_entry_value,
)

_STACK_SPACING = 0
_COL_GAP = 6
_BTN_H = 22
_BTN_W = 104
_BTN_W_SM = 68
_BTN_W_WIDE = DUAL_COL_W * 2 + _COL_GAP


class UIBuilder:
    """GraspNet 扩展主 UI：场景 Load、盒子位姿、三路相机与 ROS 配置。"""

    def __init__(self, ext_path: str):
        """解析扩展路径，创建 ``SessionController`` 与 ``SceneLoader``。"""
        self._ext_root, self._data_dir, self._robot_dir, self._box_dir = get_extension_paths(ext_path)
        self._line_edit = {}
        self._collect_fields = {}
        self._camera_topic_fields = {}
        self._camera_stream_toggles = {}
        self._camera_resolution = {}
        self._robot_topic_fields = {}
        self._robot_stream_toggles = {}
        self._live_toggles = {}
        self._buttons = {}
        self._status_label = None
        self._collect_progress_label = None
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
                on_collect_progress=self._set_collect_progress,
            )
            self._session.set_extension_root(self._ext_root)
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
        """每帧：live apply 时写盒子位姿；持续发布 /box_pose。"""
        if self._session is not None:
            self._session.on_app_update()

    def on_stage_event(self, _event):
        """Stage 开关事件（当前未使用）。"""

    def build_ui(self):
        """构建所有 CollapsableFrame 面板与按钮回调。"""
        ui_style = get_style()
        defaults = SessionTopicConfig.default()

        box_pose_frame = CollapsableFrame("Box Pose (world)", collapsed=False)
        with box_pose_frame:
            self._build_pose_fields(
                ui_style,
                "obj",
                (
                    f"{DEFAULT_BOX_CENTER[0]:.3f}",
                    f"{DEFAULT_BOX_CENTER[1]:.3f}",
                    f"{DEFAULT_BOX_CENTER[2]:.3f}",
                    f"{DEFAULT_BOX_POSE_RPY[0]:.1f}",
                    f"{DEFAULT_BOX_POSE_RPY[1]:.1f}",
                    f"{DEFAULT_BOX_POSE_RPY[2]:.1f}",
                ),
            )

        runtime_frame = CollapsableFrame("Runtime", collapsed=True)
        with runtime_frame:
            with ui.VStack(style=ui_style, spacing=_STACK_SPACING, height=0):
                self._live_toggles["live_object"] = self._add_checkbox_row(
                    ui_style, "Live apply box pose", False
                )
                self._live_toggles["kinematic_object"] = self._add_checkbox_row(
                    ui_style, "Box kinematic (no gravity)", False
                )
                with self._compact_btn_row():
                    self._buttons["physics_grasp"] = self._compact_btn(
                        "Physics ON", "Enable gravity on box", self._on_enable_physics_grasp, width=_BTN_W_SM
                    )
                    self._buttons["apply_poses"] = self._compact_btn(
                        "Apply", "Apply box pose from fields", self._on_apply_poses, width=_BTN_W_SM
                    )
                    self._buttons["sync_poses"] = self._compact_btn(
                        "Sync", "Read pose from Stage", self._on_sync_poses, width=_BTN_W_SM
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

        collect_frame = CollapsableFrame("Data Collection", collapsed=True)
        with collect_frame:
            self._build_collect_panel(ui_style)

        grasp_frame = CollapsableFrame("Box Pose ROS", collapsed=False)
        with grasp_frame:
            self._build_grasp_panel(ui_style)

        actions_frame = CollapsableFrame("Actions", collapsed=False)
        with actions_frame:
            with ui.VStack(style=ui_style, spacing=_STACK_SPACING, height=0):
                self._status_label = ui.Label("Status: IDLE", height=0)
                with self._compact_btn_row():
                    self._buttons["load"] = self._compact_btn(
                        "Load", "Load table + Nova robot + box", self._on_load
                    )
                    self._buttons["unload"] = self._compact_btn(
                        "Unload", "Stop streaming and remove scene", self._on_unload
                    )
                with self._compact_btn_row():
                    self._buttons["start_stream"] = self._compact_btn(
                        "Start ROS",
                        "Play and publish cameras + joint_states + TF + /box_pose (base_link)",
                        self._on_start_stream,
                    )
                    self._buttons["stop_stream"] = self._compact_btn(
                        "Stop ROS", "Stop publishing and simulation", self._on_stop_stream
                    )

        self.frames = [
            box_pose_frame,
            runtime_frame,
            *camera_frames,
            robot_frame,
            collect_frame,
            grasp_frame,
            actions_frame,
        ]
        self._apply_session_config()

    def _build_camera_panel(self, ui_style, key: str, cam_defaults):
        cb = self._on_param_changed
        self._camera_topic_fields[key] = {}
        self._camera_stream_toggles[key] = {}
        self._camera_resolution[key] = {}

        with ui.VStack(style=ui_style, spacing=_STACK_SPACING, height=0):
            with ui.HStack(spacing=_COL_GAP, height=0):
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    self._camera_resolution[key]["width"] = add_int_param(
                        "W (px)",
                        DEFAULT_CAMERA_WIDTH,
                        min_val=64,
                        max_val=4096,
                        on_changed=cb,
                        dual_col=True,
                    )
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    self._camera_resolution[key]["height"] = add_int_param(
                        "H (px)",
                        DEFAULT_CAMERA_HEIGHT,
                        min_val=64,
                        max_val=4096,
                        on_changed=cb,
                        dual_col=True,
                    )
            for fk, label in CAMERA_TOPIC_FIELD_SPECS:
                self._camera_topic_fields[key][fk] = self._add_string_row(
                    ui_style, label, getattr(cam_defaults, fk), full_width=True
                )
            for tk, label in CAMERA_STREAM_TOGGLE_SPECS:
                self._camera_stream_toggles[key][tk] = self._add_checkbox_row(
                    ui_style, label, getattr(cam_defaults, tk)
                )

    def _build_robot_panel(self, ui_style, robot_defaults):
        with ui.VStack(style=ui_style, spacing=_STACK_SPACING, height=0):
            for fk, label in ROBOT_TOPIC_FIELD_SPECS:
                self._robot_topic_fields[fk] = self._add_string_row(
                    ui_style, label, getattr(robot_defaults, fk), full_width=True
                )
            for tk, label in ROBOT_STREAM_TOGGLE_SPECS:
                self._robot_stream_toggles[tk] = self._add_checkbox_row(
                    ui_style, label, getattr(robot_defaults, tk)
                )

    def _build_collect_panel(self, ui_style):
        from .ui_numeric import add_float_param, add_int_param

        cb = self._on_param_changed
        with ui.VStack(style=ui_style, spacing=_STACK_SPACING, height=0):
            self._collect_fields["output_dir"] = self._add_string_row(
                ui_style, "Out dir", f"{self._ext_root}/{DATA_LOG_DIRNAME}", full_width=True
            )
            with ui.HStack(spacing=_COL_GAP, height=0):
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    self._collect_fields["num_samples"] = add_int_param(
                        "Samples",
                        DEFAULT_COLLECT_SAMPLES,
                        min_val=1,
                        max_val=10000,
                        on_changed=cb,
                        dual_col=True,
                    )
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    self._collect_fields["settle_steps"] = add_int_param(
                        "Settle",
                        DEFAULT_SETTLE_STEPS,
                        min_val=10,
                        max_val=600,
                        on_changed=cb,
                        dual_col=True,
                    )
            pairs = (
                (("tx_min", "X min"), ("tx_max", "X max"), COLLECT_TX_RANGE[0], COLLECT_TX_RANGE[1], 0.01, "%.3f", "m"),
                (("ty_min", "Y min"), ("ty_max", "Y max"), COLLECT_TY_RANGE[0], COLLECT_TY_RANGE[1], 0.01, "%.3f", "m"),
                (("tz_min", "Z min"), ("tz_max", "Z max"), COLLECT_TZ_RANGE[0], COLLECT_TZ_RANGE[1], 0.01, "%.3f", "m"),
                (("roll_min", "Roll min"), ("roll_max", "Roll max"), COLLECT_ROLL_RANGE[0], COLLECT_ROLL_RANGE[1], 1.0, "%.1f", "deg"),
                (("pitch_min", "Pitch min"), ("pitch_max", "Pitch max"), COLLECT_PITCH_RANGE[0], COLLECT_PITCH_RANGE[1], 1.0, "%.1f", "deg"),
                (("yaw_min", "Yaw min"), ("yaw_max", "Yaw max"), COLLECT_YAW_RANGE[0], COLLECT_YAW_RANGE[1], 1.0, "%.1f", "deg"),
            )
            with ui.HStack(spacing=_COL_GAP, height=0):
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    for (k1, l1), _, v1, _, step, fmt, unit in pairs:
                        self._collect_fields[k1] = add_float_param(
                            f"{l1} ({unit})",
                            v1,
                            step=step,
                            fmt=fmt,
                            on_changed=cb,
                            dual_col=True,
                        )
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    for _, (k2, l2), _, v2, step, fmt, unit in pairs:
                        self._collect_fields[k2] = add_float_param(
                            f"{l2} ({unit})",
                            v2,
                            step=step,
                            fmt=fmt,
                            on_changed=cb,
                            dual_col=True,
                        )
            with self._compact_btn_row():
                self._buttons["collect_start"] = self._compact_btn(
                    "Start collection",
                    "Random box poses (kinematic pin), render settle, Replicator capture",
                    self._on_start_collection,
                    width=_BTN_W_WIDE,
                )
            self._collect_progress_label = ui.Label("Sample: —", height=0)

    def _build_grasp_panel(self, ui_style):
        """仅保留盒子位姿话题；仿真启动后自动持续发布相对 base_link 的 PoseStamped。"""
        with ui.VStack(style=ui_style, spacing=_STACK_SPACING, height=0):
            self._collect_fields["box_pose_topic"] = self._add_string_row(
                ui_style,
                "Box",
                BOX_POSE_TOPIC,
                full_width=True,
            )
            ui.Label(
                "Auto-publishes box pose in base_link after Start ROS / Play",
                word_wrap=True,
                height=0,
            )

    def _compact_btn_row(self):
        """左对齐的紧凑按钮行（不铺满面板宽度）。"""
        return ui.HStack(spacing=4, height=0)

    def _compact_btn(self, text: str, tooltip: str, on_clicked, *, width: int = _BTN_W):
        btn = ui.Button(text, width=width, height=_BTN_H, clicked_fn=on_clicked)
        if tooltip:
            btn.set_tooltip(tooltip)
        return btn

    def _add_string_row(self, ui_style, label: str, default: str, *, full_width: bool = False):
        with ui.HStack(style=ui_style, height=ROW_HEIGHT):
            if full_width:
                ui.Label(label, width=TOPIC_LABEL_W, height=0)
                field = ui.StringField(height=0)
            else:
                ui.Label(label, width=COMPACT_COL_W // 2, height=0)
                field = ui.StringField(width=COMPACT_COL_W, height=0)
            field.model.set_value(default)
            return field

    def _build_pose_fields(self, ui_style, prefix: str, defaults):
        from .ui_numeric import add_float_param

        # 每行：平移 | 旋转（与 Data Collection 相同行内双列，避免左列被挤没）
        rows = (
            ("tx", "X (m)", "roll", "Roll", 0.01, "%.4f", 0.5, "%.2f"),
            ("ty", "Y (m)", "pitch", "Pitch", 0.01, "%.4f", 0.5, "%.2f"),
            ("tz", "Z (m)", "yaw", "Yaw", 0.01, "%.4f", 0.5, "%.2f"),
        )
        cb = self._on_param_changed
        vals = list(defaults)

        with ui.VStack(style=ui_style, spacing=0, height=0):
            with ui.HStack(spacing=_COL_GAP, height=0):
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    for i, (pos_k, pos_l, rot_k, rot_l, ps, pf, rs, rf) in enumerate(rows):
                        self._line_edit[f"{prefix}_{pos_k}"] = add_float_param(
                            pos_l,
                            float(vals[i]),
                            step=ps,
                            fmt=pf,
                            on_changed=cb,
                            dual_col=True,
                        )
                with ui.VStack(spacing=0, width=DUAL_COL_W, height=0):
                    for i, (pos_k, pos_l, rot_k, rot_l, ps, pf, rs, rf) in enumerate(rows):
                        self._line_edit[f"{prefix}_{rot_k}"] = add_float_param(
                            rot_l,
                            float(vals[3 + i]),
                            step=rs,
                            fmt=rf,
                            on_changed=cb,
                            dual_col=True,
                        )

    def _add_checkbox_row(self, ui_style, label: str, default: bool):
        with ui.HStack(height=ROW_HEIGHT):
            cb = ui.CheckBox(width=18)
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
        self.session.set_grasp_options(
            publish_gt=True,
            arm_mode="auto",
            grasp_pose_topic="/graspnet/best_grasp",
            box_pose_topic=self._read_string_field(
                self._collect_fields.get("box_pose_topic"), BOX_POSE_TOPIC
            ),
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
            Pose6D(DEFAULT_BOX_CENTER, DEFAULT_BOX_POSE_RPY),
        )

    def _on_load(self):
        """Load scene 按钮：应用配置 → Load → Sync 位姿回 UI。"""
        print("UI: loading scene...")
        self._apply_session_config()
        ok = self.session.load_scene(box_pose=self._default_box_pose())
        if ok:
            synced = self.session.sync_poses_to_ui()
            for k, v in synced.items():
                if k in self._line_edit:
                    set_entry_value(self._line_edit[k], v)
            self._apply_session_config()
            print("UI: scene loaded — Press Play or Start ROS")
        else:
            print("UI: Load scene failed — see console SceneLoader logs")
        self._set_status(self.session.status_text())

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
        self.session.pin_box_for_posing()
        if not self.session.apply_box_pose_from_ui(force=True):
            print("UI: Apply box pose failed")
        self._set_status(self.session.status_text())

    def _on_enable_physics_grasp(self):
        """一键关闭摆位模式，启用重力（不再把盒子 teleport 回 UI 坐标）。"""
        for key in ("live_object", "kinematic_object"):
            cb = self._live_toggles.get(key)
            if cb is not None:
                cb.model.set_value(False)
        self._apply_session_config()
        self.session.enable_physics_grasp()
        self._set_status(self.session.status_text())

    def _on_sync_poses(self):
        synced = self.session.sync_poses_to_ui()
        for k, v in synced.items():
            if k in self._line_edit:
                set_entry_value(self._line_edit[k], v)

    def _read_collect_range(self) -> BoxPoseRange:
        def _pair(key_min: str, key_max: str, default: tuple[float, float]) -> tuple[float, float]:
            lo = read_entry_as_float(self._collect_fields.get(key_min), default[0])
            hi = read_entry_as_float(self._collect_fields.get(key_max), default[1])
            return (min(lo, hi), max(lo, hi))

        return BoxPoseRange(
            tx=_pair("tx_min", "tx_max", COLLECT_TX_RANGE),
            ty=_pair("ty_min", "ty_max", COLLECT_TY_RANGE),
            tz=_pair("tz_min", "tz_max", COLLECT_TZ_RANGE),
            roll=_pair("roll_min", "roll_max", COLLECT_ROLL_RANGE),
            pitch=_pair("pitch_min", "pitch_max", COLLECT_PITCH_RANGE),
            yaw=_pair("yaw_min", "yaw_max", COLLECT_YAW_RANGE),
        )

    def _on_start_collection(self):
        self._apply_session_config()
        output_dir = self._read_string_field(
            self._collect_fields.get("output_dir"),
            f"{self._ext_root}/{DATA_LOG_DIRNAME}",
        )
        num_samples = read_entry_as_int(
            self._collect_fields.get("num_samples"), DEFAULT_COLLECT_SAMPLES
        )
        settle_steps = read_entry_as_int(
            self._collect_fields.get("settle_steps"), DEFAULT_SETTLE_STEPS
        )
        self.session.start_data_collection(
            num_samples=num_samples,
            pose_range=self._read_collect_range(),
            output_dir=output_dir,
            settle_steps=settle_steps,
        )
        self._set_status(self.session.status_text())

    def _set_status(self, text: str):
        if self._status_label:
            self._status_label.text = f"Status: {text}"

    def _set_collect_progress(self, current: int, total: int):
        if self._collect_progress_label is None:
            return
        if total <= 0:
            self._collect_progress_label.text = "Sample: —"
        else:
            self._collect_progress_label.text = f"Sample: {current}/{total}"
