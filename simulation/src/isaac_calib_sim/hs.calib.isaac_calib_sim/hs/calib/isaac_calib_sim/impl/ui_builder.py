# -*- coding: utf-8 -*-
"""Extension UI: board type, motion params, ROS streaming (English only)."""

from __future__ import annotations

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import btn_builder, get_style

from ..global_variables import (
    BOARD_TYPE_LABELS,
    DEFAULT_AZIM_RATE_DEG,
    DEFAULT_ARUCO_DICTIONARY,
    DEFAULT_BOARD_TYPE,
    DEFAULT_DIST_MAX,
    DEFAULT_DIST_MIN,
    DEFAULT_DIST_PERIOD_S,
    DEFAULT_ELEV_MAX_DEG,
    DEFAULT_ELEV_MIN_DEG,
    DEFAULT_ELEV_PERIOD_S,
    DEFAULT_HFOV_DEG,
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_IMAGE_WIDTH,
    DEFAULT_LOOKAT_OFFSET_M,
    DEFAULT_LOOKAT_PERIOD_S,
    DEFAULT_MARKER_ID,
    DEFAULT_MARKER_LENGTH_M,
    DEFAULT_ROLL_AMP_DEG,
    DEFAULT_ROLL_PERIOD_S,
    DEFAULT_SQUARE_LENGTH_M,
    DEFAULT_SQUARES_X,
    DEFAULT_SQUARES_Y,
    DEFAULT_STREAM_FPS,
)
from ..paths import get_extension_paths
from .aruco_markers import ARUCO_DICTIONARY_NAMES, clamp_marker_id, normalize_dictionary_name
from .board_factory import BoardSpec
from .camera_motion import OrbitParams
from .scene_loader import SceneLoader
from .session_controller import SessionController, default_orbit_params, orbit_params_for_board
from .topic_config import TOPIC_FIELD_SPECS, TopicConfig, topic_config_from_ui
from .ui_numeric import (
    add_float_param,
    read_entry_as_float,
    read_entry_as_int,
    write_entry_float,
)


class UIBuilder:
    def __init__(self, ext_path: str):
        _, _, texture_dir = get_extension_paths(ext_path)
        self._line_edit = {}
        self._topic_fields = {}
        self._buttons = {}
        self._status_label = None
        self._board_combo = None
        self._dict_combo = None
        self._board_type_ids = [k for k, _ in BOARD_TYPE_LABELS]
        self._dict_names = list(ARUCO_DICTIONARY_NAMES)
        self._param_rows = {}  # name -> ui.Widget with .visible
        self._hint_label = None
        self._motion_enabled_model = None
        self._enable_color_model = None
        self._enable_info_model = None
        self.frames = []

        self._session = SessionController(
            scene_loader=SceneLoader(texture_dir),
            on_status=self._set_status,
        )

    def cleanup(self):
        self._session.stop_streaming_only()

    def shutdown(self):
        self._session.stop_all()

    def on_timeline_event(self, event):
        self._session.on_timeline_event(event)

    def on_physics_step(self, step):
        self._session.on_physics_step(step)

    def on_app_update(self):
        self._session.on_app_update()

    def on_stage_event(self, _event):
        pass

    def build_ui(self):
        ui_style = get_style()

        board_frame = CollapsableFrame("Calibration Board", collapsed=False)
        with board_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._hint_label = ui.Label(
                    "Pick a board type; relevant params appear below. Then Apply.",
                    height=0,
                    word_wrap=True,
                )
                with ui.HStack(height=24):
                    ui.Label("Board type", width=ui.Fraction(0.45), height=0)
                    self._board_combo = ui.ComboBox(
                        0, *[label for _, label in BOARD_TYPE_LABELS]
                    )
                    try:
                        self._board_combo.model.add_item_changed_fn(
                            lambda *_a, **_k: self._on_board_type_changed()
                        )
                    except Exception:
                        pass

                # Dictionary (ArUco / ChArUco)
                dict_row = ui.HStack(height=24)
                self._param_rows["dictionary"] = dict_row
                with dict_row:
                    ui.Label("ArUco dictionary", width=ui.Fraction(0.45), height=0)
                    dict_idx = (
                        self._dict_names.index(DEFAULT_ARUCO_DICTIONARY)
                        if DEFAULT_ARUCO_DICTIONARY in self._dict_names
                        else 0
                    )
                    self._dict_combo = ui.ComboBox(dict_idx, *self._dict_names)

                # Marker ID (single) / first ID (grid / ChArUco)
                mid_row = ui.HStack(height=24)
                self._param_rows["marker_id"] = mid_row
                with mid_row:
                    ui.Label("Marker ID / first ID", width=ui.Fraction(0.45), height=0)
                    mid_model = ui.SimpleIntModel(int(DEFAULT_MARKER_ID))
                    with ui.Frame(width=ui.Fraction(0.55), height=0):
                        ui.IntDrag(model=mid_model, min=0, max=1023, step=1, height=0)
                    self._line_edit["marker_id"] = (mid_model, mid_row, 1.0, 0.0, 1023.0)

                sx_row = ui.HStack(height=24)
                self._param_rows["squares_x"] = sx_row
                with sx_row:
                    ui.Label("Squares / cols X", width=ui.Fraction(0.45), height=0)
                    sx_model = ui.SimpleIntModel(int(DEFAULT_SQUARES_X))
                    with ui.Frame(width=ui.Fraction(0.55), height=0):
                        ui.IntDrag(model=sx_model, min=1, max=40, step=1, height=0)
                    self._line_edit["squares_x"] = (sx_model, sx_row, 1.0, 1.0, 40.0)

                sy_row = ui.HStack(height=24)
                self._param_rows["squares_y"] = sy_row
                with sy_row:
                    ui.Label("Squares / rows Y", width=ui.Fraction(0.45), height=0)
                    sy_model = ui.SimpleIntModel(int(DEFAULT_SQUARES_Y))
                    with ui.Frame(width=ui.Fraction(0.55), height=0):
                        ui.IntDrag(model=sy_model, min=1, max=40, step=1, height=0)
                    self._line_edit["squares_y"] = (sy_model, sy_row, 1.0, 1.0, 40.0)

                sq_row = ui.HStack(height=24)
                self._param_rows["square_length"] = sq_row
                with sq_row:
                    ui.Label("Square / spacing (m)", width=ui.Fraction(0.45), height=0)
                    sq_model = ui.SimpleFloatModel(float(DEFAULT_SQUARE_LENGTH_M))
                    with ui.Frame(width=ui.Fraction(0.55), height=0):
                        ui.FloatDrag(
                            model=sq_model,
                            min=0.005,
                            max=0.5,
                            step=0.001,
                            format="%.4f",
                            height=0,
                        )
                    self._line_edit["square_length"] = (sq_model, sq_row, 0.001, 0.005, 0.5)

                mk_row = ui.HStack(height=24)
                self._param_rows["marker_length"] = mk_row
                with mk_row:
                    ui.Label("Marker length (m)", width=ui.Fraction(0.45), height=0)
                    mk_model = ui.SimpleFloatModel(float(DEFAULT_MARKER_LENGTH_M))
                    with ui.Frame(width=ui.Fraction(0.55), height=0):
                        ui.FloatDrag(
                            model=mk_model,
                            min=0.005,
                            max=0.5,
                            step=0.001,
                            format="%.4f",
                            height=0,
                        )
                    self._line_edit["marker_length"] = (mk_model, mk_row, 0.001, 0.005, 0.5)

                self._buttons["apply_board"] = btn_builder(
                    label="apply_board",
                    type="button",
                    text="Apply board params",
                    tooltip="Rebuild board with current params (no unload)",
                    on_clicked_fn=self._on_apply_board,
                )
                self._on_board_type_changed()

        cam_frame = CollapsableFrame("Camera", collapsed=False)
        with cam_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._line_edit["image_width"] = add_int_param(
                    "Image width", DEFAULT_IMAGE_WIDTH, min_val=64, max_val=4096
                )
                self._line_edit["image_height"] = add_int_param(
                    "Image height", DEFAULT_IMAGE_HEIGHT, min_val=64, max_val=4096
                )
                self._line_edit["hfov"] = add_float_param(
                    "Horizontal FOV (deg)",
                    DEFAULT_HFOV_DEG,
                    step=1.0,
                    fmt="%.1f",
                    min_val=20.0,
                    max_val=120.0,
                )

        motion_frame = CollapsableFrame("Camera Motion (Play)", collapsed=False)
        with motion_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                ui.Label(
                    "After Play: orbit + look-at wander + roll for diverse calib views "
                    "(board moves in image so auto-capture keeps collecting).",
                    height=0,
                    word_wrap=True,
                )
                with ui.HStack(height=22):
                    ui.Label("Enable orbit motion", width=ui.Fraction(0.7), height=0)
                    self._motion_enabled_model = ui.SimpleBoolModel(True)
                    ui.CheckBox(model=self._motion_enabled_model)
                orbit = default_orbit_params()
                self._line_edit["dist_min"] = add_float_param(
                    "Distance min (m)", orbit.dist_min, step=0.02, fmt="%.2f", min_val=0.2, max_val=3.0
                )
                self._line_edit["dist_max"] = add_float_param(
                    "Distance max (m)", orbit.dist_max, step=0.02, fmt="%.2f", min_val=0.3, max_val=4.0
                )
                self._line_edit["elev_min"] = add_float_param(
                    "Elevation min (deg)", orbit.elev_min_deg, step=1.0, fmt="%.1f", min_val=5.0, max_val=85.0
                )
                self._line_edit["elev_max"] = add_float_param(
                    "Elevation max (deg)", orbit.elev_max_deg, step=1.0, fmt="%.1f", min_val=5.0, max_val=89.0
                )
                self._line_edit["azim_rate"] = add_float_param(
                    "Azimuth rate (deg/s)", orbit.azim_rate_deg, step=1.0, fmt="%.1f", min_val=0.0, max_val=90.0
                )
                self._line_edit["dist_period"] = add_float_param(
                    "Distance period (s)", orbit.dist_period_s, step=0.5, fmt="%.1f", min_val=1.0, max_val=60.0
                )
                self._line_edit["elev_period"] = add_float_param(
                    "Elevation period (s)", orbit.elev_period_s, step=0.5, fmt="%.1f", min_val=1.0, max_val=60.0
                )
                self._line_edit["lookat_offset"] = add_float_param(
                    "Look-at offset (m)", orbit.lookat_offset_m, step=0.01, fmt="%.2f", min_val=0.0, max_val=0.4
                )
                self._line_edit["lookat_period"] = add_float_param(
                    "Look-at period (s)", orbit.lookat_period_s, step=0.5, fmt="%.1f", min_val=1.0, max_val=60.0
                )
                self._line_edit["roll_amp"] = add_float_param(
                    "Roll amplitude (deg)", orbit.roll_amp_deg, step=1.0, fmt="%.1f", min_val=0.0, max_val=35.0
                )
                self._line_edit["roll_period"] = add_float_param(
                    "Roll period (s)", orbit.roll_period_s, step=0.5, fmt="%.1f", min_val=1.0, max_val=60.0
                )

        topic_frame = CollapsableFrame("ROS Topics", collapsed=False)
        with topic_frame:
            with ui.VStack(style=ui_style, spacing=4, height=0):
                defaults = TopicConfig()
                for key, label in TOPIC_FIELD_SPECS:
                    with ui.HStack(height=22):
                        ui.Label(label, width=ui.Fraction(0.45), height=0)
                        model = ui.SimpleStringModel(getattr(defaults, key))
                        ui.StringField(model=model, height=0)
                        self._topic_fields[key] = model
                with ui.HStack(height=22):
                    ui.Label("Publish color", width=ui.Fraction(0.7), height=0)
                    self._enable_color_model = ui.SimpleBoolModel(True)
                    ui.CheckBox(model=self._enable_color_model)
                with ui.HStack(height=22):
                    ui.Label("Publish CameraInfo", width=ui.Fraction(0.7), height=0)
                    self._enable_info_model = ui.SimpleBoolModel(True)
                    ui.CheckBox(model=self._enable_info_model)

        actions = CollapsableFrame("Actions", collapsed=False)
        with actions:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._status_label = ui.Label("Status: IDLE", height=0, word_wrap=True)
                self._buttons["load"] = btn_builder(
                    label="load",
                    type="button",
                    text="Load scene",
                    tooltip="Create table + board + single camera",
                    on_clicked_fn=self._on_load,
                )
                self._buttons["start"] = btn_builder(
                    label="start",
                    type="button",
                    text="Start ROS stream",
                    tooltip="Start RGB stream and Play",
                    on_clicked_fn=self._on_start,
                )
                self._buttons["stop"] = btn_builder(
                    label="stop",
                    type="button",
                    text="Stop ROS stream",
                    tooltip="Stop ROS stream",
                    on_clicked_fn=self._on_stop,
                )
                self._buttons["unload"] = btn_builder(
                    label="unload",
                    type="button",
                    text="Unload",
                    tooltip="Unload scene",
                    on_clicked_fn=self._on_unload,
                )

        self.frames = [board_frame, cam_frame, motion_frame, topic_frame, actions]

    def _set_status(self, text: str):
        if self._status_label:
            self._status_label.text = f"Status: {text}"
        print(f"[CalibSim] {text}")

    def _selected_board_type(self) -> str:
        if self._board_combo is None:
            return DEFAULT_BOARD_TYPE
        try:
            idx = self._board_combo.model.get_item_value_model().as_int
        except Exception:
            idx = 0
        if idx < 0 or idx >= len(self._board_type_ids):
            idx = 0
        return self._board_type_ids[idx]

    def _set_row_visible(self, key: str, visible: bool) -> None:
        row = self._param_rows.get(key)
        if row is None:
            return
        try:
            row.visible = bool(visible)
        except Exception:
            pass

    def _selected_dictionary(self) -> str:
        if self._dict_combo is None:
            return DEFAULT_ARUCO_DICTIONARY
        try:
            idx = self._dict_combo.model.get_item_value_model().as_int
        except Exception:
            idx = 0
        if idx < 0 or idx >= len(self._dict_names):
            idx = 0
        return normalize_dictionary_name(self._dict_names[idx])

    def _refresh_board_param_visibility(self, board_type: str) -> None:
        """Show only params that matter for the selected board."""
        needs_dict = board_type in (
            "charuco",
            "aruco",
            "aruco_grid",
            "trihedral_charuco",
            "trihedral_aruco",
        )
        needs_marker_id = board_type in (
            "aruco",
            "aruco_grid",
            "charuco",
            "trihedral_charuco",
            "trihedral_aruco",
        )
        needs_grid = board_type != "aruco"
        needs_square = board_type != "aruco"
        needs_marker_len = board_type in (
            "aruco",
            "charuco",
            "trihedral_charuco",
        )
        self._set_row_visible("dictionary", needs_dict)
        self._set_row_visible("marker_id", needs_marker_id)
        self._set_row_visible("squares_x", needs_grid)
        self._set_row_visible("squares_y", needs_grid)
        self._set_row_visible("square_length", needs_square)
        self._set_row_visible("marker_length", needs_marker_len)

        hints = {
            "chessboard": "Chessboard: Squares X/Y = inner corners; Square = cell size.",
            "circles_symmetric": "Circles: cols×rows + spacing (center distance).",
            "circles_asymmetric": "Asymmetric circles: cols×rows + spacing.",
            "charuco": "ChArUco: cells X/Y, square, marker length, dictionary, first marker ID.",
            "aruco": "Single ArUco: dictionary + Marker ID + Marker length (m).",
            "aruco_grid": "ArUco grid: cols×rows, spacing≈pitch, dictionary, first marker ID.",
            "trihedral_chess": "Trihedral chess: inner corners n×n (uses max X/Y), square size.",
            "trihedral_charuco": "Trihedral ChArUco: n×n, square, marker length, dictionary.",
            "trihedral_aruco": "Trihedral ArUco: n×n markers, spacing, dictionary, first ID.",
        }
        if self._hint_label is not None:
            try:
                self._hint_label.text = hints.get(
                    board_type, "Pick a board type; then Apply."
                )
            except Exception:
                pass

    def _on_board_type_changed(self) -> None:
        """Auto-narrow camera motion when switching to trihedral boards."""
        board_type = self._selected_board_type()
        self._apply_orbit_preset_to_ui(board_type)
        self._refresh_board_param_visibility(board_type)
        # Sensible defaults when switching to single ArUco
        if board_type == "aruco":
            entry = self._line_edit.get("marker_length")
            if entry is not None:
                model = entry[0] if isinstance(entry, tuple) else entry
                try:
                    if float(model.get_value_as_float()) < 0.03:
                        model.set_value(0.05)
                except Exception:
                    pass
            mid = self._line_edit.get("marker_id")
            if mid is not None:
                model = mid[0] if isinstance(mid, tuple) else mid
                try:
                    model.set_value(int(DEFAULT_MARKER_ID))
                except Exception:
                    pass

    def _apply_orbit_preset_to_ui(self, board_type: str) -> None:
        enabled = True
        if self._motion_enabled_model is not None:
            enabled = bool(self._motion_enabled_model.get_value_as_bool())
        orbit = orbit_params_for_board(board_type, enabled=enabled)
        write_entry_float(self._line_edit.get("dist_min"), orbit.dist_min)
        write_entry_float(self._line_edit.get("dist_max"), orbit.dist_max)
        write_entry_float(self._line_edit.get("elev_min"), orbit.elev_min_deg)
        write_entry_float(self._line_edit.get("elev_max"), orbit.elev_max_deg)
        write_entry_float(self._line_edit.get("azim_rate"), orbit.azim_rate_deg)
        write_entry_float(self._line_edit.get("dist_period"), orbit.dist_period_s)
        write_entry_float(self._line_edit.get("elev_period"), orbit.elev_period_s)
        write_entry_float(self._line_edit.get("lookat_offset"), orbit.lookat_offset_m)
        write_entry_float(self._line_edit.get("lookat_period"), orbit.lookat_period_s)
        write_entry_float(self._line_edit.get("roll_amp"), orbit.roll_amp_deg)
        write_entry_float(self._line_edit.get("roll_period"), orbit.roll_period_s)

    def _read_board_spec(self) -> BoardSpec:
        board_type = self._selected_board_type()
        dictionary = self._selected_dictionary()
        marker_id = read_entry_as_int(self._line_edit.get("marker_id"), DEFAULT_MARKER_ID)
        marker_id = clamp_marker_id(dictionary, marker_id)
        sx = read_entry_as_int(self._line_edit.get("squares_x"), DEFAULT_SQUARES_X)
        sy = read_entry_as_int(self._line_edit.get("squares_y"), DEFAULT_SQUARES_Y)
        # Trihedral faces are square plates: force n×n inner corners (match suite).
        if board_type.startswith("trihedral_"):
            n = max(int(sx), int(sy), 3)
            # Old planar default 9×6 → use 8×8 to match suite trihedral_oneshot.yaml
            if int(sx) != int(sy) and n == max(DEFAULT_SQUARES_X, DEFAULT_SQUARES_Y):
                n = 8
            sx = sy = n
            for key, val in (("squares_x", sx), ("squares_y", sy)):
                entry = self._line_edit.get(key)
                if entry is not None:
                    model = entry[0] if isinstance(entry, tuple) else entry
                    try:
                        model.set_value(int(val))
                    except Exception:
                        pass
        elif board_type == "aruco":
            sx = max(int(sx), 1)
            sy = max(int(sy), 1)
        else:
            sx = max(int(sx), 2)
            sy = max(int(sy), 2)
        return BoardSpec(
            board_type=board_type,
            squares_x=sx,
            squares_y=sy,
            square_length_m=read_entry_as_float(
                self._line_edit.get("square_length"), DEFAULT_SQUARE_LENGTH_M
            ),
            marker_length_m=read_entry_as_float(
                self._line_edit.get("marker_length"), DEFAULT_MARKER_LENGTH_M
            ),
            dictionary=dictionary,
            marker_id=marker_id,
        )

    def _read_orbit(self) -> OrbitParams:
        enabled = True
        if self._motion_enabled_model is not None:
            enabled = bool(self._motion_enabled_model.get_value_as_bool())
        return OrbitParams(
            dist_min=read_entry_as_float(self._line_edit.get("dist_min"), DEFAULT_DIST_MIN),
            dist_max=read_entry_as_float(self._line_edit.get("dist_max"), DEFAULT_DIST_MAX),
            elev_min_deg=read_entry_as_float(self._line_edit.get("elev_min"), DEFAULT_ELEV_MIN_DEG),
            elev_max_deg=read_entry_as_float(self._line_edit.get("elev_max"), DEFAULT_ELEV_MAX_DEG),
            azim_rate_deg=read_entry_as_float(self._line_edit.get("azim_rate"), DEFAULT_AZIM_RATE_DEG),
            dist_period_s=read_entry_as_float(self._line_edit.get("dist_period"), DEFAULT_DIST_PERIOD_S),
            elev_period_s=read_entry_as_float(self._line_edit.get("elev_period"), DEFAULT_ELEV_PERIOD_S),
            lookat_offset_m=read_entry_as_float(
                self._line_edit.get("lookat_offset"), DEFAULT_LOOKAT_OFFSET_M
            ),
            lookat_period_s=read_entry_as_float(
                self._line_edit.get("lookat_period"), DEFAULT_LOOKAT_PERIOD_S
            ),
            roll_amp_deg=read_entry_as_float(self._line_edit.get("roll_amp"), DEFAULT_ROLL_AMP_DEG),
            roll_period_s=read_entry_as_float(self._line_edit.get("roll_period"), DEFAULT_ROLL_PERIOD_S),
            enabled=enabled,
        )

    def _apply_session_config(self) -> None:
        fields = {k: m.get_value_as_string() for k, m in self._topic_fields.items()}
        enable_color = (
            True
            if self._enable_color_model is None
            else bool(self._enable_color_model.get_value_as_bool())
        )
        enable_info = (
            True
            if self._enable_info_model is None
            else bool(self._enable_info_model.get_value_as_bool())
        )
        cfg = topic_config_from_ui(fields, enable_color, enable_info)
        w = read_entry_as_int(self._line_edit.get("image_width"), DEFAULT_IMAGE_WIDTH)
        h = read_entry_as_int(self._line_edit.get("image_height"), DEFAULT_IMAGE_HEIGHT)
        hfov = read_entry_as_float(self._line_edit.get("hfov"), DEFAULT_HFOV_DEG)
        self._session.configure(
            cfg, (w, h), hfov, self._read_orbit(), stream_fps=DEFAULT_STREAM_FPS
        )

    def _on_load(self):
        self._apply_orbit_preset_to_ui(self._selected_board_type())
        self._apply_session_config()
        self._session.load_scene(self._read_board_spec())

    def _on_apply_board(self):
        self._apply_orbit_preset_to_ui(self._selected_board_type())
        self._apply_session_config()
        self._session.switch_board(self._read_board_spec())

    def _on_start(self):
        self._apply_session_config()
        self._session.start_stream()

    def _on_stop(self):
        self._session.stop_streaming_only()

    def _on_unload(self):
        self._session.stop_all()
