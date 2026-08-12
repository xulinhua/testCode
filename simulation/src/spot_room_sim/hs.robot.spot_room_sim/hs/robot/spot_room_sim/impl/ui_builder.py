# -*- coding: utf-8 -*-
"""Spot Room Sim 扩展 UI。"""

from __future__ import annotations

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import btn_builder, get_style

from ..global_variables import (
    DEFAULT_IMAGE_HEIGHT,
    DEFAULT_IMAGE_WIDTH,
    UI_DRIVE_ANGULAR,
    UI_DRIVE_LINEAR,
)
from ..paths import get_extension_paths
from .scene_loader import SceneLoader
from .session_controller import SessionController
from .topic_config import STREAM_TOGGLE_SPECS, TOPIC_FIELD_SPECS, TopicConfig, topic_config_from_ui

_DRIVE_BTN_W = 64
_DRIVE_BTN_H = 36
_COMPACT_FIELD_W = 72


class UIBuilder:
    def __init__(self, ext_path: str):
        self._ext_root, self._data_dir = get_extension_paths(ext_path)
        self._topic_fields = {}
        self._stream_toggles = {}
        self._buttons = {}
        self._status_label = None
        self._cam_w_field = None
        self._cam_h_field = None
        self._drive_lin_field = None
        self._drive_ang_field = None
        self._drive_hint = None
        self._held_dirs = set()  # {"fwd","back","left","right"}
        self.frames = []

        self._session = SessionController(
            scene_loader=SceneLoader(ext_root=self._ext_root),
            on_status=self._set_status,
        )

    def cleanup(self):
        self._held_dirs.clear()
        self._session.stop_ui_drive()
        self._session.stop_all()

    def shutdown(self):
        self.cleanup()

    def on_timeline_event(self, event):
        self._session.on_timeline_event(event)
        self._refresh_status()

    def on_physics_step(self, step):
        self._session.on_physics_step(step)

    def on_app_update(self):
        self._session.on_update()

    def on_stage_event(self, _event):
        pass

    def build_ui(self):
        ui_style = get_style()

        scene_frame = CollapsableFrame("Camera", collapsed=False)
        with scene_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                # 宽高同一行，少占纵向空间
                with ui.HStack(height=24, spacing=4):
                    ui.Label("Resolution", width=70, height=0)
                    self._cam_w_field = ui.IntField(width=_COMPACT_FIELD_W, height=0)
                    self._cam_w_field.model.set_value(int(DEFAULT_IMAGE_WIDTH))
                    ui.Label("×", width=14, height=0)
                    self._cam_h_field = ui.IntField(width=_COMPACT_FIELD_W, height=0)
                    self._cam_h_field.model.set_value(int(DEFAULT_IMAGE_HEIGHT))
                    ui.Spacer()

        drive_frame = CollapsableFrame("Drive", collapsed=False)
        with drive_frame:
            self._build_drive_pad(ui_style)

        topic_frame = CollapsableFrame("ROS Topics", collapsed=True)
        with topic_frame:
            self._build_topic_fields(ui_style)

        stream_frame = CollapsableFrame("Stream Options", collapsed=True)
        with stream_frame:
            self._build_stream_toggles(ui_style)

        actions_frame = CollapsableFrame("Actions", collapsed=False)
        with actions_frame:
            with ui.VStack(style=ui_style, spacing=5, height=0):
                self._status_label = ui.Label("Status: IDLE", height=0)
                self._buttons["load"] = btn_builder(
                    label="load",
                    type="button",
                    text="Load",
                    tooltip="Load Simple Room + Spot + front camera",
                    on_clicked_fn=self._on_load,
                )
                self._buttons["unload"] = btn_builder(
                    label="unload",
                    type="button",
                    text="Unload",
                    tooltip="Stop ROS and remove scene",
                    on_clicked_fn=self._on_unload,
                )

        self.frames = [scene_frame, drive_frame, topic_frame, stream_frame, actions_frame]
        self._apply_session_config()

    def _build_drive_pad(self, ui_style) -> None:
        with ui.VStack(style=ui_style, spacing=6, height=0):
            # 线速度 / 角速度同一行
            with ui.HStack(height=24, spacing=4):
                ui.Label("Lin m/s", width=55, height=0)
                self._drive_lin_field = ui.FloatField(width=_COMPACT_FIELD_W, height=0)
                self._drive_lin_field.model.set_value(float(UI_DRIVE_LINEAR))
                ui.Spacer(width=8)
                ui.Label("Ang rad/s", width=70, height=0)
                self._drive_ang_field = ui.FloatField(width=_COMPACT_FIELD_W, height=0)
                self._drive_ang_field.model.set_value(float(UI_DRIVE_ANGULAR))
                ui.Spacer()

            ui.Spacer(height=4)
            #     [Fwd]
            # [L][Stop][R]
            #     [Back]
            with ui.HStack(height=_DRIVE_BTN_H):
                ui.Spacer()
                self._make_drive_btn("Fwd", "fwd")
                ui.Spacer()
            with ui.HStack(spacing=6, height=_DRIVE_BTN_H):
                ui.Spacer()
                self._make_drive_btn("Left", "left")
                self._make_stop_btn()
                self._make_drive_btn("Right", "right")
                ui.Spacer()
            with ui.HStack(height=_DRIVE_BTN_H):
                ui.Spacer()
                self._make_drive_btn("Back", "back")
                ui.Spacer()

            self._drive_hint = ui.Label(
                "Hold direction to drive, release to stop. Load then Play first.",
                word_wrap=True,
                height=0,
            )

    def _make_drive_btn(self, text: str, direction: str):
        """点动：按住运动、松开停止。用 Rectangle 接鼠标，比 Button 的 pressed 可靠。"""
        with ui.ZStack(width=_DRIVE_BTN_W, height=_DRIVE_BTN_H):
            hit = ui.Rectangle(
                style={
                    "background_color": 0xFF3A3A3A,
                    "border_color": 0xFF666666,
                    "border_width": 1.0,
                    "border_radius": 3.0,
                }
            )
            ui.Label(text, alignment=ui.Alignment.CENTER, height=0)
            # button: 0 = left
            hit.set_mouse_pressed_fn(
                lambda x, y, b, m, d=direction: self._on_drive_press(d) if b == 0 else None
            )
            hit.set_mouse_released_fn(
                lambda x, y, b, m, d=direction: self._on_drive_release(d) if b == 0 else None
            )
            try:
                hit.set_mouse_hovered_fn(
                    lambda hovered, d=direction: (
                        self._on_drive_release(d) if (not hovered and d in self._held_dirs) else None
                    )
                )
            except Exception:
                pass
        return hit

    def _on_drive_press(self, direction: str) -> None:
        opposites = {"fwd": "back", "back": "fwd", "left": "right", "right": "left"}
        opp = opposites.get(direction)
        if opp:
            self._held_dirs.discard(opp)
        self._held_dirs.add(direction)
        self._apply_held_dirs()

    def _on_drive_release(self, direction: str) -> None:
        self._held_dirs.discard(direction)
        self._apply_held_dirs()

    def _make_stop_btn(self) -> ui.Button:
        btn = ui.Button("Stop", width=_DRIVE_BTN_W, height=_DRIVE_BTN_H, clicked_fn=self._on_drive_stop)
        return btn

    def _drive_speeds(self) -> tuple[float, float]:
        lin = float(UI_DRIVE_LINEAR)
        ang = float(UI_DRIVE_ANGULAR)
        try:
            if self._drive_lin_field is not None:
                lin = float(self._drive_lin_field.model.get_value_as_float())
        except Exception:
            pass
        try:
            if self._drive_ang_field is not None:
                ang = float(self._drive_ang_field.model.get_value_as_float())
        except Exception:
            pass
        return max(0.0, lin), max(0.0, ang)

    def _apply_held_dirs(self) -> None:
        lin, ang = self._drive_speeds()
        vx = vy = wz = 0.0
        if "fwd" in self._held_dirs:
            vx += lin
        if "back" in self._held_dirs:
            vx -= lin
        if "left" in self._held_dirs:
            wz += ang
        if "right" in self._held_dirs:
            wz -= ang
        if self._held_dirs:
            self._session.set_ui_cmd_vel(vx, vy, wz, active=True)
            if self._drive_hint is not None:
                dirs = ",".join(sorted(self._held_dirs))
                self._drive_hint.text = f"Holding [{dirs}] vx={vx:.2f} wz={wz:.2f}"
        else:
            self._session.stop_ui_drive()
            if self._drive_hint is not None:
                self._drive_hint.text = "Hold direction to drive, release to stop. Load then Play first."

    def _on_drive_stop(self) -> None:
        self._held_dirs.clear()
        self._session.stop_ui_drive()
        if self._drive_hint is not None:
            self._drive_hint.text = "Stopped."

    def _build_topic_fields(self, ui_style):
        self._topic_fields = {}
        with ui.VStack(style=ui_style, spacing=4, height=0):
            for key, label in TOPIC_FIELD_SPECS:
                with ui.HStack(height=24):
                    ui.Label(label, width=160, height=0)
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

    def _read_cam_size(self) -> tuple[int, int]:
        width, height = DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT
        try:
            if self._cam_w_field is not None:
                width = int(self._cam_w_field.model.get_value_as_int())
        except Exception:
            pass
        try:
            if self._cam_h_field is not None:
                height = int(self._cam_h_field.model.get_value_as_int())
        except Exception:
            pass
        return max(1, width), max(1, height)

    def _apply_session_config(self) -> None:
        width, height = self._read_cam_size()
        cfg = topic_config_from_ui(self._topic_fields, self._stream_toggles, width, height)
        self._session.configure(cfg)

    def _on_load(self):
        self._apply_session_config()
        self._set_status("LOADING (async World init)...")

        async def _load():
            ok = await self._session.load_scene_async()
            self._refresh_status()
            if not ok:
                self._set_status("LOAD FAILED — see console")

        import asyncio

        asyncio.ensure_future(_load())

    def _on_unload(self):
        self._held_dirs.clear()
        self._session.stop_ui_drive()
        self._session.stop_all()
        self._refresh_status()

    def _set_status(self, text: str) -> None:
        if self._status_label is not None:
            self._status_label.text = f"Status: {text}"

    def _refresh_status(self) -> None:
        self._set_status(self._session.status_text())
