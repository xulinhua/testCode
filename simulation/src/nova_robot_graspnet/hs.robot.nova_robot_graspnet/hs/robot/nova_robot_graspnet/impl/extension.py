# Copyright (c) 2024
"""Isaac Sim 扩展入口：菜单、窗口、Timeline/Stage 事件转发到 UIBuilder。"""

import asyncio
import gc
import logging

import omni.ext
import omni.kit.app
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from ..global_variables import EXTENSION_TITLE


class Extension(omni.ext.IExt):
    """``hs.robot.nova_robot_graspnet`` 扩展生命周期：startup / shutdown / 窗口可见性。"""

    def on_startup(self, ext_id: str):
        """注册菜单、创建 ScrollingWindow；不自动弹窗、不阻塞启用 ROS2（避免卡住主线程）。"""
        logging.debug(f"Extension {EXTENSION_TITLE} starting")
        print(f"Extension {EXTENSION_TITLE} starting")

        self.ext_id = ext_id
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)

        self._window = ScrollingWindow(
            title=EXTENSION_TITLE,
            width=480,
            height=680,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Open {EXTENSION_TITLE}",
        )
        self._menu_items = [
            MenuItemDescription(
                name=EXTENSION_TITLE,
                onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"),
            )
        ]
        # 顶栏独立菜单（与 isaac_box_grasp 一致），比 Window 子菜单更易发现
        add_menu_items(self._menu_items, EXTENSION_TITLE)
        add_menu_items(self._menu_items, "Window")
        print(f"Open panel: menu bar -> {EXTENSION_TITLE}  (also under Window)")

        self._ext_path = ext_path
        self.ui_builder = None
        self._physxIFace = _physx.acquire_physx_interface()
        self._physx_subscription = None
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage_event_sub = None
        self._timeline_event_sub = None
        self._update_sub = None

        async def _show_window():
            await omni.kit.app.get_app().next_update_async()
            self._window.visible = True

        asyncio.ensure_future(_show_window())

    def _ensure_ui_builder(self) -> None:
        """延迟创建 UIBuilder，避免扩展 enable 阶段 import scene_loader / core.api。"""
        if self.ui_builder is None:
            from .ui_builder import UIBuilder

            self.ui_builder = UIBuilder(ext_path=self._ext_path)

    def on_shutdown(self):
        """移除菜单项、关闭窗口、停止会话。"""
        remove_menu_items(self._menu_items, EXTENSION_TITLE)
        remove_menu_items(self._menu_items, "Window")
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")
        if self.ui_builder is not None:
            self.ui_builder.shutdown()
        self._window = None
        self.ui_builder = None
        gc.collect()

    def _on_window(self, visible):
        """窗口显示时订阅 Stage/Timeline/Update 事件并构建 UI；隐藏时 cleanup。"""
        if self._window.visible:
            self._ensure_ui_builder()
            events = omni.usd.get_context().get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)
            app_stream = omni.kit.app.get_app().get_update_event_stream()
            self._update_sub = app_stream.create_subscription_to_pop(self._on_app_update)
            self._build_ui()
        else:
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self._update_sub = None
            if self.ui_builder is not None:
                self.ui_builder.cleanup()

    def _build_ui(self):
        """在 ScrollingWindow 内构建 UIBuilder 面板并停靠到 Viewport 左侧。"""
        self._ensure_ui_builder()
        self._window.frame.clear()
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                try:
                    self.ui_builder.build_ui()
                except Exception as exc:
                    import traceback

                    traceback.print_exc()
                    ui.Label(
                        f"UI build failed: {exc}",
                        word_wrap=True,
                        style={"color": 0xFFFF5555},
                    )

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)

            tgt = ui.Workspace.get_window("Viewport")
            if tgt:
                dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)

        asyncio.ensure_future(dock_window())

    def _menu_callback(self):
        self._window.visible = not self._window.visible

    def _on_timeline_event(self, event):
        """Play 时订阅 physics step；转发 Timeline 事件给 SessionController。"""
        if self.ui_builder is None:
            return
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(
                    self.ui_builder.on_physics_step
                )
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None
        self.ui_builder.on_timeline_event(event)

    def _on_app_update(self, _event):
        if self.ui_builder is not None:
            self.ui_builder.on_app_update()

    def _on_stage_event(self, event):
        if self.ui_builder is None:
            return
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            self._physx_subscription = None
            self.ui_builder.cleanup()
        self.ui_builder.on_stage_event(event)
