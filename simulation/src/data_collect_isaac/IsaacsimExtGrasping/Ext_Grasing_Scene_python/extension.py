# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#




import asyncio
import gc

import os
import logging

import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from .global_variables import EXTENSION_DESCRIPTION, EXTENSION_TITLE
from .ui_builder import UIBuilder

"""
扩展入口文件。

Isaac Sim 启用扩展时会创建 Extension 实例，并调用 on_startup()。
本文件只负责 Omniverse 扩展生命周期、窗口、菜单、Stage/Timeline/Physics 事件转发；
真正的抓取场景搭建、UI 控件、数据采集逻辑放在 ui_builder.py 的 UIBuilder 中。

事件转发关系：
    菜单点击 -> _menu_callback() -> UIBuilder.on_menu_callback()
    时间轴事件 -> _on_timeline_event() -> UIBuilder.on_timeline_event()
    物理步进 -> _on_physics_step() -> UIBuilder.on_physics_step()
    Stage 打开/关闭 -> _on_stage_event() -> UIBuilder.on_stage_event()
    UI 重建 -> _build_extension_ui() -> UIBuilder.build_ui()
"""




class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """扩展启动入口：注册菜单、创建窗口、初始化事件接口和 UIBuilder。"""

        # try:
        #     import pydevd_pycharm
        #     print("🔴 正在尝试连接 PyCharm 调试器...")
        #     pydevd_pycharm.settrace('localhost', port=7777, suspend=True)
        #     print("🟢 成功连接到 PyCharm 调试器！")
        # except Exception as e:
        #     print(f"❌ 连接失败: {type(e).__name__}: {e}")

        # 清屏只影响终端输出，便于开发时观察当前扩展启动日志。
        os.system('clear')
        logging.debug(f"Extension {EXTENSION_TITLE} is starting up")
        print(f"Extension {EXTENSION_TITLE} is starting up")
        
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # 创建插件主窗口。visible=False 表示启动时不自动弹出，用户从菜单点击后再显示。
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # 注册菜单动作：菜单项本身只切换窗口显示，窗口显示后再构建内部 UI。
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]

        add_menu_items(self._menu_items, EXTENSION_TITLE)

        # UIBuilder 持有业务状态：加载场景、生成物体、采集数据、注册 Writer 等。
        self.ui_builder = UIBuilder()

        # 缓存常用事件接口。真正订阅发生在窗口显示时，隐藏时释放订阅。
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.acquire_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self):
        """扩展关闭/热重载前的清理入口。"""
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible):
        """窗口可见性变化回调：显示时订阅事件并构建 UI，隐藏时释放资源。"""
        
        # logging.debug(f"Extension Window {EXTENSION_TITLE} is {'' if visible else 'not '}visible")
        # print(f"Extension Window {EXTENSION_TITLE} is {'' if visible else 'not '}visible")
        
        if self._window.visible:
            # 只有窗口打开时才订阅 Stage/Timeline 事件，避免后台无意义回调。
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        """创建窗口内容，并把窗口停靠到 Viewport 左侧。"""
        
        # logging.debug(f"Building UI for {EXTENSION_TITLE}")
        # print(f"Building UI for {EXTENSION_TITLE}")
        
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window():
            # Dock 操作需要等下一帧 UI 刷新后窗口对象可用，因此使用异步任务。
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self):
        """菜单点击回调：切换窗口显示状态，并通知 UIBuilder。"""
        
        # logging.debug(f"Menu Callback for {EXTENSION_TITLE}")
        # print(f"Menu Callback for {EXTENSION_TITLE}")
        
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_event(self, event):
        """Timeline 播放/停止事件。播放时注册物理步回调，停止时取消。"""
        
        # logging.debug(f"Timeline Event Callback for {EXTENSION_TITLE}")
        # print(f"Timeline Event Callback for {EXTENSION_TITLE}")
        
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                # 物理步回调只在仿真播放时有效；采集前让物体落稳也依赖时间轴推进。
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None

        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step):
        """每个物理仿真步触发一次，当前项目仅转发给 UIBuilder 预留扩展点。"""
        
        # logging.debug(f"Physics Step Callback for {EXTENSION_TITLE}")
        # print(f"Physics Step Callback for {EXTENSION_TITLE}")
        
        self.ui_builder.on_physics_step(step)

    def _on_stage_event(self, event):
        """Stage 打开/关闭时清理旧资源，避免引用已经失效的 Prim 或 World。"""
        
        # logging.debug(f"Stage Event Callback for {EXTENSION_TITLE}")
        # print(f"Stage Event Callback for {EXTENSION_TITLE}")
        
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # Stage 被替换后，旧的物理订阅和 UIBuilder 内部对象都可能失效。
            self._physx_subscription = None
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        """把 UIBuilder 生成的控件挂到扩展窗口中。"""
        
        # logging.debug(f"Building Extension UI for {EXTENSION_TITLE}")
        # print(f"Building Extension UI for {EXTENSION_TITLE}")
        
        # 具体按钮、输入框和采集参数在 UIBuilder.build_ui() 中定义。
        self.ui_builder.build_ui()
