# -*- coding: utf-8 -*-
"""按需启用 Isaac Kit 扩展。"""

from __future__ import annotations


def _enable(*extension_ids: str) -> bool:
    try:
        import omni.kit.app

        em = omni.kit.app.get_app().get_extension_manager()
        for ext_id in extension_ids:
            if not em.is_extension_enabled(ext_id):
                em.set_extension_enabled_immediate(ext_id, True)
                print(f"[SpotRoomSim] enabled extension: {ext_id}")
        return all(em.is_extension_enabled(ext_id) for ext_id in extension_ids)
    except Exception as exc:
        print(f"[SpotRoomSim] failed to enable {extension_ids}: {exc}")
        return False


def ensure_core_api_enabled() -> bool:
    return _enable("isaacsim.core.api")


def ensure_ros2_bridge_enabled() -> bool:
    return _enable("isaacsim.core.nodes", "isaacsim.ros2.bridge")
