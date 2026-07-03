# -*- coding: utf-8 -*-
"""将机器人 USD reference 到 Stage。"""

from __future__ import annotations

import os
from typing import Optional

from isaacsim.core.utils.stage import add_reference_to_stage

from ...defaults import ROBOT_PRIM_PATH
from ...paths import resolve_asset_file
from ..interfaces import IRobotLoader


class UsdRobotLoader(IRobotLoader):
    def __init__(self, robots_dir: str, prim_path: str = ROBOT_PRIM_PATH):
        self._robots_dir = robots_dir
        self.prim_path = prim_path
        self._loaded_path: Optional[str] = None

    @property
    def is_loaded(self) -> bool:
        return self._loaded_path is not None

    @property
    def loaded_usd_path(self) -> Optional[str]:
        return self._loaded_path

    def load_robot(self, asset_name: str, prim_path: str = "") -> bool:
        target_prim = prim_path or self.prim_path
        usd_path = resolve_asset_file(asset_name, self._robots_dir)
        if not usd_path:
            print(
                f"UsdRobotLoader: robot USD not found: {asset_name!r} "
                f"(expected under {self._robots_dir})"
            )
            return False
        try:
            add_reference_to_stage(usd_path=usd_path, prim_path=target_prim)
            self.prim_path = target_prim
            self._loaded_path = usd_path
            print(f"UsdRobotLoader: referenced {usd_path} -> {target_prim}")
            return True
        except Exception as exc:
            import traceback

            traceback.print_exc()
            print(f"UsdRobotLoader.load_robot failed: {exc}")
            self._loaded_path = None
            return False

    def unload_robot(self) -> None:
        try:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            if stage and stage.GetPrimAtPath(self.prim_path):
                stage.RemovePrim(self.prim_path)
        except Exception as exc:
            print(f"UsdRobotLoader.unload_robot: {exc}")
        self._loaded_path = None
