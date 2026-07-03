# -*- coding: utf-8 -*-
"""创建 render product 配置；实际 render product 由 ROS OmniGraph 在 Play 时创建。"""

from __future__ import annotations

from typing import Optional, Tuple

import omni.usd


class CameraPipeline:
    def __init__(self):
        self.camera_prim_path: Optional[str] = None
        self._resolution: Optional[Tuple[int, int]] = None

    def setup(self, camera_prim_path: str, resolution: Tuple[int, int]) -> bool:
        stage = omni.usd.get_context().get_stage()
        if not stage or not stage.GetPrimAtPath(camera_prim_path):
            print(f"CameraPipeline: camera prim missing: {camera_prim_path}")
            return False
        self.camera_prim_path = camera_prim_path
        self._resolution = resolution
        print(f"CameraPipeline: ready prim={camera_prim_path} res={resolution}")
        return True

    def teardown(self) -> None:
        self.camera_prim_path = None
        self._resolution = None
