# -*- coding: utf-8 -*-
"""三路相机 ROS 出流管理：每路独立 OmniGraph。"""

from __future__ import annotations

from typing import List

from ..topic_config import CameraStreamConfig, SessionTopicConfig
from .camera_publisher import SingleCameraPublisher


class MultiCameraPublisher:
    """管理 cam0/cam1/cam2 三个 ``SingleCameraPublisher`` 实例。"""

    GRAPH_PREFIX = "/NovaGraspNet/CameraGraph"

    def __init__(self):
        self._publishers: List[SingleCameraPublisher] = []

    @property
    def is_active(self) -> bool:
        """任一路相机图仍在发布时为 True。"""
        return any(p.is_active for p in self._publishers)

    def start(self, topic_config: SessionTopicConfig) -> bool:
        """为 ``topic_config.cameras`` 中每路启用的相机创建 OmniGraph。

        Returns:
            至少一路成功创建时为 True。
        """
        self.stop()
        ok_any = False
        for cam in topic_config.cameras:
            graph_path = f"{self.GRAPH_PREFIX}/{cam.key}"
            pub = SingleCameraPublisher(graph_path)
            if pub.start(cam):
                ok_any = True
                self._publishers.append(pub)
            else:
                pub.stop()
        return ok_any

    def stop(self) -> None:
        """停止并销毁所有相机 OmniGraph。"""
        for pub in self._publishers:
            pub.stop()
        self._publishers.clear()
