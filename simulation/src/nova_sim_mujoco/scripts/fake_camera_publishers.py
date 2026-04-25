#!/usr/bin/env python3
"""
MuJoCo fallback camera publishers.

Why this exists:
- Current robot URDF uses <gazebo> camera plugins (libgazebo_ros_camera.so).
- In MuJoCo flow these plugins are not executed, so image topics stay silent.

This node publishes synthetic RGB image + camera_info for camera0/1/2 so
calibration and perception pipelines can at least run end-to-end.
"""

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


@dataclass(frozen=True)
class CamSpec:
    name: str
    frame_id: str
    width: int = 1280
    height: int = 720


class FakeCameraPublishers(Node):
    def __init__(self) -> None:
        super().__init__("fake_camera_publishers")
        self.declare_parameter("publish_hz", 15.0)
        hz = float(self.get_parameter("publish_hz").value)
        self._period = 1.0 / max(1.0, hz)

        self._cams = [
            CamSpec("camera0_rgb_sensor", "camera0_link"),
            CamSpec("camera1_rgb_sensor", "camera1_link"),
            CamSpec("camera2_rgb_sensor", "camera2_link"),
        ]

        self._img_pubs = []
        self._info_pubs = []
        for cam in self._cams:
            self._img_pubs.append(self.create_publisher(Image, f"/{cam.name}/image_raw", 10))
            self._info_pubs.append(self.create_publisher(CameraInfo, f"/{cam.name}/camera_info", 10))

        self._timer = self.create_timer(self._period, self._on_timer)
        self.get_logger().warn(
            "Publishing synthetic camera streams for MuJoCo fallback: /camera{0,1,2}_rgb_sensor/*"
        )

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for idx, cam in enumerate(self._cams):
            img = Image()
            img.header.stamp = stamp
            img.header.frame_id = cam.frame_id
            img.height = cam.height
            img.width = cam.width
            img.encoding = "rgb8"
            img.is_bigendian = 0
            img.step = cam.width * 3

            # Solid color per camera to make source obvious.
            if idx == 0:
                pixel = bytes([220, 80, 80])   # reddish
            elif idx == 1:
                pixel = bytes([80, 220, 80])   # greenish
            else:
                pixel = bytes([80, 80, 220])   # bluish
            img.data = pixel * (cam.width * cam.height)

            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = cam.frame_id
            info.width = cam.width
            info.height = cam.height
            fx = 900.0
            fy = 900.0
            cx = cam.width / 2.0
            cy = cam.height / 2.0
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.distortion_model = "plumb_bob"
            info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

            self._img_pubs[idx].publish(img)
            self._info_pubs[idx].publish(info)


def main() -> None:
    rclpy.init()
    node = FakeCameraPublishers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
