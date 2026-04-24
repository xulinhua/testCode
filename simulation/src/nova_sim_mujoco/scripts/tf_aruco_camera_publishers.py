#!/usr/bin/env python3
"""
TF-based ArUco renderer for MuJoCo calibration.

MuJoCo path does not execute gazebo camera plugins in URDF. This node renders
the known ArUco boards into camera images using live TF and publishes:
  /camera{0,1,2}_rgb_sensor/image_raw
  /camera{0,1,2}_rgb_sensor/camera_info
"""

from dataclasses import dataclass
from typing import Dict, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class CamSpec:
    topic_prefix: str
    frame_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


class TfArucoCameraPublishers(Node):
    def __init__(self) -> None:
        super().__init__("tf_aruco_camera_publishers")
        self.declare_parameter("publish_hz", 15.0)
        hz = float(self.get_parameter("publish_hz").value)
        self._period = 1.0 / max(1.0, hz)

        self._cams = [
            CamSpec("camera0_rgb_sensor", "camera0_optical_frame", 1280, 720, 910.0, 910.0, 640.0, 360.0),
            CamSpec("camera1_rgb_sensor", "camera1_optical_frame", 1280, 720, 910.0, 910.0, 640.0, 360.0),
            CamSpec("camera2_rgb_sensor", "camera2_optical_frame", 1280, 720, 910.0, 910.0, 640.0, 360.0),
        ]

        # Board link -> marker id
        self._boards: Dict[str, int] = {
            "aruco_board_base_mid": 0,
            "aruco_board_base_arm1": 1,
            "aruco_board_j1_6": 2,
            "aruco_board_j2_6": 3,
        }
        self._board_size_m = 0.10
        self._half = self._board_size_m * 0.5
        self._marker_px = 400

        self._aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        self._marker_imgs = {
            marker_id: self._make_marker(marker_id) for marker_id in self._boards.values()
        }

        self._img_pubs = []
        self._info_pubs = []
        for cam in self._cams:
            self._img_pubs.append(self.create_publisher(Image, f"/{cam.topic_prefix}/image_raw", 10))
            self._info_pubs.append(self.create_publisher(CameraInfo, f"/{cam.topic_prefix}/camera_info", 10))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(self._period, self._on_timer)
        self.get_logger().info("TF ArUco camera renderer started for /camera{0,1,2}_rgb_sensor/*")

    def _make_marker(self, marker_id: int) -> np.ndarray:
        core = np.zeros((self._marker_px, self._marker_px), dtype=np.uint8)
        cv2.aruco.generateImageMarker(self._aruco_dict, marker_id, self._marker_px, core, 1)
        border = 40
        out = np.full((self._marker_px + 2 * border, self._marker_px + 2 * border), 255, dtype=np.uint8)
        out[border:-border, border:-border] = core
        return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for i, cam in enumerate(self._cams):
            frame = np.full((cam.height, cam.width, 3), 24, dtype=np.uint8)
            # Light grid background for visual sanity check
            frame[::40, :, :] = (40, 40, 40)
            frame[:, ::40, :] = (40, 40, 40)

            for board_link, marker_id in self._boards.items():
                corners_2d = self._project_board(cam, board_link)
                if corners_2d is None:
                    continue
                self._warp_marker(frame, self._marker_imgs[marker_id], corners_2d)

            self._draw_text(frame, cam.frame_id)

            img_msg = self._to_image_msg(frame, cam.frame_id, stamp)
            info_msg = self._to_info_msg(cam, stamp)
            self._img_pubs[i].publish(img_msg)
            self._info_pubs[i].publish(info_msg)

    def _project_board(self, cam: CamSpec, board_link: str) -> np.ndarray | None:
        try:
            tf_msg = self._tf_buffer.lookup_transform(cam.frame_id, board_link, rclpy.time.Time())
        except TransformException:
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rot = self._quat_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float64).reshape(3, 1)

        # Board plane on XY of board frame, facing +Z.
        pts = np.array(
            [
                [-self._half, -self._half, 0.0],
                [self._half, -self._half, 0.0],
                [self._half, self._half, 0.0],
                [-self._half, self._half, 0.0],
            ],
            dtype=np.float64,
        ).T
        pts_cam = rot @ pts + trans
        if np.any(pts_cam[2, :] <= 0.03):
            return None

        u = cam.fx * pts_cam[0, :] / pts_cam[2, :] + cam.cx
        v = cam.fy * pts_cam[1, :] / pts_cam[2, :] + cam.cy
        corners = np.stack([u, v], axis=1).astype(np.float32)

        # Basic frustum cull with small margin.
        if np.all((corners[:, 0] < -80) | (corners[:, 0] > cam.width + 80) |
                  (corners[:, 1] < -80) | (corners[:, 1] > cam.height + 80)):
            return None
        return corners

    def _warp_marker(self, frame: np.ndarray, marker: np.ndarray, dst_corners: np.ndarray) -> None:
        h, w = marker.shape[:2]
        src = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.float32)
        H = cv2.getPerspectiveTransform(src, dst_corners)
        warped = cv2.warpPerspective(marker, H, (frame.shape[1], frame.shape[0]))
        mask = cv2.warpPerspective(np.full((h, w), 255, dtype=np.uint8), H, (frame.shape[1], frame.shape[0]))
        fg = cv2.bitwise_and(warped, warped, mask=mask)
        bg = cv2.bitwise_and(frame, frame, mask=cv2.bitwise_not(mask))
        frame[:, :] = cv2.add(bg, fg)

    @staticmethod
    def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
        n = max(1e-12, x * x + y * y + z * z + w * w)
        s = 2.0 / n
        xx, yy, zz = x * x * s, y * y * s, z * z * s
        xy, xz, yz = x * y * s, x * z * s, y * z * s
        wx, wy, wz = w * x * s, w * y * s, w * z * s
        return np.array(
            [
                [1.0 - (yy + zz), xy - wz, xz + wy],
                [xy + wz, 1.0 - (xx + zz), yz - wx],
                [xz - wy, yz + wx, 1.0 - (xx + yy)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _draw_text(frame: np.ndarray, frame_id: str) -> None:
        cv2.putText(frame, f"TF render: {frame_id}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 220, 255), 2)

    @staticmethod
    def _to_image_msg(img: np.ndarray, frame_id: str, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(img.shape[0])
        msg.width = int(img.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(img.shape[1] * img.shape[2])
        msg.data = img.tobytes()
        return msg

    @staticmethod
    def _to_info_msg(cam: CamSpec, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = cam.frame_id
        msg.width = cam.width
        msg.height = cam.height
        msg.k = [cam.fx, 0.0, cam.cx, 0.0, cam.fy, cam.cy, 0.0, 0.0, 1.0]
        msg.p = [cam.fx, 0.0, cam.cx, 0.0, 0.0, cam.fy, cam.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return msg


def main() -> None:
    rclpy.init()
    node = TfArucoCameraPublishers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
