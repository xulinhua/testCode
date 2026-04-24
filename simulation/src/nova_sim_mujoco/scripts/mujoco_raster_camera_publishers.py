#!/usr/bin/env python3
"""
MuJoCo raster camera publishers.

Render real RGB/Depth images from MuJoCo model and publish ROS topics:
  /camera{0,1,2}_rgb_sensor/image_raw
  /camera{0,1,2}_rgb_sensor/camera_info
  /camera{0,1,2}_depth_sensor/depth/image_raw
  /camera{0,1,2}_depth_sensor/camera_info
"""

from dataclasses import dataclass
import math
import os
import site
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

# Force headless rendering backend for ROS launch environments without X11.
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

# Try current interpreter first; if not found, fall back to known MuJoCo venv.
try:
    import mujoco  # type: ignore
except ModuleNotFoundError:
    pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    venv_site = f"/home/hs/.ros/ros2_control/.venv/lib/{pyver}/site-packages"
    if os.path.isdir(venv_site):
        site.addsitedir(venv_site)
        import mujoco  # type: ignore
    else:
        raise


@dataclass(frozen=True)
class CamDef:
    index: int
    parent_body: str
    camera_name: str
    rgb_topic_prefix: str
    depth_topic_prefix: str
    frame_id: str
    width: int = 1280
    height: int = 720
    rel_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rel_quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    target_body: str = ""


class MujocoRasterCameraPublishers(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_raster_camera_publishers")
        self.declare_parameter("publish_hz", 15.0)
        hz = float(self.get_parameter("publish_hz").value)
        self._period = 1.0 / max(1.0, hz)

        self._cams = [
            # camera0 optical frame link is removed by URDF->MJCF converter; use a world-fixed camera.
            CamDef(
                0, "world", "camera0_rgb_sensor", "camera0_rgb_sensor", "camera0_depth_sensor",
                "camera0_optical_frame",
                rel_pos=(0.5299998, -0.4994585, 1.10),
                rel_quat_wxyz=(0.0, 0.9999996, 0.00079449, -0.00039816),
                target_body="aruco_board_base_mid",
            ),
            # Attach camera1/camera2 to surviving wrist bodies in MJCF.
            CamDef(
                1, "J1_6", "camera1_rgb_sensor", "camera1_rgb_sensor", "camera1_depth_sensor",
                "camera1_optical_frame",
                rel_pos=(0.00, -0.05, 0.05),
                rel_quat_wxyz=(0.9999999, -0.00039816, 0.0, 0.00000184),
                target_body="aruco_board_base_mid",
            ),
            CamDef(
                2, "J2_6", "camera2_rgb_sensor", "camera2_rgb_sensor", "camera2_depth_sensor",
                "camera2_optical_frame",
                rel_pos=(0.00, -0.05, 0.05),
                rel_quat_wxyz=(0.9999999, -0.00039816, 0.0, 0.00000184),
                target_body="aruco_board_base_arm1",
            ),
        ]

        self._rgb_pubs = []
        self._rgb_info_pubs = []
        self._depth_pubs = []
        self._depth_info_pubs = []
        for cam in self._cams:
            self._rgb_pubs.append(self.create_publisher(Image, f"/{cam.rgb_topic_prefix}/image_raw", 10))
            self._rgb_info_pubs.append(self.create_publisher(CameraInfo, f"/{cam.rgb_topic_prefix}/camera_info", 10))
            self._depth_pubs.append(
                self.create_publisher(Image, f"/{cam.depth_topic_prefix}/depth/image_raw", 10))
            self._depth_info_pubs.append(
                self.create_publisher(CameraInfo, f"/{cam.depth_topic_prefix}/camera_info", 10))

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._mjcf_sub = self.create_subscription(String, "/mujoco_robot_description", self._on_mjcf, qos)
        self._joint_sub = self.create_subscription(JointState, "/joint_states", self._on_joint_state, 30)

        self._model = None
        self._data = None
        self._renderer = None
        self._joint_map = {}
        self._camera_name_to_id = {}
        self._timer = self.create_timer(self._period, self._on_timer)
        self.get_logger().info("Waiting /mujoco_robot_description to start MuJoCo raster camera publishing...")

    def _on_joint_state(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_map[name] = float(msg.position[i])

    def _on_mjcf(self, msg: String) -> None:
        if self._model is not None:
            return
        try:
            xml_with_cams = self._inject_cameras(msg.data)
            self._model = mujoco.MjModel.from_xml_string(xml_with_cams)
            self._data = mujoco.MjData(self._model)
            self._renderer = mujoco.Renderer(self._model, height=self._cams[0].height, width=self._cams[0].width)
            self._camera_name_to_id = {
                mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_CAMERA, i): i
                for i in range(self._model.ncam)
            }
            self.get_logger().info(f"Loaded MuJoCo model for raster camera publishing, ncam={self._model.ncam}.")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed loading MuJoCo model for cameras: {exc}")

    def _inject_cameras(self, xml_text: str) -> str:
        root = ET.fromstring(xml_text)
        worldbody = root.find("worldbody")
        if worldbody is None:
            return xml_text

        # Ensure offscreen framebuffer is large enough for requested render size.
        max_w = max(cam.width for cam in self._cams)
        max_h = max(cam.height for cam in self._cams)
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        global_tag = visual.find("global")
        if global_tag is None:
            global_tag = ET.SubElement(visual, "global")
        cur_off_w = int(global_tag.attrib.get("offwidth", "0") or 0)
        cur_off_h = int(global_tag.attrib.get("offheight", "0") or 0)
        if cur_off_w < max_w:
            global_tag.set("offwidth", str(max_w))
        if cur_off_h < max_h:
            global_tag.set("offheight", str(max_h))

        body_by_name = {}
        for body in worldbody.iter("body"):
            name = body.attrib.get("name")
            if name:
                body_by_name[name] = body

        for cam in self._cams:
            if cam.parent_body == "world":
                target = worldbody
            else:
                target = body_by_name.get(cam.parent_body)
                if target is None:
                    continue
            has_camera = any(c.attrib.get("name") == cam.camera_name for c in target.findall("camera"))
            if has_camera:
                continue
            attrs = {
                "name": cam.camera_name,
                "mode": "targetbodycom" if cam.target_body else "fixed",
                "pos": f"{cam.rel_pos[0]} {cam.rel_pos[1]} {cam.rel_pos[2]}",
                "quat": (
                    f"{cam.rel_quat_wxyz[0]} {cam.rel_quat_wxyz[1]} "
                    f"{cam.rel_quat_wxyz[2]} {cam.rel_quat_wxyz[3]}"
                ),
                "fovy": "45",
            }
            if cam.target_body:
                attrs["target"] = cam.target_body
            ET.SubElement(target, "camera", attrs)
        return ET.tostring(root, encoding="unicode")

    def _on_timer(self) -> None:
        if self._model is None or self._data is None or self._renderer is None:
            return
        self._apply_joint_states()
        mujoco.mj_forward(self._model, self._data)
        stamp = self.get_clock().now().to_msg()

        for i, cam in enumerate(self._cams):
            cam_id = self._camera_name_to_id.get(cam.camera_name)
            if cam_id is None:
                continue

            self._renderer.update_scene(self._data, camera=cam.camera_name)
            rgb = self._renderer.render()
            rgb_msg = self._to_rgb_msg(rgb, cam.frame_id, stamp)
            info_msg = self._make_info(cam, cam_id, stamp)
            self._rgb_pubs[i].publish(rgb_msg)
            self._rgb_info_pubs[i].publish(info_msg)

            self._renderer.enable_depth_rendering()
            self._renderer.update_scene(self._data, camera=cam.camera_name)
            depth = self._renderer.render().astype(np.float32)
            self._renderer.disable_depth_rendering()
            depth_msg = self._to_depth_msg(depth, cam.frame_id, stamp)
            self._depth_pubs[i].publish(depth_msg)
            self._depth_info_pubs[i].publish(info_msg)

    def _apply_joint_states(self) -> None:
        if not self._joint_map:
            return
        for name, q in self._joint_map.items():
            try:
                j_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            except Exception:  # noqa: BLE001
                continue
            if j_id < 0:
                continue
            qpos_addr = self._model.jnt_qposadr[j_id]
            self._data.qpos[qpos_addr] = q

    @staticmethod
    def _to_rgb_msg(rgb: np.ndarray, frame_id: str, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(rgb.shape[0])
        msg.width = int(rgb.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = int(rgb.shape[1] * 3)
        msg.data = rgb.tobytes()
        return msg

    @staticmethod
    def _to_depth_msg(depth: np.ndarray, frame_id: str, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(depth.shape[0])
        msg.width = int(depth.shape[1])
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = int(depth.shape[1] * 4)
        msg.data = depth.tobytes()
        return msg

    def _make_info(self, cam: CamDef, cam_id: int, stamp) -> CameraInfo:
        fovy_deg = float(self._model.cam_fovy[cam_id])
        fy = cam.height / (2.0 * math.tan(math.radians(fovy_deg) * 0.5))
        fx = fy
        cx = cam.width * 0.5
        cy = cam.height * 0.5

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = cam.frame_id
        msg.width = cam.width
        msg.height = cam.height
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return msg


def main() -> None:
    rclpy.init()
    node = MujocoRasterCameraPublishers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
