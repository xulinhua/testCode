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


def _q_from_rpy_radians(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """URDF 固定轴 rpy（弧度）→ 四元数 x,y,z,w，与 RSP 常用 tf 约定一致。"""
    try:
        from tf_transformations import quaternion_from_euler

        q = quaternion_from_euler(roll, pitch, yaw)
    except Exception:  # noqa: BLE001
        from scipy.spatial.transform import Rotation as Rf

        q = Rf.from_euler("xyz", (roll, pitch, yaw), degrees=False).as_quat()
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _q_mult_xyzw(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    try:
        from tf_transformations import quaternion_multiply

        o = quaternion_multiply(a, b)
    except Exception:  # noqa: BLE001
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        x = aw * bx + ax * bw + ay * bz - az * by
        y = aw * by - ax * bz + ay * bw + az * bx
        z = aw * bz + ax * by - ay * bx + az * bw
        w = aw * bw - ax * bx - ay * by - az * bz
        o = (x, y, z, w)
    return (float(o[0]), float(o[1]), float(o[2]), float(o[3]))


# 与 nova_robot_position.urdf: camera*_optical_joint，parent=camera*_link, xyz=0, rpy=-pi/2 0 -pi/2
_Q_LINK_TO_OPT = _q_from_rpy_radians(-1.57079632679, 0.0, -1.57079632679)
# camera0: gantry_connector->camera0_link, xyz 0,0,-0.15, rpy 1.57,1.57,-1.5708
_Q_GANTRY0_LINK = _q_from_rpy_radians(1.57, 1.57, -1.5708)
_Q_CAM0_IN_GANTRY = _q_mult_xyzw(_Q_GANTRY0_LINK, _Q_LINK_TO_OPT)
# 龙门+opt 链在 base_link(=world 同原点) 下零关节的 camera0_optical 位姿；MJCF 无 camera0_link/gantry 时挂 base/world
_POS_BASE_CAM0_OPT = (0.52999978, -0.4994585, 1.04)
_Q_BASE_CAM0_OPT = (-0.9999996, -0.0007945, 0.0003982, 0.0)  # xyzw，与 robot_state_publisher+URDF 一致
# MuJoCo camera forward uses -Z; ROS optical uses +Z forward.
_Q_ROS_OPT_TO_MJ_CAM = _q_from_rpy_radians(3.14159265359, 0.0, 0.0)
# J1_6->camera1_link / J2_6->camera2_link 与 URDF 中 camera1_joint / camera2_joint 的 rpy 一致（fallback 为挂不到 camera*_link 时）
_RPY_J1_TO_CAM1_LINK = (0.0, -1.57, 1.5708)
_RPY_J2_TO_CAM2_LINK = (0.0, -1.57, 1.5708)
_Q_J1_TO_CAM1_LINK = _q_from_rpy_radians(*_RPY_J1_TO_CAM1_LINK)
_Q_J2_TO_CAM2_LINK = _q_from_rpy_radians(*_RPY_J2_TO_CAM2_LINK)
_Q_CAM1_IN_J = _q_mult_xyzw(_Q_J1_TO_CAM1_LINK, _Q_LINK_TO_OPT)
_Q_CAM2_IN_J = _q_mult_xyzw(_Q_J2_TO_CAM2_LINK, _Q_LINK_TO_OPT)

# 每条为 (父 body 名, 相对位姿(米), 四元数 xyzw)；优先挂在 camera*_link 上仅加 optical，避免与 world 人为机位/错误腕偏移
MountSpec = tuple[str, tuple[float, float, float], tuple[float, float, float, float]]


@dataclass(frozen=True)
class CamDef:
    index: int
    # Unique MJCF camera name used by renderer (avoid collisions with converted cameras/sensors).
    mjcf_camera_name: str
    rgb_topic_prefix: str
    depth_topic_prefix: str
    frame_id: str
    # 按序尝试；首命中的 body 上注入，与 TF/URDF 的 camera*_optical_frame 一致
    mount_chain: tuple[MountSpec, ...]
    width: int = 1280
    height: int = 720
    target_body: str = ""
    track_target: bool = False
    fovy_deg: float = 58.0
    # 2D 后处理；仅在确有镜像时再打开
    image_fliplr: bool = False
    image_rot180: bool = False


@dataclass(frozen=True)
class WorldTargetDef:
    body_name: str
    pos_world: tuple[float, float, float]


class MujocoRasterCameraPublishers(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_raster_camera_publishers")
        self.declare_parameter("publish_hz", 15.0)
        self.declare_parameter("enable_target_tracking", True)
        self.declare_parameter("enable_depth", False)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        hz = float(self.get_parameter("publish_hz").value)
        self._enable_target_tracking = bool(self.get_parameter("enable_target_tracking").value)
        self._enable_depth = bool(self.get_parameter("enable_depth").value)
        self._image_width = int(self.get_parameter("image_width").value)
        self._image_height = int(self.get_parameter("image_height").value)
        self._period = 1.0 / max(1.0, hz)

        self._cams = [
            CamDef(
                0,
                "raster_cam0",
                "camera0_rgb_sensor",
                "camera0_depth_sensor",
                "camera0_optical_frame",
                mount_chain=(
                    # Calibration path: prefer URDF camera0_link -> camera0_optical_frame.
                    ("camera0_link", (0.0, 0.0, 0.0), _q_mult_xyzw(_Q_LINK_TO_OPT, _Q_ROS_OPT_TO_MJ_CAM)),
                    ("gantry_connector", (0.0, 0.0, -0.15), _q_mult_xyzw(_Q_CAM0_IN_GANTRY, _Q_ROS_OPT_TO_MJ_CAM)),
                    ("base_link", _POS_BASE_CAM0_OPT, _q_mult_xyzw(_Q_BASE_CAM0_OPT, _Q_ROS_OPT_TO_MJ_CAM)),
                    ("world", _POS_BASE_CAM0_OPT, _q_mult_xyzw(_Q_BASE_CAM0_OPT, _Q_ROS_OPT_TO_MJ_CAM)),
                ),
                width=self._image_width,
                height=self._image_height,
                target_body="",
                track_target=False,
                fovy_deg=43.0,
                image_fliplr=True,
                image_rot180=True,
            ),
            CamDef(
                1,
                "raster_cam1",
                "camera1_rgb_sensor",
                "camera1_depth_sensor",
                "camera1_optical_frame",
                mount_chain=(
                    ("camera1_link", (0.0, 0.0, 0.0), _q_mult_xyzw(_Q_LINK_TO_OPT, _Q_ROS_OPT_TO_MJ_CAM)),
                    ("J1_6", (0.0, -0.08, 0.041), _q_mult_xyzw(_Q_CAM1_IN_J, _Q_ROS_OPT_TO_MJ_CAM)),
                ),
                width=self._image_width,
                height=self._image_height,
                target_body="aruco_board_base_mid",
                track_target=False,
                # RealSense D405-like narrower vertical FOV (was too wide at 78 deg).
                fovy_deg=58.0,
                image_fliplr=True,
                image_rot180=True,
            ),
            CamDef(
                2,
                "raster_cam2",
                "camera2_rgb_sensor",
                "camera2_depth_sensor",
                "camera2_optical_frame",
                mount_chain=(
                    (
                        "camera2_link",
                        (0.0, 0.0, 0.0),
                        _q_mult_xyzw(_Q_LINK_TO_OPT, _Q_ROS_OPT_TO_MJ_CAM),
                    ),
                    (
                        "J2_6",
                        (0.0, -0.08, 0.041),
                        _q_mult_xyzw(_Q_CAM2_IN_J, _Q_ROS_OPT_TO_MJ_CAM),
                    ),
                ),
                width=self._image_width,
                height=self._image_height,
                target_body="aruco_board_base_arm1",
                track_target=False,
                # RealSense D405-like narrower vertical FOV (was too wide at 78 deg).
                fovy_deg=58.0,
                image_fliplr=True,
                image_rot180=True,
            ),
        ]
        # URDF->MJCF conversion may strip board link bodies. Recreate stable world targets
        # so cameras can keep using targetbodycom in real raster rendering.
        self._world_targets = [
            WorldTargetDef("aruco_board_base_mid", (0.42, -0.14, 0.02)),
            WorldTargetDef("aruco_board_base_arm1", (0.65, -0.14, 0.02)),
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
            cam_names = [mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(self._model.ncam)]
            self.get_logger().info(
                f"Loaded MuJoCo model for raster camera publishing, ncam={self._model.ncam}, cams={cam_names}."
            )
            for cam in self._cams:
                cam_id = self._camera_name_to_id.get(cam.mjcf_camera_name)
                self.get_logger().info(
                    f"Raster map: /{cam.rgb_topic_prefix}/image_raw -> {cam.mjcf_camera_name} (id={cam_id})"
                )
                if cam_id is None:
                    self.get_logger().error(
                        f"未注入 {cam.mjcf_camera_name}（无对应 MJCF 父 body？或名称冲突）。"
                    )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed loading MuJoCo model for cameras: {exc}")

    def _inject_cameras(self, xml_text: str) -> str:
        def xyzw_to_wxyz(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
            return (q[3], q[0], q[1], q[2])

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

        for target in self._world_targets:
            if target.body_name in body_by_name:
                continue
            injected = ET.SubElement(
                worldbody,
                "body",
                {
                    "name": target.body_name,
                    "pos": f"{target.pos_world[0]} {target.pos_world[1]} {target.pos_world[2]}",
                },
            )
            ET.SubElement(injected, "site", {"name": f"{target.body_name}_site", "size": "0.001"})
            body_by_name[target.body_name] = injected

        for cam in self._cams:
            chosen: tuple[str, tuple[float, float, float], tuple[float, float, float, float]] | None = None
            target = None
            for spec in cam.mount_chain:
                bname, rel_pos, rel_quat = spec
                tnode = worldbody if bname == "world" else body_by_name.get(bname)
                if tnode is not None:
                    chosen = spec
                    target = tnode
                    break
            if target is None or chosen is None:
                tried = [s[0] for s in cam.mount_chain]
                self.get_logger().warning(
                    f"Skip injecting camera '{cam.mjcf_camera_name}': none of parent bodies in MJCF: {tried}.")
                continue
            has_camera = any(c.attrib.get("name") == cam.mjcf_camera_name for c in target.findall("camera"))
            if has_camera:
                continue
            bname, rel_pos, rel_quat = chosen
            self.get_logger().info(
                f"Injecting '{cam.mjcf_camera_name}' on body '{bname}' pos={rel_pos} (URDF camera*_optical 对齐)")

            target_exists = bool(cam.target_body) and cam.target_body in body_by_name
            use_tracking = self._enable_target_tracking and cam.track_target
            mode = "targetbodycom" if (use_tracking and target_exists) else "fixed"
            attrs = {
                "name": cam.mjcf_camera_name,
                "mode": mode,
                "pos": f"{rel_pos[0]} {rel_pos[1]} {rel_pos[2]}",
                "quat": " ".join(str(v) for v in xyzw_to_wxyz(rel_quat)),
                "fovy": f"{cam.fovy_deg}",
            }
            if use_tracking and target_exists:
                attrs["target"] = cam.target_body
            elif use_tracking and cam.target_body:
                self.get_logger().warning(
                    f"Camera '{cam.mjcf_camera_name}' target body '{cam.target_body}' not found, using fixed mode.")
            ET.SubElement(target, "camera", attrs)
        return ET.tostring(root, encoding="unicode")

    @staticmethod
    def _postprocess_raster(
        arr: np.ndarray, fliplr: bool, rot180: bool
    ) -> np.ndarray:
        if fliplr:
            arr = np.fliplr(arr)
        if rot180:
            arr = np.rot90(arr, 2)
        return arr

    def _on_timer(self) -> None:
        if self._model is None or self._data is None or self._renderer is None:
            return
        self._apply_joint_states()
        mujoco.mj_forward(self._model, self._data)
        stamp = self.get_clock().now().to_msg()

        for i, cam in enumerate(self._cams):
            cam_id = self._camera_name_to_id.get(cam.mjcf_camera_name)
            if cam_id is None:
                continue
            rgb_sub_count = self._rgb_pubs[i].get_subscription_count()
            depth_sub_count = self._depth_pubs[i].get_subscription_count()
            info_sub_count = self._rgb_info_pubs[i].get_subscription_count() + self._depth_info_pubs[i].get_subscription_count()
            if rgb_sub_count == 0 and depth_sub_count == 0 and info_sub_count == 0:
                continue

            info_msg = self._make_info(cam, cam_id, stamp)
            if rgb_sub_count > 0 or self._rgb_info_pubs[i].get_subscription_count() > 0:
                self._renderer.update_scene(self._data, camera=cam.mjcf_camera_name)
                rgb = self._renderer.render()
                # MuJoCo/OpenGL image origin is bottom-left; ROS image consumers expect top-left.
                rgb = np.flipud(rgb)
                rgb = self._postprocess_raster(rgb, cam.image_fliplr, cam.image_rot180)
                if rgb_sub_count > 0:
                    rgb_msg = self._to_rgb_msg(rgb, cam.frame_id, stamp)
                    self._rgb_pubs[i].publish(rgb_msg)
                if self._rgb_info_pubs[i].get_subscription_count() > 0:
                    self._rgb_info_pubs[i].publish(info_msg)

            if depth_sub_count > 0 or self._depth_info_pubs[i].get_subscription_count() > 0:
                if not self._enable_depth:
                    continue
                self._renderer.enable_depth_rendering()
                self._renderer.update_scene(self._data, camera=cam.mjcf_camera_name)
                depth = self._renderer.render().astype(np.float32)
                depth = np.flipud(depth)
                depth = self._postprocess_raster(depth, cam.image_fliplr, cam.image_rot180)
                self._renderer.disable_depth_rendering()
                if depth_sub_count > 0:
                    depth_msg = self._to_depth_msg(depth, cam.frame_id, stamp)
                    self._depth_pubs[i].publish(depth_msg)
                if self._depth_info_pubs[i].get_subscription_count() > 0:
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
