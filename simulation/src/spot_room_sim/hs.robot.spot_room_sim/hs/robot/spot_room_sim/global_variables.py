# -*- coding: utf-8 -*-
"""扩展常量与默认 prim / 话题名。"""

EXTENSION_TITLE = "Hs Robot Spot Room Sim"
EXTENSION_DESCRIPTION = (
    "Spot in Simple Room: ROS2 front camera (color/depth/points) + cmd_vel + odom/tf/joint_states"
)

# Stage prims
WORLD_PATH = "/World"
ROOM_PRIM_PATH = "/World/SimpleRoom"
SPOT_PRIM_PATH = "/World/Spot"
CLUTTER_ROOT_PATH = "/World/Clutter"
ENABLE_ROOM_CLUTTER = True  # Load 时生成带碰撞/质量的圆柱与凳子等障碍
SPOT_BODY_CANDIDATES = ("body", "base", "base_link", "torso")
CAMERA_PRIM_NAME = "front_cam"
# 解析后为 /World/Spot/<body>/front_cam 或 /World/Spot/front_cam
CAMERA_PRIM_PATH_FALLBACK = f"{SPOT_PRIM_PATH}/{CAMERA_PRIM_NAME}"

# Isaac Nucleus 相对路径（经 get_assets_root_path）
SIMPLE_ROOM_REL = "/Isaac/Environments/Simple_Room/simple_room.usd"
SPOT_USD_REL = "/Isaac/Robots/BostonDynamics/spot/spot.usd"

# Spot 初始位姿：避开中央 table_low（原先 (0,-1.5) 会钻到桌下翻车）
SPOT_SPAWN_POS = (-1.8, 1.6, 0.55)
SPOT_SPAWN_ORI = (1.0, 0.0, 0.0, 0.0)  # wxyz

# Load 后主视口：Perspective，从 Spot 斜前方俯看（避开误切到 front_cam）
VIEWPORT_EYE = (-0.3, 0.2, 1.85)
VIEWPORT_TARGET = (-1.8, 1.6, 0.35)

# Spot 面板遥控默认速度
UI_DRIVE_LINEAR = 0.6
UI_DRIVE_ANGULAR = 0.8

# 物理（Spot policy 训练为 500Hz）；渲染放宽，避免追帧 spiral
PHYSICS_DT = 1.0 / 500.0
RENDERING_DT = 1.0 / 30.0
# 每渲染帧最多追赶的物理步数，防止 2FPS 时一帧算几百步物理
MAX_PHYSICS_STEPS_PER_FRAME = 8

# 相机默认分辨率（color / depth / points 默认均发布）
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480
DEFAULT_HORIZONTAL_FOV_DEG = 70.0

# Camera on body (USD prim): looks along -Z of Camera schema.
# RPY(90,0,-90) maps that look direction onto robot +X (前方), camera up onto +Z.
# ROS: cam_0_link 与 base 同轴向（X前 Y左 Z上）；optical = link * rpy(-90,0,-90)，
# 使光学系 +Z 同样指向前方，与渲染朝向一致。
CAMERA_LOCAL_XYZ = (0.35, 0.0, 0.12)  # 相对 body，略靠前上方
CAMERA_LOCAL_RPY_DEG = (90.0, 0.0, -90.0)

# Full articulation TF every frame is expensive; odom+cam is enough for mapping
ENABLE_ROBOT_LINK_TF = True

# ROS frames / topics — 与 cam_mgr_ros CameraTfBroadcaster 对齐：
#   cam_0_link          : REP-103 机体式（X 前、Y 左、Z 上）
#   cam_0_*_frame       : 传感器安装系（与 link 同姿）
#   cam_0_*_optical_frame : REP-103 光学系（X 右、Y 下、Z 前）
#   link→optical 固定旋转与 Gazebo 一致：rpy(-π/2, 0, -π/2)
FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_CAMERA_LINK = "cam_0_link"
FRAME_COLOR = "cam_0_color_frame"
FRAME_DEPTH = "cam_0_depth_frame"
FRAME_COLOR_OPTICAL = "cam_0_color_optical_frame"
FRAME_DEPTH_OPTICAL = "cam_0_depth_optical_frame"
# UI / 默认消息系：深度光学系（点云与 CameraInfo 挂此 frame）
FRAME_CAMERA = FRAME_DEPTH_OPTICAL

TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_ODOM = "/odom"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_COLOR = "/cam_0/color/image_raw"
TOPIC_DEPTH = "/cam_0/depth/image_raw"
TOPIC_POINTS = "/cam_0/depth/points"
TOPIC_CAMERA_INFO = "/cam_0/depth/camera_info"

GRAPH_ROOT = "/SpotRoomSim"
CAMERA_GRAPH_PATH = f"{GRAPH_ROOT}/CameraGraph"
JOINT_GRAPH_PATH = f"{GRAPH_ROOT}/JointStateGraph"
