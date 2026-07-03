# Copyright (c) 2024

EXTENSION_TITLE = "Hs Table Isaac Table Scene"
EXTENSION_DESCRIPTION = (
    "Table with cassette scan mesh and table-edge camera; ROS2 color/depth/pointcloud; runtime 6D pose"
)

TABLE_PRIM_PATH = "/World/table_mz"
CUBOID_LINK_PATH = "/World/cuboid_link"
CUBOID_GEOM_PATH = "/World/cuboid_link/geometry"
CUBOID_PRIM_PATH = CUBOID_LINK_PATH  # 位姿 / TF 作用在 link

CAMERA_LINK_PATH = "/World/camera_link"
CAMERA_OPTICAL_PATH = "/World/camera_link/camera_optical_frame"
CAMERA_PRIM_PATH = "/World/camera_link/camera_optical_frame/camera"

# ROS camera_link → camera_optical_frame 固定旋转 (REP-103)
OPTICAL_FRAME_RPY_DEG = (-90.0, 0.0, -90.0)

DEFAULT_WORLD_FRAME = "map"
DEFAULT_CAMERA_LINK_FRAME = "camera_link"
DEFAULT_CAMERA_OPTICAL_FRAME = "camera_optical_frame"
DEFAULT_OBJECT_FRAME = "cuboid_link"

# mz 桌子 scale=2.5 时桌面高度约 0.78m
DEFAULT_TABLE_TOP_Z = 0.78

# Orbbec Gemini 335 深度流（1280×720 @ 30fps，90°×65° FOV）
ORBBEC_G335_DEPTH_WIDTH = 1280
ORBBEC_G335_DEPTH_HEIGHT = 720
ORBBEC_G335_DEPTH_HFOV_DEG = 90.0
ORBBEC_G335_DEPTH_VFOV_DEG = 65.0
# 未加载桌子时的回退位姿
DEFAULT_CUBOID_CENTER_XY = (0.0, 0.0)
# camera_link：桌面 +Y 边缘高度（旋转由 look-at 对准 cube 中心自动计算）
DEFAULT_CAMERA_HEIGHT_Z = 1.8
DEFAULT_CAMERA_EDGE_Y_RATIO = 0.88
# 桌边典型位姿下的 look-at 欧拉角回退（对准 cube + 成像水平）
DEFAULT_CAMERA_ROTATION_DEG = (90.0, -27.5, -90.0)

# ROS2 OmniGraph 节点命名空间（勿以 / 开头）
ROS_NODE_NAMESPACE = "table_scene"
