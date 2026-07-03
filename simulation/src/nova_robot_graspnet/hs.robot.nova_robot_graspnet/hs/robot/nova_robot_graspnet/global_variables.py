# -*- coding: utf-8 -*-
"""Nova GraspNet 扩展常量：Prim 路径、场景尺寸、相机/ROS 默认话题。"""

# ---------------------------------------------------------------------------
# 扩展与 ROS
# ---------------------------------------------------------------------------
EXTENSION_TITLE = "Hs Robot Nova Robot GraspNet"
ROS_NODE_NAMESPACE = "nova_graspnet"

# ---------------------------------------------------------------------------
# USD Prim 路径（Stage 内唯一）
# ---------------------------------------------------------------------------
ROBOT_PRIM_PATH = "/World/nova_robot"
ROBOT_ROOT_JOINT_PATH = "/World/nova_robot/root_joint"
TABLE_PRIM_PATH = "/World/graspnet_table"
BOX_LINK_PATH = "/World/grasp_box"          # 刚体根 / TF 帧
BOX_VISUAL_PATH = "/World/grasp_box/visual"  # OBJ/USD 视觉引用
BOX_COLLISION_PATH = "/World/grasp_box/collision"  # 简化盒碰撞

# ---------------------------------------------------------------------------
# 桌子（米）：2×1 桌面，顶面 z = TABLE_TOP_Z
# ---------------------------------------------------------------------------
TABLE_LENGTH = 2.0
TABLE_WIDTH = 1.0
TABLE_HEIGHT = 0.6
TABLE_TOP_Z = TABLE_HEIGHT
TABLE_CENTER = (0.0, 0.0, 0.0)
TABLE_TOP_THICKNESS = 0.05   # 桌面板厚
TABLE_LEG_SIZE = 0.08        # 腿截面边长
TABLE_LEG_INSET = 0.12       # 腿中心距桌边的内缩量
TABLE_LEG_HEIGHT = TABLE_TOP_Z - TABLE_TOP_THICKNESS

# ---------------------------------------------------------------------------
# 机器人挂载
# base_link 原点在 J1 侧基座平台顶面（z=0），不是整台几何中心
# Load 时会量 base_link 下台面 mesh 包围盒，把台面中心对齐桌心，再叠加 FINE 微调
# ---------------------------------------------------------------------------
PLATFORM_DEPTH_BELOW_BASE = 0.04  # base_link z=0 到平台网格底面（量不到时的回退）
ROBOT_MOUNT_FINE_XY = (0.0, 0.0)  # 在自动居中基础上的 xy 微调（米）
ROBOT_MOUNT_FINE_Z = -0.50  # 平台底面相对桌面的 z 微调（米，负值=降低；实测约 -0.5）

# 量不到 bbox 时用于居中的回退：base_link mesh extent 中心 (见 nova_robot_prepared.usda)
ROBOT_FOOTPRINT_CENTER_XY = (0.53, -0.18)

# 双臂 J1_1 / J2_1 在 base_link 下的 x（URDF joint origin）
ARM_J1_X = 0.0
ARM_J2_X = 1.06

# ---------------------------------------------------------------------------
# 抓取盒：尺寸与 meta 一致；prepare_box_usd.py 会更新 grasp_box_meta.json
# ---------------------------------------------------------------------------
BOX_SIZE_X = 0.173
BOX_SIZE_Y = 0.107
BOX_SIZE_Z = 0.245
BOX_MASS_KG = 0.3

# 下列为 Load 前 UI 占位；Load 后 scene_loader 会按实测 mount 更新
ROBOT_MOUNT_Z = TABLE_TOP_Z + PLATFORM_DEPTH_BELOW_BASE + ROBOT_MOUNT_FINE_Z
ROBOT_ARM_CENTER_XY = (0.0, 0.0)
DEFAULT_BOX_CENTER = (
    0.0,
    0.0,
    ROBOT_MOUNT_Z + BOX_SIZE_Z * 0.5,
)

# ---------------------------------------------------------------------------
# 相机默认分辨率
# ---------------------------------------------------------------------------
DEFAULT_CAMERA_WIDTH = 1280
DEFAULT_CAMERA_HEIGHT = 720

# 三路 RSD455：prim_suffix 相对 ROBOT_PRIM_PATH
CAMERA_DEFS = (
    {
        "key": "cam0",
        "label": "Camera 0 (base / cam0)",
        "prim_suffix": "base_link/cam0/RSD455/Camera_Pseudo_Depth",
        "frame_id": "camera0_pseudo_depth",
        "pub_color": "/camera0_rgb_sensor/image_raw",
        "pub_depth": "/camera0_depth_sensor/depth/image_raw",
        "pub_points": "/camera0_depth_sensor/depth/points",
        "pub_camera_info": "/camera0_rgb_sensor/camera_info",
    },
    {
        "key": "cam1",
        "label": "Camera 1 (J1_6 / cam1)",
        "prim_suffix": "J1_6/cam1/RSD455/Camera_Pseudo_Depth",
        "frame_id": "camera1_pseudo_depth",
        "pub_color": "/camera1_rgb_sensor/image_raw",
        "pub_depth": "/camera1_depth_sensor/depth/image_raw",
        "pub_points": "/camera1_depth_sensor/depth/points",
        "pub_camera_info": "/camera1_rgb_sensor/camera_info",
    },
    {
        "key": "cam2",
        "label": "Camera 2 (J2_6 / cam2)",
        "prim_suffix": "J2_6/cam2/RSD455/Camera_Pseudo_Depth",
        "frame_id": "camera2_pseudo_depth",
        "pub_color": "/camera2_rgb_sensor/image_raw",
        "pub_depth": "/camera2_depth_sensor/depth/image_raw",
        "pub_points": "/camera2_depth_sensor/depth/points",
        "pub_camera_info": "/camera2_rgb_sensor/camera_info",
    },
)

# ---------------------------------------------------------------------------
# 机器人 ROS 默认话题
# ---------------------------------------------------------------------------
JOINT_STATES_TOPIC = "/joint_states"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
TF_WORLD_FRAME = "world"
