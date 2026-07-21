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
BOX_COLLISION_PATH = "/World/grasp_box/collision"  # 旧 AABB 立方体（加载时删除）
BOX_COLLISION_GEO_ROOT = "/World/grasp_box/collision_geo"
BOX_COLLISION_GEO_MESH = f"{BOX_COLLISION_GEO_ROOT}/mesh"
GEMINI335_PRIM_PATH = "/World/Gemini335"
# Orbbec Gemini 335 RGB 光学相机（深度/点云由 Isaac 对该 Camera 渲染生成）
GEMINI335_RGB_CAMERA_PATH = (
    f"{GEMINI335_PRIM_PATH}/Gemini_335/camera_rgb/camera_rgb/Stream_rgb"
)
CAPTURE_LIGHTS_PATH = "/CaptureLights"
FLAT_GRID_PATH = "/FlatGrid"
# 与 bake_graspnet_scene / 采集场景一致的机器人挂载（台面居中）
SCENE_ROBOT_MOUNT_XYZ = (-0.53, 0.18, 0.64)

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
# 地面：深色程序化平面（无 Grid 纹理，Load 不依赖 Nucleus / 额外 USD）
# ---------------------------------------------------------------------------
GROUND_PLANE_PRIM_PATH = "/World/defaultGroundPlane"
GROUND_PLANE_COLOR = (0.10, 0.10, 0.12)  # 深炭灰，略偏冷

# ---------------------------------------------------------------------------
# 机器人挂载
# base_link 原点在 J1 侧基座平台顶面（z=0），不是整台几何中心
# Load 时会量 base_link 下台面 mesh 包围盒，把台面中心对齐桌心，再叠加 FINE 微调
# ---------------------------------------------------------------------------
PLATFORM_DEPTH_BELOW_BASE = 0.04  # base_link z=0 到平台网格底面（量不到时的回退）
ROBOT_MOUNT_FINE_XY = (0.0, 0.0)  # 在自动居中基础上的 xy 微调（米）
# 平台底面相对桌面顶面 (TABLE_TOP_Z) 的 z 微调；0=贴桌面，仅做小范围 ±5cm 调节
ROBOT_MOUNT_FINE_Z = 0.0

# 量不到 bbox 时用于居中的回退：base_link mesh extent 中心 (见 nova_robot_prepared.usda)
ROBOT_FOOTPRINT_CENTER_XY = (0.53, -0.18)

# 双臂 J1_1 / J2_1 在 base_link 下的 x（URDF joint origin）
ARM_J1_X = 0.0
ARM_J2_X = 1.06
# 动态刚体只能用凸包：base_link 凸包在 J1/J2 底座之间会拱起；
# 在 base_link 下补 workspace_fill：顶面 = URDF base_link.stl 的 max_z（整件 base_link 高度）。
# 注意：collisions 为 USD 实例引用，不能在其子树内 Authoring。
WORKSPACE_COLLISION_FILL_NAME = "workspace_fill"
WORKSPACE_COLLISION_INSET_M = 0.02
# 由 mesh_bounds.base_link_stl_z_bounds() 实测；与 PLATFORM_DEPTH_BELOW_BASE 对应 STL min_z
WORKSPACE_BASE_LINK_MESH_TOP_Z = 0.1348

# ---------------------------------------------------------------------------
# 抓取盒：尺寸与 meta 一致；prepare_box_usd.py 会更新 grasp_box_meta.json
# ---------------------------------------------------------------------------
BOX_SIZE_X = 0.173
BOX_SIZE_Y = 0.071
BOX_SIZE_Z = 0.219
BOX_MASS_KG = 0.1
# 盒子底面相对 workspace 顶面的最小间隙（倾斜时按包围盒最低点抬高）
BOX_SURFACE_CLEARANCE_M = 0.005

# 夹取摩擦：盒子碰撞 + 夹爪指尖共用高摩擦物理材质（PhysX combine=max）
GRASP_PHYSICS_MATERIAL_PATH = "/World/PhysicsMaterials/GraspHighFriction"
GRASP_STATIC_FRICTION = 5.0
GRASP_DYNAMIC_FRICTION = 4.0
GRASP_RESTITUTION = 0.0
# 夹爪 prismatic 驱动（USD + Play 后 Articulation 再写一遍）
GRIPPER_DRIVE_STIFFNESS = 50000.0
GRIPPER_DRIVE_DAMPING = 2000.0
GRIPPER_DRIVE_MAX_FORCE = 10000.0

# 臂关节 angular 驱动（USD 默认腕部 stiff≈20/8、肘部 maxForce 偏小 → 升降时低头/弯折）
# (joint_name, stiffness, damping, maxForce)；Load + Play 后 articulation 再写一遍
# 勿过大：过高 stiff/maxForce 会导致 PhysX 不稳定、打断 ROS /joint_states 出流
ARM_DRIVE_SPECS = (
    # 肩 yaw：抑制 ~3° 静差
    ("J1_1_joint", 4000.0, 400.0, 12000.0),
    ("J2_1_joint", 4000.0, 400.0, 12000.0),
    # 肘链：垂直升降时保持姿态
    ("J1_2_joint", 3500.0, 350.0, 4000.0),
    ("J1_3_joint", 3500.0, 350.0, 4000.0),
    ("J1_4_joint", 5000.0, 500.0, 5000.0),
    ("J2_2_joint", 3500.0, 350.0, 4000.0),
    ("J2_3_joint", 3500.0, 350.0, 4000.0),
    ("J2_4_joint", 5000.0, 500.0, 5000.0),
    # 腕部：USD 默认几乎跟不动
    ("J1_5_joint", 8000.0, 800.0, 4000.0),
    ("J1_6_joint", 6000.0, 600.0, 3000.0),
    ("J2_5_joint", 8000.0, 800.0, 4000.0),
    ("J2_6_joint", 6000.0, 600.0, 3000.0),
)
# PhysX maxJointVelocity（deg/s）。原先 J1-4=120、J5-6=240 偏慢；抓取轨迹会拖很久。
ARM_MAX_JOINT_VELOCITY_DEG_S = 360.0
ARM_WRIST_MAX_JOINT_VELOCITY_DEG_S = 540.0

# 下列为 Load 前 UI 占位；mount 高度仍由实测更新
ROBOT_MOUNT_Z = TABLE_TOP_Z + PLATFORM_DEPTH_BELOW_BASE + ROBOT_MOUNT_FINE_Z
# 双臂肩关节中点（世界系）：mount + ((J1_x+J2_x)/2, 0)
ROBOT_ARM_CENTER_XY = (
    SCENE_ROBOT_MOUNT_XYZ[0] + 0.5 * (ARM_J1_X + ARM_J2_X),  # 0.0
    SCENE_ROBOT_MOUNT_XYZ[1],  # 0.18
)
# 盒心相对肩线中点的 xy 偏移（世界系）；负 x = 朝机器人/base 靠近，便于顶抓 IK 可达
BOX_CENTER_OFFSET_XY = (-0.18, 0.0)
# 盒心放在双臂工作区略靠机器人一侧（勿用桌面几何原点 (0,0)：相对肩线偏 -Y）
DEFAULT_BOX_CENTER = (
    ROBOT_ARM_CENTER_XY[0] + BOX_CENTER_OFFSET_XY[0],
    ROBOT_ARM_CENTER_XY[1] + BOX_CENTER_OFFSET_XY[1],
    0.9,
)
DEFAULT_BOX_POSE_RPY = (0.0, 90.0, 0.0)  # roll, pitch, yaw (deg)

# 数据采集随机范围默认（UI 初始值 / BoxPoseRange 回退）；绕默认盒心小范围抖动
COLLECT_TX_RANGE = (-0.33, -0.03)
COLLECT_TY_RANGE = (0.03, 0.33)  # 以 y=0.18 为中心 ±0.15
COLLECT_TZ_RANGE = (0.9, 0.9)
COLLECT_ROLL_RANGE = (0.0, 0.0)
COLLECT_PITCH_RANGE = (0.0, 0.0)
COLLECT_YAW_RANGE = (-45.0, 45.0)

# ---------------------------------------------------------------------------
# 相机默认分辨率
# ---------------------------------------------------------------------------
DEFAULT_CAMERA_WIDTH = 1280
DEFAULT_CAMERA_HEIGHT = 720

# 相机：cam0 = Gemini335；cam1/cam2 = 腕部 RSD455
# prim_path 为绝对路径；prim_suffix 相对 ROBOT_PRIM_PATH（二者择一）
CAMERA_DEFS = (
    {
        "key": "cam0",
        "label": "Camera 0 (Gemini 335)",
        "prim_path": GEMINI335_RGB_CAMERA_PATH,
        "frame_id": "cam0",
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
JOINT_COMMAND_TOPIC = "/joint_command"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
TF_WORLD_FRAME = "world"

# ---------------------------------------------------------------------------
# 抓取盒位姿约定（Scheme A / 数据集 GT）
# ---------------------------------------------------------------------------
BOX_POSE_FRAME = "base_link"
BASE_LINK_PATH = f"{ROBOT_PRIM_PATH}/base_link"
BOX_POSE_TOPIC = "/box_pose"
BOX_POSE_PUBLISH_HZ = 1.0
# 外部 GraspNet 等发布的抓取候选（geometry_msgs/PoseArray）
GRASP_POSE_ARRAY_TOPIC = "/graspnet/best_grasp"
GRASP_POSE_PUBLISH_HZ = 1.0
# nova_grasp_moveit 抓取状态
GRASP_STATUS_TOPIC = "/nova_grasp/status"
GRASP_ARM_AUTO = "auto"
GRASP_ARM_J1 = "arm1"
GRASP_ARM_J2 = "arm2"

# 臂链关节名（与 nova_robot USD 一致）
ARM1_JOINTS = (
    "J1_1_joint",
    "J1_2_joint",
    "J1_3_joint",
    "J1_4_joint",
    "J1_5_joint",
    "J1_6_joint",
)
ARM2_JOINTS = (
    "J2_1_joint",
    "J2_2_joint",
    "J2_3_joint",
    "J2_4_joint",
    "J2_5_joint",
    "J2_6_joint",
)
ARM1_GRIPPER_JOINTS = ("J1_7_joint", "J1_8_joint")
ARM2_GRIPPER_JOINTS = ("J2_7_joint", "J2_8_joint")
ARM1_EE_LINK = "J1_6"
ARM2_EE_LINK = "J2_6"
ARM1_GRIPPER_FINGERS = ("J1_7", "J1_8")
ARM2_GRIPPER_FINGERS = ("J2_7", "J2_8")
ARM1_BASE_LINK = "J1_1"
ARM2_BASE_LINK = "J2_1"
ARM1_BASE_PATH = f"{ROBOT_PRIM_PATH}/{ARM1_BASE_LINK}"
ARM2_BASE_PATH = f"{ROBOT_PRIM_PATH}/{ARM2_BASE_LINK}"

# 夹爪 prismatic 目标（米）：两指反向极限，对中开合（Isaac 同向轴时同号会整侧平移）
# J*_7 ∈ [-0.04, 0.02]，J*_8 ∈ [-0.02, 0.04]
GRIPPER1_OPEN = (0.02, -0.02)
GRIPPER1_CLOSED = (-0.04, 0.04)
GRIPPER2_OPEN = (0.02, -0.02)
GRIPPER2_CLOSED = (-0.04, 0.04)

# Load 时双臂绕底座整体摆开（度）：只转 J*_1，其余关节 0
# 两臂同向 -90°（世界系朝同一侧），避免一上一下
DEFAULT_JOINT_ANGLES_DEG = {
    "J1_1_joint": -90.0,
    "J2_1_joint": -90.0,
}

# 双臂复位 / 避让（复位：base_link 下夹爪 xyzrpy；避让仍为关节角）
DEFAULT_ARM1_RESET_XYZRPY = (0.20, 0.00, 0.50, 0.0, -77.0, 0.0)  # m, deg
DEFAULT_ARM2_RESET_XYZRPY = (0.86, 0.00, 0.50, 0.0, -77.0, 0.0)
DEFAULT_ARM1_PARK_JOINTS = (-1.5708, -0.55, 0.75, 0.0, -1.35, 0.0)  # 同向 ≈-90°
DEFAULT_ARM2_PARK_JOINTS = (-1.5708, -0.55, 0.75, 0.0, -1.35, 0.0)

# 数据采集默认输出目录（相对扩展根）
DATA_LOG_DIRNAME = "data_log"
DEFAULT_COLLECT_SAMPLES = 20
DEFAULT_SETTLE_STEPS = 10
DEFAULT_RENDER_SUBFRAMES = 8
