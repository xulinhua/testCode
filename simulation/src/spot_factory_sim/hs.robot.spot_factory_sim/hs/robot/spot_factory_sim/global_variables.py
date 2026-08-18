# -*- coding: utf-8 -*-
"""扩展常量与默认 prim / 话题名。"""

EXTENSION_TITLE = "Hs Robot Spot Factory Sim"
EXTENSION_DESCRIPTION = (
    "Factory warehouse: Spot or Carter cart + ROS2 camera/IMU/RTX lidar + cmd_vel + odom/tf/joint_states"
)

# Robot selection (UI ComboBox)
ROBOT_TYPE_SPOT = "spot"
ROBOT_TYPE_CART = "cart"
ROBOT_TYPE_LABELS = (
    (ROBOT_TYPE_SPOT, "Spot (dog)"),
    (ROBOT_TYPE_CART, "Cart (Carter)"),
)
DEFAULT_ROBOT_TYPE = ROBOT_TYPE_SPOT

# Stage prims
WORLD_PATH = "/World"
FACTORY_PRIM_PATH = "/World/Factory"
SPOT_PRIM_PATH = "/World/Spot"
CART_PRIM_PATH = "/World/Cart"
CLUTTER_ROOT_PATH = "/World/Clutter"
ENABLE_FACTORY_CLUTTER = True  # Load 时在过道再放带碰撞的桶/箱，便于障碍建图
SPOT_BODY_CANDIDATES = ("body", "base", "base_link", "torso")
CART_BODY_CANDIDATES = ("chassis_link", "chassis", "base_link", "base", "body")
CAMERA_PRIM_NAME = "front_cam"
IMU_PRIM_NAME = "imu"
LIDAR_PRIM_NAME = "lidar"
# 解析后为 /World/<Robot>/<body>/front_cam 或回退到机器人根
CAMERA_PRIM_PATH_FALLBACK = f"{SPOT_PRIM_PATH}/{CAMERA_PRIM_NAME}"
IMU_PRIM_PATH_FALLBACK = f"{SPOT_PRIM_PATH}/{IMU_PRIM_NAME}"
LIDAR_PRIM_PATH_FALLBACK = f"{SPOT_PRIM_PATH}/{LIDAR_PRIM_NAME}"

# Isaac Nucleus 相对路径（经 get_assets_root_path）
# 官方无独立 Factory 名；多货架工厂库房用 Simple_Warehouse/full_warehouse.usd
FACTORY_USD_REL = "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
SPOT_USD_REL = "/Isaac/Robots/BostonDynamics/spot/spot.usd"
CART_USD_REL = "/Isaac/Robots/NVIDIA/Carter/carter_v1.usd"

# Spot 初始位姿：大仓库主过道（full_warehouse 货架很多，放在中央空地）
SPOT_SPAWN_POS = (0.0, 0.0, 0.55)
SPOT_SPAWN_ORI = (1.0, 0.0, 0.0, 0.0)  # wxyz
# Carter V1：略抬高，避免轮子穿地
CART_SPAWN_POS = (0.0, 0.0, 0.15)
CART_SPAWN_ORI = (1.0, 0.0, 0.0, 0.0)  # wxyz

# Load 后主视口：Perspective，从过道斜上方俯看（避开误切到 front_cam）
VIEWPORT_EYE = (12.0, -14.0, 8.0)
VIEWPORT_TARGET = (0.0, 0.0, 0.35)

# 面板遥控默认速度
UI_DRIVE_LINEAR = 0.6
UI_DRIVE_ANGULAR = 0.8
# Carter V1 差速（Isaac test_carter_v1 / OG：radius=0.24, base=0.56）
CART_WHEEL_RADIUS = 0.24
CART_WHEEL_BASE = 0.56
CART_WHEEL_DOF_NAMES = ("left_wheel", "right_wheel")

# 物理：策略训练是 500Hz，建图交互不必跟那么高。
# 200Hz + decimation=4 → 策略仍约 50Hz；30FPS 视口每帧约 7 步，能进追赶上限。
PHYSICS_HZ = 200
POLICY_CONTROL_HZ = 50
PHYSICS_DT = 1.0 / float(PHYSICS_HZ)
POLICY_DECIMATION = max(1, int(round(PHYSICS_HZ / float(POLICY_CONTROL_HZ))))
RENDERING_DT = 1.0 / 30.0
# 每渲染帧最多追赶的物理步数，防止低 FPS 时一帧算几百步
MAX_PHYSICS_STEPS_PER_FRAME = 8

# 相机默认分辨率（color / depth / points 默认均发布）
DEFAULT_IMAGE_WIDTH = 640
DEFAULT_IMAGE_HEIGHT = 480
DEFAULT_HORIZONTAL_FOV_DEG = 70.0

# Camera on body (USD prim): looks along -Z of Camera schema.
# RPY(90,0,-90) maps that look direction onto robot +X (前方), camera up onto +Z.
# ROS: cam_0_link 与 base 同轴向（X前 Y左 Z上）；optical = link * rpy(-90,0,-90)，
# 使光学系 +Z 同样指向前方，与渲染朝向一致。
CAMERA_LOCAL_XYZ = (0.35, 0.0, 0.12)  # Spot：相对 body，略靠前上方
CAMERA_LOCAL_RPY_DEG = (90.0, 0.0, -90.0)
CART_CAMERA_LOCAL_XYZ = (0.30, 0.0, 0.35)  # Carter chassis 前方偏上

# IMU / 激光雷达相对 body：REP-103 机体式（X 前 Y 左 Z 上），与 base_link 同向
IMU_LOCAL_XYZ = (0.0, 0.0, 0.04)
CART_IMU_LOCAL_XYZ = (0.0, 0.0, 0.20)
IMU_SENSOR_PERIOD = 0.01  # 100 Hz
LIDAR_LOCAL_XYZ = (0.10, 0.0, 0.22)  # Spot：背部上方，避开前视相机
CART_LIDAR_LOCAL_XYZ = (0.0, 0.0, 0.45)
LIDAR_CONFIG = "Example_Rotary"  # Isaac RTX 3D 旋转雷达配置名
LIDAR_FRAME_SKIP = 1  # 隔帧发布，减轻 RTX 负担

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
FRAME_IMU = "imu_link"
FRAME_LIDAR = "lidar_link"

TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_ODOM = "/odom"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_COLOR = "/cam_0/color/image_raw"
TOPIC_DEPTH = "/cam_0/depth/image_raw"
TOPIC_POINTS = "/cam_0/depth/points"
TOPIC_CAMERA_INFO = "/cam_0/depth/camera_info"
TOPIC_IMU = "/imu"
TOPIC_LIDAR_POINTS = "/lidar/points"
TOPIC_LIDAR_SCAN = "/scan"

GRAPH_ROOT = "/SpotFactorySim"
CAMERA_GRAPH_PATH = f"{GRAPH_ROOT}/CameraGraph"
JOINT_GRAPH_PATH = f"{GRAPH_ROOT}/JointStateGraph"
IMU_GRAPH_PATH = f"{GRAPH_ROOT}/ImuGraph"
LIDAR_GRAPH_PATH = f"{GRAPH_ROOT}/LidarGraph"
