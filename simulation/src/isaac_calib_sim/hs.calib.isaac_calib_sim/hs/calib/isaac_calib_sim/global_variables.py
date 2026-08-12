# -*- coding: utf-8 -*-
"""Extension constants (English UI labels only — avoid CJK mojibake in Isaac)."""

EXTENSION_TITLE = "Hs Calib Isaac Calib Sim"
EXTENSION_DESCRIPTION = (
    "Single camera + switchable calibration boards; animated viewpoints"
)

ROOT_PRIM = "/World/CalibSim"
GROUND_PATH = f"{ROOT_PRIM}/Ground"
TABLE_PATH = f"{ROOT_PRIM}/Table"
BOARD_PATH = f"{ROOT_PRIM}/CalibBoard"
BOARD_MESH_PATH = f"{ROOT_PRIM}/CalibBoard/Mesh"
BOARD_LOOKAT_PATH = f"{ROOT_PRIM}/BoardCenter"
CAMERA_LINK_PATH = f"{ROOT_PRIM}/camera_link"
CAMERA_OPTICAL_PATH = f"{ROOT_PRIM}/camera_link/camera_optical_frame"
CAMERA_PRIM_PATH = f"{ROOT_PRIM}/camera_link/camera_optical_frame/camera"
LIGHT_PATH = f"{ROOT_PRIM}/DomeLight"
GRAPH_PATH = f"{ROOT_PRIM}/ROS2CameraGraph"

OPTICAL_FRAME_RPY_DEG = (-90.0, 0.0, -90.0)

DEFAULT_WORLD_FRAME = "map"
DEFAULT_CAMERA_LINK_FRAME = "camera_link"
DEFAULT_CAMERA_OPTICAL_FRAME = "camera_optical_frame"

ROS_NODE_NAMESPACE = "calib_sim"

DEFAULT_IMAGE_WIDTH = 1280
DEFAULT_IMAGE_HEIGHT = 720
DEFAULT_HFOV_DEG = 70.0
# ROS / timeline publish rate (physics disabled — no collision)
DEFAULT_STREAM_FPS = 30.0

TABLE_TOP_Z = 0.75
TABLE_SIZE_XY = (1.2, 1.0)
TABLE_THICKNESS = 0.04
BOARD_CENTER_XY = (0.0, 0.0)

DEFAULT_BOARD_TYPE = "chessboard"
# Trihedral / suite use inner-corner count; planar chess default stays 9×6 via UI.
DEFAULT_SQUARES_X = 8
DEFAULT_SQUARES_Y = 8
DEFAULT_SQUARE_LENGTH_M = 0.025
DEFAULT_MARKER_LENGTH_M = 0.018

DEFAULT_DIST_MIN = 0.2
DEFAULT_DIST_MAX = 1.1
DEFAULT_ELEV_MIN_DEG = 12.0
DEFAULT_ELEV_MAX_DEG = 72.0
DEFAULT_AZIM_RATE_DEG = 22.0
DEFAULT_DIST_PERIOD_S = 6.5
DEFAULT_ELEV_PERIOD_S = 9.7
# Board-plane look-at wander (m) + camera roll — drives capture diversity
DEFAULT_LOOKAT_OFFSET_M = 0.08
DEFAULT_LOOKAT_PERIOD_S = 7.3
DEFAULT_ROLL_AMP_DEG = 12.0
DEFAULT_ROLL_PERIOD_S = 5.5

# Trihedral: keep camera inside the open-corner viewing cone so all 3 faces
# stay in frame. Body-diagonal ≈ 35° elev; avoid near-nadir (lost side faces)
# and very low elev (lost top face). Mild look-at wander only.
TRI_DIST_MIN = 0.55
TRI_DIST_MAX = 0.80
TRI_ELEV_MIN_DEG = 32.0
TRI_ELEV_MAX_DEG = 52.0
TRI_AZIM_RATE_DEG = 6.0
TRI_DIST_PERIOD_S = 9.0
TRI_ELEV_PERIOD_S = 7.0
TRI_LOOKAT_OFFSET_M = 0.012
TRI_LOOKAT_PERIOD_S = 8.0
TRI_ROLL_AMP_DEG = 2.5
TRI_ROLL_PERIOD_S = 8.0

# English-only labels (Isaac UI fonts often break CJK)
BOARD_TYPE_LABELS = (
    ("chessboard", "Chessboard"),
    ("circles_symmetric", "Circles grid (symmetric)"),
    ("circles_asymmetric", "Circles grid (asymmetric)"),
    ("charuco", "ChArUco"),
    ("aruco_grid", "ArUco / AprilTag grid"),
    ("trihedral_chess", "Trihedral — chessboard (3 faces)"),
    ("trihedral_charuco", "Trihedral — ChArUco (3 faces)"),
    ("trihedral_aruco", "Trihedral — ArUco / Tag (3 faces)"),
)
