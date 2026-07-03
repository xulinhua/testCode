# -*- coding: utf-8 -*-
"""扩展默认资源名与机器人参数（可在 UI 覆盖）。"""

# 场景 USD：空表示暂不加载；就绪后填入 data/scenes/ 下文件名
DEFAULT_SCENE_USD = ""
DEFAULT_ROBOT_USD = "HAD2503-D-DEX1.usd"

SCENE_PRIM_PATH = "/World/scene"
ROBOT_PRIM_PATH = "/World/robot"

# 左臂抓取链（不含夹爪）
LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
)
LEFT_FINGER_JOINTS = (
    "left_dex1_finger_joint_1",
    "left_dex1_finger_joint_2",
)
LEFT_EE_LINK_NAME = "left_wrist_roll_link"

# 差速底盘（continuous 轮关节）
LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"
DEFAULT_WHEEL_RADIUS_M = 0.1
DEFAULT_WHEEL_BASE_M = 0.5

# 抓取默认话题
DEFAULT_SUB_CMD_VEL = "/box_grasp/cmd_vel"

# 夹爪关节目标（弧度，按实际限位再调）
GRIPPER_OPEN = (0.0, 0.0)
GRIPPER_CLOSED = (0.5, 0.5)
