#!/usr/bin/env python3
"""
手势识别算法模块
提供独立于ROS的手势识别功能
"""

import math
import numpy as np
from collections import deque

# 手部连接关系常量
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),  # 拇指
    (0, 5), (5, 6), (6, 7), (7, 8),  # 食指
    (0, 9), (9, 10), (10, 11), (11, 12),  # 中指
    (0, 13), (13, 14), (14, 15), (15, 16),  # 无名指
    (0, 17), (17, 18), (18, 19), (19, 20),  # 小拇指
    (5, 9), (9, 13), (13, 17)  # 手掌
]


def vector_2d_angle(v1, v2):
    """
    计算两个二维向量v1和v2之间的夹角（0~180度）。
    若计算异常则返回65535.
    """
    v1_x, v1_y = v1
    v2_x, v2_y = v2
    try:
        dot = v1_x * v2_x + v1_y * v2_y
        norm1 = math.sqrt(v1_x**2 + v1_y**2)
        norm2 = math.sqrt(v2_x**2 + v2_y**2)
        angle_ = math.degrees(math.acos(dot / (norm1 * norm2)))
    except:
        angle_ = 65535.
    if angle_ > 180.:
        angle_ = 65535.
    return angle_


def vector_3d_angle(v1, v2):
    """
    计算两个三维向量v1和v2之间的夹角（0~180度）。
    若计算异常则返回65535.
    """
    try:
        dot = np.dot(v1, v2)
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        angle_ = math.degrees(math.acos(dot / (norm1 * norm2)))
    except:
        angle_ = 65535.
    if angle_ > 180.:
        angle_ = 65535.
    return angle_


def hand_angle(hand_landmarks):
    """
    计算手指关节的角度列表：
    返回 [拇指, 食指, 中指, 无名指, 小拇指] 各自的角度。
    每个角度通过对比指根与指尖的方向得到。
    """
    angle_list = []
    # 拇指
    angle = vector_2d_angle(
        ((hand_landmarks[0][0] - hand_landmarks[2][0]), (hand_landmarks[0][1] - hand_landmarks[2][1])),
        ((hand_landmarks[3][0] - hand_landmarks[4][0]), (hand_landmarks[3][1] - hand_landmarks[4][1]))
    )
    angle_list.append(angle)

    # 食指
    angle = vector_2d_angle(
        ((hand_landmarks[0][0] - hand_landmarks[6][0]), (hand_landmarks[0][1] - hand_landmarks[6][1])),
        ((hand_landmarks[7][0] - hand_landmarks[8][0]), (hand_landmarks[7][1] - hand_landmarks[8][1]))
    )
    angle_list.append(angle)

    # 中指
    angle = vector_2d_angle(
        ((hand_landmarks[0][0] - hand_landmarks[10][0]), (hand_landmarks[0][1] - hand_landmarks[10][1])),
        ((hand_landmarks[11][0] - hand_landmarks[12][0]), (hand_landmarks[11][1] - hand_landmarks[12][1]))
    )
    angle_list.append(angle)

    # 无名指
    angle = vector_2d_angle(
        ((hand_landmarks[0][0] - hand_landmarks[14][0]), (hand_landmarks[0][1] - hand_landmarks[14][1])),
        ((hand_landmarks[15][0] - hand_landmarks[16][0]), (hand_landmarks[15][1] - hand_landmarks[16][1]))
    )
    angle_list.append(angle)

    # 小拇指
    angle = vector_2d_angle(
        ((hand_landmarks[0][0] - hand_landmarks[18][0]), (hand_landmarks[0][1] - hand_landmarks[18][1])),
        ((hand_landmarks[19][0] - hand_landmarks[20][0]), (hand_landmarks[19][1] - hand_landmarks[20][1]))
    )
    angle_list.append(angle)

    return angle_list


def h_gesture(angle_list, palm_angle, palm_facing_angle_degrees, fingers_pointing_toward_screen, is_all_fingers_open, history_x,
              hand_local, normal, move_thr=30, wave_state=None, wave_position_history=None, min_wave_changes=3):
    """
    根据手指角度列表及手掌、手指朝向信息、历史位置等，判断手势名称。

    angle_list：手指角度列表
    palm_angle：手掌角度（平面内）
    palm_facing_angle_degrees：手掌朝向角度(0=正对, 90=侧对)
    fingers_pointing_toward_screen：手指是否朝向屏幕（z值对比）
    is_all_fingers_open：是否所有手指打开
    history_x：手腕x坐标的历史，用于检测水平移动（如Wave）
    hand_local：当前手的21个关键点坐标(x,y,z)
    normal：手掌法线向量
    move_thr：Wave手势检测移动阈值
    wave_state：Wave手势检测状态 {'direction': None, 'change_count': 0}
    wave_position_history：Wave手势位置历史记录
    min_wave_changes：Wave手势识别所需的最小方向变化次数（默认3次）

    返回：gesture_str（手势名称字符串）
    """
    thr_angle = 65.
    thr_angle_s = 49.
    gesture_str = ""

    def finger_straight(a): return a < thr_angle_s
    def finger_bent(a): return a > thr_angle

    # 手指伸直/弯曲状态判定
    thumb_straight = finger_straight(angle_list[0])
    index_straight = finger_straight(angle_list[1])
    middle_straight = finger_straight(angle_list[2])
    ring_straight = finger_straight(angle_list[3])
    pinky_straight = finger_straight(angle_list[4])

    thumb_bent = finger_bent(angle_list[0])
    index_bent = finger_bent(angle_list[1])
    middle_bent = finger_bent(angle_list[2])
    ring_bent = finger_bent(angle_list[3])
    pinky_bent = finger_bent(angle_list[4])

    # 手部关键点坐标（用于判断相对位置和距离）
    wrist_x, wrist_y, wrist_z = hand_local[0]
    thumb_tip_x, thumb_tip_y, thumb_tip_z = hand_local[4]
    index_tip_x, index_tip_y, index_tip_z = hand_local[8]

    if 65535. not in angle_list:

        # Wave与某些条件下的Handshake、Palm_up、Stop依次判断
        if is_all_fingers_open and len(history_x) >= 2:
            # 检测Wave手势：需要来回移动（至少3次方向变化）
            is_wave = False
            if wave_state is not None and wave_position_history is not None and len(wave_position_history) >= 7:
                # 分析最近的移动方向
                positions = list(wave_position_history)
                for i in range(len(positions) - 3):
                    move1 = positions[i+1] - positions[i]
                    move2 = positions[i+2] - positions[i+1]
                    move3 = positions[i+3] - positions[i+2]
                    # 两次移动都必须超过阈值
                    if abs(move1) > move_thr and abs(move2) > move_thr and abs(move3) > move_thr:
                        # 检测方向变化
                        if move1 > 0 and move2 < 0 and move3 > 0:
                            # 从左向右移动，然后从右向左
                            wave_state['change_count'] += 1
                            is_wave = True
                            break
                        elif move1 < 0 and move2 > 0 and move3 < 0:
                            # 从右向左移动，然后从左向右
                            wave_state['change_count'] += 1
                            is_wave = True
                            break

                # 需要至少3次方向变化才认为是Wave
                if is_wave and wave_state['change_count'] >= 1:
                    gesture_str = "Wave"
                    # 清空状态,避免重复识别
                    wave_state['change_count'] = 0
                    wave_position_history.clear()
                else:
                    # 无明显来回移动，根据palm_facing_angle区分Stop和Handshake
                    # Stop:手掌正对/背对摄像头(palm_facing_angle_degrees较小)
                    # Handshake:手掌侧对摄像头(palm_facing_angle_degrees较大)
                    if palm_facing_angle_degrees < 30:
                        gesture_str = "Stop"
                    else:
                        gesture_str = "Handshake"
            else:
                # 历史数据不足，不识别为Wave，根据手掌朝向判断为Stop或Handshake
                if palm_facing_angle_degrees < 30:
                    gesture_str = "Stop"
                else:
                    gesture_str = "Handshake"

        # Heart_single：拇指、食指弯曲状态与关节距离判断
        elif thumb_straight and index_bent and middle_bent and ring_bent and pinky_bent:
            thumb_ip_x, thumb_ip_y = hand_local[3][0], hand_local[3][1]
            index_dip_x, index_dip_y = hand_local[7][0], hand_local[7][1]
            dist = np.linalg.norm(np.array([thumb_ip_x, thumb_ip_y]) - np.array([index_dip_x, index_dip_y]))
            if dist < 50:
                gesture_str = "Heart_single"
            elif thumb_tip_y < wrist_y:
                gesture_str = "like"
            else:
                gesture_str = "dislike"

        # ILY手势判断
        elif thumb_straight and index_straight and pinky_straight and middle_bent and ring_bent:
            gesture_str = "ILY"

        # Insult手势判断
        elif middle_straight and thumb_bent and index_bent and ring_bent and pinky_bent and palm_facing_angle_degrees > 90:
            gesture_str = "Insult"

        # one
        elif index_straight and thumb_bent and middle_bent and ring_bent and pinky_bent:
            gesture_str = "one"

        # fist
        elif thumb_bent and index_bent and middle_bent and ring_bent and pinky_bent:
            gesture_str = "fist"

        # peace
        elif index_straight and middle_straight and thumb_bent and ring_bent and pinky_bent:
            gesture_str = "peace"

        # call
        elif thumb_straight and pinky_straight and index_bent and middle_bent and ring_bent:
            gesture_str = "call"

        # ok
        elif index_bent and middle_straight and ring_straight and pinky_straight:
            gesture_str = "ok"

        # four
        elif thumb_bent and index_straight and middle_straight and ring_straight and pinky_straight:
            gesture_str = "four"

        # three
        elif thumb_straight and index_straight and middle_straight and ring_bent and pinky_bent:
            gesture_str = "three"

        # three2
        elif index_straight and middle_straight and ring_straight and thumb_bent and pinky_bent:
            gesture_str = "three2"

    return gesture_str


class GestureDetector:
    """
    手势识别类
    封装手势识别算法，独立于ROS
    """

    def __init__(self, move_thr=50, min_wave_changes=3):
        """
        初始化手势识别器

        Args:
            move_thr: Wave手势检测移动阈值
            min_wave_changes: Wave手势识别所需的最小方向变化次数（默认3次）
        """
        self.move_thr = move_thr
        self.min_wave_changes = min_wave_changes
        self.gesture_history = {}
        self.thr_angle_s = 49.
        # Wave手势检测的历史记录，存储最近的位置
        # 增加历史记录长度以支持3次方向变化检测（至少需要7个点）
        self.wave_position_history = {}  # {hand_idx: deque(maxlen=20)}
        # Wave手势检测的状态记录
        self.wave_state = {}  # {hand_idx: {'direction': None, 'change_count': 0}}

    def process_hand_landmarks(self, hand_landmarks, hand_idx=0, image_width=640, image_height=480, **kwargs):
        """
        处理手部关键点并识别手势

        Args:
            hand_landmarks: MediaPipe返回的手部关键点列表（21个点，每个点有x,y,z坐标）
            hand_idx: 手的索引（用于多手追踪）
            image_width: 图像宽度，用于坐标转换
            image_height: 图像高度，用于坐标转换
            **kwargs: 其他参数,如hand_info={'handedness': 'Left'/'Right'}

        Returns:
            dict: 包含手势名称和相关信息的字典
            {
                'gesture': '手势名称',
                'hand_local': [(x,y,z), ...],  # 21个关键点
                'angle_list': [拇指角, 食指角, ...],  # 5个角度
                'palm_angle': 手掌角度,
                'palm_facing_angle': 手掌朝向角度,
                'is_all_fingers_open': 是否所有手指打开
            }
        """
        # 提取手部关键点坐标
        hand_local = []
        for lm in hand_landmarks:
            x = int(lm.x * image_width)
            y = int(lm.y * image_height)
            z = lm.z
            hand_local.append((x, y, z))

        if not hand_local:
            return None

        # 基于2D坐标计算手指角度
        hand_local_2d = [(p[0], p[1]) for p in hand_local]
        angle_list = hand_angle(hand_local_2d)

        # 计算palm_angle
        wrist = hand_local[0]
        middle_finger_base = hand_local[9]
        vector_palm = (middle_finger_base[0] - wrist[0], middle_finger_base[1] - wrist[1])
        palm_angle = math.degrees(math.atan2(vector_palm[1], vector_palm[0]))
        if palm_angle < 0:
            palm_angle += 360

        is_all_fingers_open = all(a < self.thr_angle_s for a in angle_list)

        # 判断手指是否指向屏幕（通过z坐标判断）
        wrist_3d = np.array(hand_local[0])
        fingers_z = [hand_local[i][2] for i in [8, 12, 16, 20]]
        fingers_pointing_toward_screen = all(z < wrist_3d[2] for z in fingers_z)

        # 计算手掌法线normal
        wrist_3d = np.array(hand_local[0])
        index_mcp_3d = np.array(hand_local[5])
        pinky_mcp_3d = np.array(hand_local[17])
        v1 = index_mcp_3d - wrist_3d
        v2 = pinky_mcp_3d - wrist_3d
        normal = np.cross(v1, v2)
        normal_norm = np.linalg.norm(normal)
        if normal_norm != 0:
            normal = normal / normal_norm
        else:
            normal = np.array([0, 0, 1])

        # 获取手的左右手信息(如果提供)
        hand_info = kwargs.get('hand_info', {})
        hand_label = hand_info.get('handedness', None)

        # 使用2D平面的手指宽度来判断手掌朝向
        # Stop:手掌正对摄像头,手指指尖在2D平面上横向距离较大
        # Handshake:手掌侧对摄像头,手指指尖在2D平面上横向距离较小
        thumb_tip = np.array([hand_local[4][0], hand_local[4][1]])
        index_tip = np.array([hand_local[8][0], hand_local[8][1]])
        middle_tip = np.array([hand_local[12][0], hand_local[12][1]])
        ring_tip = np.array([hand_local[16][0], hand_local[16][1]])
        pinky_tip = np.array([hand_local[20][0], hand_local[20][1]])

        # 计算最外侧两个指尖(拇指和小拇指)的x坐标距离
        hand_width_2d = abs(hand_local[4][0] - hand_local[20][0])
        # 计算手腕到中指尖的y距离(作为归一化)
        hand_height_2d = abs(hand_local[0][1] - hand_local[12][1])

        # 归一化的手部宽高比,用于判断手掌朝向
        # 正对摄像头时比例较大,侧对时比例较小
        palm_facing_ratio = hand_width_2d / max(hand_height_2d, 1) if hand_height_2d > 0 else 0

        # 转换为角度(便于与原逻辑兼容)
        # 比例越大(正对),角度越小; 比例越小(侧对),角度越大
        palm_facing_angle_degrees = max(0.0, 90.0 - palm_facing_ratio * 90.0)

        # 使用hand_label作为键来区分左右手的状态
        hand_key = hand_label if hand_label else f"hand_{hand_idx}"

        # 更新历史x坐标用于Wave判定
        if hand_key not in self.gesture_history:
            self.gesture_history[hand_key] = deque(maxlen=2)
        self.gesture_history[hand_key].append(int(wrist[0]))

        # 更新Wave手势检测的位置历史
        if hand_key not in self.wave_position_history:
            self.wave_position_history[hand_key] = deque(maxlen=20)
        self.wave_position_history[hand_key].append(int(wrist[0]))

        # 更新Wave状态记录
        if hand_key not in self.wave_state:
            self.wave_state[hand_key] = {'direction': None, 'change_count': 0}

        # 获取手势名称
        gesture_str = h_gesture(
            angle_list,
            palm_angle,
            palm_facing_angle_degrees,
            fingers_pointing_toward_screen,
            is_all_fingers_open,
            self.gesture_history[hand_key],
            hand_local,
            normal,
            move_thr=self.move_thr,
            wave_state=self.wave_state[hand_key],
            wave_position_history=self.wave_position_history[hand_key],
            min_wave_changes=self.min_wave_changes
        )

        return {
            'gesture': gesture_str,
            'hand_local': hand_local,
            'angle_list': angle_list,
            'palm_angle': palm_angle,
            'palm_facing_angle': palm_facing_angle_degrees,
            'is_all_fingers_open': is_all_fingers_open
        }

    def reset_history(self):
        """重置手势历史记录"""
        self.gesture_history = {}
        self.wave_position_history = {}
        self.wave_state = {}
