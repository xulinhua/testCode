#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HAD2503 调试 UI：底盘差速 + 双臂关节。"""

from __future__ import annotations

import math
import sys
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as MsgDuration

from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

LEFT_ARM = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
]
RIGHT_ARM = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]
LEFT_GRIP = ["left_dex1_finger_joint_1", "left_dex1_finger_joint_2"]
RIGHT_GRIP = ["right_dex1_finger_joint_1", "right_dex1_finger_joint_2"]
TORSO = [
    "ankle_pitch_joint",
    "knee_pitch_joint",
    "hip_pitch_joint",
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "head_yaw_joint",
    "head_pitch_joint",
]

DEG = 180.0 / math.pi
RAD = math.pi / 180.0


class HadRosBridge(Node):
    def __init__(self) -> None:
        super().__init__("had2503_debug_ui")
        self._joints: Dict[str, float] = {}
        self._lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self._cmd_pub = self.create_publisher(
            Twist, "/diff_drive_controller/cmd_vel_unstamped", 10
        )
        self._traj_pubs = {
            "left_arm": self.create_publisher(
                JointTrajectory, "/left_arm_controller/joint_trajectory", 10
            ),
            "right_arm": self.create_publisher(
                JointTrajectory, "/right_arm_controller/joint_trajectory", 10
            ),
            "left_grip": self.create_publisher(
                JointTrajectory, "/left_gripper_controller/joint_trajectory", 10
            ),
            "right_grip": self.create_publisher(
                JointTrajectory, "/right_gripper_controller/joint_trajectory", 10
            ),
            "torso": self.create_publisher(
                JointTrajectory, "/torso_controller/joint_trajectory", 10
            ),
        }

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            for n, p in zip(msg.name, msg.position):
                self._joints[n] = float(p)

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._joints)

    def publish_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def publish_traj(
        self,
        key: str,
        joint_names: List[str],
        positions_rad: List[float],
        duration_sec: float = 1.0,
    ) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions_rad]
        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)
        point.time_from_start = MsgDuration(sec=sec, nanosec=nsec)
        traj.points = [point]
        self._traj_pubs[key].publish(traj)


class ArmPanel(QWidget):
    def __init__(
        self,
        title: str,
        joint_names: List[str],
        traj_key: str,
        use_deg: bool = True,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.joint_names = joint_names
        self.traj_key = traj_key
        self.use_deg = use_deg
        self.dirty = False
        layout = QVBoxLayout(self)

        head = QHBoxLayout()
        head.addWidget(QLabel(f"<b>{title}</b>"))
        self.realtime = QCheckBox("实时模式")
        self.realtime.setToolTip("勾选后改目标会立刻下发")
        head.addWidget(self.realtime)
        head.addStretch(1)
        layout.addLayout(head)

        grid = QGridLayout()
        grid.addWidget(QLabel("关节"), 0, 0)
        grid.addWidget(QLabel("当前"), 0, 1)
        grid.addWidget(QLabel("目标"), 0, 2)
        self.cur_labels: List[QLabel] = []
        self.spins: List[QDoubleSpinBox] = []
        for i, name in enumerate(joint_names):
            short = name.replace("_joint", "").replace("left_", "L_").replace("right_", "R_")
            grid.addWidget(QLabel(short), i + 1, 0)
            cur = QLabel("—")
            cur.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            cur.setMinimumWidth(64)
            spin = QDoubleSpinBox()
            spin.setDecimals(3 if not use_deg else 2)
            if use_deg:
                spin.setRange(-360.0, 360.0)
                spin.setSingleStep(1.0)
                spin.setSuffix(" °")
            else:
                spin.setRange(-0.1, 0.1)
                spin.setSingleStep(0.001)
                spin.setSuffix(" m")
            spin.valueChanged.connect(self._on_edit)
            self.cur_labels.append(cur)
            self.spins.append(spin)
            grid.addWidget(cur, i + 1, 1)
            grid.addWidget(spin, i + 1, 2)
        layout.addLayout(grid)

        btns = QHBoxLayout()
        self.btn_apply = QPushButton("应用")
        self.btn_reset = QPushButton("复位目标=0")
        self.btn_sync = QPushButton("目标←当前")
        btns.addWidget(self.btn_apply)
        btns.addWidget(self.btn_reset)
        btns.addWidget(self.btn_sync)
        layout.addLayout(btns)
        layout.addStretch(1)

        self._rt_timer = QTimer(self)
        self._rt_timer.setSingleShot(True)
        self._rt_timer.setInterval(80)
        self._rt_timer.timeout.connect(lambda: self.btn_apply.click())

    def _on_edit(self, *_args) -> None:
        self.dirty = True
        if self.realtime.isChecked():
            self._rt_timer.start()

    def sync_from(self, joints: Dict[str, float]) -> None:
        for i, name in enumerate(self.joint_names):
            if name not in joints:
                continue
            v = joints[name]
            show = v * DEG if self.use_deg else v
            self.cur_labels[i].setText(f"{show:.3f}" if not self.use_deg else f"{show:.2f}")
            if not self.dirty:
                self.spins[i].blockSignals(True)
                self.spins[i].setValue(show)
                self.spins[i].blockSignals(False)

    def target_rad(self) -> List[float]:
        out = []
        for spin in self.spins:
            v = spin.value()
            out.append(v * RAD if self.use_deg else v)
        return out

    def zero_targets(self) -> None:
        for spin in self.spins:
            spin.blockSignals(True)
            spin.setValue(0.0)
            spin.blockSignals(False)
        self.dirty = True


class ChassisPanel(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        box = QGroupBox("差速底盘 (cmd_vel)")
        form = QFormLayout(box)
        self.vx = QDoubleSpinBox()
        self.vx.setRange(-5.0, 5.0)
        self.vx.setSingleStep(0.1)
        self.vx.setDecimals(3)
        self.vx.setSuffix(" m/s")
        self.wz = QDoubleSpinBox()
        self.wz.setRange(-6.0, 6.0)
        self.wz.setSingleStep(0.1)
        self.wz.setDecimals(3)
        self.wz.setSuffix(" rad/s")
        self.realtime = QCheckBox("实时下发（定时发送）")
        form.addRow("线速度 vx", self.vx)
        form.addRow("角速度 wz", self.wz)
        form.addRow(self.realtime)
        layout.addWidget(box)

        btns = QHBoxLayout()
        self.btn_go = QPushButton("发送")
        self.btn_stop = QPushButton("停止")
        self.btn_fwd = QPushButton("前进 1.0")
        self.btn_back = QPushButton("后退 1.0")
        self.btn_left = QPushButton("左转")
        self.btn_right = QPushButton("右转")
        for b in (self.btn_go, self.btn_stop, self.btn_fwd, self.btn_back, self.btn_left, self.btn_right):
            btns.addWidget(b)
        layout.addLayout(btns)

        tip = QLabel("话题: /diff_drive_controller/cmd_vel_unstamped")
        tip.setStyleSheet("color: gray;")
        layout.addWidget(tip)
        layout.addStretch(1)

        self.btn_fwd.clicked.connect(lambda: self._preset(1.0, 0.0))
        self.btn_back.clicked.connect(lambda: self._preset(-1.0, 0.0))
        self.btn_left.clicked.connect(lambda: self._preset(0.0, 1.0))
        self.btn_right.clicked.connect(lambda: self._preset(0.0, -1.0))

    def _preset(self, vx: float, wz: float) -> None:
        self.vx.setValue(vx)
        self.wz.setValue(wz)
        self.btn_go.click()


class MainWindow(QMainWindow):
    def __init__(self, bridge: HadRosBridge) -> None:
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("HAD2503 调试 (底盘 / 机械臂)")
        self.resize(980, 720)

        tabs = QTabWidget()
        self.chassis = ChassisPanel()
        self.left_arm = ArmPanel("左臂 (deg)", LEFT_ARM, "left_arm", True)
        self.right_arm = ArmPanel("右臂 (deg)", RIGHT_ARM, "right_arm", True)
        self.left_grip = ArmPanel("左爪 (m)", LEFT_GRIP, "left_grip", False)
        self.right_grip = ArmPanel("右爪 (m)", RIGHT_GRIP, "right_grip", False)
        self.torso = ArmPanel("躯干/头部 (deg)", TORSO, "torso", True)

        arms = QWidget()
        arms_l = QHBoxLayout(arms)
        arms_l.addWidget(self._scroll(self.left_arm))
        arms_l.addWidget(self._scroll(self.right_arm))

        grips = QWidget()
        grips_l = QHBoxLayout(grips)
        grips_l.addWidget(self.left_grip)
        grips_l.addWidget(self.right_grip)

        tabs.addTab(self.chassis, "底盘")
        tabs.addTab(arms, "双臂")
        tabs.addTab(grips, "夹爪")
        tabs.addTab(self._scroll(self.torso), "躯干")
        self.setCentralWidget(tabs)

        self.status = QLabel("等待 /joint_states …")
        self.statusBar().addWidget(self.status)

        self.chassis.btn_go.clicked.connect(self._send_cmd)
        self.chassis.btn_stop.clicked.connect(self._stop_cmd)
        self.chassis.vx.valueChanged.connect(self._maybe_rt_cmd)
        self.chassis.wz.valueChanged.connect(self._maybe_rt_cmd)

        for panel in (self.left_arm, self.right_arm, self.left_grip, self.right_grip, self.torso):
            panel.btn_apply.clicked.connect(lambda _=False, p=panel: self._apply_panel(p))
            panel.btn_reset.clicked.connect(lambda _=False, p=panel: self._reset_panel(p))
            panel.btn_sync.clicked.connect(lambda _=False, p=panel: self._force_sync(p))

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._tick)
        self._ui_timer.start(100)

        self._cmd_timer = QTimer(self)
        self._cmd_timer.timeout.connect(self._send_cmd)
        self._cmd_timer.setInterval(50)
        self.chassis.realtime.toggled.connect(self._toggle_cmd_timer)

    @staticmethod
    def _scroll(w: QWidget) -> QScrollArea:
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(w)
        return area

    def _toggle_cmd_timer(self, on: bool) -> None:
        if on:
            self._cmd_timer.start()
        else:
            self._cmd_timer.stop()
            self._stop_cmd()

    def _maybe_rt_cmd(self, *_args) -> None:
        if self.chassis.realtime.isChecked():
            self._send_cmd()

    def _send_cmd(self) -> None:
        self.bridge.publish_cmd(self.chassis.vx.value(), self.chassis.wz.value())

    def _stop_cmd(self) -> None:
        self.chassis.vx.setValue(0.0)
        self.chassis.wz.setValue(0.0)
        self.bridge.publish_cmd(0.0, 0.0)

    def _apply_panel(self, panel: ArmPanel) -> None:
        self.bridge.publish_traj(panel.traj_key, panel.joint_names, panel.target_rad(), 1.0)
        panel.dirty = True
        self.status.setText(f"已下发 {panel.traj_key}")

    def _reset_panel(self, panel: ArmPanel) -> None:
        panel.zero_targets()
        self._apply_panel(panel)

    def _force_sync(self, panel: ArmPanel) -> None:
        panel.dirty = False
        panel.sync_from(self.bridge.snapshot())

    def _tick(self) -> None:
        js = self.bridge.snapshot()
        if not js:
            self.status.setText("等待 /joint_states …（请先启动 Gazebo）")
            return
        self.status.setText(f"关节数={len(js)}")
        for panel in (self.left_arm, self.right_arm, self.left_grip, self.right_grip, self.torso):
            panel.sync_from(js)

    def closeEvent(self, event) -> None:  # noqa: N802
        self._stop_cmd()
        self._ui_timer.stop()
        self._cmd_timer.stop()
        super().closeEvent(event)


def main() -> None:
    rclpy.init()
    bridge = HadRosBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    win = MainWindow(bridge)
    win.show()
    code = app.exec_()

    executor.shutdown()
    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
