# -*- coding: utf-8 -*-
"""MoveIt2 抓取执行节点：订阅位姿，经本包 ArmPose 话题驱动双臂。"""

from __future__ import annotations

import copy
import threading
import time
from typing import Callable, Optional

import rclpy
from nova_grasp_moveit.msg import ArmPose
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .grasp_planner import format_plan, plan_grasp_from_pose, pose_stamped_from_plan_step

DEFAULT_GRASP_TOPIC = "/graspnet/best_grasp"
DEFAULT_BOX_TOPIC = "/box_pose"
DEFAULT_TARGET_ARM_POSE_TOPIC = "/nova_target_arm_pose"
DEFAULT_GRIPPER_TOPIC = "/nova_gripper_goal"
DEFAULT_STATUS_TOPIC = "/nova_grasp/status"
DEFAULT_POSE_LOG_TOPIC = "/nova_pose_log"
DEFAULT_POSE_FRAME = "base_link"


def _box_pose_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def format_pose_stamped(msg: PoseStamped) -> str:
    p = msg.pose.position
    q = msg.pose.orientation
    frame = msg.header.frame_id or "(empty)"
    return (
        f"frame={frame} pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
        f"quat=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})"
    )


class GraspMoveItNode(Node):
    """GraspNet / box_pose → /nova_target_arm_pose + /nova_gripper_goal。"""

    def __init__(
        self,
        *,
        grasp_topic: str = DEFAULT_GRASP_TOPIC,
        box_topic: str = DEFAULT_BOX_TOPIC,
        target_arm_pose_topic: str = DEFAULT_TARGET_ARM_POSE_TOPIC,
        gripper_topic: str = DEFAULT_GRIPPER_TOPIC,
        status_topic: str = DEFAULT_STATUS_TOPIC,
        pose_log_topic: str = DEFAULT_POSE_LOG_TOPIC,
        pose_frame: str = DEFAULT_POSE_FRAME,
        arm_split_x: float = 0.53,
        pregrasp_z_offset: float = 0.12,
        lift_z_offset: float = 0.08,
        step_settle_sec: float = 2.0,
        gripper_settle_sec: float = 0.8,
        use_top_down_if_identity: bool = True,
        auto_execute_on_grasp: bool = False,
        on_log: Optional[Callable[[str], None]] = None,
        on_status: Optional[Callable[[str], None]] = None,
        on_grasp_pose: Optional[Callable[[PoseStamped], None]] = None,
        on_joint_state: Optional[Callable[[JointState], None]] = None,
    ) -> None:
        super().__init__("grasp_moveit_node")

        self._grasp_topic = grasp_topic.strip() or DEFAULT_GRASP_TOPIC
        self._box_topic = box_topic.strip() or DEFAULT_BOX_TOPIC
        self._target_topic = target_arm_pose_topic.strip() or DEFAULT_TARGET_ARM_POSE_TOPIC
        self._gripper_topic = gripper_topic.strip() or DEFAULT_GRIPPER_TOPIC
        self._status_topic = status_topic.strip() or DEFAULT_STATUS_TOPIC
        self._pose_log_topic = pose_log_topic.strip() or DEFAULT_POSE_LOG_TOPIC
        self._pose_frame = pose_frame.strip() or DEFAULT_POSE_FRAME
        self._arm_split_x = float(arm_split_x)
        self._pregrasp_z = float(pregrasp_z_offset)
        self._lift_z = float(lift_z_offset)
        self._step_settle = max(0.1, float(step_settle_sec))
        self._gripper_settle = max(0.1, float(gripper_settle_sec))
        self._use_top_down = bool(use_top_down_if_identity)
        self._auto_execute = bool(auto_execute_on_grasp)
        self._on_log = on_log
        self._on_status = on_status
        self._on_grasp_pose = on_grasp_pose
        self._on_joint_state = on_joint_state

        self._last_grasp_pose: Optional[PoseStamped] = None
        self._last_status = "idle"
        self._busy = False
        self._lock = threading.Lock()

        self._arm_pose_pub = self.create_publisher(ArmPose, self._target_topic, 10)
        self._gripper_pub = self.create_publisher(String, self._gripper_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(PoseArray, self._grasp_topic, self._on_grasp_array, 10)
        self.create_subscription(PoseStamped, self._box_topic, self._on_box_pose, _box_pose_qos())
        self.create_subscription(String, self._pose_log_topic, self._on_pose_log, 20)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state_msg, 50)
        self.create_service(Trigger, "~/execute", self._on_execute_service)

        self._log_ui(
            f"ready grasp={self._grasp_topic} box={self._box_topic} "
            f"target={self._target_topic} gripper={self._gripper_topic}"
        )
        self._publish_status("ready")

    @property
    def is_busy(self) -> bool:
        with self._lock:
            return self._busy

    @property
    def last_grasp_pose(self) -> Optional[PoseStamped]:
        return copy.deepcopy(self._last_grasp_pose) if self._last_grasp_pose else None

    @property
    def last_status(self) -> str:
        return self._last_status

    def set_auto_execute(self, enabled: bool) -> None:
        self._auto_execute = bool(enabled)
        self._log_ui(f"auto_execute_on_grasp={self._auto_execute}")

    def set_use_top_down_if_identity(self, enabled: bool) -> None:
        self._use_top_down = bool(enabled)
        self._log_ui(f"use_top_down_if_identity={self._use_top_down}")

    def update_timing(
        self,
        *,
        pregrasp_z_offset: Optional[float] = None,
        lift_z_offset: Optional[float] = None,
        step_settle_sec: Optional[float] = None,
        gripper_settle_sec: Optional[float] = None,
        arm_split_x: Optional[float] = None,
    ) -> None:
        if pregrasp_z_offset is not None:
            self._pregrasp_z = float(pregrasp_z_offset)
        if lift_z_offset is not None:
            self._lift_z = float(lift_z_offset)
        if step_settle_sec is not None:
            self._step_settle = max(0.1, float(step_settle_sec))
        if gripper_settle_sec is not None:
            self._gripper_settle = max(0.1, float(gripper_settle_sec))
        if arm_split_x is not None:
            self._arm_split_x = float(arm_split_x)

    def _log_ui(self, text: str) -> None:
        self.get_logger().info(text)
        if self._on_log:
            self._on_log(text)

    def _publish_status(self, text: str) -> None:
        self._last_status = text
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        if self._on_status:
            self._on_status(text)
        self._log_ui(f"[status] {text}")

    def _notify_grasp_pose(self, stamped: PoseStamped) -> None:
        if self._on_grasp_pose:
            self._on_grasp_pose(copy.deepcopy(stamped))

    def _on_pose_log(self, msg: String) -> None:
        line = msg.data.strip()
        if line:
            prefix = "[ERROR]" if line.startswith("[ERROR]") else "[pose_log]"
            self._log_ui(f"{prefix} {line}")

    def _on_joint_state_msg(self, msg: JointState) -> None:
        if self._on_joint_state:
            self._on_joint_state(msg)

    def _on_grasp_array(self, msg: PoseArray) -> None:
        if not msg.poses:
            return
        stamped = PoseStamped()
        stamped.header = copy.deepcopy(msg.header)
        if not stamped.header.frame_id:
            stamped.header.frame_id = self._pose_frame
        stamped.pose = copy.deepcopy(msg.poses[0])
        self._last_grasp_pose = stamped
        self._log_ui(f"<<< {self._grasp_topic} {format_pose_stamped(stamped)}")
        self._notify_grasp_pose(stamped)
        if self._auto_execute:
            self.execute_grasp()

    def _on_box_pose(self, msg: PoseStamped) -> None:
        self._last_grasp_pose = copy.deepcopy(msg)
        if not self._last_grasp_pose.header.frame_id:
            self._last_grasp_pose.header.frame_id = self._pose_frame
        self._notify_grasp_pose(self._last_grasp_pose)

    def _on_execute_service(self, _request, response):
        started = self.execute_grasp()
        response.success = started
        if self._last_grasp_pose is None:
            response.message = "no grasp pose yet"
        elif started:
            response.message = "grasp sequence started"
        else:
            response.message = "grasp already running or no pose"
        return response

    def execute_grasp(self) -> bool:
        with self._lock:
            if self._busy or self._last_grasp_pose is None:
                return False
            self._busy = True
        thread = threading.Thread(target=self._run_grasp_sequence, daemon=True)
        thread.start()
        return True

    def send_gripper(self, cmd: str) -> None:
        self._send_gripper(cmd)

    def send_arm_pose_manual(self, arm_id: int, pose: PoseStamped) -> None:
        self._send_arm_pose(int(arm_id), pose)

    def preview_plan_text(self) -> str:
        if self._last_grasp_pose is None:
            return "no grasp pose yet"
        frame = self._last_grasp_pose.header.frame_id or self._pose_frame
        plan = plan_grasp_from_pose(
            self._last_grasp_pose.pose,
            frame_id=frame,
            arm_split_x=self._arm_split_x,
            pregrasp_z_offset=self._pregrasp_z,
            lift_z_offset=self._lift_z,
            use_top_down_if_identity=self._use_top_down,
        )
        g = plan.grasp.position
        p = plan.pregrasp.position
        l = plan.lift.position
        return (
            f"{format_plan(plan)}\n"
            f"pregrasp z={p.z:.3f}  grasp z={g.z:.3f}  lift z={l.z:.3f}\n"
            f"settle step={self._step_settle:.1f}s gripper={self._gripper_settle:.1f}s"
        )

    def _run_grasp_sequence(self) -> None:
        try:
            pose_msg = copy.deepcopy(self._last_grasp_pose)
            if pose_msg is None:
                return
            frame = pose_msg.header.frame_id or self._pose_frame
            plan = plan_grasp_from_pose(
                pose_msg.pose,
                frame_id=frame,
                arm_split_x=self._arm_split_x,
                pregrasp_z_offset=self._pregrasp_z,
                lift_z_offset=self._lift_z,
                use_top_down_if_identity=self._use_top_down,
            )
            self._publish_status(f"EXECUTE {format_plan(plan)}")

            stamp = self.get_clock().now().to_msg()
            self._send_arm_pose(plan.arm_id, pose_stamped_from_plan_step(plan, plan.pregrasp, stamp))
            self._publish_status("step pregrasp")
            time.sleep(self._step_settle)

            stamp = self.get_clock().now().to_msg()
            self._send_arm_pose(plan.arm_id, pose_stamped_from_plan_step(plan, plan.grasp, stamp))
            self._publish_status("step grasp")
            time.sleep(self._step_settle)

            self._send_gripper("close")
            self._publish_status("gripper close")
            time.sleep(self._gripper_settle)

            stamp = self.get_clock().now().to_msg()
            self._send_arm_pose(plan.arm_id, pose_stamped_from_plan_step(plan, plan.lift, stamp))
            self._publish_status("step lift")
            time.sleep(self._step_settle)

            self._publish_status("DONE")
        except Exception as exc:
            self.get_logger().error(f"grasp sequence failed: {exc}")
            self._publish_status(f"ERROR {exc}")
        finally:
            with self._lock:
                self._busy = False

    def _send_arm_pose(self, arm_id: int, pose: PoseStamped) -> None:
        msg = ArmPose()
        msg.arm_id = int(arm_id)
        msg.pose = copy.deepcopy(pose)
        self._arm_pose_pub.publish(msg)
        p = pose.pose.position
        self._log_ui(
            f">>> {self._target_topic} arm_id={arm_id} "
            f"frame={pose.header.frame_id} pos=({p.x:.3f},{p.y:.3f},{p.z:.3f})"
        )

    def _send_gripper(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._gripper_pub.publish(msg)
        self._log_ui(f">>> {self._gripper_topic} {cmd}")


def main() -> None:
    rclpy.init()
    node = GraspMoveItNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
