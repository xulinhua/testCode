#!/usr/bin/env python3
"""Append kit module sources to CMakeLists.txt if missing."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CMAKE = ROOT / "CMakeLists.txt"
text = CMAKE.read_text(encoding="utf-8")

modules = [
    "rosbag_workbench", "topic_lab", "joint_monitor", "moveit_debug",
    "grasp_pose_gen", "multi_tcp_manager", "odometry_analyzer", "wheel_calib", "nav2_panel",
    "foot_contact_monitor", "legged_imu_panel", "rl_policy_monitor", "humanoid_joint_monitor",
    "balance_panel", "path_compare", "obstacle_editor", "pointcloud_viewer", "depth_analyzer",
    "lidar_cam_projection", "sim_control_panel", "sim_time_monitor", "sim2real_compare",
    "inference_monitor", "detection_overlay",
]

extra = [
    "src/ui/workbench_module_registry.cpp",
]

for m in modules:
    extra += [
        f"src/module/{m}_module.cpp",
        f"src/ui/{m}_widget.cpp",
        f"src/manage/{m}_data_manager.cpp",
    ]

for path in extra:
    if path not in text:
        anchor = "  src/manage/feature_data_manager_hub.cpp"
        text = text.replace(anchor, f"  {path}\n{anchor}")

CMAKE.write_text(text, encoding="utf-8")
print("CMakeLists updated")
