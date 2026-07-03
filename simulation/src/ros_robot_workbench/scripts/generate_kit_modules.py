#!/usr/bin/env python3
"""Generate config / data_manager / module / widget scaffolds for workbench kit modules."""

from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INC = ROOT / "include/ros_robot_workbench"
SRC = ROOT / "src"
CFG = ROOT / "config"

NEW_MODULES = [
    # (snake_name, kit, title_zh, summary, yaml_defaults)
    ("rosbag_workbench", "通用", "Rosbag工作台", "录制、回放、裁剪与 topic 导出。", {"bag_dir": "", "record_topics": ["/camera/image_raw"]}),
    ("topic_lab", "通用", "话题调试", "浏览在线 topic、Echo、测 Hz，并支持 String 单次发布。", {"default_topic": "/chatter", "msg_type": "std_msgs/String"}),
    ("joint_monitor", "机械臂", "关节监视", "订阅 joint_states，表格与曲线显示。", {"joint_states_topic": "/joint_states"}),
    ("moveit_debug", "机械臂", "MoveIt调试", "IK / 规划 / 执行状态调试。", {"compute_ik_service": "/compute_ik", "planning_group": "arm"}),
    ("grasp_pose_gen", "机械臂", "抓取姿态", "基于物体位姿生成候选 grasp pose。", {"object_frame": "object", "ee_frame": "tool0"}),
    ("multi_tcp_manager", "机械臂", "多TCP管理", "多套 tool frame 管理与导出。", {"active_tcp": "default", "tcp_config_dir": ""}),
    ("odometry_analyzer", "移动机器人", "里程计分析", "odom / TF / GPS 对比与漂移评估。", {"odom_topic": "/odom", "base_frame": "base_link"}),
    ("wheel_calib", "移动机器人", "轮速标定", "差速/阿克曼轮速与几何参数标定。", {"cmd_vel_topic": "/cmd_vel", "track_width_m": 0.3}),
    ("nav2_panel", "移动机器人", "Nav2状态", "导航 action 状态与路径预览。", {"action_name": "navigate_to_pose", "global_frame": "map"}),
    ("foot_contact_monitor", "机器狗", "足端监视", "四足接触力/相位/足端位置监视。", {"contact_topic": "/foot_contact", "leg_count": 4}),
    ("legged_imu_panel", "机器狗", "IMU姿态", "四足 IMU 姿态与震荡检测。", {"imu_topic": "/imu/data", "fall_threshold_deg": 45.0}),
    ("rl_policy_monitor", "机器狗", "RL策略监视", "Policy obs/action/reward 实时监视。", {"obs_topic": "/rl/observation", "action_topic": "/rl/action"}),
    ("humanoid_joint_monitor", "人形", "全身关节", "人形机器人分组关节状态监视。", {"joint_states_topic": "/joint_states", "group": "full_body"}),
    ("balance_panel", "人形", "平衡面板", "CoM / ZMP / 支撑足简易估算。", {"imu_topic": "/imu/data", "foot_force_topic": "/foot_force"}),
    ("path_compare", "路径规划", "路径对比", "多条路径长度/平滑度/耗时对比。", {"path_topic_a": "/plan_a", "path_topic_b": "/plan_b"}),
    ("obstacle_editor", "路径规划", "障碍编辑", "2D 障碍编辑与 planner 测试导出。", {"map_topic": "/map", "resolution": 0.05}),
    ("pointcloud_viewer", "3D感知", "点云查看", "PointCloud2 订阅与基础统计。", {"cloud_topic": "/points", "max_points_display": 50000}),
    ("depth_analyzer", "3D感知", "深度分析", "深度图空洞率与噪声统计。", {"depth_topic": "/camera/depth/image_raw"}),
    ("lidar_cam_projection", "3D感知", "点云投影", "LiDAR 投影到相机验证外参。", {"cloud_topic": "/points", "image_topic": "/camera/image_raw"}),
    ("sim_control_panel", "仿真", "仿真控制", "pause/reset/step 与场景切换。", {"control_topic": "/sim/control", "clock_topic": "/clock"}),
    ("sim_time_monitor", "仿真", "Sim Time", "仿真时间与 real-time factor 监视。", {"clock_topic": "/clock"}),
    ("sim2real_compare", "仿真", "Sim2Real对比", "仿真与实机 bag/topic 曲线对比。", {"sim_topic": "/sim/joint_states", "real_topic": "/joint_states"}),
    ("inference_monitor", "深度学习", "推理监视", "模型推理延迟、FPS 与 GPU 占用。", {"result_topic": "/detections", "latency_warn_ms": 100.0}),
    ("detection_overlay", "深度学习", "检测叠加", "2D 检测框叠加显示与阈值调参。", {"image_topic": "/camera/image_raw", "det_topic": "/detections"}),
]


def pascal(snake: str) -> str:
    return "".join(p.capitalize() for p in snake.split("_"))


def guard(path_parts: str) -> str:
    return "ROS_ROBOT_WORKBENCH__" + path_parts.upper().replace("/", "__").replace(".", "_")


def write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        return
    path.write_text(content, encoding="utf-8")


def gen_config(name: str, defaults: dict) -> None:
    lines = [f"# {name}", f"module: {name}", f"enabled: true"]
    for k, v in defaults.items():
        if isinstance(v, list):
            lines.append(f"{k}:")
            for item in v:
                lines.append(f"  - {item}")
        elif isinstance(v, bool):
            lines.append(f"{k}: {'true' if v else 'false'}")
        elif isinstance(v, float):
            lines.append(f"{k}: {v}")
        elif v == "":
            lines.append(f'{k}: ""')
        else:
            lines.append(f"{k}: {v}")
    write(CFG / f"{name}.yaml", "\n".join(lines) + "\n")


def gen_dm(name: str) -> None:
    cls = pascal(name) + "DataManager"
    h = guard(f"manage/{name}_data_manager.hpp")
    write(
        INC / "manage" / f"{name}_data_manager.hpp",
        f"""#ifndef {h}
#define {h}

#include "ros_robot_workbench/manage/feature_data_manager_base.hpp"

namespace ros_robot_workbench::manage
{{

class {cls} : public FeatureDataManagerBase
{{
public:
  {cls}();
  void EnsureDefaults() override;
}};

}}  // namespace ros_robot_workbench::manage

#endif
""",
    )
    write(
        SRC / "manage" / f"{name}_data_manager.cpp",
        f"""#include "ros_robot_workbench/manage/{name}_data_manager.hpp"

namespace ros_robot_workbench::manage
{{

{cls}::{cls}()
: FeatureDataManagerBase("{name}.yaml")
{{
}}

void {cls}::EnsureDefaults()
{{
  FeatureDataManagerBase::EnsureDefaults();
  if (!data_["module"]) {{
    data_["module"] = "{name}";
  }}
  if (!data_["enabled"]) {{
    data_["enabled"] = true;
  }}
}}

}}  // namespace ros_robot_workbench::manage
""",
    )


def gen_module(name: str, summary: str) -> None:
    h = guard(f"module/{name}_module.h")
    write(
        INC / "module" / f"{name}_module.h",
        f"""#ifndef {h}
#define {h}

#include <QString>

namespace ros_robot_workbench::ui
{{

QString {pascal(name)}ModuleSummary();

}}  // namespace ros_robot_workbench::ui

#endif
""",
    )
    write(
        SRC / "module" / f"{name}_module.cpp",
        f"""#include "ros_robot_workbench/module/{name}_module.h"

namespace ros_robot_workbench::ui
{{

QString {pascal(name)}ModuleSummary()
{{
  return QStringLiteral("{summary}");
}}

}}  // namespace ros_robot_workbench::ui
""",
    )


def gen_widget(name: str, title: str, summary: str) -> None:
    cls = pascal(name) + "Widget"
    h = guard(f"ui/{name}_widget.h")
    write(
        INC / "ui" / f"{name}_widget.h",
        f"""#ifndef {h}
#define {h}

#include <QWidget>

namespace ros_robot_workbench::ui
{{

class {cls} : public QWidget
{{
public:
  explicit {cls}(QWidget * parent = nullptr);
}};

}}  // namespace ros_robot_workbench::ui

#endif
""",
    )
    write(
        SRC / "ui" / f"{name}_widget.cpp",
        f"""#include "ros_robot_workbench/ui/{name}_widget.h"

#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "ros_robot_workbench/manage/{name}_data_manager.hpp"
#include "ros_robot_workbench/module/calibration_module.h"
#include "ros_robot_workbench/module/{name}_module.h"

namespace ros_robot_workbench::ui
{{

{cls}::{cls}(QWidget * parent)
: QWidget(parent)
{{
  manage::{pascal(name)}DataManager dm;
  dm.SetConfigPath(ResolveDefaultConfigYamlPath("{name}.yaml").toStdString());
  dm.Load();

  QVBoxLayout * root = new QVBoxLayout(this);
  root->setContentsMargins(8, 6, 8, 6);
  root->setSpacing(6);

  QLabel * title = new QLabel("{title}");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);

  QLabel * hint = new QLabel({pascal(name)}ModuleSummary());
  hint->setWordWrap(true);
  hint->setStyleSheet("color:#445566;font-size:12px;");
  root->addWidget(hint);

  QGroupBox * cfg_group = new QGroupBox("配置");
  QFormLayout * form = new QFormLayout(cfg_group);
  QLineEdit * cfg_path = new QLineEdit(QString::fromStdString(dm.GetConfigPath()));
  cfg_path->setReadOnly(true);
  form->addRow("配置文件:", cfg_path);
  root->addWidget(cfg_group);

  QPlainTextEdit * log = new QPlainTextEdit();
  log->setReadOnly(true);
  log->setPlaceholderText("模块日志与运行输出…");
  root->addWidget(log, 1);

  QPushButton * refresh = new QPushButton("刷新状态");
  QObject::connect(refresh, &QPushButton::clicked, [log, summary = QString({pascal(name)}ModuleSummary())]() {{
    log->appendPlainText(summary);
    log->appendPlainText("模块已加载，可在后续版本接入 ROS 接口与业务逻辑。");
  }});
  root->addWidget(refresh);
}}

}}  // namespace ros_robot_workbench::ui
""",
    )


def main() -> None:
    for name, _kit, title, summary, defaults in NEW_MODULES:
        gen_config(name, defaults)
        gen_dm(name)
        gen_module(name, summary)
        gen_widget(name, title, summary)
        print("generated", name)
    print(f"done: {len(NEW_MODULES)} modules")


if __name__ == "__main__":
    main()
