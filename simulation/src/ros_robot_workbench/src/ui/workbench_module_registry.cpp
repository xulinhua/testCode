#include "ros_robot_workbench/ui/workbench_module_registry.hpp"

#include "ros_robot_workbench/workbench_build_config.hpp"

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#if WORKBENCH_KIT_GENERAL
#include "ros_robot_workbench/ui/image_viewer_widget.h"
#include "ros_robot_workbench/ui/rosbag_workbench_widget.h"
#include "ros_robot_workbench/ui/topic_lab_widget.h"
#endif
#if WORKBENCH_KIT_KINEMATICS
#include "ros_robot_workbench/ui/kinematics_solver_widget.h"
#include "ros_robot_workbench/ui/pose_transform_widget.h"
#include "ros_robot_workbench/ui/tf_viewer_widget.h"
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
#include "ros_robot_workbench/ui/board_generator_widget.h"
#include "ros_robot_workbench/ui/handeye_calibration_widget.h"
#endif
#include "ros_robot_workbench/ui/intrinsic_calibration_widget.h"
#include "ros_robot_workbench/ui/multi_sensor_calibration_widget.h"
#include "ros_robot_workbench/ui/stereo_calibration_widget.h"
#include "ros_robot_workbench/ui/tcp_calibration_widget.h"
#endif
#if WORKBENCH_KIT_ARM
#include "ros_robot_workbench/ui/grasp_pose_gen_widget.h"
#include "ros_robot_workbench/ui/joint_monitor_widget.h"
#if WORKBENCH_WITH_MOVEIT
#include "ros_robot_workbench/ui/moveit_debug_widget.h"
#endif
#include "ros_robot_workbench/ui/multi_tcp_manager_widget.h"
#endif
#if WORKBENCH_KIT_MOBILE
#include "ros_robot_workbench/ui/nav2_panel_widget.h"
#include "ros_robot_workbench/ui/odometry_analyzer_widget.h"
#include "ros_robot_workbench/ui/wheel_calib_widget.h"
#endif
#if WORKBENCH_KIT_LEGGED
#include "ros_robot_workbench/ui/foot_contact_monitor_widget.h"
#include "ros_robot_workbench/ui/legged_imu_panel_widget.h"
#include "ros_robot_workbench/ui/rl_policy_monitor_widget.h"
#endif
#if WORKBENCH_KIT_HUMANOID
#include "ros_robot_workbench/ui/balance_panel_widget.h"
#include "ros_robot_workbench/ui/humanoid_joint_monitor_widget.h"
#endif
#if WORKBENCH_KIT_PLANNING
#include "ros_robot_workbench/ui/obstacle_editor_widget.h"
#include "ros_robot_workbench/ui/path_compare_widget.h"
#endif
#if WORKBENCH_KIT_PERCEPTION3D
#include "ros_robot_workbench/ui/depth_analyzer_widget.h"
#include "ros_robot_workbench/ui/lidar_cam_projection_widget.h"
#include "ros_robot_workbench/ui/pointcloud_viewer_widget.h"
#endif
#if WORKBENCH_KIT_SIMULATION
#include "ros_robot_workbench/ui/sim2real_compare_widget.h"
#include "ros_robot_workbench/ui/sim_control_panel_widget.h"
#include "ros_robot_workbench/ui/sim_time_monitor_widget.h"
#include "ros_robot_workbench/ui/usd_converter_widget.h"
#endif
#if WORKBENCH_KIT_DL
#include "ros_robot_workbench/ui/detection_overlay_widget.h"
#include "ros_robot_workbench/ui/inference_monitor_widget.h"
#endif

#include "ros_robot_workbench/ui/lazy_feature_page.hpp"
#include "ros_robot_workbench/ui/system_status_widget.h"

namespace ros_robot_workbench::ui
{
namespace
{

template<typename WidgetT>
WorkbenchPageFactory MakeFactory()
{
  return [](QWidget * parent) -> QWidget * { return new WidgetT(parent); };
}

}  // namespace

std::vector<WorkbenchModuleSpec> GetWorkbenchModuleSpecs()
{
  std::vector<WorkbenchModuleSpec> specs;
  specs.reserve(40);

  // 系统状态：始终可用
  specs.push_back({"通用", "系统状态", MakeFactory<SystemStatusWidget>(), false});

#if WORKBENCH_KIT_GENERAL
  specs.push_back({"通用", "Rosbag工作台", MakeFactory<RosbagWorkbenchWidget>(), true});
  specs.push_back({"通用", "话题调试", MakeFactory<TopicLabWidget>(), true});
  specs.push_back({"通用", "图像查看", MakeFactory<ImageViewerWidget>(), true});
#endif
#if WORKBENCH_KIT_KINEMATICS
  specs.push_back({"运动学", "姿态转换", MakeFactory<PoseTransformWidget>(), true});
  specs.push_back({"运动学", "运动学计算", MakeFactory<KinematicsSolverWidget>(), true});
  specs.push_back({"运动学", "TF查看", MakeFactory<TfViewerWidget>(), true});
#endif
#if WORKBENCH_KIT_CALIBRATION
#if WORKBENCH_WITH_OPENCV
  specs.push_back({"标定", "标定板生成", MakeFactory<BoardGeneratorWidget>(), true});
#endif
  specs.push_back({"标定", "内参标定", MakeFactory<IntrinsicCalibrationWidget>(), true});
  specs.push_back({"标定", "双目标定", MakeFactory<StereoCalibrationWidget>(), true});
  specs.push_back({"标定", "多传感器标定", MakeFactory<MultiSensorCalibrationWidget>(), true});
#if WORKBENCH_WITH_OPENCV
  specs.push_back({"标定", "手眼标定", MakeFactory<HandeyeCalibrationWidget>(), true});
#endif
  specs.push_back({"标定", "TCP标定", MakeFactory<TcpCalibrationWidget>(), true});
#endif
#if WORKBENCH_KIT_ARM
  specs.push_back({"机械臂", "关节监视", MakeFactory<JointMonitorWidget>(), true});
#if WORKBENCH_WITH_MOVEIT
  specs.push_back({"机械臂", "MoveIt调试", MakeFactory<MoveitDebugWidget>(), true});
#endif
  specs.push_back({"机械臂", "抓取姿态", MakeFactory<GraspPoseGenWidget>(), true});
  specs.push_back({"机械臂", "多TCP管理", MakeFactory<MultiTcpManagerWidget>(), true});
#endif
#if WORKBENCH_KIT_MOBILE
  specs.push_back({"移动机器人", "里程计分析", MakeFactory<OdometryAnalyzerWidget>(), true});
  specs.push_back({"移动机器人", "轮速标定", MakeFactory<WheelCalibWidget>(), true});
  specs.push_back({"移动机器人", "Nav2状态", MakeFactory<Nav2PanelWidget>(), true});
#endif
#if WORKBENCH_KIT_LEGGED
  specs.push_back({"机器狗", "足端监视", MakeFactory<FootContactMonitorWidget>(), true});
  specs.push_back({"机器狗", "IMU姿态", MakeFactory<LeggedImuPanelWidget>(), true});
  specs.push_back({"机器狗", "RL策略监视", MakeFactory<RlPolicyMonitorWidget>(), true});
#endif
#if WORKBENCH_KIT_HUMANOID
  specs.push_back({"人形", "全身关节", MakeFactory<HumanoidJointMonitorWidget>(), true});
  specs.push_back({"人形", "平衡面板", MakeFactory<BalancePanelWidget>(), true});
#endif
#if WORKBENCH_KIT_PLANNING
  specs.push_back({"路径规划", "路径对比", MakeFactory<PathCompareWidget>(), true});
  specs.push_back({"路径规划", "障碍编辑", MakeFactory<ObstacleEditorWidget>(), true});
#endif
#if WORKBENCH_KIT_PERCEPTION3D
  specs.push_back({"3D感知", "点云查看", MakeFactory<PointcloudViewerWidget>(), true});
  specs.push_back({"3D感知", "深度分析", MakeFactory<DepthAnalyzerWidget>(), true});
  specs.push_back({"3D感知", "点云投影", MakeFactory<LidarCamProjectionWidget>(), true});
#endif
#if WORKBENCH_KIT_SIMULATION
  specs.push_back({"仿真", "仿真控制", MakeFactory<SimControlPanelWidget>(), true});
  specs.push_back({"仿真", "Sim Time", MakeFactory<SimTimeMonitorWidget>(), true});
  specs.push_back({"仿真", "Sim2Real对比", MakeFactory<Sim2realCompareWidget>(), true});
  specs.push_back({"仿真", "USD转换", MakeFactory<UsdConverterWidget>(), true});
#endif
#if WORKBENCH_KIT_DL
  specs.push_back({"深度学习", "推理监视", MakeFactory<InferenceMonitorWidget>(), true});
  specs.push_back({"深度学习", "检测叠加", MakeFactory<DetectionOverlayWidget>(), true});
#endif

  return specs;
}

QWidget * BuildKitNavigationPanel(
  QStackedWidget * stack,
  QStringList * flat_page_names,
  int * image_viewer_stack_index)
{
  if (flat_page_names) {
    flat_page_names->clear();
  }
  if (image_viewer_stack_index) {
    *image_viewer_stack_index = -1;
  }

  QWidget * nav_root = new QWidget();
  nav_root->setFixedWidth(168);
  nav_root->setStyleSheet("background-color: #2c3e50;");

  QVBoxLayout * outer = new QVBoxLayout(nav_root);
  outer->setContentsMargins(8, 16, 8, 12);
  outer->setSpacing(8);

  QLabel * title = new QLabel("Robot Workbench");
  title->setStyleSheet("color: white; font-size: 16px; font-weight: bold;");
  title->setAlignment(Qt::AlignCenter);
  outer->addWidget(title);

  QScrollArea * scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setStyleSheet("QScrollArea{background:transparent;} QScrollBar:vertical{width:8px;}");
  QWidget * scroll_body = new QWidget();
  QVBoxLayout * nav_layout = new QVBoxLayout(scroll_body);
  nav_layout->setContentsMargins(0, 0, 0, 0);
  nav_layout->setSpacing(6);

  QString current_kit;
  int page_index = 0;
  for (const WorkbenchModuleSpec & spec : GetWorkbenchModuleSpecs()) {
    if (spec.kit != current_kit) {
      current_kit = spec.kit;
      QLabel * kit_label = new QLabel(current_kit);
      kit_label->setStyleSheet("color:#95a5a6;font-size:11px;font-weight:600;margin-top:6px;");
      nav_layout->addWidget(kit_label);
    }

    QWidget * page = nullptr;
    if (spec.lazy) {
      page = new LazyFeaturePage(spec.factory);
    } else {
      page = spec.factory(scroll_body);
    }
    stack->addWidget(page);

    if (flat_page_names) {
      flat_page_names->append(spec.title);
    }
    if (image_viewer_stack_index && spec.title == "图像查看") {
      *image_viewer_stack_index = page_index;
    }

    QPushButton * btn = new QPushButton(spec.title);
    btn->setFixedHeight(34);
    btn->setStyleSheet(
      "QPushButton{background-color:#34495e;color:white;border:none;border-radius:4px;font-size:12px;text-align:left;padding-left:8px;}"
      "QPushButton:hover{background-color:#1abc9c;}");
    QObject::connect(btn, &QPushButton::clicked, [stack, page_index]() {
      stack->setCurrentIndex(page_index);
    });
    nav_layout->addWidget(btn);
    ++page_index;
  }

  nav_layout->addStretch();
  scroll->setWidget(scroll_body);
  outer->addWidget(scroll, 1);
  return nav_root;
}

}  // namespace ros_robot_workbench::ui
