// Qt 图形界面：多臂关节滑动条、位姿输入、夹爪与 IK 调用（与 moveit2_arm_executor 协同）。
#include <memory>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <array>
#include <unordered_map>
#include <map>
#include <set>
#include <regex>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPalette>
#include <QPushButton>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QScrollArea>
#include <QSpinBox>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QtConcurrent/QtConcurrentRun>

#include "rclcpp/executors/single_threaded_executor.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
const std::vector<std::string> kJointOrder = {
  "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint", "J1_7_joint", "J1_8_joint",
  "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint", "J2_7_joint", "J2_8_joint",
  "J3_1_joint", "J3_2_joint", "J3_3_joint", "J3_4_joint", "J3_5_joint", "J3_6_joint",
  "J4_1_joint", "J4_2_joint", "J4_3_joint", "J4_4_joint", "J4_5_joint", "J4_6_joint"};
const std::vector<int> kEePoseRows = {0, 1, 2, 3};

struct JointStateUiBatch {
  std::vector<std::pair<std::string, double>> position_updates;
  std::set<int> arm_ids;
  std::unordered_map<int, std::set<int>> joint_indices_by_arm;
};

JointStateUiBatch build_joint_state_batch(const sensor_msgs::msg::JointState & msg)
{
  JointStateUiBatch batch;
  static const std::regex kJointPattern(R"(J(\d+)_(\d+)_joint)");
  for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
    batch.position_updates.emplace_back(msg.name[i], msg.position[i]);
    std::smatch m;
    if (std::regex_match(msg.name[i], m, kJointPattern)) {
      const int arm_num = std::stoi(m[1].str());
      const int joint_num = std::stoi(m[2].str());
      const int arm_id = arm_num - 1;
      if (arm_id >= 0) {
        batch.arm_ids.insert(arm_id);
        batch.joint_indices_by_arm[arm_id].insert(joint_num);
      }
    }
  }
  return batch;
}

struct EePoseRowResult {
  int arm_id{0};
  QString ref_text;
  QString ee_text;
  QString pose_text;
  bool tf_ok{false};
  std::array<double, 7> pose{};
};
}  // namespace

class NovaControlWindow : public QMainWindow
{
public:
  explicit NovaControlWindow(const std::shared_ptr<rclcpp::Node> & node)
  : node_(node)
  {
    cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);
    arm_id_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/nova_arm_id", 10);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/nova_target_pose", 10);
    gripper_pub_ = node_->create_publisher<std_msgs::msg::String>("/nova_gripper_goal", 10);
    pose_log_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/nova_pose_log", 20,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        const QString text = QString::fromStdString(msg->data);
        const bool err = text.startsWith("[ERROR]");
        QTimer::singleShot(0, this, [this, text, err]() { append_pose_log(text, err); });
      });
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });
    ik_client_ = node_->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

    setWindowTitle("Nova Control UI (Qt/C++)");
    resize(900, 780);
    setMinimumSize(860, 720);
    build_ui();

    // Debounced realtime publish
    joint_publish_timer_ = new QTimer(this);
    joint_publish_timer_->setSingleShot(true);
    joint_publish_timer_->setInterval(80);
    connect(joint_publish_timer_, &QTimer::timeout, [this]() { publish_joint_command(); });

    pose_publish_timer_ = new QTimer(this);
    pose_publish_timer_->setSingleShot(true);
    pose_publish_timer_->setInterval(120);
    connect(pose_publish_timer_, &QTimer::timeout, [this]() { send_pose_goal_impl(false); });

    // IK availability monitor
    ik_watchdog_timer_ = new QTimer(this);
    ik_watchdog_timer_->setInterval(1000);
    connect(ik_watchdog_timer_, &QTimer::timeout, [this]() { refresh_ik_status(); });
    ik_watchdog_timer_->start();
    refresh_ik_status();

    ee_pose_timer_ = new QTimer(this);
    ee_pose_timer_->setInterval(200);
    connect(ee_pose_timer_, &QTimer::timeout, [this]() { schedule_ee_pose_refresh(); });
    ee_pose_timer_->start();

    ros_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    ros_executor_->add_node(node_);
    ros_spin_thread_ = std::thread([this]() {
      while (rclcpp::ok() && !ros_spin_exit_.load()) {
        if (ros_executor_) {
          ros_executor_->spin_some(std::chrono::milliseconds(15));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  ~NovaControlWindow() override
  {
    ros_spin_exit_.store(true);
    if (ros_executor_) {
      ros_executor_->cancel();
    }
    if (ros_spin_thread_.joinable()) {
      ros_spin_thread_.join();
    }
  }

private:
  void build_ui()
  {
    setStyleSheet(
      "QMainWindow { background: #eef1f4; }"
      "QTabWidget::pane { border: 1px solid #c9d0d8; background: #f8fafc; }"
      "QTabBar::tab { background: #e2e6eb; border: 1px solid #c4ccd6; padding: 6px 12px; margin-right: 2px; }"
      "QTabBar::tab:selected { background: #ffffff; border-bottom-color: #ffffff; }"
      "QGroupBox { border: 1px solid #cfd6df; border-radius: 6px; margin-top: 10px; font-weight: 600; background: #f5f7fa; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
      "QLabel { color: #222b35; }"
      "QDoubleSpinBox, QComboBox, QLineEdit { background: #ffffff; border: 1px solid #c9d2dc; border-radius: 4px; padding: 2px 5px; min-height: 22px; }"
      "QPushButton { background: #e7ebf0; border: 1px solid #c1c9d3; border-radius: 4px; padding: 5px 10px; }"
      "QPushButton:hover { background: #dbe1e8; }"
      "QScrollArea { border: 1px solid #cfd6df; background: #fbfcfd; }");

    auto * central = new QWidget(this);
    auto * root_layout = new QVBoxLayout(central);
    root_layout->setContentsMargins(8, 8, 8, 8);
    root_layout->setSpacing(6);
    auto * tabs = new QTabWidget(central);
    root_layout->addWidget(tabs);

    auto * joint_tab = new QWidget(tabs);
    auto * pose_tab = new QWidget(tabs);
    tabs->addTab(joint_tab, "Joint Control");
    tabs->addTab(pose_tab, "Pose Control");

    // Joint tab
    auto * joint_layout = new QVBoxLayout(joint_tab);
    auto * scroll = new QScrollArea(joint_tab);
    scroll->setWidgetResizable(true);
    auto * scroll_content = new QWidget(scroll);
    auto * joint_groups_layout = new QGridLayout(scroll_content);
    joint_groups_layout->setHorizontalSpacing(8);
    joint_groups_layout->setVerticalSpacing(8);
    joint_groups_layout->setContentsMargins(8, 8, 8, 8);
    scroll_content->setLayout(joint_groups_layout);
    scroll->setWidget(scroll_content);
    joint_layout->addWidget(scroll);

    std::map<int, QGridLayout *> arm_grids;
    std::map<int, int> arm_rows;
    static const std::regex kJointPattern(R"(J(\d+)_(\d+)_joint)");

    for (int i = 0; i < static_cast<int>(kJointOrder.size()); ++i) {
      int arm_num = 0;
      std::smatch m;
      if (std::regex_match(kJointOrder[i], m, kJointPattern)) {
        arm_num = std::stoi(m[1].str());
      }
      if (arm_grids.find(arm_num) == arm_grids.end()) {
        auto * arm_group = new QGroupBox(QString("Arm Group J%1").arg(arm_num), scroll_content);
        arm_group->setFixedSize(430, 305);
        auto * arm_group_layout = new QVBoxLayout(arm_group);
        auto * arm_grid = new QGridLayout();
        arm_grid->setHorizontalSpacing(6);
        arm_grid->setVerticalSpacing(4);
        arm_grid->setColumnStretch(0, 0);
        arm_grid->setColumnStretch(1, 0);
        arm_grid->setColumnStretch(2, 0);
        arm_grid->setColumnStretch(3, 1);
        arm_grid->addWidget(new QLabel("Joint", arm_group), 0, 0);
        arm_grid->addWidget(new QLabel("Current(readonly)", arm_group), 0, 1);
        arm_grid->addWidget(new QLabel("Target", arm_group), 0, 2);
        arm_group_layout->addLayout(arm_grid);
        const int arm_id = std::max(0, arm_num - 1);
        const int col = (arm_id >= 2) ? 1 : 0;   // left: arm 0/1, right: arm 2/3
        const int row = (arm_id >= 2) ? (arm_id - 2) : arm_id;
        joint_groups_layout->addWidget(arm_group, row, col, Qt::AlignTop | Qt::AlignLeft);
        arm_grids[arm_num] = arm_grid;
        arm_rows[arm_num] = 1;
      }

      auto * label =
        new QLabel(QString("%1 %2").arg(i, 2, 10, QLatin1Char('0')).arg(kJointOrder[i].c_str()), scroll_content);
      auto * current = new QLabel("--", scroll_content);
      current->setTextInteractionFlags(Qt::TextSelectableByMouse);
      auto * spin = new QDoubleSpinBox(scroll_content);
      label->setMinimumWidth(118);
      label->setMaximumWidth(140);
      current->setMinimumWidth(60);
      current->setMaximumWidth(66);
      current->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
      spin->setDecimals(4);
      spin->setRange(-6.28, 6.28);
      spin->setSingleStep(0.05);
      spin->setValue(0.0);
      spin->setMaximumWidth(92);
      joint_current_labels_.push_back(current);
      joint_spins_.push_back(spin);
      const int row = arm_rows[arm_num]++;
      arm_grids[arm_num]->addWidget(label, row, 0, Qt::AlignLeft);
      arm_grids[arm_num]->addWidget(current, row, 1, Qt::AlignLeft);
      arm_grids[arm_num]->addWidget(spin, row, 2, Qt::AlignLeft);
      connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), [this](double) {
        if (joint_realtime_check_ && joint_realtime_check_->isChecked()) {
          joint_publish_timer_->start();
        }
      });
    }
    joint_groups_layout->setColumnStretch(0, 0);
    joint_groups_layout->setColumnStretch(1, 0);
    joint_groups_layout->setColumnMinimumWidth(0, 438);
    joint_groups_layout->setColumnMinimumWidth(1, 438);
    joint_groups_layout->setColumnStretch(2, 1);
    auto * joint_btns = new QHBoxLayout();
    auto * btn_publish = new QPushButton("Publish Once", joint_tab);
    auto * btn_zero = new QPushButton("Set All Zero", joint_tab);
    auto * btn_copy_current = new QPushButton("Copy Current -> Target", joint_tab);
    joint_realtime_check_ = new QCheckBox("Realtime", joint_tab);
    joint_realtime_check_->setChecked(true);
    joint_status_label_ = new QLabel("Status: idle", joint_tab);
    joint_btns->addWidget(btn_publish);
    joint_btns->addWidget(btn_zero);
    joint_btns->addWidget(btn_copy_current);
    joint_btns->addWidget(joint_realtime_check_);
    joint_btns->addWidget(joint_status_label_);
    joint_btns->addStretch();
    joint_layout->addLayout(joint_btns);
    connect(btn_publish, &QPushButton::clicked, [this]() { publish_joint_command(); });
    connect(btn_zero, &QPushButton::clicked, [this]() {
      for (auto * s : joint_spins_) {
        s->setValue(0.0);
      }
      publish_joint_command();
    });
    connect(btn_copy_current, &QPushButton::clicked, [this]() { copy_current_joint_to_target(); });

    // Pose tab
    auto * pose_layout = new QVBoxLayout(pose_tab);

    auto * group_arm = new QGroupBox("Arm / Frame", pose_tab);
    auto * arm_form = new QFormLayout(group_arm);
    arm_form->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    arm_form->setHorizontalSpacing(12);
    arm_form->setVerticalSpacing(7);
    arm_id_combo_ = new QComboBox(group_arm);
    arm_id_combo_->setMaximumWidth(90);
    arm_id_combo_->addItem("0");
    frame_id_combo_ = new QComboBox(group_arm);
    frame_id_combo_->setMaximumWidth(180);
    frame_id_combo_->setInsertPolicy(QComboBox::NoInsert);
    frame_id_combo_->addItems({"base_link", "world"});
    arm_form->addRow("arm_id", arm_id_combo_);
    arm_form->addRow("frame_id", frame_id_combo_);
    pose_layout->addWidget(group_arm);

    auto * group_pose = new QGroupBox("Target Pose", pose_tab);
    auto * pose_split = new QHBoxLayout(group_pose);
    auto * pose_form_widget = new QWidget(group_pose);
    pose_form_widget->setMinimumWidth(340);
    auto * pose_form = new QFormLayout(pose_form_widget);
    pose_form->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    pose_form->setHorizontalSpacing(14);
    pose_form->setVerticalSpacing(8);
    orientation_mode_ = new QComboBox(group_pose);
    orientation_mode_->addItems({"rpy", "quaternion"});
    px_ = create_spin(group_pose, -5.0, 5.0, 0.35);
    py_ = create_spin(group_pose, -5.0, 5.0, -0.10);
    pz_ = create_spin(group_pose, -5.0, 5.0, 0.25);
    roll_ = create_spin(group_pose, -3.1416, 3.1416, 0.0);
    pitch_ = create_spin(group_pose, -3.1416, 3.1416, 0.0);
    yaw_ = create_spin(group_pose, -3.1416, 3.1416, 0.0);
    qx_ = create_spin(group_pose, -1.0, 1.0, 0.0);
    qy_ = create_spin(group_pose, -1.0, 1.0, 0.0);
    qz_ = create_spin(group_pose, -1.0, 1.0, 0.0);
    qw_ = create_spin(group_pose, -1.0, 1.0, 1.0);
    pose_form->addRow("orientation_mode", orientation_mode_);
    pose_form->addRow("x", px_);
    pose_form->addRow("y", py_);
    pose_form->addRow("z", pz_);
    roll_label_ = new QLabel("roll", group_pose);
    pitch_label_ = new QLabel("pitch", group_pose);
    yaw_label_ = new QLabel("yaw", group_pose);
    qx_label_ = new QLabel("qx", group_pose);
    qy_label_ = new QLabel("qy", group_pose);
    qz_label_ = new QLabel("qz", group_pose);
    qw_label_ = new QLabel("qw", group_pose);
    pose_form->addRow(roll_label_, roll_);
    pose_form->addRow(pitch_label_, pitch_);
    pose_form->addRow(yaw_label_, yaw_);
    pose_form->addRow(qx_label_, qx_);
    pose_form->addRow(qy_label_, qy_);
    pose_form->addRow(qz_label_, qz_);
    pose_form->addRow(qw_label_, qw_);
    pose_form_widget->setLayout(pose_form);
    pose_split->addWidget(pose_form_widget, 0);

    auto * pose_log_group = new QGroupBox("Pose Log", group_pose);
    pose_log_group->setMinimumWidth(390);
    auto * pose_log_layout = new QVBoxLayout(pose_log_group);
    pose_log_text_ = new QPlainTextEdit(pose_log_group);
    pose_log_text_->setReadOnly(true);
    pose_log_text_->setMaximumBlockCount(200);
    pose_log_text_->setPlaceholderText("Pose validation and execution logs...");
    pose_log_text_->setMinimumWidth(280);
    pose_log_layout->addWidget(pose_log_text_);
    pose_split->addWidget(pose_log_group, 1);
    pose_split->setSpacing(12);
    pose_split->setStretch(0, 1);
    pose_split->setStretch(1, 2);
    pose_layout->addWidget(group_pose);

    auto * group_gripper = new QGroupBox("Gripper (optional)", pose_tab);
    auto * grip_form = new QFormLayout(group_gripper);
    grip_form->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    grip_form->setHorizontalSpacing(12);
    grip_form->setVerticalSpacing(7);
    gripper_mode_ = new QComboBox(group_gripper);
    gripper_mode_->addItems({"none", "open", "close", "width"});
    gripper_width_ = create_spin(group_gripper, 0.0, 0.06, 0.03);
    gripper_status_label_ = new QLabel("Gripper: idle", group_gripper);
    grip_form->addRow("mode", gripper_mode_);
    grip_form->addRow("width(m)", gripper_width_);
    grip_form->addRow("status", gripper_status_label_);
    pose_layout->addWidget(group_gripper);

    auto * group_ee = new QGroupBox("Current EE Poses (Read Only)", pose_tab);
    auto * ee_grid = new QGridLayout(group_ee);
    ee_grid->addWidget(new QLabel("arm_id", group_ee), 0, 0);
    ee_grid->addWidget(new QLabel("frame_id(ref)", group_ee), 0, 1);
    ee_grid->addWidget(new QLabel("ee_frame(child)", group_ee), 0, 2);
    ee_value_header_label_ = new QLabel("pose (x y z | r p y)", group_ee);
    ee_grid->addWidget(ee_value_header_label_, 0, 3);
    for (int i = 0; i < static_cast<int>(kEePoseRows.size()); ++i) {
      auto * arm_label = new QLabel(QString::number(kEePoseRows[i]), group_ee);
      auto * ref_label = new QLabel(QString::fromStdString(current_frame_id()), group_ee);
      auto * frame_label = new QLabel(QString::fromStdString(frame_for_arm_id(kEePoseRows[i])), group_ee);
      auto * pose_label = new QLabel("--", group_ee);
      pose_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
      ee_ref_frame_labels_.push_back(ref_label);
      ee_frame_labels_.push_back(frame_label);
      ee_pose_labels_.push_back(pose_label);
      ee_grid->addWidget(arm_label, i + 1, 0);
      ee_grid->addWidget(ref_label, i + 1, 1);
      ee_grid->addWidget(frame_label, i + 1, 2);
      ee_grid->addWidget(pose_label, i + 1, 3);
    }
    auto * btn_copy_pose = new QPushButton("Copy Current -> Target Pose", group_ee);
    connect(btn_copy_pose, &QPushButton::clicked, [this]() { copy_current_pose_to_target(true); });
    ee_grid->addWidget(btn_copy_pose, static_cast<int>(kEePoseRows.size()) + 1, 3);
    pose_layout->addWidget(group_ee);

    auto * pose_btns = new QHBoxLayout();
    auto * btn_send_pose = new QPushButton("Send Pose", pose_tab);
    auto * btn_send_gripper = new QPushButton("Send Gripper Only", pose_tab);
    pose_realtime_check_ = new QCheckBox("Realtime Pose", pose_tab);
    pose_realtime_check_->setChecked(true);
    ik_status_label_ = new QLabel("IK: checking...", pose_tab);
    pose_btns->addWidget(btn_send_pose);
    pose_btns->addWidget(btn_send_gripper);
    pose_btns->addWidget(pose_realtime_check_);
    pose_btns->addWidget(ik_status_label_);
    pose_btns->addStretch();
    pose_layout->addLayout(pose_btns);

    connect(btn_send_pose, &QPushButton::clicked, [this]() { send_pose_goal_impl(true); });
    connect(btn_send_gripper, &QPushButton::clicked, [this]() { send_gripper_only(); });
    pose_send_btn_ = btn_send_pose;
    pose_gripper_btn_ = btn_send_gripper;
    pose_group_arm_ = group_arm;
    pose_group_pose_ = group_pose;

    auto pose_changed_cb = [this](auto) {
      if (pose_realtime_check_ && pose_realtime_check_->isChecked()) {
        pose_publish_timer_->start();
      }
    };

    arm_combo_debounce_ = new QTimer(this);
    arm_combo_debounce_->setSingleShot(true);
    arm_combo_debounce_->setInterval(45);
    connect(arm_combo_debounce_, &QTimer::timeout, this, [this]() {
      if (pose_realtime_check_ && pose_realtime_check_->isChecked()) {
        pose_publish_timer_->start();
      }
    });
    connect(arm_id_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
      if (suppress_combo_reactions_) {
        return;
      }
      arm_combo_debounce_->start();
    });

    frame_combo_debounce_ = new QTimer(this);
    frame_combo_debounce_->setSingleShot(true);
    frame_combo_debounce_->setInterval(120);
    connect(frame_combo_debounce_, &QTimer::timeout, this, [this]() {
      schedule_ee_pose_refresh();
      QTimer::singleShot(0, this, [this]() { copy_current_pose_to_target(false); });
      if (pose_realtime_check_ && pose_realtime_check_->isChecked()) {
        pose_publish_timer_->start();
      }
    });
    connect(frame_id_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
      if (suppress_combo_reactions_) {
        return;
      }
      frame_combo_debounce_->start();
    });

    orientation_combo_debounce_ = new QTimer(this);
    orientation_combo_debounce_->setSingleShot(true);
    orientation_combo_debounce_->setInterval(60);
    connect(orientation_combo_debounce_, &QTimer::timeout, this, [this]() {
      const bool use_rpy = orientation_mode_ && orientation_mode_->currentText() == QStringLiteral("rpy");
      update_orientation_widgets(use_rpy);
      if (pose_realtime_check_ && pose_realtime_check_->isChecked()) {
        pose_publish_timer_->start();
      }
    });
    connect(orientation_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
      if (suppress_combo_reactions_) {
        return;
      }
      orientation_combo_debounce_->start();
    });

    gripper_combo_debounce_ = new QTimer(this);
    gripper_combo_debounce_->setSingleShot(true);
    gripper_combo_debounce_->setInterval(80);
    connect(gripper_combo_debounce_, &QTimer::timeout, this, [this]() { publish_gripper_realtime(); });
    connect(gripper_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
      if (suppress_combo_reactions_) {
        return;
      }
      gripper_combo_debounce_->start();
    });

    connect(px_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(py_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(pz_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(roll_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(pitch_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(yaw_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(qx_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(qy_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(qz_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(qw_, qOverload<double>(&QDoubleSpinBox::valueChanged), pose_changed_cb);
    connect(gripper_width_, qOverload<double>(&QDoubleSpinBox::valueChanged), [this](double) {
      publish_gripper_realtime();
    });
    suppress_combo_reactions_ = true;
    orientation_mode_->setCurrentText("rpy");
    update_orientation_widgets(true);
    suppress_combo_reactions_ = false;

    setCentralWidget(central);
  }

  QDoubleSpinBox * create_spin(QWidget * parent, double lo, double hi, double v)
  {
    auto * s = new QDoubleSpinBox(parent);
    s->setDecimals(4);
    s->setRange(lo, hi);
    s->setSingleStep(0.01);
    s->setValue(v);
    s->setMaximumWidth(110);
    return s;
  }

  void publish_joint_command()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(joint_spins_.size());
    for (auto * s : joint_spins_) {
      msg.data.push_back(s->value());
    }
    cmd_pub_->publish(msg);
    if (joint_status_label_) {
      joint_status_label_->setText("Status: published");
    }
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto batch = build_joint_state_batch(*msg);
    QTimer::singleShot(0, this, [this, batch = std::move(batch)]() mutable { apply_joint_state_ui(std::move(batch)); });
  }

  void apply_joint_state_ui(JointStateUiBatch batch)
  {
    for (const auto & pr : batch.position_updates) {
      current_joint_map_[pr.first] = pr.second;
    }
    bool arm_changed = false;
    if (!batch.arm_ids.empty() && batch.arm_ids != available_arm_ids_) {
      available_arm_ids_ = batch.arm_ids;
      arm_changed = true;
    }
    for (const auto & kv : batch.joint_indices_by_arm) {
      int chosen_joint_idx = -1;
      if (kv.second.count(6) > 0) {
        chosen_joint_idx = 6;
      } else {
        for (int idx : kv.second) {
          if (idx <= 6 && idx > chosen_joint_idx) {
            chosen_joint_idx = idx;
          }
        }
        if (chosen_joint_idx < 0 && !kv.second.empty()) {
          chosen_joint_idx = *kv.second.rbegin();
        }
      }
      if (chosen_joint_idx < 0) {
        continue;
      }
      const std::string ee = "J" + std::to_string(kv.first + 1) + "_" + std::to_string(chosen_joint_idx);
      if (ee_frame_by_arm_id_[kv.first] != ee) {
        ee_frame_by_arm_id_[kv.first] = ee;
        arm_changed = true;
      }
    }
    if (arm_changed) {
      refresh_arm_frame_options();
    }
    for (int i = 0; i < static_cast<int>(kJointOrder.size()) && i < static_cast<int>(joint_current_labels_.size());
      ++i)
    {
      const auto it = current_joint_map_.find(kJointOrder[i]);
      if (it != current_joint_map_.end()) {
        joint_current_labels_[i]->setText(QString::number(it->second, 'f', 4));
      }
    }
  }

  void copy_current_joint_to_target()
  {
    for (int i = 0; i < static_cast<int>(kJointOrder.size()) && i < static_cast<int>(joint_spins_.size()); ++i) {
      const auto it = current_joint_map_.find(kJointOrder[i]);
      if (it != current_joint_map_.end()) {
        joint_spins_[i]->setValue(it->second);
      }
    }
    if (joint_status_label_) {
      joint_status_label_->setText("Status: copied current->target");
    }
  }

  void publish_arm_id(int arm_id)
  {
    std_msgs::msg::Int32 msg;
    msg.data = arm_id;
    arm_id_pub_->publish(msg);
  }

  void send_pose_goal_impl(bool show_error_popup)
  {
    if (!ik_ready_) {
      append_pose_log("IK unavailable: /compute_ik is unavailable.", true);
      if (show_error_popup) {
        QMessageBox::warning(this, "IK Unavailable", "/compute_ik is unavailable. Start MoveIt first.");
      }
      return;
    }

    const int arm_id = current_arm_id();
    if (arm_id < 0 || arm_id > 3) {
      append_pose_log(QString("Invalid arm_id=%1, expected 0~3.").arg(arm_id), true);
      return;
    }

    QString validation_error;
    if (!validate_target_pose_inputs(&validation_error)) {
      append_pose_log(validation_error, true);
      if (show_error_popup) {
        QMessageBox::warning(this, "Invalid Target Pose", validation_error);
      }
      return;
    }

    publish_arm_id(arm_id);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = current_frame_id();
    pose.header.stamp = node_->now();
    pose.pose.position.x = px_->value();
    pose.pose.position.y = py_->value();
    pose.pose.position.z = pz_->value();
    if (orientation_mode_ && orientation_mode_->currentText() == "rpy") {
      const double roll = roll_->value();
      const double pitch = pitch_->value();
      const double yaw = yaw_->value();
      const double cr = std::cos(roll * 0.5);
      const double sr = std::sin(roll * 0.5);
      const double cp = std::cos(pitch * 0.5);
      const double sp = std::sin(pitch * 0.5);
      const double cy = std::cos(yaw * 0.5);
      const double sy = std::sin(yaw * 0.5);
      pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
      pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
      pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
      pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    } else {
      pose.pose.orientation.x = qx_->value();
      pose.pose.orientation.y = qy_->value();
      pose.pose.orientation.z = qz_->value();
      pose.pose.orientation.w = qw_->value();
    }
    pose_pub_->publish(pose);
    append_pose_log(
      QString("Pose sent: arm_id=%1 frame=%2 pos=(%3,%4,%5)")
        .arg(arm_id)
        .arg(QString::fromStdString(pose.header.frame_id))
        .arg(pose.pose.position.x, 0, 'f', 3)
        .arg(pose.pose.position.y, 0, 'f', 3)
        .arg(pose.pose.position.z, 0, 'f', 3),
      false);

    if (gripper_mode_->currentText() != "none") {
      publish_gripper();
    }
  }

  void send_gripper_only()
  {
    const int arm_id = current_arm_id();
    if (arm_id > 1) {
      QMessageBox::warning(this, "Invalid arm_id", "Gripper only supports arm_id 0/1.");
      return;
    }
    publish_arm_id(arm_id);
    publish_gripper();
  }

  void refresh_ik_status()
  {
    ik_ready_ = ik_client_ && ik_client_->wait_for_service(std::chrono::milliseconds(50));
    if (ik_status_label_) {
      ik_status_label_->setText(ik_ready_ ? "IK: ready" : "IK: unavailable");
    }
    if (pose_send_btn_) {
      pose_send_btn_->setEnabled(ik_ready_);
    }
    if (pose_group_arm_) {
      pose_group_arm_->setEnabled(ik_ready_);
    }
    if (pose_group_pose_) {
      pose_group_pose_->setEnabled(ik_ready_);
    }
    if (pose_realtime_check_) {
      pose_realtime_check_->setEnabled(ik_ready_);
    }
  }

  void publish_gripper()
  {
    std_msgs::msg::String msg;
    const auto mode = gripper_mode_->currentText();
    if (mode == "width") {
      const double width = gripper_width_->value();
      if (!std::isfinite(width) || width < 0.0 || width > 0.06) {
        if (gripper_status_label_) {
          gripper_status_label_->setText("Gripper: invalid width");
        }
        RCLCPP_ERROR(node_->get_logger(), "Invalid gripper width=%.6f, expect [0.0, 0.06]", width);
        return;
      }
      msg.data = "width:" + std::to_string(width);
    } else {
      msg.data = mode.toStdString();
    }
    gripper_pub_->publish(msg);
    if (gripper_status_label_) {
      gripper_status_label_->setText("Gripper: published");
    }
  }

  void publish_gripper_realtime()
  {
    if (!gripper_mode_ || gripper_mode_->currentText() == "none") {
      if (gripper_status_label_) {
        gripper_status_label_->setText("Gripper: idle");
      }
      return;
    }

    const int arm_id = current_arm_id();
    if (arm_id > 1) {
      if (gripper_status_label_) {
        gripper_status_label_->setText("Gripper: invalid arm_id (0/1)");
      }
      RCLCPP_ERROR(node_->get_logger(), "Gripper realtime rejected: arm_id=%d, expect 0/1", arm_id);
      return;
    }

    publish_arm_id(arm_id);
    publish_gripper();
  }

  void update_orientation_widgets(bool use_rpy)
  {
    if (roll_label_) roll_label_->setVisible(use_rpy);
    if (pitch_label_) pitch_label_->setVisible(use_rpy);
    if (yaw_label_) yaw_label_->setVisible(use_rpy);
    if (roll_) roll_->setVisible(use_rpy);
    if (pitch_) pitch_->setVisible(use_rpy);
    if (yaw_) yaw_->setVisible(use_rpy);

    if (qx_label_) qx_label_->setVisible(!use_rpy);
    if (qy_label_) qy_label_->setVisible(!use_rpy);
    if (qz_label_) qz_label_->setVisible(!use_rpy);
    if (qw_label_) qw_label_->setVisible(!use_rpy);
    if (qx_) qx_->setVisible(!use_rpy);
    if (qy_) qy_->setVisible(!use_rpy);
    if (qz_) qz_->setVisible(!use_rpy);
    if (qw_) qw_->setVisible(!use_rpy);

    if (ee_value_header_label_) {
      ee_value_header_label_->setText(use_rpy ? "pose (x y z | r p y)" : "pose (x y z | qx qy qz qw)");
    }
    schedule_ee_pose_refresh();
  }

  void copy_current_pose_to_target(bool show_error_popup)
  {
    if (!arm_id_combo_ || !px_ || !py_ || !pz_ || !qx_ || !qy_ || !qz_ || !qw_ || !roll_ || !pitch_ || !yaw_) {
      append_pose_log("Copy pose failed: target controls not ready.", true);
      return;
    }
    const int arm_id = current_arm_id();
    if (arm_id < 0) {
      append_pose_log("Copy pose failed: invalid arm_id or TF buffer unavailable.", true);
      return;
    }
    const std::string ref_frame = current_frame_id();
    const std::string ee_frame = frame_for_arm_id(arm_id);
    // Use the latest successfully refreshed EE pose cache for this arm.
    const auto it = cached_ee_pose_by_arm_.find(arm_id);
    if (it == cached_ee_pose_by_arm_.end()) {
      const QString msg =
        QString("Copy pose failed: no cached pose for arm_id=%1 (%2 <- %3)")
          .arg(arm_id)
          .arg(QString::fromStdString(ref_frame))
          .arg(QString::fromStdString(ee_frame));
      append_pose_log(msg, true);
      if (show_error_popup) {
        QMessageBox::warning(this, "Copy Pose Failed", msg);
      }
      return;
    }
    const auto & p = it->second;
    px_->setValue(p[0]);
    py_->setValue(p[1]);
    pz_->setValue(p[2]);
    qx_->setValue(p[3]);
    qy_->setValue(p[4]);
    qz_->setValue(p[5]);
    qw_->setValue(p[6]);

    // quaternion -> rpy
    const double sinr_cosp = 2.0 * (p[6] * p[3] + p[4] * p[5]);
    const double cosr_cosp = 1.0 - 2.0 * (p[3] * p[3] + p[4] * p[4]);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);
    const double sinp = 2.0 * (p[6] * p[4] - p[5] * p[3]);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) {
      pitch = std::copysign(1.57079632679, sinp);
    } else {
      pitch = std::asin(sinp);
    }
    const double siny_cosp = 2.0 * (p[6] * p[5] + p[3] * p[4]);
    const double cosy_cosp = 1.0 - 2.0 * (p[4] * p[4] + p[5] * p[5]);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);
    roll_->setValue(roll);
    pitch_->setValue(pitch);
    yaw_->setValue(yaw);
    append_pose_log(
      QString("Copied current pose: T(%1 <- %2) = (%3,%4,%5)")
        .arg(QString::fromStdString(ref_frame))
        .arg(QString::fromStdString(ee_frame))
        .arg(p[0], 0, 'f', 3)
        .arg(p[1], 0, 'f', 3)
        .arg(p[2], 0, 'f', 3),
      false);
  }

  void schedule_ee_pose_refresh()
  {
    if (!tf_buffer_ || ee_pose_labels_.size() != kEePoseRows.size() || ee_frame_labels_.size() != kEePoseRows.size() ||
      ee_ref_frame_labels_.size() != kEePoseRows.size())
    {
      return;
    }
    if (ee_refresh_running_.exchange(true)) {
      return;
    }

    const std::string ref_frame = current_frame_id();
    std::vector<std::pair<int, std::string>> arm_ee;
    arm_ee.reserve(kEePoseRows.size());
    for (int arm_id : kEePoseRows) {
      arm_ee.emplace_back(arm_id, frame_for_arm_id(arm_id));
    }
    const bool use_rpy = orientation_mode_ && orientation_mode_->currentText() == "rpy";
    const auto tf_buf = tf_buffer_;

    (void)QtConcurrent::run([this, ref_frame, arm_ee, use_rpy, tf_buf]() {
      std::vector<EePoseRowResult> rows;
      rows.reserve(arm_ee.size());
      for (const auto & pr : arm_ee) {
        EePoseRowResult row;
        row.arm_id = pr.first;
        row.ref_text = QString::fromStdString(ref_frame);
        row.ee_text = QString::fromStdString(pr.second);
        try {
          const auto tf = tf_buf->lookupTransform(ref_frame, pr.second, tf2::TimePointZero);
          const auto & t = tf.transform.translation;
          const auto & q = tf.transform.rotation;
          row.pose = {t.x, t.y, t.z, q.x, q.y, q.z, q.w};
          row.tf_ok = true;
          if (use_rpy) {
            const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
            const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
            const double roll = std::atan2(sinr_cosp, cosr_cosp);
            const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
            double pitch = 0.0;
            if (std::abs(sinp) >= 1.0) {
              pitch = std::copysign(1.57079632679, sinp);
            } else {
              pitch = std::asin(sinp);
            }
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            const double yaw = std::atan2(siny_cosp, cosy_cosp);
            row.pose_text = QString("(%1 %2 %3 | %4 %5 %6)")
              .arg(t.x, 0, 'f', 3)
              .arg(t.y, 0, 'f', 3)
              .arg(t.z, 0, 'f', 3)
              .arg(roll, 0, 'f', 3)
              .arg(pitch, 0, 'f', 3)
              .arg(yaw, 0, 'f', 3);
          } else {
            row.pose_text = QString("(%1 %2 %3 | %4 %5 %6 %7)")
              .arg(t.x, 0, 'f', 3)
              .arg(t.y, 0, 'f', 3)
              .arg(t.z, 0, 'f', 3)
              .arg(q.x, 0, 'f', 3)
              .arg(q.y, 0, 'f', 3)
              .arg(q.z, 0, 'f', 3)
              .arg(q.w, 0, 'f', 3);
          }
        } catch (const tf2::TransformException &) {
          row.tf_ok = false;
          row.pose_text = QStringLiteral("N/A");
        }
        rows.push_back(std::move(row));
      }

      QTimer::singleShot(0, this, [this, rows = std::move(rows)]() mutable {
        apply_ee_pose_rows(std::move(rows));
        ee_refresh_running_.store(false);
      });
    });
  }

  void apply_ee_pose_rows(std::vector<EePoseRowResult> rows)
  {
    for (size_t i = 0; i < rows.size() && i < ee_pose_labels_.size(); ++i) {
      const auto & r = rows[i];
      ee_ref_frame_labels_[i]->setText(r.ref_text);
      ee_frame_labels_[i]->setText(r.ee_text);
      if (r.tf_ok) {
        cached_ee_pose_by_arm_[r.arm_id] = r.pose;
        ee_pose_labels_[i]->setText(r.pose_text);
      } else {
        cached_ee_pose_by_arm_.erase(r.arm_id);
        ee_pose_labels_[i]->setText(r.pose_text);
      }
    }
  }

  bool validate_target_pose_inputs(QString * error_text) const
  {
    if (!error_text) {
      return false;
    }
    const auto frame = current_frame_id();
    if (frame.empty()) {
      *error_text = "Invalid frame_id: empty frame.";
      return false;
    }
    if (!std::isfinite(px_->value()) || !std::isfinite(py_->value()) || !std::isfinite(pz_->value())) {
      *error_text = "Invalid position: x/y/z must be finite values.";
      return false;
    }

    if (orientation_mode_ && orientation_mode_->currentText() == "quaternion") {
      const double qx = qx_->value();
      const double qy = qy_->value();
      const double qz = qz_->value();
      const double qw = qw_->value();
      if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw)) {
        *error_text = "Invalid quaternion: qx/qy/qz/qw must be finite values.";
        return false;
      }
      const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
      if (norm < 1e-6) {
        *error_text = "Invalid quaternion: norm is near zero.";
        return false;
      }
    }
    return true;
  }

  void append_pose_log(const QString & text, bool is_error)
  {
    if (!pose_log_text_) {
      return;
    }
    QString line;
    if (text.startsWith("[ERROR]") || text.startsWith("[INFO]")) {
      line = text;
    } else {
      line = (is_error ? QStringLiteral("[ERROR] ") : QStringLiteral("[INFO] ")) + text;
    }
    const bool line_is_error =
      is_error || line.startsWith(QStringLiteral("[ERROR]"));

    QTextCursor cursor(pose_log_text_->document());
    cursor.movePosition(QTextCursor::End);
    QTextCharFormat fmt;
    if (line_is_error) {
      fmt.setForeground(QBrush(QColor(200, 40, 40)));
    } else {
      fmt.setForeground(QBrush(pose_log_text_->palette().color(QPalette::WindowText)));
    }
    cursor.setCharFormat(fmt);
    cursor.insertText(line + QLatin1Char('\n'));
    pose_log_text_->moveCursor(QTextCursor::End);
  }

  int current_arm_id() const
  {
    if (!arm_id_combo_) {
      return 0;
    }
    bool ok = false;
    const int value = arm_id_combo_->currentText().toInt(&ok);
    return ok ? value : 0;
  }

  std::string current_frame_id() const
  {
    if (!frame_id_combo_ || frame_id_combo_->currentText().isEmpty()) {
      return "base_link";
    }
    return frame_id_combo_->currentText().toStdString();
  }

  std::string frame_for_arm_id(int arm_id) const
  {
    const auto it = ee_frame_by_arm_id_.find(arm_id);
    if (it != ee_frame_by_arm_id_.end()) {
      return it->second;
    }
    return "J" + std::to_string(arm_id + 1) + "_6";
  }

  void refresh_arm_frame_options()
  {
    if (!arm_id_combo_ || !frame_id_combo_) {
      return;
    }

    suppress_combo_reactions_ = true;
    const QString old_arm = arm_id_combo_->currentText();
    arm_id_combo_->blockSignals(true);
    arm_id_combo_->clear();
    if (available_arm_ids_.empty()) {
      arm_id_combo_->addItem("0");
    } else {
      for (int id : available_arm_ids_) {
        arm_id_combo_->addItem(QString::number(id));
      }
    }
    int arm_idx = arm_id_combo_->findText(old_arm);
    if (arm_idx < 0) {
      arm_idx = 0;
    }
    arm_id_combo_->setCurrentIndex(arm_idx);
    arm_id_combo_->blockSignals(false);

    const QString old_frame = frame_id_combo_->currentText();
    frame_id_combo_->blockSignals(true);
    frame_id_combo_->clear();
    frame_id_combo_->addItem("base_link");
    frame_id_combo_->addItem("world");
    std::set<std::string> added_frames = {"base_link", "world"};
    for (int id : available_arm_ids_) {
      const std::string ee = frame_for_arm_id(id);
      if (added_frames.insert(ee).second) {
        frame_id_combo_->addItem(QString::fromStdString(ee));
      }
    }
    int frame_idx = frame_id_combo_->findText(old_frame);
    if (frame_idx < 0) {
      frame_idx = 0;
    }
    frame_id_combo_->setCurrentIndex(frame_idx);
    frame_id_combo_->blockSignals(false);
    suppress_combo_reactions_ = false;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_id_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pose_log_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool ik_ready_{false};

  std::vector<QDoubleSpinBox *> joint_spins_;
  std::vector<QLabel *> joint_current_labels_;
  std::unordered_map<std::string, double> current_joint_map_;
  QCheckBox * joint_realtime_check_{nullptr};
  QLabel * joint_status_label_{nullptr};
  QComboBox * arm_id_combo_{nullptr};
  QComboBox * frame_id_combo_{nullptr};
  QDoubleSpinBox * px_{nullptr};
  QDoubleSpinBox * py_{nullptr};
  QDoubleSpinBox * pz_{nullptr};
  QComboBox * orientation_mode_{nullptr};
  QLabel * roll_label_{nullptr};
  QLabel * pitch_label_{nullptr};
  QLabel * yaw_label_{nullptr};
  QDoubleSpinBox * roll_{nullptr};
  QDoubleSpinBox * pitch_{nullptr};
  QDoubleSpinBox * yaw_{nullptr};
  QLabel * qx_label_{nullptr};
  QLabel * qy_label_{nullptr};
  QLabel * qz_label_{nullptr};
  QLabel * qw_label_{nullptr};
  QDoubleSpinBox * qx_{nullptr};
  QDoubleSpinBox * qy_{nullptr};
  QDoubleSpinBox * qz_{nullptr};
  QDoubleSpinBox * qw_{nullptr};
  QPlainTextEdit * pose_log_text_{nullptr};
  QComboBox * gripper_mode_{nullptr};
  QDoubleSpinBox * gripper_width_{nullptr};
  QLabel * gripper_status_label_{nullptr};
  QCheckBox * pose_realtime_check_{nullptr};
  QLabel * ik_status_label_{nullptr};
  QPushButton * pose_send_btn_{nullptr};
  QPushButton * pose_gripper_btn_{nullptr};
  QGroupBox * pose_group_arm_{nullptr};
  QGroupBox * pose_group_pose_{nullptr};
  std::vector<QLabel *> ee_pose_labels_;
  std::vector<QLabel *> ee_ref_frame_labels_;
  std::vector<QLabel *> ee_frame_labels_;
  QLabel * ee_value_header_label_{nullptr};
  std::unordered_map<int, std::array<double, 7>> cached_ee_pose_by_arm_;
  std::set<int> available_arm_ids_;
  std::unordered_map<int, std::string> ee_frame_by_arm_id_;
  QTimer * joint_publish_timer_{nullptr};
  QTimer * pose_publish_timer_{nullptr};
  QTimer * ik_watchdog_timer_{nullptr};
  QTimer * ee_pose_timer_{nullptr};
  QTimer * arm_combo_debounce_{nullptr};
  QTimer * frame_combo_debounce_{nullptr};
  QTimer * orientation_combo_debounce_{nullptr};
  QTimer * gripper_combo_debounce_{nullptr};
  bool suppress_combo_reactions_{false};

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
  std::thread ros_spin_thread_;
  std::atomic<bool> ros_spin_exit_{false};
  std::atomic<bool> ee_refresh_running_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("nova_control_ui_qt_node");
  NovaControlWindow win(node);
  win.show();

  // Ensure Qt app exits promptly when ROS receives SIGINT/SIGTERM.
  QTimer ros_shutdown_watchdog;
  ros_shutdown_watchdog.setInterval(100);
  QObject::connect(&ros_shutdown_watchdog, &QTimer::timeout, [&app]() {
    if (!rclcpp::ok()) {
      app.quit();
    }
  });
  ros_shutdown_watchdog.start();

  QObject::connect(&app, &QCoreApplication::aboutToQuit, []() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  });

  const int ret = app.exec();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return ret;
}
