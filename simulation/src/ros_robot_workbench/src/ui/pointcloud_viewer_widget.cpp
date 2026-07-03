#include "ros_robot_workbench/ui/pointcloud_viewer_widget.h"

#include <cmath>
#include <mutex>

#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ros_robot_workbench/module/pointcloud_viewer_module.h"
#include "ros_robot_workbench/ui/shared_ui_executor.hpp"

namespace ros_robot_workbench::ui
{

PointcloudViewerWidget::PointcloudViewerWidget(QWidget * parent)
: QWidget(parent)
, node_(rclcpp::Node::make_shared("pointcloud_viewer_ui", rclcpp::NodeOptions()))
{
  QVBoxLayout * root = new QVBoxLayout(this);
  QLabel * title = new QLabel("点云查看");
  title->setStyleSheet("font-size: 20px; font-weight: 600; color: #22415b;");
  root->addWidget(title);
  root->addWidget(new QLabel(PointcloudViewerModuleSummary()));

  QGroupBox * cfg = new QGroupBox("订阅");
  QFormLayout * form = new QFormLayout(cfg);
  QLineEdit * topic = new QLineEdit("/points");
  form->addRow("PointCloud2:", topic);
  root->addWidget(cfg);

  QPlainTextEdit * log = new QPlainTextEdit();
  log->setReadOnly(true);
  root->addWidget(log, 1);

  struct Stats
  {
    std::mutex mu;
    sensor_msgs::msg::PointCloud2 last;
  };
  auto stats = std::make_shared<Stats>();

  auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic->text().toStdString(), rclcpp::SensorDataQoS(),
    [stats](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(stats->mu);
      stats->last = *msg;
    });
  (void)sub;
  SharedUiExecutor::instance().add_node(node_);

  QPushButton * analyze = new QPushButton("统计点云");
  QObject::connect(analyze, &QPushButton::clicked, [log, stats]() {
    sensor_msgs::msg::PointCloud2 cloud;
    {
      std::lock_guard<std::mutex> lock(stats->mu);
      cloud = stats->last;
    }
    if (cloud.width == 0 || cloud.data.empty()) {
      log->appendPlainText("尚未收到 PointCloud2");
      return;
    }
    const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
    double min_z = 1e9;
    double max_z = -1e9;
    size_t valid = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
      for (size_t i = 0; i < n; ++i, ++iter_z) {
        const float z = *iter_z;
        if (!std::isfinite(z)) {
          continue;
        }
        ++valid;
        min_z = std::min(min_z, static_cast<double>(z));
        max_z = std::max(max_z, static_cast<double>(z));
      }
    } catch (const std::runtime_error & e) {
      log->appendPlainText(QString("解析失败: %1").arg(e.what()));
      return;
    }
    log->appendPlainText(
      QString("frame=%1  points=%2  valid=%3  z=[%4, %5]")
        .arg(QString::fromStdString(cloud.header.frame_id))
        .arg(n)
        .arg(valid)
        .arg(min_z, 0, 'g', 4)
        .arg(max_z, 0, 'g', 4));
  });
  root->addWidget(analyze);
}

}  // namespace ros_robot_workbench::ui
