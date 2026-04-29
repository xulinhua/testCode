#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QMainWindow>

#include <opencv2/core/mat.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class QComboBox;
class QGridLayout;
class QLabel;
class QPoint;
class QPointF;
class QProcess;
class QPushButton;
class QSpinBox;
class QTimer;
class QToolButton;
class QWidget;

namespace ImageDebug {

class ImageViewWidget;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override = default;

private:
  void setupMenuBar();
  void setupCentralWidget();
  void setupRosTimers();
  void setToolBoxVisible(bool visible);
  void saveVisibleImages();
  void zoomVisibleImages(bool zoom_in);
  void toggleRosbagRecording();
  void updateRecordButtons();
  void handleSyncHover(int source_index, const QPoint &pixel, bool active);
  void handleSyncTransform(int source_index, double scale_factor, const QPointF &pan_offset);
  void relayoutViewerCards(int active_count);
  void updateViewerVisibility();
  void refreshTopicList();
  void subscribeToTopic(int viewer_index, const QString &topic_name);
  void updateImageInfo(int viewer_index, const QString &image_type, int width, int height);
  void showAboutDialog();

  static bool isImageTopicType(const std::vector<std::string> &types);
  static bool rosImageToDisplayAndRaw(const sensor_msgs::msg::Image::SharedPtr &msg,
                                      QImage *display_image, cv::Mat *raw_image,
                                      QString *encoding);

  std::shared_ptr<rclcpp::Node> node_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;

  QSpinBox *viewer_count_spin_;
  QGridLayout *viewer_grid_;
  QAction *toolbox_action_;
  QAction *sync_view_action_;
  QProcess *rosbag_process_;
  bool rosbag_recording_;
  bool syncing_transform_;
  std::vector<QWidget *> viewer_cards_;
  std::vector<QWidget *> overlay_toolbars_;
  std::vector<QToolButton *> record_buttons_;
  std::vector<QComboBox *> topic_combos_;
  std::vector<QLabel *> image_info_labels_;
  std::vector<ImageViewWidget *> image_views_;
  QTimer *spin_timer_;
  QTimer *topic_timer_;
};

}  // namespace ImageDebug
