// CalibSimUiNode：订阅图像与状态字符串，定时 imshow 刷新。
#include "calib_sim/calib_ui_node.hpp"

#include <chrono>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace calib_sim
{

CalibSimUiNode::CalibSimUiNode(const rclcpp::NodeOptions & options)
: Node("calib_sim_ui_node", options)
{
  this->declare_parameter("status_topic", std::string("/calib_sim/status"));
  this->declare_parameter("raw_image_topic", std::string("/calib_sim/raw_image"));
  this->declare_parameter("result_image_topic", std::string("/calib_sim/result_image"));
  this->declare_parameter("window_name", std::string("calib_sim_ui"));

  status_topic_ = this->get_parameter("status_topic").as_string();
  raw_image_topic_ = this->get_parameter("raw_image_topic").as_string();
  result_image_topic_ = this->get_parameter("result_image_topic").as_string();
  window_name_ = this->get_parameter("window_name").as_string();

  status_sub_ = this->create_subscription<std_msgs::msg::String>(
    status_topic_, 10, [this](const std_msgs::msg::String::SharedPtr msg) { status_ = msg->data; });
  raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    raw_image_topic_, 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) { raw_ = toBgr(*msg); });
  result_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    result_image_topic_, 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { result_ = toBgr(*msg); });

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { render(); });
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
  RCLCPP_INFO(get_logger(), "calib_sim_ui_node started.");
}

cv::Mat CalibSimUiNode::toBgr(const sensor_msgs::msg::Image & msg)
{
  if (msg.height == 0 || msg.width == 0) {
    return cv::Mat();
  }
  cv::Mat raw(
    static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
    const_cast<unsigned char *>(msg.data.data()), msg.step);
  if (msg.encoding == "bgr8") {
    return raw.clone();
  }
  if (msg.encoding == "rgb8") {
    cv::Mat bgr;
    cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  return raw.clone();
}

void CalibSimUiNode::render()
{
  cv::Mat left = raw_.empty() ? cv::Mat::zeros(480, 640, CV_8UC3) : raw_;
  cv::Mat right = result_.empty() ? cv::Mat::zeros(480, 640, CV_8UC3) : result_;
  if (left.size() != right.size()) {
    cv::resize(right, right, left.size());
  }
  cv::Mat canvas(left.rows + 60, left.cols * 2, CV_8UC3, cv::Scalar(25, 25, 25));
  left.copyTo(canvas(cv::Rect(0, 60, left.cols, left.rows)));
  right.copyTo(canvas(cv::Rect(left.cols, 60, right.cols, right.rows)));
  cv::putText(canvas, "RAW", cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 255), 2);
  cv::putText(
    canvas, "RESULT", cv::Point(left.cols + 10, 45),
    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 255, 200), 2);
  cv::putText(
    canvas, "Status: " + status_, cv::Point(260, 35),
    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  cv::imshow(window_name_, canvas);
  cv::waitKey(1);
}

}  // namespace calib_sim
