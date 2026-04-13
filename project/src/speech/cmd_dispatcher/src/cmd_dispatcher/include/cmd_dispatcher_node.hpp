#pragma once
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs_comm/msg/interactive_command.hpp"
#include "custom_msgs_comm/msg/voice_command.hpp"
#include "custom_msgs_comm/msg/speech_status.hpp"  // 添加SpeechStatus消息头文件
#include "custom_msgs_comm/msg/voice_reply.hpp"   
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "speech_commands.hpp"
#include "std_msgs/msg/string.hpp"

namespace cmd_dispatcher
{
  /* 返回最高分 Detection 的 3D 中心；若无此类别返回 std::nullopt */
  std::optional<geometry_msgs::msg::Point> get_best_detection_center(
        const vision_msgs::msg::Detection2DArray & det_array,const std::string & target_class);
class CmdDispatcherNode : public rclcpp::Node
{
public:
  explicit CmdDispatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void voice_callback(const custom_msgs_comm::msg::VoiceCommand::SharedPtr msg);
  void awake_status_callback(const custom_msgs_comm::msg::SpeechStatus::SharedPtr msg);  // 添加语音唤醒状态回调函数
  void ai_status_callback(const std_msgs::msg::String::SharedPtr msg);
  int  parse_command(const std::string & voice); // 返回 -1/0/1/2
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  rclcpp::Subscription<custom_msgs_comm::msg::VoiceCommand>::SharedPtr voice_sub_;
  rclcpp::Subscription<custom_msgs_comm::msg::SpeechStatus>::SharedPtr awake_status_sub_;  // 添加语音唤醒状态订阅者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_status_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Publisher<custom_msgs_comm::msg::InteractiveCommand>::SharedPtr icmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reply_sub_;  // 修改为String类型
  std::unordered_set<int32_t> sent_;
  bool is_awake_ = false;  // 添加语音唤醒状态变量，默认为休眠状态
  vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
  std::string class_name_;
};

}  // namespace cmd_dispatcher