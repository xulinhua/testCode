#include "cmd_dispatcher_node.hpp"
#include "voice_type.hpp"
#include "custom_msgs_comm/msg/speech_status.hpp"  // 添加SpeechStatus消息头文件
using std::placeholders::_1;

namespace cmd_dispatcher
{

std::optional<geometry_msgs::msg::Point>
get_best_detection_center(const vision_msgs::msg::Detection2DArray & det_array,
                          const std::string & target_class)
{
  const vision_msgs::msg::Detection2D * best_det = nullptr;
  double best_score = -1.0;

  for (const auto & det : det_array.detections) {
    if (det.results.empty()) continue;
    for (const auto & hyp : det.results) {
      if (hyp.hypothesis.class_id == target_class &&
          hyp.hypothesis.score > best_score)
      {
        best_score = hyp.hypothesis.score;
        best_det = &det;
      }
    }
  }

   if (!best_det || best_det->results.empty())  return std::nullopt;

  /* 取 3D pose 中心（假设已有有效 3D 坐标） */
  return best_det->results.front().pose.pose.position;
}
CmdDispatcherNode::CmdDispatcherNode(const rclcpp::NodeOptions & options)
: Node("cmd_dispatcher", options)
{
  voice_sub_ = create_subscription<custom_msgs_comm::msg::VoiceCommand>(
    "/voice_command", 10, std::bind(&CmdDispatcherNode::voice_callback, this, _1));

  ai_status_sub_ = create_subscription<std_msgs::msg::String>(
  "/ai_status", 10, std::bind(&CmdDispatcherNode::ai_status_callback, this, _1));

  det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/det_res", 10, std::bind(&CmdDispatcherNode::detection_callback, this, _1));
    
  // 添加语音状态订阅者
  awake_status_sub_ = create_subscription<custom_msgs_comm::msg::SpeechStatus>(
    "/speech/awake_status", 10, std::bind(&CmdDispatcherNode::awake_status_callback, this, _1));

  icmd_pub_ = create_publisher<custom_msgs_comm::msg::InteractiveCommand>(
    "/interactive_command", 10);
  
  reply_sub_ = create_publisher<std_msgs::msg::String>(
    "/voice_reply", 10);
    
  // 初始化latest_detections_
  latest_detections_ = std::make_shared<vision_msgs::msg::Detection2DArray>();

  RCLCPP_INFO(get_logger(), "CmdDispatcher(voice+vision) ready.");
}

void CmdDispatcherNode::detection_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  latest_detections_ = msg;   // 缓存最新一帧
}

void CmdDispatcherNode::voice_callback(const custom_msgs_comm::msg::VoiceCommand::SharedPtr msg)
{
   RCLCPP_INFO(get_logger(), "进入语音调度回调");
  // 检查是否处于唤醒状态，如果不是唤醒状态则忽略指令
  if (!is_awake_) 
  {
    RCLCPP_INFO(get_logger(), "设备处于休眠状态，忽略语音指令: %s", msg->txt.c_str());
    return;
  }
  
  int nVoiceID = msg->voice_id;   
  if (sent_.count(nVoiceID)) 
  {
    RCLCPP_INFO(get_logger(), "语音 id=%d 已处理过，跳过", nVoiceID);
    return;
  }
  cmd_dispatcher::SpeechCmdType cmd_type = cmd_dispatcher::SpeechCmdType::UNKNOWN_SPEECH;
  if (msg->voice_type == static_cast<uint8_t>(cmd_dispatcher::Voice_Type::WAKEUP_ENABLE)) 
  {// 当前声音为：唤醒词激活
    RCLCPP_INFO(get_logger(), "接收到唤醒模式开启指令: %s", msg->txt.c_str());
    cmd_type = cmd_dispatcher::SpeechCmdType::WELCOME_SPEECH;
  } 
  else if (msg->voice_type == static_cast<uint8_t>(cmd_dispatcher::Voice_Type::WAKEUP_DISABLE)) 
  {// 当前声音为：关闭播报
    RCLCPP_INFO(get_logger(), "接收到关闭播报指令: %s", msg->txt.c_str());
    cmd_type = cmd_dispatcher::SpeechCmdType::TURN_OFF_SPEECH;
  }
  else
  {
    cmd_type = cmd_dispatcher::get_speech_command(msg->txt);// 根据文本获取解析的命令类型
  }
  RCLCPP_INFO(get_logger(), "识别结果: %s, 命令类型: %d", msg->txt.c_str(), cmd_type);
  // if (cmd_type == cmd_dispatcher::SpeechCmdType::UNKNOWN_SPEECH) 
  // {
  //   RCLCPP_INFO(get_logger(), "语音『%s』相似度不足，忽略", msg->txt.c_str());   
  //   return;
  // }
  auto vreply_msg = std_msgs::msg::String();  // 修改为String类型
  vreply_msg.data = get_speech_reply(cmd_type, msg->txt);// 获取语音命令的回复内容
  reply_sub_->publish(vreply_msg);// 发布回复消息
  RCLCPP_INFO(get_logger(), "发布 voice_reply: %s", vreply_msg.data.c_str());
  const int cmdID = cmd_dispatcher::get_act_task_id(cmd_type);
  if (cmdID >= 18)
  {
    auto icmd = custom_msgs_comm::msg::InteractiveCommand();
    icmd.act_command_type = cmdID;//获取动作任务ID
    icmd.data = msg->txt;
    //icmd.pose.header.frame_id = "base_link";
    icmd.pose.header.frame_id = "camera_link";
    icmd.pose.header.stamp = now();
    // 检查latest_detections_是否已初始化
    if (!latest_detections_) {
      RCLCPP_WARN(get_logger(), "视觉检测数据未初始化，跳过目标定位");
      return;
    }
    // std::string class_name = "";
    if(icmd.act_command_type == 18)
    {
      class_name_ = "box";
    }
    else if(icmd.act_command_type == 19)
    {
      class_name_ = "desk";
    }
    else if(icmd.act_command_type == 20)
    {
      class_name_ = "laptop";
    }
    auto center = get_best_detection_center(*latest_detections_, class_name_);
    if (!center) 
    {
      RCLCPP_WARN(get_logger(), "未找到目标");
      return;
    }
    icmd.pose.pose.position.x = center->x;
    icmd.pose.pose.position.y = center->y;
    icmd.pose.pose.position.z = center->z;
    icmd_pub_->publish(icmd);
    // 使用新的接口获取命令的中文语义
    RCLCPP_INFO(get_logger(), "发布 InteractiveCommand: type=%d , data=%s", icmd.act_command_type, icmd.data.c_str());
  }
  sent_.insert(nVoiceID);
}

void CmdDispatcherNode::ai_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if(msg->data == "arrived_final_pose")
  {
    return;
  }
  else if(msg->data == "continue_ai_scan")
  {
    auto center = get_best_detection_center(*latest_detections_, class_name_);
    if (!center) {
      RCLCPP_WARN(get_logger(), "ai未找到目标");
      return;
    }
    auto icmd = custom_msgs_comm::msg::InteractiveCommand();
    
    icmd.act_command_type = 18;
    icmd.data = msg->data;
    icmd.pose.header.frame_id = "base_link";
    icmd.pose.header.stamp = now();
    icmd.pose.pose.position.x = center->x;
    icmd.pose.pose.position.y = center->y;
    icmd.pose.pose.position.z = center->z;
    icmd_pub_->publish(icmd);
    RCLCPP_INFO(get_logger(), "发布 InteractiveCommand: type=%d , data=%s",
                icmd.act_command_type, icmd.data.c_str());
  }
  
}

void CmdDispatcherNode::awake_status_callback(const custom_msgs_comm::msg::SpeechStatus::SharedPtr msg)
{
  if (msg->awake_status == custom_msgs_comm::msg::SpeechStatus::SLEEP_STATUS) 
  {
    is_awake_ = false;
    auto vreply_msg = std_msgs::msg::String();  // 修改为String类型
    cmd_dispatcher::SpeechCmdType cmd_type = cmd_dispatcher::SpeechCmdType::TURN_OFF_SPEECH;
    vreply_msg.data = get_speech_reply(cmd_type, "");// 获取语音命令的回复内容
    reply_sub_->publish(vreply_msg);// 发布回复消息
    RCLCPP_INFO(get_logger(), "发布 voice_reply: %s", vreply_msg.data.c_str());
    RCLCPP_INFO(get_logger(), "设备进入休眠状态");
  }
  else
  {
    is_awake_ = true;
    RCLCPP_INFO(get_logger(), "设备进入唤醒状态");
  }
}

}  // namespace cmd_dispatcher

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cmd_dispatcher::CmdDispatcherNode>());
  rclcpp::shutdown();
  return 0;
}