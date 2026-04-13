// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <chrono>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <portaudio.h>  // 添加PortAudio头文件以使用格式常量

// 条件包含espeak-ng头文件
#ifdef HAVE_ESPEAK_NG
#include <espeak-ng/speak_lib.h>
#endif

#include "audio_common/tts_node.hpp"
#include "audio_common/wave_file.hpp"
#include "audio_common_msgs/action/tts.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

TtsNode::TtsNode() : Node("tts_node") {
  this->declare_parameter("chunk", 4096);
  this->declare_parameter("frame_id", "");
  this->declare_parameter("save_audio_files", true);  // 添加控制是否保存音频文件的参数

  this->chunk_ = this->get_parameter("chunk").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();
  this->save_audio_files_ = this->get_parameter("save_audio_files").as_bool();  // 获取参数值

  this->player_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio/speaker", rclcpp::SensorDataQoS());
  
  this->text_sub =
      this->create_subscription<std_msgs::msg::String>(
          "/voice_reply", rclcpp::SensorDataQoS(),
          std::bind(&TtsNode::execute_callback, this, _1));

  // // Action server
  // this->action_server_ = rclcpp_action::create_server<TTS>(
  //     this, "say", std::bind(&TtsNode::handle_goal, this, _1, _2),
  //     std::bind(&TtsNode::handle_cancel, this, _1),
  //     std::bind(&TtsNode::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "TTS node started");
}

// rclcpp_action::GoalResponse
// TtsNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
//                      std::shared_ptr<const TTS::Goal> goal) {
//   (void)uuid;
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

// rclcpp_action::CancelResponse
// TtsNode::handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle) {
//   RCLCPP_INFO(this->get_logger(), "Canceling TTS...");
//   (void)goal_handle;
//   return rclcpp_action::CancelResponse::ACCEPT;
// }

// void TtsNode::handle_accepted(
//     const std::shared_ptr<GoalHandleTTS> goal_handle) {
//   std::unique_lock<std::mutex> lock(this->goal_lock_);
//   if (this->goal_handle_ != nullptr && this->goal_handle_->is_active()) {
//     auto result = std::make_shared<TTS::Result>();
//     this->goal_handle_->abort(result);
//     this->goal_handle_ = goal_handle;
//   }

//   std::thread{std::bind(&TtsNode::execute_callback, this, _1), goal_handle}
//       .detach();
// }

void TtsNode::execute_callback(const std_msgs::msg::String::SharedPtr text_msg) 
{
  std::string text = text_msg->data;
  // 如果文本为空，直接返回
  if (text.empty()) 
  {
    RCLCPP_WARN(this->get_logger(), "接收到空文本，跳过TTS处理");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "开始TTS处理文本: %s", text.c_str());
  
  // 使用espeak-ng直接生成音频数据
  this->synthesize_and_publish(text);
  
  RCLCPP_INFO(this->get_logger(), "TTS处理完成");
}

#ifdef HAVE_ESPEAK_NG
// 添加全局变量来存储音频数据
static std::vector<int16_t> g_audio_data;
static int g_sample_rate = 0;
static int g_channels = 1;

// espeak-ng回调函数
int SynthCallback(short* wav, int numsamples, espeak_EVENT* events) {
  if (wav != nullptr && numsamples > 0) {
    // 将音频数据添加到全局缓冲区
    g_audio_data.insert(g_audio_data.end(), wav, wav + numsamples);
  }
  
  // 检查是否是结束事件
  espeak_EVENT* event = events;
  while (event->type != espeakEVENT_LIST_TERMINATED) {
    if (event->type == espeakEVENT_MSG_TERMINATED) {
      // 合成完成
      break;
    }
    event++;
  }
  
  return 0; // 继续合成
}

void TtsNode::synthesize_and_publish(const std::string& text) {
  // 清空之前的音频数据
  g_audio_data.clear();
  
  // 初始化espeak-ng
  g_sample_rate = espeak_Initialize(AUDIO_OUTPUT_RETRIEVAL, 200, nullptr, 0);
  if (g_sample_rate == -1) {
    RCLCPP_ERROR(this->get_logger(), "espeak-ng初始化失败");
    return;
  }
  
  // 设置回调函数
  espeak_SetSynthCallback(SynthCallback);
  
  // 设置参数
  espeak_SetParameter(espeakRATE, 150, 0);  // 语速
  espeak_SetParameter(espeakVOLUME, 100, 0);  // 音量
  
  // 合成语音
  espeak_ERROR result = espeak_Synth(text.c_str(), text.length(), 0, POS_CHARACTER, 0, 
                                    espeakCHARS_AUTO, nullptr, nullptr);
  
  if (result != EE_OK) {
    RCLCPP_ERROR(this->get_logger(), "TTS合成失败");
    return;
  }
  
  // 等待合成完成
  espeak_Synchronize();
  
  // 直接发布音频数据
  if (!g_audio_data.empty()) {
    this->publish_audio_data(g_audio_data, g_channels, g_sample_rate);
  } else {
    RCLCPP_ERROR(this->get_logger(), "未生成音频数据");
  }
  
  // 终止espeak-ng
  espeak_Terminate();
}
#else
void TtsNode::synthesize_and_publish(const std::string& text) {
  std::string language = "zh";  // 设置为中文
  int rate = 150;  // 语速
  int volume = 100;  // 音量

  // 创建临时文件名
  char temp_file[] = "/tmp/tts_audio.wav";
  
  std::stringstream cmd;

  // 使用espeak-ng生成中文语音
  cmd << "espeak-ng -v" << language << " -s" << rate << " -a" << volume
      << " -w " << temp_file << " \"" << text << "\"";

  RCLCPP_INFO(this->get_logger(), "执行TTS命令: %s", cmd.str().c_str());
  int result = std::system(cmd.str().c_str());
  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "TTS命令执行失败");
    return;
  }

  // 读取生成的音频文件
  audio_common::WaveFile wf(temp_file);
  if (!wf.open()) {
    RCLCPP_ERROR(this->get_logger(), "打开音频文件失败: %s", temp_file);
    std::remove(temp_file);
    return;
  }
  
  // 检查是否为支持的格式
  if (wf.get_bits_per_sample() != 16) {
    RCLCPP_ERROR(this->get_logger(), "不支持的音频位深度: %d，仅支持16位PCM", wf.get_bits_per_sample());
    std::remove(temp_file);
    return;
  }

  // 读取所有音频数据
  std::vector<float> float_data;
  std::vector<float> chunk(wf.get_sample_rate() / 10); // 每次读取0.1秒的数据
  while (wf.read(chunk, chunk.size())) {
    float_data.insert(float_data.end(), chunk.begin(), chunk.end());
  }
  
  // 将float数据转换为int16数据
  std::vector<int16_t> int16_data(float_data.size());
  for (size_t i = 0; i < float_data.size(); ++i) {
    // 将float范围[-1.0, 1.0]转换为int16范围[-32768, 32767]
    int16_data[i] = static_cast<int16_t>(float_data[i] * 32767.0f);
  }
  
  // 直接发布音频数据，不需要重采样
  this->publish_audio_data(int16_data, wf.get_num_channels(), wf.get_sample_rate());
  
  // 清理临时文件
  std::remove(temp_file);
}
#endif

void TtsNode::publish_audio_data(const std::vector<int16_t>& audio_data, int channels, int sample_rate) {
  auto msg = audio_common_msgs::msg::AudioStamped();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = this->frame_id_;
  
  // 设置音频数据
  msg.audio.audio_data.int16_data = audio_data;
  
  // 设置音频信息，格式与piper_ros保持一致
  msg.audio.info.format = 8;  // paInt16 (与piper_ros中的值一致)
  msg.audio.info.channels = channels;
  msg.audio.info.chunk = audio_data.size();
  msg.audio.info.rate = sample_rate;

  // 发布消息
  this->player_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "发布音频数据，长度: %ld, 采样率: %d", audio_data.size(), sample_rate);
}
