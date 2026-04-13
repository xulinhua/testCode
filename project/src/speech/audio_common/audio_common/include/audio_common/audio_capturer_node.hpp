// MIT License
//
// Copyright (c) 2024 Intel Corporation. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef AUDIO_COMMON__AUDIO_CAPTURER_NODE_HPP_
#define AUDIO_COMMON__AUDIO_CAPTURER_NODE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>
#include <deque>  // 添加deque头文件
#include <vector>
#include <fstream>  // 添加文件操作头文件

// 尝试包含WebRTC VAD头文件
#ifdef HAVE_WEBRTC_VAD
#include <webrtc_vad.h>
#endif

#include "audio_common_msgs/msg/audio_stamped.hpp"
#include <custom_msgs_comm/msg/speech_status.hpp>  // 添加SpeechStatus消息头文件
#include <custom_msgs_comm/msg/audio_device_status.hpp>  // 添加AudioDeviceStatus消息头文件
#include "audio_common/wake_word_detector.hpp"
#include "audio_common/wave_file.hpp"
#include "voice_type.hpp"  // 使用cmd_dispatcher中的voice_type定义
#include <rclcpp/rclcpp.hpp>

namespace audio_common {

class AudioCapturerNode : public rclcpp::Node {
public:
  AudioCapturerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());  // 修改构造函数声明
  ~AudioCapturerNode() override;

  // 添加初始化唤醒词检测器的方法
  void initialize_wake_word_detector();

  // 发布音频数据，同时指定声音类型
  void publish_audio_data(const std::vector<int16_t>& audio_data, uint8_t voice_type);
  
  void set_wake_word_detector(std::shared_ptr<audio_common::WakeWordDetector> node) {
      wake_word_detector_ = node;
    }
private:
  PaStream *stream_;
  int format_;
  int channels_;
  int rate_;
  int chunk_;
  int chunk_duration_ms_;  // 音频块时长（毫秒），避免重复计算
  std::string frame_id_;
  std::string device_name_;  // 添加设备名称成员变量
  
  // 添加配置参数：是否保存音频数据，默认为true
  bool save_audio_data_;
  
  // 音频设备状态
  uint8_t audio_device_status_;
  
  // 唤醒词检测相关成员
  std::shared_ptr<audio_common::WakeWordDetector> wake_word_detector_;
  
  // VAD和音频缓冲相关成员
  std::deque<int16_t> audio_buffer_;  // 音频数据缓冲区
  int silent_chunks_;                 // 静音块计数器
  bool is_speaking_;                  // 是否正在说话的标志
  int max_silence_ms_;                // 最大静音时长（毫秒）
  
  // WebRTC VAD相关成员
  #ifdef HAVE_WEBRTC_VAD
  VadInst* vad_inst_;                 // WebRTC VAD实例
  #endif
  
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr audio_pub_;
  rclcpp::Publisher<custom_msgs_comm::msg::SpeechStatus>::SharedPtr awake_status_pub_;  // 唤醒状态发布者
  rclcpp::Subscription<custom_msgs_comm::msg::AudioDeviceStatus>::SharedPtr audio_status_sub_;  // 音频设备状态订阅者
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_; // 用于延迟初始化的定时器
  rclcpp::TimerBase::SharedPtr device_check_timer_; // 用于定期检查设备状态的定时器

  // Methods
  template <typename T> std::vector<T> read_data();
  
  // 新增接口：获取实时音频数据流，实现完整的静默监听
  bool record_audio_streaming(std::vector<int16_t>& audio_data);
  
  // 唤醒词相关接口
  // 新增接口：对音频数据进行唤醒词检测
  bool detect_wake_word(const std::vector<int16_t>& audio_data);
  bool is_wake_reday();
  bool is_wake_status_on();
  void set_wake_status(int wake_status);
  void updata_wake_time();
  // 唤醒停止回调
  void wake_status_off_callback();
  
  // 检查音频数据是否包含语音信号（保持原有实现）
  bool is_voice_present(const std::vector<int16_t>& audio_data);
  
  // 使用WebRTC VAD检查音频数据是否包含语音信号
  bool is_voice_present_webrtc(const std::vector<int16_t>& audio_data);
  
  // 保存音频数据为WAV文件
  bool save_audio_to_wav(const std::vector<int16_t>& audio_data, const std::string& filename);
  
  // 定时器回调函数
  void timer_callback();

  std::thread audio_thread_;
  std::atomic<bool> audio_thread_running_;
  std::mutex audio_mutex_;
  std::condition_variable audio_cv_;
  void audio_thread_func();

  
  // 设备状态检查回调函数
  void device_check_callback();
  
  // 重新初始化音频流
  bool reinitialize_stream();
};

} // namespace audio_common

#endif