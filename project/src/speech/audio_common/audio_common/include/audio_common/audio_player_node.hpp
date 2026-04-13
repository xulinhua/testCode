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

#ifndef AUDIO_COMMON__AUDIO_PLAYER_NODE
#define AUDIO_COMMON__AUDIO_PLAYER_NODE

#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>
#include <atomic>  // 添加atomic头文件支持

#include "audio_common_msgs/msg/audio_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
// #include "audio_common/audio_device_status.hpp"  // 移除旧的头文件引用
#include <custom_msgs_comm/msg/audio_device_status.hpp>  // 添加AudioDeviceStatus消息头文件

namespace audio_common {

class AudioPlayerNode : public rclcpp::Node {
public:
  AudioPlayerNode();
  ~AudioPlayerNode() override;

private:
  // ROS 2 subscription for audio messages
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      audio_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  // PortAudio stream dictionary
  std::unordered_map<std::string, PaStream *> stream_dict_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;  // 用于延迟初始化的定时器
  
  // 添加播放状态跟踪标志
  std::atomic<bool> is_playing_;
  
  // Parameters
  int channels_;
  int device_;
  double voice_gain_;

  // 重新采样函数实现
  std::vector<int16_t> resample_audio_data(
      const std::vector<int16_t>& input_data,
      double input_sample_rate,
      double output_sample_rate,
      int num_channels);
      
  // 新增的ROS音频数据重采样接口
  std::vector<int16_t> resample_audio_data_ros(
      const audio_common_msgs::msg::AudioStamped::SharedPtr msg,
      double device_sample_rate);

  // 创建音频流的接口
  bool create_audio_stream(const std::string& stream_key, 
                          int format, 
                          int rate, 
                          int channels);

  // 播放音频数据的接口
  template <typename T>
  void play_audio_data(const std::vector<T>& audio_data,
                      int channels,
                      int chunk,
                      const std::string& stream_key);

  bool load_and_play_audio_file(const std::string& file_path);
  // Methods
  void timer_callback();
  void audio_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);

  template <typename T>
  void write_data(const std::vector<T> &data, int channels, int chunk,
                  const std::string &stream_key);

  template <typename T>
  void write_data_with_resampling(const std::vector<T> &input_data, 
                                  int channels,
                                  int chunk, 
                                  int input_rate,
                                  int output_rate,
                                  const std::string &stream_key);
};

} // namespace audio_common

#endif