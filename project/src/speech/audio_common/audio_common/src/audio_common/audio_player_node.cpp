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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// 添加重采样支持
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <cstring>  // 添加 for memcpy
#include <cstdint>  // 添加 for uint32_t, uint16_t
#include <climits>  // 添加这个头文件，定义 SHRT_MIN, SHRT_MAX
#include <limits>   // 添加这个头文件，定义 std::numeric_limits
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common/audio_player_node.hpp"
#include "audio_common/audio_device_manager.hpp"
#include <custom_msgs_comm/msg/audio_device_status.hpp>  // 添加AudioDeviceStatus消息头文件
#include "audio_common_msgs/msg/audio.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

// 添加WAV文件头结构
// 修改 WAV 文件头结构
#pragma pack(push, 1)
struct WavHeader {
    char riff[4];           // "RIFF"
    uint32_t file_size;     // 文件总大小-8
    char wave[4];           // "WAVE"
    char fmt[4];            // "fmt "
    uint32_t fmt_size;      // fmt chunk大小 (通常是16)
    uint16_t audio_format;  // 音频格式（1=PCM）
    uint16_t num_channels;  // 声道数
    uint32_t sample_rate;   // 采样率
    uint32_t byte_rate;     // 每秒字节数
    uint16_t block_align;   // 每个样本的字节数
    uint16_t bits_per_sample; // 位深度
    // 注意：有些WAV文件这里可能还有其他字段
    char data[4];           // "data"
    uint32_t data_size;     // 数据大小
};
#pragma pack(pop)

using namespace audio_common;
using std::placeholders::_1;
//std::string audio_file = "/home/user/testCode/project/src/speech/audio_common/audio_common/samples/elevator.wav";
std::string audio_file = "/home/user/testCode/project/resampled_audio_1.wav";
// 添加保存音频数据到文件的函数声明
void save_audio_data_to_file(const std::vector<int16_t>& audio_data, int sample_rate, const std::string& filename);

AudioPlayerNode::AudioPlayerNode() : Node("audio_player_node"), init_timer_(nullptr), is_playing_(false) {
  // Declare parameters
  // 注意：不在构造函数中初始化音频设备状态发布器，避免使用shared_from_this()
  // 音频设备状态发布器将在节点完全构造后通过定时器初始化
  
  // 创建一次性定时器，用于延迟初始化音频设备状态发布器
  auto init_timer_period = std::chrono::milliseconds(100); // 100毫秒后初始化
  init_timer_ = this->create_wall_timer(init_timer_period, [this]() {
    // 初始化音频设备状态发布器（使用新的AudioDeviceManager）
    auto device_manager = audio_common::AudioDeviceManager::getInstance();
    device_manager->set_audio_device_status(custom_msgs_comm::msg::AudioDeviceStatus::IDLE_STATUS, shared_from_this());
    // 取消定时器，确保只执行一次
    if (init_timer_) {
      init_timer_->cancel();
    }
  });
  
  voice_gain_ = 0.5; // 默认音量0.5
  this->declare_parameter<int>("channels", 1);
  this->declare_parameter<int>("device", -1);
  this->declare_parameter<double>("voice_gain", voice_gain_);
  this->declare_parameter<std::string>("device_name", "hw:3,0");  // 默认值设为空字符串，让配置文件决定实际值

  // Get parameters
  this->channels_ = this->get_parameter("channels").as_int();
  this->device_ = this->get_parameter("device").as_int();
  this->voice_gain_ = this->get_parameter("voice_gain").as_double();
  std::string device_name = this->get_parameter("device_name").as_string();
  
  // 从配置文件读取额外参数（如果存在）
  try {
    if (this->has_parameter("device_name") && !this->get_parameter("device_name").as_string().empty()) {
      device_name = this->get_parameter("device_name").as_string();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取设备名称: %s", device_name.c_str());
    } else if (this->has_parameter("audio_player.device_name")) {
      // 尝试从嵌套参数读取
      std::string nested_device_name = this->get_parameter("audio_player.device_name").as_string();
      if (!nested_device_name.empty()) {
        device_name = nested_device_name;
        RCLCPP_INFO(this->get_logger(), "从配置文件读取设备名称: %s", device_name.c_str());
      }
    }
    
    if (this->has_parameter("channels")) {
      this->channels_ = this->get_parameter("channels").as_int();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取声道数: %d", this->channels_);
    } else if (this->has_parameter("audio_player.channels")) {
      this->channels_ = this->get_parameter("audio_player.channels").as_int();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取声道数: %d", this->channels_);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "读取配置文件参数时出错: %s，使用默认值", e.what());
  }
  
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to initialize PortAudio");
  }
  
  // Use device name to find device ID if provided
  if (!device_name.empty()) {
    RCLCPP_INFO(this->get_logger(), "正在查找设备名称: %s", device_name.c_str());
    int found_device = audio_common::AudioDeviceManager::find_device_id_by_name(device_name, false, this->get_logger());
    if (found_device >= 0) {
      this->device_ = found_device;
      RCLCPP_INFO(this->get_logger(), "成功找到音频播放设备，设备ID: %d", this->device_);
    } else {
      RCLCPP_WARN(this->get_logger(), "未找到指定名称的设备，使用设备ID: %d", this->device_);
    }
  }
  
  const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(this->device_);
  if (deviceInfo == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "无法获取设备ID %d 的信息", this->device_);
    throw std::runtime_error("无法获取设备信息");
  }

  // Subscription to audio topic
  auto qos_profile = rclcpp::SensorDataQoS();
  this->audio_sub_ =
      this->create_subscription<audio_common_msgs::msg::AudioStamped>(
          "audio/speaker", qos_profile,
          std::bind(&AudioPlayerNode::audio_callback, this, _1));
  this->status_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("audio/speaker_status",
                                                  rclcpp::SensorDataQoS());
  timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&AudioPlayerNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "音频播放节点启动成功");
  RCLCPP_INFO(this->get_logger(), "设备信息: %s", deviceInfo->name);
  RCLCPP_INFO(this->get_logger(), "通道数: %d", this->channels_);
  
  // 添加当前扬声器设备支持的采样率和当前默认的采样率
  RCLCPP_INFO(this->get_logger(), "设备默认采样率: %.0f Hz", deviceInfo->defaultSampleRate);
  // 打印设备支持的采样率范围
  RCLCPP_INFO(this->get_logger(), "设备最大输出通道数: %d", deviceInfo->maxOutputChannels);
  //load_and_play_audio_file(audio_file); //已验证可以播放音频
}

// 改进的 load_and_play_audio_file 函数
// 直接计算数据位置的简化版本
// 重新采样版本（如果需要）
bool AudioPlayerNode::load_and_play_audio_file(const std::string& file_path) {
    RCLCPP_INFO(this->get_logger(), "加载音频文件: %s", file_path.c_str());
    
    FILE* file = fopen(file_path.c_str(), "rb");
    if (!file) return false;

    WavHeader header;
    if (fread(&header, 1, sizeof(header), file) != sizeof(header)) 
    {
        fclose(file);
        RCLCPP_INFO(this->get_logger(), "读取音频文件失败");
        return false;
    }

    if (std::string(header.riff, 4) != "RIFF" || 
        std::string(header.wave, 4) != "WAVE") 
    {
        fclose(file);
        RCLCPP_INFO(this->get_logger(), "音频文件格式错误");
        return false;
    }

    long data_start = sizeof(header) + (header.fmt_size - 16);
    fseek(file, data_start, SEEK_SET);
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    long data_size = file_size - data_start;
    fseek(file, data_start, SEEK_SET);

    std::vector<uint8_t> raw_data(data_size);
    size_t read_size = fread(raw_data.data(), 1, data_size, file);
    fclose(file);

    if (read_size != (size_t)data_size) 
    {
        RCLCPP_INFO(this->get_logger(), "读取音频文件失败");
        return false;
    }

    // 如果采样率不是44100，重新采样
    double target_sample_rate = 48000.0;
    if (header.sample_rate != target_sample_rate) {
        RCLCPP_INFO(this->get_logger(), "重新采样: %d Hz -> %.0f Hz", header.sample_rate, target_sample_rate);
        // 这里可以添加重新采样逻辑
        // 暂时直接使用原始数据，但修改采样率
        header.sample_rate = target_sample_rate;
    }

    auto audio_msg = std::make_shared<audio_common_msgs::msg::AudioStamped>();
    audio_msg->audio.info.rate = header.sample_rate;
    audio_msg->audio.info.channels = header.num_channels;
    audio_msg->audio.info.format = paInt16;
    audio_msg->audio.audio_data.int16_data.resize(data_size / 2);
    std::memcpy(audio_msg->audio.audio_data.int16_data.data(), raw_data.data(), data_size);
    audio_msg->audio.info.chunk = audio_msg->audio.audio_data.int16_data.size() / header.num_channels;

    RCLCPP_INFO(this->get_logger(), "开始播放音频...");
    this->audio_callback(audio_msg);
    
    return true;
}

void AudioPlayerNode::timer_callback(){
  std_msgs::msg::Bool msg;
  msg.data = true;
  status_pub_->publish(msg);
}

AudioPlayerNode::~AudioPlayerNode() {
  // Close all open streams and terminate PortAudio
  for (auto &stream_pair : this->stream_dict_) {
    Pa_StopStream(stream_pair.second);
    Pa_CloseStream(stream_pair.second);
  }
  Pa_Terminate();
  
  // 注释掉析构函数中的状态设置，避免与播放完成后的状态设置产生冲突
  // 设置音频设备状态为空闲状态（使用新的AudioDeviceManager）
  // auto device_manager = audio_common::AudioDeviceManager::getInstance();
  // device_manager->set_audio_device_status(custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS);
}

// 重新采样函数实现
std::vector<int16_t> AudioPlayerNode::resample_audio_data(
    const std::vector<int16_t>& input_data,
    double input_sample_rate,
    double output_sample_rate,
    int num_channels) 
{
    
    if (input_sample_rate == output_sample_rate) {
        RCLCPP_DEBUG(this->get_logger(), "采样率相同，无需重新采样");
        return input_data;
    }

    RCLCPP_DEBUG(this->get_logger(), "开始重新采样: %.0f Hz -> %.0f Hz", 
                input_sample_rate, output_sample_rate);

    double ratio = output_sample_rate / input_sample_rate;
    size_t output_size = static_cast<size_t>(input_data.size() * ratio);
    std::vector<int16_t> output_data(output_size);

    double gain = 1.0;
    if (fabs(voice_gain_ - 0.5) > 0.01 && voice_gain_ > 0.5)
      gain = voice_gain_ * 5;
    else
      gain = voice_gain_ * 2;
    if (gain > 5.0)
      gain = 5.0;
    else if (gain < 0.4)
      gain = 0.4;
    RCLCPP_INFO(this->get_logger(), "开始重新采样，归一化增益：%.1f ,增益: %.1f ",voice_gain_, gain);

    // 简单的线性插值重新采样
    for (size_t i = 0; i < output_size; ++i) {
        double input_index = i / ratio;
        size_t index1 = static_cast<size_t>(input_index);
        size_t index2 = std::min(index1 + 1, input_data.size() - 1);
        
        double fraction = input_index - index1;
        
        // 对每个声道分别处理
        for (int ch = 0; ch < num_channels; ++ch) {
            if (index1 * num_channels + ch < input_data.size() && 
                index2 * num_channels + ch < input_data.size()) {
                double sample1 = input_data[index1 * num_channels + ch];
                double sample2 = input_data[index2 * num_channels + ch];
                double interpolated = sample1 + fraction * (sample2 - sample1);
                int32_t amplified = static_cast<int32_t>(interpolated) * gain;
                // 限制在int16_t范围内
                output_data[i * num_channels + ch] = static_cast<int16_t>(std::clamp(amplified, 
                    static_cast<int32_t>(SHRT_MIN), static_cast<int32_t>(SHRT_MAX)));
                //output_data[i * num_channels + ch] = static_cast<int16_t>(interpolated);
            }
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "重新采样完成: %zu -> %zu 样本", 
                input_data.size(), output_data.size());
    bool bSaveAudio = false;
    if(bSaveAudio)
    {
      // 添加调试信息，保存重采样后的音频数据到文件
      static int resampled_file_counter = 0;
      resampled_file_counter++;
      std::string resampled_filename = "resampled_audio_" + std::to_string(resampled_file_counter) + ".wav";
      double real_sample_rate = output_sample_rate;
      int real_num_channels = num_channels;
      // 打印重采样后的音频数据的详细信息
      RCLCPP_DEBUG(this->get_logger(), "=== 重采样后音频数据详细信息 ===");
      RCLCPP_DEBUG(this->get_logger(), "文件名: %s", resampled_filename.c_str());
      RCLCPP_DEBUG(this->get_logger(), "原采样率: %.0f Hz", input_sample_rate);
      RCLCPP_DEBUG(this->get_logger(), "目标采样率: %.0f Hz", output_sample_rate);
      RCLCPP_DEBUG(this->get_logger(), "重采样后实际采样率: %.0f Hz", real_sample_rate);  // 实际采样率就是目标采样率
      RCLCPP_DEBUG(this->get_logger(), "重采样后声道数: %d", real_num_channels);
      RCLCPP_DEBUG(this->get_logger(), "重采样后音频数据长度: %ld 采样点", output_data.size());
      
      // 计算重采样后音频时长（使用实际数据）
      if (real_sample_rate > 0) {
        double duration = (double)output_data.size() / (double)real_sample_rate;
        RCLCPP_DEBUG(this->get_logger(), "重采样后音频时长: %.3f 秒", duration);
      }
      
      // 计算数据大小（使用实际数据）
      size_t data_size = output_data.size() * sizeof(int16_t);
      RCLCPP_DEBUG(this->get_logger(), "重采样后数据大小: %ld 字节", data_size);
      
      // 检查数据范围（使用实际数据）
      if (!output_data.empty()) {
        int16_t min_val = output_data[0];
        int16_t max_val = output_data[0];
        for (const auto& sample : output_data) {
          if (sample < min_val) min_val = sample;
          if (sample > max_val) max_val = sample;
        }
        RCLCPP_DEBUG(this->get_logger(), "重采样后音频数据范围: %d 到 %d", min_val, max_val);
        
        // 检查是否为静音（使用实际数据）
        bool is_silent = (min_val == 0 && max_val == 0);
        RCLCPP_DEBUG(this->get_logger(), "重采样后是否静音: %s", is_silent ? "是" : "否");
      }
      
      RCLCPP_DEBUG(this->get_logger(), "保存重采样后音频数据到文件: %s", resampled_filename.c_str());
      // 保存重采样后的音频数据（使用实际数据和实际采样率）
      save_audio_data_to_file(output_data, static_cast<int>(real_sample_rate), resampled_filename);
    }
    return output_data;
}

// 新增的ROS音频数据重采样接口
std::vector<int16_t> AudioPlayerNode::resample_audio_data_ros(
    const audio_common_msgs::msg::AudioStamped::SharedPtr msg,
    double device_sample_rate)
{
  RCLCPP_DEBUG(this->get_logger(), "进入ROS音频数据重采样接口");

  // 获取音频数据采样率
  double audio_sample_rate = static_cast<double>(msg->audio.info.rate);
  
  RCLCPP_DEBUG(this->get_logger(), "设备采样率: %.0f Hz, 音频数据采样率: %.0f Hz", 
              device_sample_rate, audio_sample_rate);
  
  // 检查是否需要重采样
  bool need_resample = (device_sample_rate != audio_sample_rate);
  std::vector<int16_t> resampled_audio_data;
  
  if (need_resample) {
    RCLCPP_DEBUG(this->get_logger(), "采样率不匹配，开始重采样处理");
    // 根据不同格式进行重采样
    switch (msg->audio.info.format) {
      case paInt16:
        resampled_audio_data = this->resample_audio_data(
            msg->audio.audio_data.int16_data,
            audio_sample_rate,
            device_sample_rate,
            msg->audio.info.channels);
        break;
      case paUInt8:
        {
          // 转换uint8_t到int16_t再重采样
          std::vector<int16_t> converted_data(msg->audio.audio_data.uint8_data.size());
          for (size_t i = 0; i < msg->audio.audio_data.uint8_data.size(); ++i) {
            converted_data[i] = (static_cast<int16_t>(msg->audio.audio_data.uint8_data[i]) - 128) * 256;
          }
          resampled_audio_data = this->resample_audio_data(
              converted_data,
              audio_sample_rate,
              device_sample_rate,
              msg->audio.info.channels);
        }
        break;
      default:
        resampled_audio_data = this->resample_audio_data(
            msg->audio.audio_data.int16_data,
            audio_sample_rate,
            device_sample_rate,
            msg->audio.info.channels);
        break;
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "采样率匹配，无需重采样");
    // 如果不需要重采样，但格式是uint8_t，需要转换为int16_t
    switch (msg->audio.info.format) {
      case paUInt8:
        {
          resampled_audio_data.resize(msg->audio.audio_data.uint8_data.size());
          for (size_t i = 0; i < msg->audio.audio_data.uint8_data.size(); ++i) {
            resampled_audio_data[i] = (static_cast<int16_t>(msg->audio.audio_data.uint8_data[i]) - 128) * 256;
          }
        }
        break;
      case paInt16:
        // 直接复制数据，而不是引用，以确保数据可用
        resampled_audio_data = msg->audio.audio_data.int16_data;
        // 添加调试信息
        if (!resampled_audio_data.empty()) {
          RCLCPP_INFO(this->get_logger(), "复制原始int16数据成功，大小: %ld", resampled_audio_data.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "原始int16数据为空！");
        }
        break;
      default:
        resampled_audio_data = msg->audio.audio_data.int16_data;
        break;
    }
  }
  
  bool bSave_resampled_audio = false;// 根据控制变量决定是否保存重采样后的音频数据到文件
  if (bSave_resampled_audio && !resampled_audio_data.empty()) 
  {
    // 添加调试信息，保存重采样后的音频数据到文件
    static int resampled_file_counter = 0;
    resampled_file_counter++;
    std::string resampled_filename = "resampled_audio_from_ros_" + std::to_string(resampled_file_counter) + ".wav";
    RCLCPP_INFO(this->get_logger(), "保存重采样后的音频数据到文件: %s", resampled_filename.c_str());
    // 保存重采样后的音频数据
    save_audio_data_to_file(resampled_audio_data, static_cast<int>(device_sample_rate), resampled_filename);
    
    // 打印重采样后音频数据的详细信息
    RCLCPP_INFO(this->get_logger(), "=== 重采样后音频数据详细信息 ===");
    RCLCPP_INFO(this->get_logger(), "文件名: %s", resampled_filename.c_str());
    RCLCPP_INFO(this->get_logger(), "采样率: %.0f Hz", device_sample_rate);
    RCLCPP_INFO(this->get_logger(), "声道数: %d", msg->audio.info.channels);
    RCLCPP_INFO(this->get_logger(), "音频格式: %d", msg->audio.info.format);
    RCLCPP_INFO(this->get_logger(), "重采样后音频数据长度: %ld 采样点", resampled_audio_data.size());
    
    // 计算重采样后音频时长
    if (device_sample_rate > 0) {
      double duration = (double)resampled_audio_data.size() / (double)device_sample_rate;
      RCLCPP_INFO(this->get_logger(), "重采样后音频时长: %.3f 秒", duration);
    }
    
    // 计算数据大小
    size_t data_size = resampled_audio_data.size() * sizeof(int16_t);
    RCLCPP_INFO(this->get_logger(), "重采样后数据大小: %ld 字节", data_size);
    
    // 检查数据范围
    if (!resampled_audio_data.empty()) {
      int16_t min_val = resampled_audio_data[0];
      int16_t max_val = resampled_audio_data[0];
      for (const auto& sample : resampled_audio_data) {
        if (sample < min_val) min_val = sample;
        if (sample > max_val) max_val = sample;
      }
      RCLCPP_INFO(this->get_logger(), "重采样后音频数据范围: %d 到 %d", min_val, max_val);
      
      // 检查是否为静音
      bool is_silent = (min_val == 0 && max_val == 0);
      RCLCPP_INFO(this->get_logger(), "重采样后是否静音: %s", is_silent ? "是" : "否");
    }
  }
  return resampled_audio_data;
}

// 创建音频流的接口
bool AudioPlayerNode::create_audio_stream(const std::string& stream_key, 
                                         int format, 
                                         int rate, 
                                         int channels)
{
  RCLCPP_DEBUG(this->get_logger(), "创建音频流: %s", stream_key.c_str());
  
  // 检查流是否已存在
  if (this->stream_dict_.find(stream_key) != this->stream_dict_.end()) {
    RCLCPP_DEBUG(this->get_logger(), "音频流已存在: %s", stream_key.c_str());
    return true;
  }
  
  PaStreamParameters outputParameters;
  outputParameters.device =
      (this->device_ >= 0) ? this->device_ : Pa_GetDefaultOutputDevice();
  outputParameters.channelCount = channels;
  outputParameters.sampleFormat = format;
  outputParameters.suggestedLatency =
      Pa_GetDeviceInfo(outputParameters.device)->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = nullptr;

  // 创建音频流
  PaError err = Pa_OpenStream(&this->stream_dict_[stream_key], nullptr,
                              &outputParameters, rate,
                              paFramesPerBufferUnspecified, paClipOff,
                              nullptr, nullptr);

  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream: %s",
                 Pa_GetErrorText(err));
    return false;
  }
  
  PaError start_err = Pa_StartStream(this->stream_dict_[stream_key]);
  if (start_err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream: %s",
                 Pa_GetErrorText(start_err));
    return false;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "音频流创建成功: %s", stream_key.c_str());
  return true;
}

void AudioPlayerNode::audio_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg) 
{
  RCLCPP_INFO(this->get_logger(), "进入播放回调，采样率: %d Hz", msg->audio.info.rate);
  
  // 设置音频设备状态为播放状态（使用新的AudioDeviceManager）
  auto device_manager = audio_common::AudioDeviceManager::getInstance();
  RCLCPP_INFO(this->get_logger(), "audio_player_node获取到的AudioDeviceManager实例地址: %p", device_manager.get());
  RCLCPP_INFO(this->get_logger(), "audio_player_node进程ID: %d", getpid());
  device_manager->set_audio_device_status(custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS, shared_from_this());
  is_playing_ = true;
  
  // 添加调试信息，保存接收到的音频数据到文件
  static int debug_file_counter = 0;
  debug_file_counter++;
  std::string debug_filename = "debug_audio_" + std::to_string(debug_file_counter) + ".wav";
  
  // 打印保存的音频数据的详细信息
  RCLCPP_DEBUG(this->get_logger(), "=== 原始音频数据详细信息 ===");
  RCLCPP_DEBUG(this->get_logger(), "文件名: %s", debug_filename.c_str());
  RCLCPP_DEBUG(this->get_logger(), "采样率: %d Hz", msg->audio.info.rate);
  RCLCPP_DEBUG(this->get_logger(), "声道数: %d", msg->audio.info.channels);
  RCLCPP_DEBUG(this->get_logger(), "音频格式: %d", msg->audio.info.format);
  RCLCPP_DEBUG(this->get_logger(), "数据块大小: %d", msg->audio.info.chunk);
  
  // 根据音频格式计算实际数据长度
  size_t audio_data_size = 0;
  switch (msg->audio.info.format) {
    case paFloat32:
      audio_data_size = msg->audio.audio_data.float32_data.size();
      RCLCPP_DEBUG(this->get_logger(), "音频数据长度: %ld 采样点", audio_data_size);
      break;
    case paInt32:
      audio_data_size = msg->audio.audio_data.int32_data.size();
      RCLCPP_DEBUG(this->get_logger(), "音频数据长度: %ld 采样点", audio_data_size);
      break;
    case paInt16:
      audio_data_size = msg->audio.audio_data.int16_data.size();
      RCLCPP_DEBUG(this->get_logger(), "音频数据长度: %ld 采样点", audio_data_size);
      // 添加调试信息，检查原始数据
      if (!msg->audio.audio_data.int16_data.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "原始int16数据非空，前10个样本: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    msg->audio.audio_data.int16_data[0], msg->audio.audio_data.int16_data[1],
                    msg->audio.audio_data.int16_data[2], msg->audio.audio_data.int16_data[3],
                    msg->audio.audio_data.int16_data[4], msg->audio.audio_data.int16_data[5],
                    msg->audio.audio_data.int16_data[6], msg->audio.audio_data.int16_data[7],
                    msg->audio.audio_data.int16_data[8], msg->audio.audio_data.int16_data[9]);
      } else {
        RCLCPP_WARN(this->get_logger(), "原始int16数据为空！");
      }
      break;
    case paInt8:
      audio_data_size = msg->audio.audio_data.int8_data.size();
      RCLCPP_DEBUG(this->get_logger(), "音频数据长度: %ld 采样点", audio_data_size);
      break;
    case paUInt8:
      audio_data_size = msg->audio.audio_data.uint8_data.size();
      RCLCPP_DEBUG(this->get_logger(), "音频数据长度: %ld 采样点", audio_data_size);
      break;
    default:
      audio_data_size = msg->audio.audio_data.int16_data.size();
      RCLCPP_DEBUG(this->get_logger(), "音频数据长度: %ld 采样点", audio_data_size);
      break;
  }
  
  // 计算音频时长
  if (msg->audio.info.rate > 0) {
    double duration = (double)audio_data_size / (double)msg->audio.info.rate;
    RCLCPP_DEBUG(this->get_logger(), "音频时长: %.3f 秒", duration);
  }
  
  // 计算数据大小
  size_t data_size = 0;
  switch (msg->audio.info.format) {
    case paFloat32:
      data_size = msg->audio.audio_data.float32_data.size() * sizeof(float);
      break;
    case paInt32:
      data_size = msg->audio.audio_data.int32_data.size() * sizeof(int32_t);
      break;
    case paInt16:
      data_size = msg->audio.audio_data.int16_data.size() * sizeof(int16_t);
      break;
    case paInt8:
      data_size = msg->audio.audio_data.int8_data.size() * sizeof(int8_t);
      break;
    case paUInt8:
      data_size = msg->audio.audio_data.uint8_data.size() * sizeof(uint8_t);
      break;
    default:
      data_size = msg->audio.audio_data.int16_data.size() * sizeof(int16_t);
      break;
  }
  RCLCPP_DEBUG(this->get_logger(), "数据大小: %ld 字节", data_size);
  
  // 检查数据范围
  switch (msg->audio.info.format) {
    case paInt16: {
      if (!msg->audio.audio_data.int16_data.empty()) {
        int16_t min_val = msg->audio.audio_data.int16_data[0];
        int16_t max_val = msg->audio.audio_data.int16_data[0];
        for (const auto& sample : msg->audio.audio_data.int16_data) {
          if (sample < min_val) min_val = sample;
          if (sample > max_val) max_val = sample;
        }
        RCLCPP_DEBUG(this->get_logger(), "音频数据范围: %d 到 %d", min_val, max_val);
        
        // 检查是否为静音
        bool is_silent = (min_val == 0 && max_val == 0);
        RCLCPP_DEBUG(this->get_logger(), "是否静音: %s", is_silent ? "是" : "否");
      }
      break;
    }
    case paUInt8: {
      if (!msg->audio.audio_data.uint8_data.empty()) {
        uint8_t min_val = msg->audio.audio_data.uint8_data[0];
        uint8_t max_val = msg->audio.audio_data.uint8_data[0];
        for (const auto& sample : msg->audio.audio_data.uint8_data) {
          if (sample < min_val) min_val = sample;
          if (sample > max_val) max_val = sample;
        }
        RCLCPP_DEBUG(this->get_logger(), "音频数据范围: %d 到 %d", min_val, max_val);
        
        // 检查是否为静音
        bool is_silent = (min_val == 128 && max_val == 128);  // 8位无符号音频的静音值是128
        RCLCPP_DEBUG(this->get_logger(), "是否静音: %s", is_silent ? "是" : "否");
      }
      break;
    }
    // 其他格式可以类似处理
  }
  
  RCLCPP_DEBUG(this->get_logger(), "保存调试音频数据到文件: %s", debug_filename.c_str());
  static bool bSaveFile = false;
  // 保存音频数据到文件
  switch (msg->audio.info.format) {
    case paInt16:
      if (bSaveFile)
        save_audio_data_to_file(msg->audio.audio_data.int16_data, msg->audio.info.rate, debug_filename);
      break;
    case paUInt8:
      {
        // 转换uint8_t到int16_t再保存
        std::vector<int16_t> converted_data(msg->audio.audio_data.uint8_data.size());
        for (size_t i = 0; i < msg->audio.audio_data.uint8_data.size(); ++i) {
          converted_data[i] = (static_cast<int16_t>(msg->audio.audio_data.uint8_data[i]) - 128) * 256;
        }
        if (bSaveFile)
          save_audio_data_to_file(converted_data, msg->audio.info.rate, debug_filename);
      }
      break;
    default:
      if (bSaveFile)
        save_audio_data_to_file(msg->audio.audio_data.int16_data, msg->audio.info.rate, debug_filename);
      break;
  }

  // 获取设备信息以检查采样率
  const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(
      (this->device_ >= 0) ? this->device_ : Pa_GetDefaultOutputDevice());
  if (deviceInfo == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "无法获取设备信息");
    return;
  }
  
  // 获取设备采样率
  double device_sample_rate = deviceInfo->defaultSampleRate;
  double audio_sample_rate = static_cast<double>(msg->audio.info.rate);
  
  RCLCPP_DEBUG(this->get_logger(), "设备默认采样率: %.0f Hz, 音频数据采样率: %.0f Hz", 
              device_sample_rate, audio_sample_rate);
  
  // 调用重采样接口处理音频数据
  std::vector<int16_t> processed_audio_data = this->resample_audio_data_ros(msg, device_sample_rate);
  
  // 输出重采样以后新的音频数据的format、rate、channels等信息
  RCLCPP_DEBUG(this->get_logger(), "=== 重采样后音频信息 ===");
  RCLCPP_DEBUG(this->get_logger(), "音频格式: %d", msg->audio.info.format);
  RCLCPP_DEBUG(this->get_logger(), "原始采样率: %d Hz", msg->audio.info.rate);
  RCLCPP_DEBUG(this->get_logger(), "设备采样率: %.0f Hz", device_sample_rate);
  RCLCPP_DEBUG(this->get_logger(), "使用采样率: %d Hz", (device_sample_rate != audio_sample_rate) ? 
              static_cast<int>(device_sample_rate) : msg->audio.info.rate);
  RCLCPP_DEBUG(this->get_logger(), "声道数: %d", msg->audio.info.channels);
  RCLCPP_DEBUG(this->get_logger(), "数据块大小: %d", msg->audio.info.chunk);
  RCLCPP_DEBUG(this->get_logger(), "重采样后数据长度: %ld 采样点", processed_audio_data.size());
  
  // Create a unique stream key based on format, rate, and channels
  // 使用设备采样率创建流键
  int stream_rate = (device_sample_rate != audio_sample_rate) ? 
                    static_cast<int>(device_sample_rate) : msg->audio.info.rate;
  std::string stream_key = std::to_string(msg->audio.info.format) + "_" +
                           std::to_string(stream_rate) + "_" +
                           std::to_string(this->channels_);

  // 调用创建音频流接口
  // 确保使用正确的PortAudio格式常量
  PaSampleFormat pa_format;
  switch (msg->audio.info.format) {
    case paFloat32:
      pa_format = paFloat32;
      break;
    case paInt16:
      pa_format = paInt16;
      break;
    case paInt8:
      pa_format = paInt8;
      break;
    case paUInt8:
      pa_format = paUInt8;
      break;
    default:
      pa_format = paInt16;  // 默认使用paInt16
      RCLCPP_WARN(this->get_logger(), "未知的音频格式，使用默认paInt16格式");
      break;
  }
  
  if (!this->create_audio_stream(stream_key, pa_format, stream_rate, this->channels_)) {
    RCLCPP_ERROR(this->get_logger(), "创建音频流失败");
    return;
  }

  // 调用播放音频数据接口
  // 使用重采样后的数据（如果进行了重采样）
  RCLCPP_DEBUG(this->get_logger(), "Switch时的音频格式: %d", msg->audio.info.format);
  RCLCPP_DEBUG(this->get_logger(), "paInt8值: %lu, paInt16값: %lu", static_cast<unsigned long>(paInt8), static_cast<unsigned long>(paInt16));
  
  switch (msg->audio.info.format)
  {
  case paFloat32:
    RCLCPP_DEBUG(this->get_logger(), "进入paFloat32的case, 音频格式: %d", msg->audio.info.format);
    RCLCPP_DEBUG(this->get_logger(), "原始声道数: %d, 目标声道数: %d", 
                  msg->audio.info.channels, this->channels_);
    this->template play_audio_data<float>(msg->audio.audio_data.float32_data,
                         msg->audio.info.channels, msg->audio.info.chunk,
                         stream_key);
    break;

  case paInt32:
    RCLCPP_DEBUG(this->get_logger(), "进入paInt32的case, 音频格式: %d", msg->audio.info.format);
    RCLCPP_DEBUG(this->get_logger(), "原始声道数: %d, 目标声道数: %d", 
                  msg->audio.info.channels, this->channels_);
    this->template play_audio_data<int32_t>(msg->audio.audio_data.int32_data, msg->audio.info.channels,
                         msg->audio.info.chunk, stream_key);
    break;

  case paInt16:
    RCLCPP_DEBUG(this->get_logger(), "进入paInt16的case, 音频格式: %d", msg->audio.info.format);
    RCLCPP_DEBUG(this->get_logger(), "原始声道数: %d, 目标声道数: %d", 
                  msg->audio.info.channels, this->channels_);
    // 如果进行了重采样，使用重采样后的数据
    if (device_sample_rate != audio_sample_rate || msg->audio.info.format == paUInt8) 
    {
      // 检查重采样后的数据是否为空
      if (processed_audio_data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "错误：重采样后的int16数据为空！");
        return;
      }
      
      // 计算新的chunk大小 - 注意这里应该使用原始数据的声道数而不是成员变量
      int resampled_chunk = static_cast<int>(
          processed_audio_data.size() / msg->audio.info.channels);
      RCLCPP_DEBUG(this->get_logger(), "使用重采样后的int16数据");
      RCLCPP_DEBUG(this->get_logger(), "重采样前chunk: %d, 重采样后chunk: %d", 
                  msg->audio.info.chunk, resampled_chunk);
      RCLCPP_DEBUG(this->get_logger(), "重采样后数据大小: %ld", processed_audio_data.size());
      RCLCPP_DEBUG(this->get_logger(), "原始声道数: %d, 目标声道数: %d", 
                  msg->audio.info.channels, this->channels_);
      
      // 确保数据被正确传递
      RCLCPP_DEBUG(this->get_logger(), "即将调用play_audio_data，数据非空");
      RCLCPP_DEBUG(this->get_logger(), "重采样后数据的前5个样本: %d,%d,%d,%d,%d", 
                  processed_audio_data[0], processed_audio_data[1], 
                  processed_audio_data[2], processed_audio_data[3], 
                  processed_audio_data[4]);
      
      // 显式调用模板函数，确保使用正确的类型
      this->template play_audio_data<int16_t>(processed_audio_data, msg->audio.info.channels,
                           resampled_chunk, stream_key);
    } else 
    {
      RCLCPP_DEBUG(this->get_logger(), "采样率匹配，无需计算新的chunk");
      this->template play_audio_data<int16_t>(msg->audio.audio_data.int16_data, msg->audio.info.channels,
                           msg->audio.info.chunk, stream_key);
    }
    break;

  case paInt8:
    RCLCPP_DEBUG(this->get_logger(), "进入paInt8的case, 音频格式: %d", msg->audio.info.format);
    RCLCPP_DEBUG(this->get_logger(), "原始声道数: %d, 目标声道数: %d", 
                  msg->audio.info.channels, this->channels_);
    this->template play_audio_data<int8_t>(msg->audio.audio_data.int8_data, msg->audio.info.channels,
                         msg->audio.info.chunk, stream_key);
    break;

  case paUInt8:
    RCLCPP_DEBUG(this->get_logger(), "进入paUInt8的case, 音频格式: %d", msg->audio.info.format);
    RCLCPP_DEBUG(this->get_logger(), "原始声道数: %d, 目标声道数: %d", 
                  msg->audio.info.channels, this->channels_);
    // 对于uint8格式，使用转换后的数据
    {
      // 检查重采样后的数据是否为空
      if (processed_audio_data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "错误：重采样后的uint8数据为空！");
        return;
      }
      
      int resampled_chunk = static_cast<int>(
          processed_audio_data.size() / this->channels_);
      RCLCPP_INFO(this->get_logger(), "重采样后数据大小: %ld", processed_audio_data.size());
      this->template play_audio_data<int16_t>(processed_audio_data, msg->audio.info.channels,
                           resampled_chunk, stream_key);
    }
    break;

  default:
    RCLCPP_ERROR(this->get_logger(), "Unsupported format");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "音频数据播放完成");
  
  // 播放完成后设置音频设备状态为录音状态（使用新的AudioDeviceManager）
  device_manager->set_audio_device_status(custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS, shared_from_this());
  is_playing_ = false;
  
  // 添加一个小延迟，确保状态更新能被其他节点及时获取到
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 播放音频数据的接口
template <typename T>
void AudioPlayerNode::play_audio_data(const std::vector<T>& audio_data,
                                     int channels,
                                     int chunk,
                                     const std::string& stream_key)
{
  RCLCPP_DEBUG(this->get_logger(), "进入play_audio_data函数");
  RCLCPP_DEBUG(this->get_logger(), "参数 - 音频数据长度: %ld", audio_data.size());
  RCLCPP_DEBUG(this->get_logger(), "参数 - 声道数: %d", channels);
  RCLCPP_DEBUG(this->get_logger(), "参数 - chunk: %d", chunk);
  RCLCPP_DEBUG(this->get_logger(), "参数 - 流键: %s", stream_key.c_str());
  RCLCPP_DEBUG(this->get_logger(), "成员变量 - this->channels_: %d", this->channels_);
  
  // 检查音频数据是否为空
  if (audio_data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "错误：音频数据为空，无法播放");
    return;
  }
  
  // 确保数据在播放前被正确复制
  std::vector<T> data(audio_data.begin(), audio_data.end()); // 使用迭代器复制音频数据
  
  // 添加调试信息，检查数据复制
  RCLCPP_DEBUG(this->get_logger(), "复制数据后的大小: %ld", data.size());
  
  // 检查数据是否有效
  if (data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "错误：复制后的数据为空！");
    return;
  }
  
  // 打印前几个样本值，确保数据有效
  if (data.size() >= 5) {
    RCLCPP_DEBUG(this->get_logger(), "数据样本值: %d,%d,%d,%d,%d", 
                static_cast<int>(data[0]), static_cast<int>(data[1]), 
                static_cast<int>(data[2]), static_cast<int>(data[3]), 
                static_cast<int>(data[4]));
  }

  // Handle mono-to-stereo or stereo-to-mono conversions if necessary
  if (channels != this->channels_) {
    if (channels == 1 && this->channels_ == 2) {
      // Mono to stereo conversion
      data.resize(audio_data.size() * 2);
      for (size_t i = 0; i < audio_data.size(); ++i) {
        data[2 * i] = audio_data[i];
        data[2 * i + 1] = audio_data[i];
      }
    } else if (channels == 2 && this->channels_ == 1) {
      // Stereo to mono conversion
      data.resize(audio_data.size() / 2);
      for (size_t i = 0; i < data.size(); ++i) {
        data[i] =
            static_cast<T>((audio_data[2 * i] + audio_data[2 * i + 1]) / 2);
      }
    }
  } else {
    // No conversion needed
    data = audio_data;
  }

  // Make sure chunk size is correct for frames (not samples)
  // 修正chunk大小计算，确保不会超过实际数据大小
  int actual_chunk = std::min(chunk, static_cast<int>(data.size() / this->channels_));
  
  // 添加详细的调试信息
  RCLCPP_DEBUG(this->get_logger(), "=== 播放数据详细信息 ===");
  RCLCPP_DEBUG(this->get_logger(), "原始chunk大小: %d", chunk);
  RCLCPP_DEBUG(this->get_logger(), "数据大小: %ld 样本", data.size());
  RCLCPP_DEBUG(this->get_logger(), "声道数: %d", this->channels_);
  RCLCPP_DEBUG(this->get_logger(), "帧数(数据大小/声道数): %ld", data.size() / this->channels_);
  RCLCPP_DEBUG(this->get_logger(), "计算后的actual_chunk: %d", actual_chunk);
  RCLCPP_DEBUG(this->get_logger(), "流键: %s", stream_key.c_str());
  
  if (actual_chunk <= 0) {
    RCLCPP_ERROR(this->get_logger(),
                "Insufficient data (%ld) for requested chunk size (%d). 无法播放。",
                data.size(), chunk);
    
    // 添加更多调试信息
    RCLCPP_ERROR(this->get_logger(), "数据.size(): %ld, 声道数: %d, chunk: %d", 
                 data.size(), this->channels_, chunk);
    
    // 尝试使用数据本身的大小作为chunk
    if (!data.empty()) {
      int fallback_chunk = static_cast<int>(data.size() / this->channels_);
      RCLCPP_WARN(this->get_logger(), "尝试使用回退chunk大小: %d", fallback_chunk);
      if (fallback_chunk > 0) {
        actual_chunk = fallback_chunk;
      } else {
        return;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "数据为空，无法播放");
      return;
    }
  }

  // Write to PortAudio stream
  PaError err =
      Pa_WriteStream(this->stream_dict_[stream_key], data.data(), actual_chunk);
  if (err != paNoError && err != paOutputUnderflowed) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio write error: %s",
                 Pa_GetErrorText(err));
  }
  
  RCLCPP_INFO(this->get_logger(), "音频数据播放完成");
}

template <typename T>
void AudioPlayerNode::write_data(const std::vector<T> &input_data, int channels,
                                 int chunk, const std::string &stream_key) {
  std::vector<T> data; // Buffer for the actual data to write

  // Handle mono-to-stereo or stereo-to-mono conversions if necessary
  if (channels != this->channels_) {
    if (channels == 1 && this->channels_ == 2) {
      // Mono to stereo conversion
      data.resize(input_data.size() * 2);
      for (size_t i = 0; i < input_data.size(); ++i) {
        data[2 * i] = input_data[i];
        data[2 * i + 1] = input_data[i];
      }
    } else if (channels == 2 && this->channels_ == 1) {
      // Stereo to mono conversion
      data.resize(input_data.size() / 2);
      for (size_t i = 0; i < data.size(); ++i) {
        data[i] =
            static_cast<T>((input_data[2 * i] + input_data[2 * i + 1]) / 2);
      }
    }
  } else {
    // No conversion needed
    data = input_data;
  }

  // Make sure chunk size is correct for frames (not samples)
  // 修正chunk大小计算，确保不会超过实际数据大小
  int actual_chunk = std::min(chunk, static_cast<int>(data.size() / this->channels_));
  
  // 添加详细的调试信息
  RCLCPP_DEBUG(this->get_logger(), "=== 播放数据详细信息 ===");
  RCLCPP_DEBUG(this->get_logger(), "原始chunk大小: %d", chunk);
  RCLCPP_DEBUG(this->get_logger(), "数据大小: %ld 样本", data.size());
  RCLCPP_DEBUG(this->get_logger(), "声道数: %d", this->channels_);
  RCLCPP_DEBUG(this->get_logger(), "帧数(数据大小/声道数): %ld", data.size() / this->channels_);
  RCLCPP_DEBUG(this->get_logger(), "计算后的actual_chunk: %d", actual_chunk);
  RCLCPP_DEBUG(this->get_logger(), "流键: %s", stream_key.c_str());
  
  if (actual_chunk <= 0) {
    RCLCPP_ERROR(this->get_logger(),
                "Insufficient data (%ld) for requested chunk size (%d). 无法播放。",
                data.size(), chunk);
    
    // 添加更多调试信息
    RCLCPP_ERROR(this->get_logger(), "数据.size(): %ld, 声道数: %d, chunk: %d", 
                 data.size(), this->channels_, chunk);
    
    // 尝试使用数据本身的大小作为chunk
    if (!data.empty()) {
      int fallback_chunk = static_cast<int>(data.size() / this->channels_);
      RCLCPP_WARN(this->get_logger(), "尝试使用回退chunk大小: %d", fallback_chunk);
      if (fallback_chunk > 0) {
        actual_chunk = fallback_chunk;
      } else {
        return;
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "数据为空，无法播放");
      return;
    }
  }

  // Write to PortAudio stream
  PaError err =
      Pa_WriteStream(this->stream_dict_[stream_key], data.data(), actual_chunk);
  if (err != paNoError && err != paOutputUnderflowed) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio write error: %s",
                 Pa_GetErrorText(err));
  }
}

// 实现保存音频数据到文件的函数
void save_audio_data_to_file(const std::vector<int16_t>& audio_data, int sample_rate, const std::string& filename) {
  // 创建WAV文件头
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    return;
  }

  // WAV文件头
  uint32_t chunk_size = 36 + audio_data.size() * sizeof(int16_t);
  uint32_t subchunk2_size = audio_data.size() * sizeof(int16_t);
  
  file.write("RIFF", 4);
  file.write(reinterpret_cast<const char*>(&chunk_size), 4);
  file.write("WAVE", 4);
  file.write("fmt ", 4);
  uint32_t subchunk1_size = 16;
  file.write(reinterpret_cast<const char*>(&subchunk1_size), 4);
  uint16_t audio_format = 1; // PCM
  file.write(reinterpret_cast<const char*>(&audio_format), 2);
  uint16_t num_channels = 1;
  file.write(reinterpret_cast<const char*>(&num_channels), 2);
  file.write(reinterpret_cast<const char*>(&sample_rate), 4);
  uint32_t byte_rate = sample_rate * num_channels * sizeof(int16_t);
  file.write(reinterpret_cast<const char*>(&byte_rate), 4);
  uint16_t block_align = num_channels * sizeof(int16_t);
  file.write(reinterpret_cast<const char*>(&block_align), 2);
  uint16_t bits_per_sample = 16;
  file.write(reinterpret_cast<const char*>(&bits_per_sample), 2);
  file.write("data", 4);
  file.write(reinterpret_cast<const char*>(&subchunk2_size), 4);
  
  // 写入音频数据
  file.write(reinterpret_cast<const char*>(audio_data.data()), audio_data.size() * sizeof(int16_t));
  
  file.close();
}

// 添加一个新的函数来保存uint8_t格式的音频数据
void save_audio_data_to_file_uint8(const std::vector<uint8_t>& audio_data, int sample_rate, const std::string& filename) {
  // 创建WAV文件头
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    return;
  }

  // WAV文件头
  uint32_t chunk_size = 36 + audio_data.size() * sizeof(uint8_t);
  uint32_t subchunk2_size = audio_data.size() * sizeof(uint8_t);
  
  file.write("RIFF", 4);
  file.write(reinterpret_cast<const char*>(&chunk_size), 4);
  file.write("WAVE", 4);
  file.write("fmt ", 4);
  uint32_t subchunk1_size = 16;
  file.write(reinterpret_cast<const char*>(&subchunk1_size), 4);
  uint16_t audio_format = 1; // PCM
  file.write(reinterpret_cast<const char*>(&audio_format), 2);
  uint16_t num_channels = 1;
  file.write(reinterpret_cast<const char*>(&num_channels), 2);
  file.write(reinterpret_cast<const char*>(&sample_rate), 4);
  uint32_t byte_rate = sample_rate * num_channels * sizeof(uint8_t);
  file.write(reinterpret_cast<const char*>(&byte_rate), 4);
  uint16_t block_align = num_channels * sizeof(uint8_t);
  file.write(reinterpret_cast<const char*>(&block_align), 2);
  uint16_t bits_per_sample = 8;
  file.write(reinterpret_cast<const char*>(&bits_per_sample), 2);
  file.write("data", 4);
  file.write(reinterpret_cast<const char*>(&subchunk2_size), 4);
  
  // 写入音频数据
  file.write(reinterpret_cast<const char*>(audio_data.data()), audio_data.size() * sizeof(uint8_t));
  
  file.close();
}