#include "audio_common/audio_capturer_node.hpp"
#include "audio_common/audio_device_manager.hpp"

// 添加SpeechStatus消息头文件
#include <custom_msgs_comm/msg/speech_status.hpp>
#include "voice_type.hpp"  // 使用cmd_dispatcher中的voice_type定义
#include <custom_msgs_comm/msg/audio_device_status.hpp>  // 添加AudioDeviceStatus消息头文件
#include <rclcpp/rclcpp.hpp>
#include "audio_common/wake_word_detector.hpp"
#include "audio_common/wave_file.hpp"
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <deque>  // 添加deque头文件用于音频缓冲
#include <cmath>  // 添加数学库用于sqrt函数
#include <fstream>  // 添加文件操作头文件
#include <iomanip>  // 添加iomanip头文件用于时间戳格式化
#include <chrono>   // 添加chrono头文件用于时间戳

// 尝试包含WebRTC VAD头文件
#ifdef HAVE_WEBRTC_VAD
#include <webrtc_vad.h>
#endif

using namespace audio_common;

AudioCapturerNode::AudioCapturerNode(const rclcpp::NodeOptions & options)
: Node("audio_capturer", options), 
  is_speaking_(false),
  max_silence_ms_(500),
  stream_(nullptr),
  format_(paInt16),
  channels_(1),
  rate_(16000),
  chunk_(512),
  frame_id_("microphone"),
  init_timer_(nullptr),
  device_check_timer_(nullptr),
  // 初始化音频设备状态为录音状态
  audio_device_status_(custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS)
  // 条件编译初始化vad_inst_
  #ifdef HAVE_WEBRTC_VAD
  , vad_inst_(nullptr)
  #endif
{
  // 声明参数并设置默认值
  this->declare_parameter<int>("format", paInt16);
  this->declare_parameter<int>("channels", 1);
  this->declare_parameter<int>("rate", 44100);  // 恢复为44100Hz，设备支持的采样率
  this->declare_parameter<int>("chunk_duration_ms", 30);   // 音频块时长（毫秒）
  this->declare_parameter<int>("device", -1);
  this->declare_parameter<std::string>("device_name", "hw:0,0");  // 默认值设为空字符串，让配置文件决定实际值
  this->declare_parameter<std::string>("frame_id", "");
  this->declare_parameter<bool>("auto_search_device", true);  // 新增参数：是否自动搜寻设备
  this->declare_parameter<bool>("save_audio_data", false);  // 新增参数：是否保存音频数据，默认为true

  // 获取参数
  this->format_ = this->get_parameter("format").as_int();
  this->channels_ = this->get_parameter("channels").as_int();
  this->rate_ = this->get_parameter("rate").as_int();
  int chunk_duration_ms = this->get_parameter("chunk_duration_ms").as_int();  // 获取音频块时长
  this->chunk_ = int(this->rate_ * chunk_duration_ms / 1000);  // 根据时长计算块大小
  this->chunk_duration_ms_ = chunk_duration_ms;  // 保存音频块时长
  int device = this->get_parameter("device").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();
  bool auto_search_device = this->get_parameter("auto_search_device").as_bool();  // 获取新参数
  this->save_audio_data_ = this->get_parameter("save_audio_data").as_bool();  // 获取保存音频数据参数
  
  // 确保chunk_至少为1
  if (this->chunk_ < 1) {
    this->chunk_ = 1;
  }
  
  // 打印调试信息
  RCLCPP_INFO(this->get_logger(), "配置参数: 采样率=%d, chunk_duration_ms=%d, 计算得出的chunk_=%d, save_audio_data_=%s", 
              this->rate_, chunk_duration_ms, this->chunk_, this->save_audio_data_ ? "true" : "false");
  
  // 从配置文件读取额外参数（如果存在）
  // 注意：ROS 2参数系统中，嵌套参数需要使用点号分隔，例如"audio_capturer.rate"
  // 但在YAML文件中，它们是嵌套结构
  try {
    // 优先从嵌套参数读取（配置文件中的参数）
    if (this->has_parameter("audio_capturer.device_name")) {
      // 尝试从嵌套参数读取
      std::string nested_device_name = this->get_parameter("audio_capturer.device_name").as_string();
      if (!nested_device_name.empty()) {
        this->device_name_ = nested_device_name;
        RCLCPP_INFO(this->get_logger(), "从配置文件读取设备名称: %s", this->device_name_.c_str());
      }
    } else if (this->has_parameter("device_name") && !this->get_parameter("device_name").as_string().empty()) {
      this->device_name_ = this->get_parameter("device_name").as_string();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取设备名称: %s", this->device_name_.c_str());
    }
    
    if (this->has_parameter("audio_capturer.rate")) {
      this->rate_ = this->get_parameter("audio_capturer.rate").as_int();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取采样率: %d Hz", this->rate_);
    } else if (this->has_parameter("rate")) {
      this->rate_ = this->get_parameter("rate").as_int();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取采样率: %d Hz", this->rate_);
    }
    
    if (this->has_parameter("audio_capturer.channels")) {
      this->channels_ = this->get_parameter("audio_capturer.channels").as_int();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取声道数: %d", this->channels_);
    } else if (this->has_parameter("channels")) {
      this->channels_ = this->get_parameter("channels").as_int();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取声道数: %d", this->channels_);
    }
    
    if (this->has_parameter("audio_capturer.chunk_duration_ms")) {
      chunk_duration_ms = this->get_parameter("audio_capturer.chunk_duration_ms").as_int();
      this->chunk_ = int(this->rate_ * chunk_duration_ms / 1000);
      this->chunk_duration_ms_ = chunk_duration_ms;
      RCLCPP_INFO(this->get_logger(), "从配置文件读取音频块时长: %d ms", chunk_duration_ms);
    } else if (this->has_parameter("chunk_duration_ms")) {
      chunk_duration_ms = this->get_parameter("chunk_duration_ms").as_int();
      this->chunk_ = int(this->rate_ * chunk_duration_ms / 1000);
      this->chunk_duration_ms_ = chunk_duration_ms;
      RCLCPP_INFO(this->get_logger(), "从配置文件读取音频块时长: %d ms", chunk_duration_ms);
    }
    
    // 读取save_audio_data参数
    if (this->has_parameter("audio_capturer.save_audio_data")) {
      this->save_audio_data_ = this->get_parameter("audio_capturer.save_audio_data").as_bool();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取保存音频数据参数: %s", this->save_audio_data_ ? "true" : "false");
    } else if (this->has_parameter("save_audio_data")) {
      this->save_audio_data_ = this->get_parameter("save_audio_data").as_bool();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取保存音频数据参数: %s", this->save_audio_data_ ? "true" : "false");
    }
    
    // 确保chunk_至少为1
    if (this->chunk_ < 1) {
      this->chunk_ = 1;
    }
    
    // 打印调试信息
    RCLCPP_INFO(this->get_logger(), "最终参数: 采样率=%d, chunk_duration_ms=%d, chunk_=%d, save_audio_data_=%s", 
                this->rate_, this->chunk_duration_ms_, this->chunk_, this->save_audio_data_ ? "true" : "false");
    
    if (this->has_parameter("audio_capturer.auto_search_device")) {
      auto_search_device = this->get_parameter("audio_capturer.auto_search_device").as_bool();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取自动搜寻设备参数: %s", auto_search_device ? "true" : "false");
    } else if (this->has_parameter("auto_search_device")) {
      auto_search_device = this->get_parameter("auto_search_device").as_bool();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取自动搜寻设备参数: %s", auto_search_device ? "true" : "false");
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "读取配置文件参数时出错: %s，使用默认值", e.what());
  }
  
  // 初始化PortAudio
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio错误: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("初始化PortAudio失败");
  }
  
  // 如果提供了设备名称，则使用设备名称查找设备ID
  if (!this->device_name_.empty()) 
  {
    RCLCPP_INFO(this->get_logger(), "正在查找设备名称: %s", this->device_name_.c_str());
    // 打印设备详细信息
    audio_common::AudioDeviceManager::print_device_details(this->device_name_, this->get_logger());
    
    int found_device = audio_common::AudioDeviceManager::find_device_id_by_name(this->device_name_, true, this->get_logger());
    if (found_device >= 0) 
    {
      device = found_device;
      RCLCPP_INFO(this->get_logger(), "成功找到音频采集设备，设备ID: %d", device);
    } 
    else 
    {
      // 根据auto_search_device参数决定行为
      if (auto_search_device) 
      {
        RCLCPP_WARN(this->get_logger(), "未找到指定名称的设备: %s，自动寻找可用的输入设备", this->device_name_.c_str());
        // 自动寻找可用的输入设备
        int available_device = audio_common::AudioDeviceManager::find_available_input_device(this->get_logger());
        if (available_device >= 0) {
          device = available_device;
          RCLCPP_INFO(this->get_logger(), "自动找到可用的音频采集设备，设备ID: %d", device);
          // 获取设备信息并显示
          const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(device);
          if (deviceInfo != nullptr) {
            RCLCPP_INFO(this->get_logger(), "自动选择的设备信息: %s", deviceInfo->name);
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "未找到可用的输入设备，使用默认设备ID: %d", Pa_GetDefaultInputDevice());
          device = Pa_GetDefaultInputDevice();
        }
      } else {
        // 不自动搜寻设备，直接报警并使用默认设备
        RCLCPP_ERROR(this->get_logger(), "未找到指定名称的设备: %s，且未启用自动搜寻设备功能", this->device_name_.c_str());
        device = Pa_GetDefaultInputDevice();
      }
    }
  }
  
  const PaDeviceInfo *deviceInfo = Pa_GetDeviceInfo(device);
  if (deviceInfo == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "无法获取设备ID %d 的信息", device);
    throw std::runtime_error("无法获取设备信息");
  }
  
  double defaultSampleRate = deviceInfo->defaultSampleRate;
  RCLCPP_INFO(this->get_logger(), "设备默认采样率: %.0f Hz", defaultSampleRate);
  RCLCPP_INFO(this->get_logger(), "配置采样率: %d Hz", this->rate_);
  
  // 检查设备是否支持配置的采样率
  bool sampleRateSupported = false;
  double sampleRates[] = {8000.0, 11025.0, 16000.0, 22050.0, 32000.0, 44100.0, 48000.0, 88200.0, 96000.0};
  int numSampleRates = sizeof(sampleRates) / sizeof(sampleRates[0]);
  
  // 首先尝试使用配置的采样率
  for (int i = 0; i < numSampleRates; i++) {
    if ((int)sampleRates[i] == this->rate_) {
      sampleRateSupported = true;
      break;
    }
  }
  
  // 如果配置的采样率不被支持，尝试使用设备默认采样率
  if (!sampleRateSupported) {
    RCLCPP_WARN(this->get_logger(), "配置的采样率 %d Hz 可能不被设备支持，尝试使用设备默认采样率 %.0f Hz", 
                this->rate_, defaultSampleRate);
    this->rate_ = (int)defaultSampleRate;
  }

  PaStreamParameters inputParameters;
  inputParameters.device = (device >= 0) ? device : Pa_GetDefaultInputDevice();
  inputParameters.channelCount = this->channels_;
  inputParameters.sampleFormat = this->format_;
  inputParameters.suggestedLatency =
      Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = nullptr;

  RCLCPP_INFO(this->get_logger(), "正在打开音频流: 采样率=%d, 通道数=%d, 数据块大小=%d", 
              this->rate_, this->channels_, this->chunk_);
  err = Pa_OpenStream(&this->stream_, &inputParameters,
                      nullptr, // 输出参数（未使用）
                      this->rate_, this->chunk_, paClipOff, nullptr, nullptr);

  if (err == paNoError) 
  {
    RCLCPP_INFO(this->get_logger(), "成功打开音频流✅");
    RCLCPP_INFO(this->get_logger(), "流参数: 采样率=%d, 通道数=%d, 数据块大小=%d", 
                this->rate_, this->channels_, this->chunk_);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "打开音频流失败: %s", Pa_GetErrorText(err));
    RCLCPP_ERROR(this->get_logger(), "请求的参数: 采样率=%d, 通道数=%d, 格式=%d", 
                 this->rate_, this->channels_, this->format_);
    RCLCPP_ERROR(this->get_logger(), "设备信息: %s, 默认采样率=%.0f, 最大输入通道数=%d", 
                 deviceInfo->name, deviceInfo->defaultSampleRate, deviceInfo->maxInputChannels);
    throw std::runtime_error("打开PortAudio流失败");
  }

  err = Pa_StartStream(this->stream_);
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "启动音频流失败: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("启动PortAudio流失败");
  }

  this->audio_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio/microphone", rclcpp::SensorDataQoS());
          
  // 创建唤醒状态发布者
  this->awake_status_pub_ =
      this->create_publisher<custom_msgs_comm::msg::SpeechStatus>(
          "speech/awake_status", 10);
          
  // 创建音频设备状态订阅者
  this->audio_status_sub_ =
      this->create_subscription<custom_msgs_comm::msg::AudioDeviceStatus>(
          "audio_device_status", 10,
          [this](const custom_msgs_comm::msg::AudioDeviceStatus::SharedPtr msg) {
              // 更新本地音频设备状态
              this->audio_device_status_ = msg->audio_status;
              
              // 打印日志信息
              const char* status_name = (msg->audio_status == custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS) ? "RECORDING" : 
                                       (msg->audio_status == custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS) ? "PLAYING" : "IDLE";
              
              RCLCPP_INFO(this->get_logger(), "接收到音频设备状态更新: %s", status_name);
          });

  // 注意：不在构造函数中初始化音频设备状态发布器，避免使用shared_from_this()
  // 音频设备状态发布器将在节点完全构造后通过定时器初始化
  
  // 创建一次性定时器，用于延迟初始化音频设备状态发布器和唤醒词检测器
  auto init_timer_period = std::chrono::milliseconds(100); // 100毫秒后初始化
  init_timer_ = this->create_wall_timer(init_timer_period, [this]() {
    // 初始化音频设备状态发布器（使用新的AudioDeviceManager）
    auto device_manager = audio_common::AudioDeviceManager::getInstance();
    device_manager->set_audio_device_status(custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS, shared_from_this());
    
    // 初始化唤醒词检测器
    this->initialize_wake_word_detector();
    
    // 启动音频采集线程
    audio_thread_running_ = true;
    audio_thread_ = std::thread(&AudioCapturerNode::audio_thread_func, this);
    
    // 取消定时器，确保只执行一次
    if (init_timer_) {
      init_timer_->cancel();
    }
  });
  
  // 创建设备状态检查定时器，每5秒检查一次设备状态
  auto device_check_period = std::chrono::seconds(5);
  device_check_timer_ = this->create_wall_timer(device_check_period, std::bind(&AudioCapturerNode::device_check_callback, this));

  RCLCPP_INFO(this->get_logger(), "音频采集节点启动成功");
  RCLCPP_INFO(this->get_logger(), "设备信息: %s", deviceInfo->name);
  RCLCPP_INFO(this->get_logger(), "采样率: %d Hz, 通道数: %d, 数据块大小: %d", 
              this->rate_, this->channels_, this->chunk_);
  RCLCPP_INFO(this->get_logger(), "保存音频数据: %s", this->save_audio_data_ ? "启用" : "禁用");
              
  // 初始化VAD相关参数
  silent_chunks_ = 0;
  is_speaking_ = false;
  max_silence_ms_ = 1500;  // 最大静音时长1.5秒
  
  // 初始化WebRTC VAD
  #ifdef HAVE_WEBRTC_VAD
  RCLCPP_INFO(this->get_logger(), "尝试初始化WebRTC VAD");
  vad_inst_ = WebRtcVad_Create();
  if (vad_inst_ != nullptr) {
    RCLCPP_INFO(this->get_logger(), "WebRTC VAD创建成功");
    int vad_init_result = WebRtcVad_Init(vad_inst_);
    if (vad_init_result == 0) {
      RCLCPP_INFO(this->get_logger(), "WebRTC VAD初始化成功");
      // 设置VAD的敏感度模式（0-3，3最敏感）
      int vad_mode_result = WebRtcVad_set_mode(vad_inst_, 1);
      if (vad_mode_result == 0) {
        RCLCPP_INFO(this->get_logger(), "WebRTC VAD设置模式成功");
      } else {
        RCLCPP_WARN(this->get_logger(), "WebRTC VAD设置模式失败: %d", vad_mode_result);
        WebRtcVad_Free(vad_inst_);
        vad_inst_ = nullptr;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "WebRTC VAD初始化失败: %d", vad_init_result);
      WebRtcVad_Free(vad_inst_);
        vad_inst_ = nullptr;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "WebRTC VAD创建失败");
  }
  #endif
  
  // 注意：不在构造函数中初始化唤醒词检测器，避免使用shared_from_this()
  // 唤醒词检测器将在节点完全构造后通过定时器初始化
  wake_word_detector_ = nullptr;
}

// 添加初始化唤醒词检测器的方法
void AudioCapturerNode::initialize_wake_word_detector() 
{
  // 初始化唤醒词检测器（在节点完全构造后）
  //wake_word_detector_ = std::make_shared<audio_common::WakeWordDetector>(shared_from_this());
  // wake_word_detector_ = std::make_shared<audio_common::WakeWordDetector>();
  //rclcpp::spin(wake_word_detector_);
  if (wake_word_detector_ == nullptr)
  {
    RCLCPP_ERROR(this->get_logger(), "初始化唤醒词检测器对象失败");
    return;
  }
  
  // 使用 lambda 捕获 this
  wake_word_detector_->set_wake_status_callback([this]() {
    this->wake_status_off_callback();
  });
  if (!wake_word_detector_->initialize()) 
  {
    RCLCPP_ERROR(this->get_logger(), "初始化唤醒词检测器失败");
  }
  
  if (wake_word_detector_->is_enabled()) {
    RCLCPP_INFO(this->get_logger(), "唤醒词检测功能已启用");
  } else {
    RCLCPP_INFO(this->get_logger(), "唤醒词检测功能已禁用");
  }
}

AudioCapturerNode::~AudioCapturerNode() 
{
  // 释放WebRTC VAD资源
  #ifdef HAVE_WEBRTC_VAD
  if (vad_inst_ != nullptr) {
    WebRtcVad_Free(vad_inst_);
    vad_inst_ = nullptr;
  }
  #endif
  
  audio_thread_running_ = false;
  if (audio_thread_.joinable()) 
  {
    audio_thread_.join();
  }

  if (this->stream_) 
  {
    Pa_StopStream(this->stream_);
    Pa_CloseStream(this->stream_);
  }
  Pa_Terminate();
}

void AudioCapturerNode::device_check_callback() 
{
  // 定期检查设备是否仍然连接
  if (!this->device_name_.empty()) 
  {
    // 检查流是否停止或出错
    if (this->stream_ && (Pa_IsStreamStopped(this->stream_) || Pa_IsStreamActive(this->stream_) == 0)) 
    {
      RCLCPP_WARN(this->get_logger(), "检测到音频流异常，尝试重新初始化音频流");
      if (!this->reinitialize_stream()) {
        RCLCPP_ERROR(this->get_logger(), "重新初始化音频流失败");
      }
    }
  }
}

bool AudioCapturerNode::reinitialize_stream() 
{
  // 停止并关闭当前流
  if (this->stream_) {
    Pa_StopStream(this->stream_);
    Pa_CloseStream(this->stream_);
    this->stream_ = nullptr;
  }
  
  // 获取auto_search_device参数
  bool auto_search_device = false;
  try 
  {
    if (this->has_parameter("auto_search_device")) 
    {
      auto_search_device = this->get_parameter("auto_search_device").as_bool();
    }
    if (this->has_parameter("audio_capturer.auto_search_device")) 
    {
      auto_search_device = this->get_parameter("audio_capturer.auto_search_device").as_bool();
    }
  } catch (const std::exception& e) 
  {
    RCLCPP_WARN(this->get_logger(), "读取auto_search_device参数时出错: %s，使用默认值false", e.what());
  }
  
  // 重新查找设备
  int device = -1;
  if (!this->device_name_.empty()) 
  {
    RCLCPP_INFO(this->get_logger(), "重新查找设备名称: %s", this->device_name_.c_str());
    device = audio_common::AudioDeviceManager::find_device_id_by_name(this->device_name_, true, this->get_logger());
    if (device < 0) {
      // 根据auto_search_device参数决定行为
      if (auto_search_device) {
        RCLCPP_WARN(this->get_logger(), "重新查找指定设备失败: %s，自动寻找可用的输入设备", this->device_name_.c_str());
        // 自动寻找可用的输入设备
        int available_device = audio_common::AudioDeviceManager::find_available_input_device(this->get_logger());
        if (available_device >= 0) {
          device = available_device;
          RCLCPP_INFO(this->get_logger(), "自动找到可用的音频采集设备，设备ID: %d", device);
          // 获取设备信息并显示
          const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(device);
          if (deviceInfo != nullptr) {
            RCLCPP_INFO(this->get_logger(), "自动选择的设备信息: %s", deviceInfo->name);
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "未找到可用的输入设备，使用默认设备");
          device = Pa_GetDefaultInputDevice();
        }
      } else {
        // 不自动搜寻设备，直接报警
        RCLCPP_ERROR(this->get_logger(), "重新查找指定设备失败: %s，且未启用自动搜寻设备功能", this->device_name_.c_str());
        device = Pa_GetDefaultInputDevice();
      }
    }
  }
  
  // 如果仍然没有找到设备，使用默认设备
  if (device < 0) 
  {
    device = Pa_GetDefaultInputDevice();
    RCLCPP_WARN(this->get_logger(), "使用默认输入设备，设备ID: %d", device);
  }
  
  // 检查设备是否有效
  const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(device);
  if (deviceInfo == nullptr) 
  {
    RCLCPP_ERROR(this->get_logger(), "无法获取设备ID %d 的信息", device);
    return false;
  }
  
  // 重新打开音频流
  PaStreamParameters inputParameters;
  inputParameters.device = device;
  inputParameters.channelCount = this->channels_;
  inputParameters.sampleFormat = this->format_;
  inputParameters.suggestedLatency = deviceInfo->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = nullptr;

  PaError err = Pa_OpenStream(&this->stream_, &inputParameters,
                      nullptr, // 输出参数（未使用）
                      this->rate_, this->chunk_, paClipOff, nullptr, nullptr);

  if (err != paNoError) 
  {
    RCLCPP_ERROR(this->get_logger(), "重新打开音频流失败: %s", Pa_GetErrorText(err));
    return false;
  }

  err = Pa_StartStream(this->stream_);
  if (err != paNoError) 
  {
    RCLCPP_ERROR(this->get_logger(), "重新启动音频流失败: %s", Pa_GetErrorText(err));
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "成功重新初始化音频流");
  return true;
}

bool AudioCapturerNode::record_audio_streaming(std::vector<int16_t>& audio_data) {
  /**
   * 获取实时音频数据流，实现完整的静默监听
   */
  
  // 清空传入的音频数据向量
  audio_data.clear();
  
  // 重置VAD状态
  silent_chunks_ = 0;
  is_speaking_ = false;
  
  // 使用预先计算好的音频块时长，避免重复计算
  // 计算最大静音块数
  int max_silent_chunks = max_silence_ms_ / chunk_duration_ms_;
  if (max_silent_chunks < 1) max_silent_chunks = 1;
  
  // 打印调试信息
  RCLCPP_DEBUG(this->get_logger(), "开始录音: chunk_duration_ms_=%d, max_silent_chunks=%d", 
               chunk_duration_ms_, max_silent_chunks);
  
  // 临时缓冲区用于存储音频数据
  std::deque<int16_t> temp_buffer;
  
  // 添加语音活动计数器，避免噪声误判
  int voice_activity_count = 0;
  const int min_voice_chunks = 3; // 至少需要3个连续的语音块才认为是真正的语音
  
  try {
    int iteration_count = 0;
    while (rclcpp::ok()) {  // 确保ROS节点仍在运行
      iteration_count++;
      
      // 读取音频数据块
      std::vector<int16_t> chunk_data;
      switch (this->format_) {
        case paInt16: {
          chunk_data = this->read_data<int16_t>();
          break;
        }
        default:
          RCLCPP_ERROR(this->get_logger(), "当前仅支持16位整数格式的VAD处理");
          return false;
      }
      
      // 检查音频块是否包含语音，优先使用WebRTC VAD
      bool is_voice = is_voice_present_webrtc(chunk_data);
      
      // 打印调试信息
      if (iteration_count % 100 == 0) 
      {
        if (0)//暂时屏蔽
        {
          RCLCPP_INFO(this->get_logger(), "迭代 %d: is_voice=%s, is_speaking_=%s, silent_chunks_=%d, buffer_size=%ld, voice_activity_count=%d", 
                     iteration_count, is_voice ? "true" : "false", is_speaking_ ? "true" : "false", 
                     silent_chunks_, temp_buffer.size(), voice_activity_count);
        }
      }
      
      if (is_voice) {
        voice_activity_count++;
        // 只有当检测到足够的连续语音活动时，才认为真正开始说话
        if (voice_activity_count >= min_voice_chunks && !is_speaking_) {
          // 刚开始说话，清空临时缓冲区（但保留一部分前导静音）
          // 保留约300ms的前导静音
          size_t keep_samples = this->rate_ * 0.3;  // 300ms
          if (temp_buffer.size() > keep_samples) {
            temp_buffer.erase(temp_buffer.begin(), temp_buffer.end() - keep_samples);
          }
          RCLCPP_INFO(this->get_logger(), "检测到语音开始，voice_activity_count=%d", voice_activity_count);
          is_speaking_ = true;
          silent_chunks_ = 0;  // 重置静音计数器
        } else if (is_speaking_) {
          // 已经在说话状态，重置静音计数器
          silent_chunks_ = 0;
        }
      } else {
        // 重置语音活动计数器
        voice_activity_count = 0;
        
        if (is_speaking_) {
          silent_chunks_++;  // 如果之前在说话且现在是静音，则增加静音计数器
        }
      }
      
      // 将音频数据块添加到临时缓冲区
      for (const auto& sample : chunk_data) 
      {
        temp_buffer.push_back(sample);
      }
      
      // 如果检测到说话后的静音，并且静音时间超过阈值，则停止录音
      if (is_speaking_ && silent_chunks_ > max_silent_chunks) 
      {
        RCLCPP_INFO(this->get_logger(), "检测到静音，结束录音");
        break;
      }
      
      // 防止缓冲区过大（超过10秒的数据）
      if (temp_buffer.size() > (size_t)(this->rate_ * 10 * this->channels_)) 
      {
        // 即使缓冲区过大，如果检测到语音活动，也应该处理数据而不是直接清空
        if (is_speaking_) 
        {
          // 如果正在说话，发布当前缓冲区的数据
          RCLCPP_INFO(this->get_logger(), "缓冲区过大但在说话，发布当前数据");
          break;
        } else {
          //RCLCPP_WARN(this->get_logger(), "音频缓冲区过大，清空缓冲区");
          // 如果没有在说话，清空缓冲区
          temp_buffer.clear();
          is_speaking_ = false;
          silent_chunks_ = 0;
          voice_activity_count = 0;
        }
      }
    }
    
    // 将临时缓冲区的数据复制到输出参数
    audio_data.reserve(temp_buffer.size());
    for (const auto& sample : temp_buffer) 
    {
      audio_data.push_back(sample);
    }
    
    // 输出录音结束的详细信息
    RCLCPP_INFO(this->get_logger(), "录音结束: 音频数据大小=%ld samples, 采样率=%d Hz, 声道数=%d", 
                audio_data.size(), this->rate_, this->channels_);
    
    // 计算音频时长（秒）
    if (this->rate_ > 0 && this->channels_ > 0) {
      double duration = (double)audio_data.size() / (this->rate_ * this->channels_);
      RCLCPP_INFO(this->get_logger(), "录音时长: %.3f 秒", duration);
    }
    
    return !audio_data.empty();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "录音过程中发生错误: %s", e.what());
    return false;
  }
}

bool AudioCapturerNode::is_voice_present(const std::vector<int16_t>& audio_data) {
  /**
   * 检查音频数据是否包含语音信号（保持原有实现）
   * 改进实现，使其更接近VAD检测方法
   */
  if (audio_data.empty()) {
    return false;
  }
  
  // 计算音频数据的能量
  long long sum_square = 0;
  for (const auto& sample : audio_data) {
    sum_square += (long long)sample * sample;
  }
  
  // 计算RMS能量
  double rms = std::sqrt((double)sum_square / audio_data.size());
  
  // 使用更合理的阈值判断是否有语音
  // 根据音频数据的特性动态调整阈值
  // 提高阈值以减少噪声误判
  double threshold = 1000.0; // 从500.0提高到1000.0
  
  // 同时检查最大振幅，避免纯噪声被误判为语音
  int16_t max_amplitude = 0;
  for (const auto& sample : audio_data) 
  {
    if (std::abs(sample) > max_amplitude) 
    {
      max_amplitude = std::abs(sample);
    }
  }
  
  // 提高最大振幅阈值
  int16_t amplitude_threshold = 2000; // 从1000提高到2000
  
  // 打印调试信息
  RCLCPP_DEBUG(this->get_logger(), "能量检测: rms=%.2f, max_amplitude=%d, threshold=%.2f, amplitude_threshold=%d", 
               rms, max_amplitude, threshold, amplitude_threshold);
  
  // 只有当RMS能量超过阈值且最大振幅足够大时，才认为是语音
  bool result = (rms > threshold) && (max_amplitude > amplitude_threshold);
  RCLCPP_DEBUG(this->get_logger(), "能量检测结果: %s", result ? "true" : "false");
  
  return result;
}

bool AudioCapturerNode::is_voice_present_webrtc(const std::vector<int16_t>& audio_data) 
{
  /**
   * 使用WebRTC VAD检查音频数据是否包含语音信号
   */
  #ifdef HAVE_WEBRTC_VAD
  if (vad_inst_ == nullptr || audio_data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "VAD实例为空或音频数据为空");
    return false;
  }
  
  // WebRTC VAD要求特定的采样率（8000, 16000, 32000, 48000）
  // 并且要求音频数据长度为10, 20或30毫秒
  int vad_sample_rate = 0;
  int frame_length = 0;
  
  // 根据采样率选择合适的VAD采样率
  // WebRTC VAD只支持特定的采样率
  if (rate_ == 8000) {
    vad_sample_rate = 8000;
    frame_length = 80;   // 10ms: 8000 * 0.01
  } else if (rate_ == 16000) {
    vad_sample_rate = 16000;
    frame_length = 160;  // 10ms: 16000 * 0.01
  } else if (rate_ == 32000) {
    vad_sample_rate = 32000;
    frame_length = 320;  // 10ms: 32000 * 0.01
  } else if (rate_ == 48000) {
    vad_sample_rate = 48000;
    frame_length = 480;  // 10ms: 48000 * 0.01
  } else {
    // 对于不支持的采样率，回退到能量检测
    RCLCPP_INFO(this->get_logger(), "采样率 %d 不被WebRTC VAD支持，回退到能量检测", rate_);
    return is_voice_present(audio_data);
  }
  
  // 打印调试信息
  RCLCPP_INFO(this->get_logger(), "VAD参数: rate_=%d, vad_sample_rate=%d, frame_length=%d, audio_data.size()=%ld", 
               rate_, vad_sample_rate, frame_length, audio_data.size());
  
  // 检查音频数据长度是否符合要求（10, 20, 或 30ms）
  // WebRTC VAD支持的帧长度对应于10ms、20ms、30ms的音频数据
  bool valid_frame_length = (frame_length == audio_data.size()) || 
                            (frame_length * 2 == audio_data.size()) || 
                            (frame_length * 3 == audio_data.size());
  
  if (!valid_frame_length) {
    RCLCPP_INFO(this->get_logger(), "音频数据长度 %ld 不符合WebRTC VAD要求（应为 %d, %d, 或 %d）", 
                 audio_data.size(), frame_length, frame_length * 2, frame_length * 3);
    
    // 如果音频数据太短，直接返回false
    if (audio_data.size() < frame_length) {
      RCLCPP_INFO(this->get_logger(), "音频数据太短，返回false");
      return false;
    }
    
    // 如果音频数据长度不是frame_length的整数倍，调整为最接近的倍数
    int adjusted_frame_length = frame_length;
    if (audio_data.size() >= frame_length * 3) {
      adjusted_frame_length = frame_length * 3;  // 30ms
    } else if (audio_data.size() >= frame_length * 2) {
      adjusted_frame_length = frame_length * 2;  // 20ms
    }
    // 否则使用10ms (frame_length)
    
    // 使用调整后的长度
    std::vector<int16_t> vad_data(audio_data.begin(), audio_data.begin() + adjusted_frame_length);
    
    // 调用WebRTC VAD接口
    int result = WebRtcVad_Process(vad_inst_, vad_sample_rate, vad_data.data(), adjusted_frame_length);
    
    // 打印调试信息
    RCLCPP_INFO(this->get_logger(), "调整后WebRTC VAD结果: %d", result);
    
    // WebRtcVad_Process返回1表示检测到语音，0表示未检测到语音，-1表示错误
    if (result == -1) {
      RCLCPP_WARN(this->get_logger(), "WebRTC VAD处理错误");
      return false;
    }
    
    return (result == 1);
  }
  
  // 音频数据长度符合要求，直接使用
  // 调用WebRTC VAD接口
  int result = WebRtcVad_Process(vad_inst_, vad_sample_rate, audio_data.data(), audio_data.size());
  
  // 打印调试信息
  RCLCPP_INFO(this->get_logger(), "WebRTC VAD结果: %d", result);
  
  // WebRtcVad_Process返回1表示检测到语音，0表示未检测到语音，-1表示错误
  if (result == -1) {
    RCLCPP_WARN(this->get_logger(), "WebRTC VAD处理错误");
    return false;
  }
  
  return (result == 1);
  #else
  // 如果没有WebRTC VAD库，回退到原有的能量检测方法
  RCLCPP_DEBUG(this->get_logger(), "使用能量检测方法");
  return is_voice_present(audio_data);
  #endif
}

bool AudioCapturerNode::detect_wake_word(const std::vector<int16_t>& audio_data) 
{
  /**
   * 对音频数据进行唤醒词检测
   */
  if (!is_wake_reday())
    return false;
  return wake_word_detector_->detect(audio_data, this->rate_, this->channels_);
}

bool AudioCapturerNode::is_wake_reday()
{
  if (!wake_word_detector_ || !wake_word_detector_->is_enabled()) 
  {
    return false;
  }
  return true;
}

bool AudioCapturerNode::is_wake_status_on()
{
  if (!is_wake_reday())
    return false;
  bool bRes = wake_word_detector_->is_wake_status_on();
  // if (bRes)
  //   RCLCPP_INFO(this->get_logger(), "当前是唤醒状态！");
  // else
  //   RCLCPP_INFO(this->get_logger(), "当前不是唤醒状态！");
  return bRes;
}

void AudioCapturerNode::set_wake_status(int wake_status)
{
  //if (is_wake_reday())
    //wake_word_detector_->set_wake_status(wake_status);
}

void AudioCapturerNode::updata_wake_time()
{
  RCLCPP_INFO(this->get_logger(), "发布语音, 刷新唤醒记时，重新开始计时！");
  if (!is_wake_reday())
    return;
  wake_word_detector_->updata_wake_time();
}

// 唤醒停止回调
void AudioCapturerNode::wake_status_off_callback()
{
  
  RCLCPP_INFO(this->get_logger(), "进入回调，唤醒状态结束，可以重新唤醒！");
  
  // 发布语音休眠状态
  auto status_msg = custom_msgs_comm::msg::SpeechStatus();
  status_msg.awake_status = custom_msgs_comm::msg::SpeechStatus::SLEEP_STATUS;  // 使用新的字段名
  this->awake_status_pub_->publish(status_msg);
  RCLCPP_INFO(this->get_logger(), "发布语音唤醒状态: SLEEP");
}

void AudioCapturerNode::publish_audio_data(const std::vector<int16_t>& audio_data, uint8_t voice_type)
{
  /**
   * 发布音频数据
   */
  if (audio_data.empty()) 
  {
    RCLCPP_WARN(this->get_logger(), "尝试发布空的音频数据");
    return;
  }
  // 输出音频数据的完整信息
  RCLCPP_DEBUG(this->get_logger(), "音频数据信息: 大小=%ld samples, 格式=%d, 采样率=%d Hz, 声道数=%d, 数据块大小=%d", 
              audio_data.size(), this->format_, this->rate_, this->channels_, this->chunk_);
  
  // 计算音频时长（秒）
  if (this->rate_ > 0) {
    double duration = (double)audio_data.size() / (this->rate_ * this->channels_);
    RCLCPP_DEBUG(this->get_logger(), "音频时长: %.3f 秒", duration);
  }
  
  // 如果启用了保存音频数据功能，则保存音频数据
  if (this->save_audio_data_) {
    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << "debug_audio/audio_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count() << ".wav";
    
    std::string filename = ss.str();
    if (save_audio_to_wav(audio_data, filename)) {
      RCLCPP_DEBUG(this->get_logger(), "音频数据已保存到: %s", filename.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "保存音频数据失败: %s", filename.c_str());
    }
  }
  
  auto msg = audio_common_msgs::msg::AudioStamped();
  msg.header.frame_id = this->frame_id_;
  msg.header.stamp = this->get_clock()->now();
  
  // 将音频数据复制到消息中
  msg.audio.audio_data.int16_data.reserve(audio_data.size());
  for (const auto& sample : audio_data) {
    msg.audio.audio_data.int16_data.push_back(sample);
  }
  
  msg.audio.info.format = this->format_;
  msg.audio.info.channels = this->channels_;
  msg.audio.info.chunk = this->chunk_;
  msg.audio.info.rate = this->rate_;
  msg.audio.info.voice_type = voice_type; // 使用传入的声音类型参数
  
  this->audio_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "发布音频数据，大小: %ld, 声音类型: %d", audio_data.size(), voice_type);
  
  // 如果是唤醒状态，发布语音唤醒状态
  if (voice_type == static_cast<uint8_t>(cmd_dispatcher::Voice_Type::WAKEUP_ENABLE)) 
  {
    auto status_msg = custom_msgs_comm::msg::SpeechStatus();
    status_msg.awake_status = custom_msgs_comm::msg::SpeechStatus::AWAKE_STATUS;  // 使用新的字段名
    this->awake_status_pub_->publish(status_msg);
    RCLCPP_INFO(this->get_logger(), "发布语音唤醒状态: AWAKE");
  }
}

// 添加音频线程函数
void AudioCapturerNode::audio_thread_func() 
{
    RCLCPP_DEBUG(this->get_logger(), "音频采集线程启动");
    
    while (rclcpp::ok() && audio_thread_running_) 
    {
        auto start_time = std::chrono::steady_clock::now();
        
        // 检查流状态
        if (!this->stream_ || Pa_IsStreamStopped(this->stream_) || Pa_IsStreamActive(this->stream_) == 0) 
        {
            RCLCPP_WARN(this->get_logger(), "音频流未激活或已停止，尝试重新初始化");
            if (!this->reinitialize_stream()) 
            {
                RCLCPP_ERROR(this->get_logger(), "重新初始化音频流失败，等待后重试");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
        }
        
        // 获取实时音频数据流
        std::vector<int16_t> audio_data;
        RCLCPP_DEBUG(this->get_logger(), "开始调用record_audio_streaming");
        
        if (is_wake_reday() && record_audio_streaming(audio_data)) 
        {
            RCLCPP_DEBUG(this->get_logger(), "record_audio_streaming返回成功，音频数据大小: %ld", audio_data.size());
            
            // 使用互斥锁保护共享资源访问
            std::lock_guard<std::mutex> lock(audio_mutex_);
            
            // 检查当前是否处于播音状态（使用本地状态变量）
            // auto device_manager = audio_common::AudioDeviceManager::getInstance();
            // const uint8_t status = device_manager->get_audio_device_status();
            const uint8_t status = this->audio_device_status_;
            if (status == custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS) 
            {
                RCLCPP_INFO(this->get_logger(), "当前处于播音状态，只有检测到唤醒词才发布音频数据");
                
                // 打印日志信息
                const char* status_name = (status == custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS) ? "RECORDING" : 
                                        (status == custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS) ? "PLAYING" : "IDLE";
                
                RCLCPP_INFO(this->get_logger(), "当前音频设备状态: %s", status_name);

                // 在播音状态下，只有检测到唤醒词才发布音频数据
                if (detect_wake_word(audio_data)) 
                {
                    RCLCPP_INFO(this->get_logger(), "播音状态下检测到唤醒词，发布音频数据！");
                    publish_audio_data(audio_data, static_cast<uint8_t>(cmd_dispatcher::Voice_Type::WAKEUP_ENABLE));
                } 
                else 
                {
                    RCLCPP_INFO(this->get_logger(), "播音状态下未检测到唤醒词，不发布音频数据");
                }
            }
            else 
            {
                // 录音状态下正常处理
                RCLCPP_INFO(this->get_logger(), "当前处于录音状态，开始检测唤醒词");
                
                if (detect_wake_word(audio_data)) 
                {
                    RCLCPP_INFO(this->get_logger(), "检测到唤醒词，开始计时，并置为唤醒状态！");
                    // 设置音频设备状态为播放状态（使用新的AudioDeviceManager）
                    this->audio_device_status_ = custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS;
                    // 如果检测到唤醒词，则发布音频数据，声音类型为WAKEUP_ENABLE
                    publish_audio_data(audio_data, static_cast<uint8_t>(cmd_dispatcher::Voice_Type::WAKEUP_ENABLE));
                } 
                else 
                {
                    if (is_wake_status_on()) // 唤醒状态下才发布语音
                    {
                        RCLCPP_INFO(this->get_logger(), "已经是唤醒模式，检测到语音");
                        updata_wake_time();
                        // 设置音频设备状态为播放状态（使用新的AudioDeviceManager）
                        this->audio_device_status_ = custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS;
                        // 如果未检测到唤醒词，则发布音频数据，声音类型为NORMAL_COMMAND
                        publish_audio_data(audio_data, static_cast<uint8_t>(cmd_dispatcher::Voice_Type::NORMAL_COMMAND));
                    }
                    else{
                      RCLCPP_INFO(this->get_logger(), "唤醒词检测失败");
                    } 
                }   
            }
        } 
        else 
        {
            RCLCPP_DEBUG(this->get_logger(), "record_audio_streaming返回失败或空数据");
        }
        audio_data.clear();
        // 计算本次处理耗时，并调整睡眠时间以保持10ms间隔
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (elapsed < std::chrono::milliseconds(10)) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10) - elapsed);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "音频采集线程退出");
}

void AudioCapturerNode::timer_callback() 
{
  static int count = 0;
  count++;
  
  // 检查流是否有效，使用更准确的检查方法
  if (!this->stream_ || Pa_IsStreamStopped(this->stream_) || Pa_IsStreamActive(this->stream_) == 0) {
    RCLCPP_WARN(this->get_logger(), "音频流未激活或已停止，尝试重新初始化");
    if (!this->reinitialize_stream()) {
      RCLCPP_ERROR(this->get_logger(), "重新初始化音频流失败，跳过本次采集");
      return;
    }
  }
  
  // 获取实时音频数据流
  std::vector<int16_t> audio_data;
  RCLCPP_DEBUG(this->get_logger(), "开始调用record_audio_streaming");
  if (record_audio_streaming(audio_data)) 
  {
    RCLCPP_DEBUG(this->get_logger(), "record_audio_streaming返回成功，音频数据大小: %ld", audio_data.size());
    // 对音频数据进行唤醒词检测
    if (detect_wake_word(audio_data)) 
    {
      // 如果检测到唤醒词，则发布音频数据，声音类型为WAKEUP_ENABLE
      publish_audio_data(audio_data, static_cast<uint8_t>(cmd_dispatcher::Voice_Type::WAKEUP_ENABLE));
    } else {
      // 如果未检测到唤醒词，则发布音频数据，声音类型为NORMAL_COMMAND
      publish_audio_data(audio_data, static_cast<uint8_t>(cmd_dispatcher::Voice_Type::NORMAL_COMMAND));
      RCLCPP_DEBUG(this->get_logger(), "唤醒词检测失败");
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "record_audio_streaming返回失败或空数据");
  }
  
  // 每100次迭代打印一次信息
  if (count % 100 == 0) 
  {
    RCLCPP_INFO(this->get_logger(), "已处理 %d 个音频块", count);
  }
}

template <typename T> std::vector<T> AudioCapturerNode::read_data() 
{
  std::vector<T> data(this->chunk_ * this->channels_);
  PaError err = Pa_ReadStream(this->stream_, data.data(), this->chunk_);
  // 处理输入溢出错误
  if (err != paNoError && err != paInputOverflowed) 
  {
    RCLCPP_WARN(this->get_logger(), "读取音频流时出错: %s", Pa_GetErrorText(err));
  }
  
  return data;
}

bool AudioCapturerNode::save_audio_to_wav(const std::vector<int16_t>& audio_data, const std::string& filename) 
{
  /**
   * 保存音频数据为WAV文件
   * 参考Python版本的save_audio_data函数实现
   */
  try {
    // 创建debug_audio目录（如果不存在）
    std::string dir = "debug_audio";
    // 简单的目录创建方法（在Windows上）
    #ifdef _WIN32
        system("if not exist debug_audio mkdir debug_audio");
    #else
        system("mkdir -p debug_audio");  // ✅ Linux/macOS 命令
    #endif
    
    // 打开文件
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", filename.c_str());
      return false;
    }
    
    // WAV文件头信息
    const int sample_rate = this->rate_;
    const int channels = this->channels_;
    const int bits_per_sample = 16;
    const int byte_rate = sample_rate * channels * bits_per_sample / 8;
    const int block_align = channels * bits_per_sample / 8;
    const int subchunk2_size = audio_data.size() * (bits_per_sample / 8);
    const int chunk_size = 36 + subchunk2_size;
    
    // 写入RIFF头
    file.write("RIFF", 4);
    file.write(reinterpret_cast<const char*>(&chunk_size), 4);
    file.write("WAVE", 4);
    
    // 写入fmt子块
    file.write("fmt ", 4);
    const int subchunk1_size = 16;
    file.write(reinterpret_cast<const char*>(&subchunk1_size), 4);
    const short audio_format = 1; // PCM
    file.write(reinterpret_cast<const char*>(&audio_format), 2);
    file.write(reinterpret_cast<const char*>(&channels), 2);
    file.write(reinterpret_cast<const char*>(&sample_rate), 4);
    file.write(reinterpret_cast<const char*>(&byte_rate), 4);
    file.write(reinterpret_cast<const char*>(&block_align), 2);
    file.write(reinterpret_cast<const char*>(&bits_per_sample), 2);
    
    // 写入data子块
    file.write("data", 4);
    file.write(reinterpret_cast<const char*>(&subchunk2_size), 4);
    
    // 写入音频数据
    file.write(reinterpret_cast<const char*>(audio_data.data()), subchunk2_size);
    
    file.close();
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "保存音频文件时出错: %s", e.what());
    return false;
  }
}

