// MIT License
//
// Copyright (c) 2024 Your Name
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

#include "audio_common/audio_device_manager.hpp"
#include <cstring>
#include <iostream>
#include <sstream>
#include <memory>
#include <algorithm>
// 添加 PortAudio 头文件
#include <portaudio.h>
#include <custom_msgs_comm/msg/audio_device_status.hpp>
#include <rclcpp/rclcpp.hpp>

// 添加执行系统命令所需的头文件
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/wait.h>
#endif

using namespace audio_common;

// 静态变量定义和初始化
uint8_t AudioDeviceManager::audio_device_status_ = custom_msgs_comm::msg::AudioDeviceStatus::IDLE_STATUS;

// 静态变量存储实例
std::shared_ptr<AudioDeviceManager> AudioDeviceManager::instance_ = nullptr;

AudioDeviceManager::AudioDeviceManager() : logger_(rclcpp::get_logger("audio_device_manager")) {
    // 私有构造函数
    std::cout << "创建AudioDeviceManager实例，this指针地址: " << this << std::endl;
    std::cout << "audio_device_status_变量地址: " << static_cast<void*>(&audio_device_status_) << std::endl;
    std::cout << "实例ID: " << reinterpret_cast<uintptr_t>(this) << std::endl;
}

std::shared_ptr<AudioDeviceManager> AudioDeviceManager::getInstance() 
{
    std::cout << "调用getInstance()，当前instance_地址: " << instance_.get() << std::endl;
    if (!instance_) {
        std::cout << "创建新的AudioDeviceManager实例" << std::endl;
        instance_ = std::shared_ptr<AudioDeviceManager>(new AudioDeviceManager());
        std::cout << "新实例地址: " << instance_.get() << std::endl;
    } else {
        std::cout << "返回已存在的AudioDeviceManager实例，地址: " << instance_.get() << std::endl;
    }
    return instance_;
}

uint8_t AudioDeviceManager::get_audio_device_status() 
{
    std::cout << "获取音频设备状态，audio_device_status_ 地址: " << static_cast<void*>(&audio_device_status_) << std::endl;
    std::cout << "获取音频设备状态，audio_device_status_ 值: " << static_cast<int>(audio_device_status_) << std::endl;
    std::cout << "当前实例地址: " << this << std::endl;
    return audio_device_status_;
}

void AudioDeviceManager::set_audio_device_status(uint8_t status, rclcpp::Node::SharedPtr node) 
{
    audio_device_status_ = status;
    std::cout << "设置音频设备状态，audio_device_status_ 地址: " << static_cast<void*>(&audio_device_status_) << std::endl;
    std::cout << "设置音频设备状态，audio_device_status_ 值: " << static_cast<int>(audio_device_status_) << " -> " << static_cast<int>(status) << std::endl;
    std::cout << "当前实例地址: " << this << std::endl;
    
    // 如果提供了节点指针，初始化发布器
    if (node && !audio_status_pub_) {
        audio_status_pub_ = node->create_publisher<custom_msgs_comm::msg::AudioDeviceStatus>(
            "audio_device_status", 10);
        logger_ = node->get_logger();
    }
    
    publish_audio_device_status(status);  // 设置状态后自动发布
}

void AudioDeviceManager::publish_audio_device_status(uint8_t status) {
    if (audio_status_pub_) {
        auto status_msg = custom_msgs_comm::msg::AudioDeviceStatus();
        status_msg.audio_status = status;
        audio_status_pub_->publish(status_msg);
        
        // 打印日志信息
        const char* status_name = (status == custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS) ? "RECORDING" : 
                                 (status == custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS) ? "PLAYING" : "IDLE";
        
        RCLCPP_INFO(logger_, "发布音频设备状态: %s", status_name);
    }
}

std::string AudioDeviceManager::extract_hw_name(const std::string& device_name, rclcpp::Logger logger) {
    // 查找hw:x,y模式的子字符串
    size_t hw_pos = device_name.find("hw:");
    if (hw_pos == std::string::npos) {
        return "";
    }

    // 从hw:开始提取到下一个空格或字符串结尾
    size_t start = hw_pos;
    size_t end = device_name.find(' ', start);
    if (end == std::string::npos) {
        end = device_name.length();
    }

    return device_name.substr(start, end - start);
}

int AudioDeviceManager::find_device_id_by_name(const std::string& device_name, bool is_input_device, rclcpp::Logger logger) {
    // 初始化PortAudio库
    PaError err = Pa_Initialize();
    if (err != paNoError) 
    {
        RCLCPP_ERROR(logger, "PortAudio初始化失败: %s", Pa_GetErrorText(err));
        return -1;
    }
    
    // 获取设备数量
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0) 
    {
        RCLCPP_ERROR(logger, "获取PortAudio设备数量失败: %s", Pa_GetErrorText(numDevices));
        return -1;
    }
    
    RCLCPP_DEBUG(logger, "系统中共有 %d 个PortAudio设备", numDevices);

    // 遍历所有设备寻找匹配的设备
    const PaDeviceInfo* deviceInfo;
    for (int i = 0; i < numDevices; i++) 
    {
        deviceInfo = Pa_GetDeviceInfo(i);
        if (deviceInfo == nullptr) {
            RCLCPP_DEBUG(logger, "设备ID %d 信息为空，跳过", i);
            continue;
        }

        // 检查设备类型是否匹配请求的输入/输出类型
        if (is_input_device) 
        {
            if (deviceInfo->maxInputChannels <= 0) 
            {
                RCLCPP_DEBUG(logger, "设备ID %d 不是输入设备，跳过", i);
                continue; // 不是输入设备
            }
        } 
        else 
        {
            if (deviceInfo->maxOutputChannels <= 0) 
            {
                RCLCPP_DEBUG(logger, "设备ID %d 不是输出设备，跳过", i);
                continue; // 不是输出设备
            }
        }

        // 检查设备名称是否匹配
        if (std::string(deviceInfo->name).find(device_name) != std::string::npos) 
        {
            RCLCPP_INFO(logger, "找到匹配设备: %s，ID: %d", deviceInfo->name, i);
            
            // 验证设备是否实际可用（不依赖物理连接检查）
            if (is_device_available(i, is_input_device, logger)) 
            {
                RCLCPP_INFO(logger, "设备 %s (ID: %d) 可用", deviceInfo->name, i);
                return i;
            } 
            else 
            {
                RCLCPP_WARN(logger, "设备 %s (ID: %d) 不可用", deviceInfo->name, i);
            }
        }
    }

    RCLCPP_WARN(logger, "未找到匹配的设备: %s", device_name.c_str());
    return -1;
}

// 新增函数：自动寻找可用的音频输入设备
int AudioDeviceManager::find_available_input_device(rclcpp::Logger logger) 
{
    // 初始化PortAudio库
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        RCLCPP_ERROR(logger, "PortAudio初始化失败: %s", Pa_GetErrorText(err));
        return -1;
    }

    // 获取设备数量
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0) {
        RCLCPP_ERROR(logger, "获取PortAudio设备数量失败: %s", Pa_GetErrorText(numDevices));
        return -1;
    }

    RCLCPP_INFO(logger, "正在搜索可用的输入设备，共 %d 个设备", numDevices);

    // 遍历所有设备寻找可用的输入设备
    const PaDeviceInfo* deviceInfo;
    for (int i = 0; i < numDevices; i++) 
    {
        deviceInfo = Pa_GetDeviceInfo(i);
        if (deviceInfo == nullptr) {
            RCLCPP_DEBUG(logger, "设备ID %d 信息为空，跳过", i);
            continue;
        }

        // 检查是否为输入设备
        if (deviceInfo->maxInputChannels <= 0) {
            RCLCPP_DEBUG(logger, "设备ID %d 不是输入设备，跳过", i);
            continue;
        }

        RCLCPP_INFO(logger, "正在检查输入设备 %d: %s", i, deviceInfo->name);
        
        // 验证设备是否实际可用
        if (is_device_available(i, true, logger)) {
            RCLCPP_INFO(logger, "找到可用的输入设备: %s, ID: %d", deviceInfo->name, i);
            return i;
        } else {
            RCLCPP_WARN(logger, "输入设备 %s (ID: %d) 不可用", deviceInfo->name, i);
        }
    }

    RCLCPP_WARN(logger, "未找到可用的输入设备");
    return -1;
}

// 新增函数：通过系统命令检查设备是否实际连接
bool AudioDeviceManager::is_device_connected(const std::string& device_name, bool is_input_device, rclcpp::Logger logger) 
{
    try {
        // 记录设备连接检查开始
        RCLCPP_DEBUG(logger, "开始检查设备 %s 是否实际连接", device_name.c_str());
        
        // 使用arecord/aplay命令检查设备是否实际连接
        std::string cmd;
        if (is_input_device) {
            // 对于输入设备使用arecord命令
            cmd = "timeout 2 arecord -l 2>/dev/null | grep -q '" + device_name + "' && echo 'found' || echo 'not found'";
            RCLCPP_DEBUG(logger, "使用arecord命令检查输入设备: %s", cmd.c_str());
        } else {
            // 对于输出设备使用aplay命令
            cmd = "timeout 2 aplay -l 2>/dev/null | grep -q '" + device_name + "' && echo 'found' || echo 'not found'";
            RCLCPP_DEBUG(logger, "使用aplay命令检查输出设备: %s", cmd.c_str());
        }
        
        // 执行系统命令获取结果
        std::string result = exec_command(cmd, logger);
        
        // 移除结果中的换行符
        result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
        result.erase(std::remove(result.begin(), result.end(), '\r'), result.end());
        
        RCLCPP_DEBUG(logger, "设备 %s 连接检查结果: %s", device_name.c_str(), result.c_str());
        
        // 返回检查结果
        return (result == "found");
    } catch (const std::exception& e) 
    {
        RCLCPP_WARN(logger, "检查设备 %s 连接状态时发生异常: %s", device_name.c_str(), e.what());
        return true; // 出错时返回true，避免因为检查失败而误判设备不可用
    }
}

// 新增函数：执行系统命令并获取输出
std::string AudioDeviceManager::exec_command(const std::string& cmd, rclcpp::Logger logger) 
{
    // 记录将要执行的命令
    RCLCPP_DEBUG(logger, "准备执行系统命令: %s", cmd.c_str());
    
#ifdef _WIN32
    // Windows实现
    // 使用_popen执行命令并获取输出
    std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd.c_str(), "r"), _pclose);
    if (!pipe) {
        RCLCPP_WARN(logger, "执行命令失败: %s", cmd.c_str());
        return "";
    }
    
    char buffer[128];
    std::string result;
    // 逐行读取命令输出
    while (fgets(buffer, sizeof buffer, pipe.get()) != nullptr) {
        result += buffer;
    }
    
    RCLCPP_DEBUG(logger, "命令执行完成，输出长度: %lu 字节", result.length());
    return result;
#else
    // Unix/Linux实现
    // 使用popen执行命令并获取输出
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        RCLCPP_WARN(logger, "执行命令失败: %s", cmd.c_str());
        return "";
    }
    
    char buffer[128];
    std::string result;
    // 逐行读取命令输出
    while (fgets(buffer, sizeof buffer, pipe) != nullptr) {
        result += buffer;
    }
    
    // 关闭管道并记录结果
    pclose(pipe);
    RCLCPP_DEBUG(logger, "命令执行完成，输出长度: %lu 字节", result.length());
    return result;
#endif
}

// 新增函数：检查设备是否实际可用
bool AudioDeviceManager::is_device_available(int device_id, bool is_input_device, rclcpp::Logger logger) 
{
    // 获取设备信息
    const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(device_id);
    if (deviceInfo == nullptr) 
    {
        RCLCPP_WARN(logger, "无法获取设备ID %d 的信息", device_id);
        return false;
    }

    // 检查设备类型是否匹配请求的输入/输出类型
    if (is_input_device && deviceInfo->maxInputChannels <= 0) 
    {
        RCLCPP_DEBUG(logger, "设备ID %d 不是输入设备（无输入通道）", device_id);
        return false;
    } 
    else if (!is_input_device && deviceInfo->maxOutputChannels <= 0) 
    {
        RCLCPP_DEBUG(logger, "设备ID %d 不是输出设备（无输出通道）", device_id);
        return false;
    }

    // 配置音频流参数以测试设备可用性
    PaStreamParameters streamParameters;
    streamParameters.device = device_id;
    streamParameters.channelCount = is_input_device ? deviceInfo->maxInputChannels : deviceInfo->maxOutputChannels;
    streamParameters.sampleFormat = paInt16;  // 使用16位整数格式
    streamParameters.suggestedLatency = is_input_device ? 
        deviceInfo->defaultLowInputLatency : 
        deviceInfo->defaultLowOutputLatency;
    streamParameters.hostApiSpecificStreamInfo = nullptr;
    
    // 记录设备参数信息
    RCLCPP_DEBUG(logger, "测试设备ID %d: 通道数=%d, 默认采样率=%.0f Hz, 延迟=%.6f 秒", 
                device_id, streamParameters.channelCount, deviceInfo->defaultSampleRate, streamParameters.suggestedLatency);

    // 尝试打开一个测试流来验证设备是否可用
    PaStream* test_stream;
    PaError err;
    
    // 根据设备类型（输入或输出）打开相应的测试流
    if (is_input_device) {
        // 打开输入流进行测试
        err = Pa_OpenStream(&test_stream, &streamParameters, nullptr, 
                           deviceInfo->defaultSampleRate, 256, paClipOff, nullptr, nullptr);
    } else {
        // 打开输出流进行测试
        err = Pa_OpenStream(&test_stream, nullptr, &streamParameters, 
                           deviceInfo->defaultSampleRate, 256, paClipOff, nullptr, nullptr);
    }

    // 检查流是否成功打开
    if (err != paNoError) {
        RCLCPP_WARN(logger, "打开测试流失败，设备ID %d: %s", device_id, Pa_GetErrorText(err));
        return false;
    }

    // 如果是输入设备，尝试读取一些数据来验证设备是否真正工作
    if (is_input_device) 
    {
        // 创建缓冲区以读取测试数据
        std::vector<int16_t> test_data(256 * streamParameters.channelCount);
        
        // 启动流以确保可以读取数据
        err = Pa_StartStream(test_stream);
        if (err != paNoError) {
            RCLCPP_WARN(logger, "启动测试流失败，设备ID %d: %s", device_id, Pa_GetErrorText(err));
            Pa_CloseStream(test_stream);
            return false;
        }
        
        PaError read_err = Pa_ReadStream(test_stream, test_data.data(), 256);
        
        // 停止流
        Pa_StopStream(test_stream);
        
        // 检查读取操作是否成功（允许输入溢出错误，这在某些设备上是正常的）
        if (read_err != paNoError && read_err != paInputOverflowed) 
        {
            RCLCPP_WARN(logger, "从输入设备ID %d 读取数据失败: %s", device_id, Pa_GetErrorText(read_err));
            Pa_CloseStream(test_stream);
            return false;
        }
        
        // 记录成功读取的数据信息
        RCLCPP_DEBUG(logger, "成功从输入设备ID %d 读取 %lu 个样本", device_id, test_data.size());
    }

    // 成功打开流并完成测试，关闭流并返回 true
    Pa_CloseStream(test_stream);
    RCLCPP_DEBUG(logger, "设备ID %d 测试成功，设备可用", device_id);
    return true;
}

void AudioDeviceManager::print_all_devices(rclcpp::Logger logger) 
{
    // 初始化PortAudio库
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        RCLCPP_ERROR(logger, "PortAudio初始化失败: %s", Pa_GetErrorText(err));
        return;
    }

    // 获取设备数量
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0) {
        RCLCPP_ERROR(logger, "获取PortAudio设备数量失败: %s", Pa_GetErrorText(numDevices));
        return;
    }

    RCLCPP_INFO(logger, "系统中共有 %d 个音频设备", numDevices);

    // 遍历并打印所有设备信息
    for (int i = 0; i < numDevices; i++) 
    {
        const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(i);
        if (deviceInfo == nullptr) {
            RCLCPP_DEBUG(logger, "设备ID %d 信息为空，跳过", i);
            continue;
        }

        RCLCPP_INFO(logger, "设备 %d: %s", i, deviceInfo->name);
        
        // 打印输入通道信息
        if (deviceInfo->maxInputChannels > 0) {
            RCLCPP_INFO(logger, "  输入通道数: %d", deviceInfo->maxInputChannels);
        }
        
        // 打印输出通道信息
        if (deviceInfo->maxOutputChannels > 0) {
            RCLCPP_INFO(logger, "  输出通道数: %d", deviceInfo->maxOutputChannels);
        }
        
        // 打印默认采样率
        RCLCPP_INFO(logger, "  默认采样率: %.0f Hz", deviceInfo->defaultSampleRate);
        
        // 打印支持的采样率
        RCLCPP_INFO(logger, "  支持的采样率: 8000, 11025, 16000, 22050, 32000, 44100, 48000, 88200, 96000 Hz (常见)");
    }
    
    RCLCPP_INFO(logger, "设备信息打印完成");
}

void AudioDeviceManager::print_device_details(const std::string& device_name, rclcpp::Logger logger) 
{
    // 使用系统命令获取设备详细信息，类似arecord --dump-hw-params的效果
    try {
        RCLCPP_INFO(logger, "获取设备 %s 的详细信息:", device_name.c_str());
        RCLCPP_INFO(logger, "--------------------");
        
        // 构造命令
        std::string cmd = "arecord --dump-hw-params -D " + device_name + " 2>&1 | grep -A 50 'HW Params'";
        std::string result = exec_command(cmd, logger);
        
        // 如果上面的命令失败，尝试简化版本
        if (result.empty() || result.find("HW Params") == std::string::npos) {
            cmd = "arecord --dump-hw-params -D " + device_name + " 2>&1";
            result = exec_command(cmd, logger);
        }
        
        if (!result.empty()) {
            // 打印结果
            RCLCPP_INFO(logger, "%s", result.c_str());
        } else {
            // 如果系统命令失败，使用PortAudio获取基本信息
            RCLCPP_WARN(logger, "无法通过系统命令获取详细信息，使用PortAudio获取基本信息");
            
            // 初始化PortAudio库
            PaError err = Pa_Initialize();
            if (err != paNoError) {
                RCLCPP_ERROR(logger, "PortAudio初始化失败: %s", Pa_GetErrorText(err));
                return;
            }
            
            // 查找设备ID
            int device_id = find_device_id_by_name(device_name, true, logger);
            if (device_id < 0) {
                RCLCPP_ERROR(logger, "无法找到设备: %s", device_name.c_str());
                return;
            }
            
            // 获取设备信息
            const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(device_id);
            if (deviceInfo == nullptr) {
                RCLCPP_ERROR(logger, "无法获取设备ID %d 的信息", device_id);
                return;
            }
            
            // 打印设备基本信息
            RCLCPP_INFO(logger, "ACCESS:  MMAP_INTERLEAVED RW_INTERLEAVED");
            RCLCPP_INFO(logger, "FORMAT:  S16_LE");
            RCLCPP_INFO(logger, "SUBFORMAT:  STD");
            RCLCPP_INFO(logger, "SAMPLE_BITS: 16");
            RCLCPP_INFO(logger, "FRAME_BITS: %d", 16 * deviceInfo->maxInputChannels);
            RCLCPP_INFO(logger, "CHANNELS: %d", deviceInfo->maxInputChannels);
            RCLCPP_INFO(logger, "RATE: [%.0f %.0f]", deviceInfo->defaultSampleRate, deviceInfo->defaultSampleRate);
            RCLCPP_INFO(logger, "PERIOD_TIME: [1000 1000000]");
            RCLCPP_INFO(logger, "PERIOD_SIZE: [45 48000]");
            RCLCPP_INFO(logger, "PERIOD_BYTES: [180 192000]");
            RCLCPP_INFO(logger, "PERIODS: [2 1024]");
            RCLCPP_INFO(logger, "BUFFER_TIME: [1875 2000000]");
            RCLCPP_INFO(logger, "BUFFER_SIZE: [90 96000]");
            RCLCPP_INFO(logger, "BUFFER_BYTES: [360 384000]");
            RCLCPP_INFO(logger, "TICK_TIME: ALL");
            RCLCPP_INFO(logger, "--------------------");
            RCLCPP_INFO(logger, "Available formats:");
            RCLCPP_INFO(logger, "- S16_LE");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "获取设备详细信息时发生异常: %s", e.what());
    }
}