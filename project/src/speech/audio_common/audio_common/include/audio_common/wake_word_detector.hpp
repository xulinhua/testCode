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

#ifndef AUDIO_COMMON__WAKE_WORD_DETECTOR_HPP
#define AUDIO_COMMON__WAKE_WORD_DETECTOR_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

// Python头文件
#include <Python.h>
#include <functional>

namespace audio_common {

enum WakeStatus
{
    WakeStatus_On = 0,     //已唤醒
    WakeStatus_Off = 1,    //没被唤醒
};

typedef std::function<void()> wake_status_callback;

class WakeWordDetector : public rclcpp::Node{
public:
    //using SharedPtr = std::shared_ptr<WakeWordDetector>;
    /**
     * @brief 构造函数
     * @param node ROS节点指针，用于参数获取和日志记录
     */
    explicit WakeWordDetector(/*rclcpp::Node::SharedPtr node*/);
    
    /**
     * @brief 析构函数
     */
    ~WakeWordDetector();
    
    /**
     * @brief 初始化检测器
     * @return true表示初始化成功，false表示失败
     */
    bool initialize();
    
    /**
     * @brief 检测音频数据中是否包含唤醒词
     * @param audio_data 音频数据
     * @param sample_rate 采样率
     * @param channels 声道数
     * @return true表示检测到唤醒词，false表示未检测到
     */
    bool detect(const std::vector<int16_t>& audio_data, int sample_rate, int channels, float sensitivity = 0.5);
    
    /**
     * @brief 检查唤醒词检测功能是否启用
     * @return true表示启用，false表示禁用
     */
    bool is_enabled() const;

    //唤醒状态
    bool is_wake_status_on();
    void set_wake_status(WakeStatus ststus);

    //唤醒记时
    void updata_wake_time();
    void wake_time_callback();
    void set_wake_status_callback(wake_status_callback wakestatuscallback);
private:
    //rclcpp::Node::SharedPtr node_;
    bool enabled_;
    std::string model_path_;
    std::string resource_path_;
    float sensitivity_;
    
    // Python相关的成员变量
    PyObject* pModule_;
    PyObject* pClass_;
    PyObject* pInstance_;

    // 唤醒状态
    WakeStatus wake_status_;
    rclcpp::TimerBase::SharedPtr wake_timer_;
    wake_status_callback wakestatuscallback_;
    
    /**
     * @brief 初始化Python环境和snowboy检测器
     * @return true表示初始化成功，false表示失败
     */
    bool initialize_detector();
    
    /**
     * @brief 清理Python资源
     */
    void cleanup();
};

} // namespace audio_common

#endif // AUDIO_COMMON__WAKE_WORD_DETECTOR_HPP