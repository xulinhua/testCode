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

#include "audio_common/wake_word_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>

using namespace audio_common;

WakeWordDetector::WakeWordDetector() 
    : Node("wake_word"), enabled_(true), sensitivity_(0.5), 
      pModule_(nullptr), pClass_(nullptr), pInstance_(nullptr), wakestatuscallback_(nullptr) {
    
    // 声明参数
    this->declare_parameter("enabled", true);
    this->declare_parameter("model_path", "");
    this->declare_parameter("resource_path", "");
    this->declare_parameter("sensitivity", 0.5);
    
    // 获取参数
    this->get_parameter("enabled", enabled_);
    if (this->has_parameter("wake_word.model_path")) {
      // 尝试从嵌套参数读取
      std::string model_path = this->get_parameter("wake_word.model_path").as_string();
      if (!model_path.empty()) {
        this->model_path_ = model_path;
        RCLCPP_INFO(this->get_logger(), "从配置文件读取模板路径: %s", this->model_path_.c_str());
      }
    } else if (this->has_parameter("model_path") && !this->get_parameter("model_path").as_string().empty()) {
      this->model_path_ = this->get_parameter("model_path").as_string();
      RCLCPP_INFO(this->get_logger(), "从配置文件读取模板路径: %s", this->model_path_.c_str());
    }

    //this->get_parameter("model_path", model_path_);
    this->get_parameter("resource_path", resource_path_);
    this->get_parameter("sensitivity", sensitivity_);
    
    // 如果没有设置路径，则使用默认路径
    // 使用相对于工作目录的正确路径
    if (model_path_.empty()) {
        RCLCPP_INFO(this->get_logger(), "配置文件加载唤醒模型失败，使用其他路径");
        model_path_ = "src/speech/snowboy/src/resources/models/snowboy.pmdl";
    }
    else
        RCLCPP_INFO(this->get_logger(), "配置文件加载唤醒模型路径成功！");
    
    if (resource_path_.empty()) {
        RCLCPP_INFO(this->get_logger(), "配置文件加载唤醒模型失败，使用其他路径");
        resource_path_ = "src/speech/snowboy/src/resources/common.res";
    }
    wake_status_ = WakeStatus::WakeStatus_Off;

    // 注意：不在构造函数中初始化Python检测器，避免使用shared_from_this()
}

WakeWordDetector::~WakeWordDetector() {
    cleanup();
}

bool WakeWordDetector::initialize() {
    // 初始化Python检测器
    if (enabled_) {
        if (!initialize_detector()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize wake word detector");
            enabled_ = false;
            return false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Wake word detector initialized successfully");
            return true;
        }
    }
    return true;
}

bool WakeWordDetector::initialize_detector() {
    // 初始化Python解释器
    Py_Initialize();
    
    if (!Py_IsInitialized()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Python interpreter");
        return false;
    }
    
    // 添加当前目录到Python路径
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('../../snowboy/src/snowboy_python')");
    
    try {
        // 导入snowboy_python模块
        pModule_ = PyImport_ImportModule("snowboy_python.snowboy_detector_wrapper");
        if (!pModule_) {
            PyErr_Print();
            RCLCPP_ERROR(this->get_logger(), "Failed to import snowboy_detector_wrapper module");
            return false;
        }
        
        // 获取SnowboyDetectorWrapper类
        pClass_ = PyObject_GetAttrString(pModule_, "SnowboyDetectorWrapper");
        if (!pClass_) {
            PyErr_Print();
            RCLCPP_ERROR(this->get_logger(), "Failed to get SnowboyDetectorWrapper class");
            return false;
        }
        
        // 创建SnowboyDetectorWrapper实例
        PyObject* pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(model_path_.c_str()));
        PyTuple_SetItem(pArgs, 1, PyUnicode_FromString(resource_path_.c_str()));
        PyTuple_SetItem(pArgs, 2, PyFloat_FromDouble(sensitivity_));
        
        pInstance_ = PyObject_CallObject(pClass_, pArgs);
        Py_DECREF(pArgs);
        
        if (!pInstance_) {
            PyErr_Print();
            RCLCPP_ERROR(this->get_logger(), "Failed to create SnowboyDetectorWrapper instance");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully initialized snowboy wake word detector");
        RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Resource path: %s", resource_path_.c_str());
        
        return true;
    } catch (...) {
        PyErr_Print();
        RCLCPP_ERROR(this->get_logger(), "Exception occurred while initializing wake word detector");
        return false;
    }
}

bool WakeWordDetector::detect(const std::vector<int16_t>& audio_data, int sample_rate, int channels, float sensitivity) {
    if (!enabled_ || !pInstance_) {
        return false;
    }
    
    try {
        // 准备音频数据
        PyObject* pAudioData = PyBytes_FromStringAndSize(
            reinterpret_cast<const char*>(audio_data.data()), 
            audio_data.size() * sizeof(int16_t)
        );
        
        if (!pAudioData) {
            PyErr_Print();
            RCLCPP_ERROR(this->get_logger(), "Failed to create audio data bytes");
            return false;
        }
        
        // 调用detect方法
        PyObject* pFunc = PyObject_GetAttrString(pInstance_, "detect");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            PyErr_Print();
            RCLCPP_ERROR(this->get_logger(), "Failed to get detect method");
            Py_DECREF(pAudioData);
            return false;
        }
        
        // 准备参数
        PyObject* pArgs = PyTuple_New(4);
        PyTuple_SetItem(pArgs, 0, pAudioData);
        PyTuple_SetItem(pArgs, 1, PyLong_FromLong(sample_rate));
        PyTuple_SetItem(pArgs, 2, PyLong_FromLong(channels));
        PyTuple_SetItem(pArgs, 3, PyFloat_FromDouble(sensitivity_));
        
        // 调用方法
        PyObject* pResult = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pArgs);
        
        if (!pResult) {
            PyErr_Print();
            RCLCPP_ERROR(this->get_logger(), "Failed to call detect method");
            return false;
        }
        
        // 获取结果
        bool result = PyObject_IsTrue(pResult);
        Py_DECREF(pResult);
        
        if (result)
        {
            RCLCPP_INFO(this->get_logger(), "系统已被唤醒!");
            printf("系统已被唤醒!");
            wake_status_ = WakeStatus_On;
            updata_wake_time();
        }   

        return result;
    } catch (...) {
        PyErr_Print();
        RCLCPP_ERROR(this->get_logger(), "Exception occurred while detecting wake word");
        return false;
    }
}

bool WakeWordDetector::is_enabled() const {
    return enabled_;
}

    //唤醒状态
bool WakeWordDetector::is_wake_status_on()
{
    bool bRes = (wake_status_ == WakeStatus_On);
    return bRes;
}
void WakeWordDetector::set_wake_status(WakeStatus ststus)
{
    wake_status_ = ststus;
}

void WakeWordDetector::updata_wake_time()
{
    RCLCPP_INFO(this->get_logger(), "刷新唤醒记时，重新开始计时！");
    // 关键：取消之前的定时器
    if (wake_timer_) {
        wake_timer_->cancel();
        wake_timer_.reset();
    }
    wake_timer_ = this->create_wall_timer(
        std::chrono::seconds(20),
        std::bind(&WakeWordDetector::wake_time_callback, this)
    );
}

void WakeWordDetector::wake_time_callback()
{
    // 计时结束，唤醒状态结束，可以重新唤醒
    wake_status_ = WakeStatus_Off;
    RCLCPP_INFO(this->get_logger(), "计时结束，唤醒状态结束，可以重新唤醒！");
    wake_timer_->cancel();
    if (wakestatuscallback_) 
    {
        wakestatuscallback_();  // 调用 std::function
    }
}

void WakeWordDetector::set_wake_status_callback(wake_status_callback wakestatuscallback)
{
    wakestatuscallback_ = wakestatuscallback;
}

void WakeWordDetector::cleanup() {
    if (pInstance_) {
        Py_DECREF(pInstance_);
        pInstance_ = nullptr;
    }
    
    if (pClass_) {
        Py_DECREF(pClass_);
        pClass_ = nullptr;
    }
    
    if (pModule_) {
        Py_DECREF(pModule_);
        pModule_ = nullptr;
    }
    
    if (Py_IsInitialized()) {
        Py_Finalize();
    }
}