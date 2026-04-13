#include "nova_robot_ctrl/nova_gripper_ctrl.h"
#include "SCSCL.h"
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <fstream>
#include <thread>
#include <chrono>
#include <dirent.h>
#include <algorithm>
#include <cmath>
// 包含SCSCL类定义
#include "SCSCL.h"

namespace nova_robot_ctrl
{

// 初始化静态成员变量
int NovaGripperCtrl::scan_id_min_ = 0;
int NovaGripperCtrl::scan_id_max_ = 31;

NovaGripperCtrl::NovaGripperCtrl(const std::string& port, int id_name, const std::vector<int>& servo_pos)
    : port_(port), servo_id_(id_name), connected_(false), baudrate_(1000000), command_lock_(),
      scservo_(nullptr)
{
    //百分比
    min_position_ = 0;
    max_position_ = 100;
    min_speed_ = 0;
    max_speed_ = 100;
    min_force_ = 0;
    max_force_ = 100;
    
    //伺服最大位置速度位置控制//舵机支持的最大范围是0-4096
    min_servo_pos_ = servo_pos[0]>=0?servo_pos[0]:0;
    max_servo_pos_ = servo_pos[1]<=4096?servo_pos[1]:4096;
    min_servo_speed_ = 0;
    max_servo_speed_ = 4096;
    min_servo_force_ = 0;
    max_servo_force_ = 100;
    
    scservo_ = new SCSCL(0);
    disable_log_=false; // 测试时关闭日志，默认为false
    //if (!disable_log_) {
    //    LOG_INFO(PROJECT_NAME, __FILE__, __FUNCTION__, __LINE__, "NovaGripperCtrl initialized with port: %s, id: %d, baudrate: %d", port.c_str(), id_name, baudrate_);
    //}
}

bool NovaGripperCtrl::connect()
{
    if (connected_) {
        if (!disable_log_) {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪已连接" << std::endl;
        }
        return true;  // 已经连接
    }
    
    try {
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在连接夹爪，端口: " << port_ << " 波特率: " << baudrate_ << std::endl;
        }
        
        // 检查串口设备是否存在
        if (access(port_.c_str(), F_OK) == -1) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 串口设备不存在: " << port_ << std::endl;
            }
            return false;
        }
        
        // 检查串口设备是否可读写
        if (access(port_.c_str(), R_OK | W_OK) == -1) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 串口无读写权限: " << port_ << std::endl;
            }
            return false;
        }
        
        // 使用 SCServo SDK 初始化
        // 根据SCServo SDK，可能需要单独设置串口和波特率
        
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在初始化SCServo，波特率: " << baudrate_ << " 端口: " << port_ << std::endl;
        }
        //std::cout << "About to call scservo_->begin with baudrate: " << baudrate_ << std::endl;
        if (scservo_->begin(baudrate_, port_.c_str()) != true) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 打开串口失败: " << port_ << " 波特率: " << baudrate_ << std::endl;
            }
            return false;
        }
        
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功打开串口: " << port_ << std::endl;
        }
        
        // 等待一段时间让连接稳定
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 等待连接稳定..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 测试通信
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在尝试ping SCServo，ID: " << servo_id_ << std::endl;
        }
        if (!ping()) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - ping SCServo失败，ID: " << servo_id_ << std::endl;
            }
            // 尝试再次ping
            if (!disable_log_) {
                std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在重试ping..." << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!ping()) {
                if (!disable_log_) {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第二次ping尝试也失败了" << std::endl;
                }
                return false;
            }
        }
        
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功ping通SCServo，ID: " << servo_id_ << std::endl;
        }
        
        set_latency_timer();
        connected_ = true;
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪连接成功" << std::endl;
        }
        return true;
        
    } catch (const std::exception& e) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪连接期间发生异常: " << e.what() << std::endl;
        }
        connected_ = false;
    } catch (...) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪连接期间发生未知异常" << std::endl;
        }
        connected_ = false;
    }
    return false;
}

bool NovaGripperCtrl::is_connected() const
{
    return connected_;
}

NovaGripperCtrl::~NovaGripperCtrl()
{
    try {
        disconnect();
        if (scservo_) {
            delete scservo_;
            scservo_ = nullptr;
        }
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - NovaGripperCtrl已销毁" << std::endl;
        }
    } catch (const std::exception& e) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 析构函数中发生异常: " << e.what() << std::endl;
        }
    } catch (...) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 析构函数中发生未知异常" << std::endl;
        }
    }
}

void NovaGripperCtrl::set_latency_timer()
{
    // Extract port name from full path
    size_t pos = port_.find_last_of("/");
    if (pos != std::string::npos) {
        std::string port_name = port_.substr(pos + 1);
        std::string latency_path = "/sys/bus/usb-serial/devices/" + port_name + "/latency_timer";
        
        // Set latency timer to 1ms for better performance
        std::ofstream latency_file(latency_path);
        if (latency_file.is_open()) {
            latency_file << "1";
            latency_file.close();
            if (!disable_log_) {
                std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已设置端口的延迟计时器: " << port_name << std::endl;
            }
        }
    }
}

bool NovaGripperCtrl::ping()
{
    if (!scservo_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo未初始化" << std::endl;
        }
        return false;
    }
    
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在向舵机ID发送ping: " << servo_id_ << std::endl;
    }
    
    int result = scservo_->Ping(servo_id_);
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - Ping结果: " << result << std::endl;
    }
    
    // SCServo的Ping方法通常返回servo ID表示成功，或者-1表示失败
    if (result != servo_id_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo ping失败，结果: " << result << "，期望: " << servo_id_ << std::endl;
        }
        // 添加更多诊断信息
        if (result == -1) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo ping返回-1，表示设备无响应" << std::endl;
            }
        }
        return false;
    }
    
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo ping成功，结果: " << result << std::endl;
    }
    return true;
}

bool NovaGripperCtrl::set_torque_limit(int limit)
{
    if (!scservo_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo未初始化" << std::endl;
        }
        return false;
    }
    
    // TODO: 需要查阅具体的SCServo API文档来实现扭矩限制设置
    // 暂时返回成功，等待确认正确的API调用
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo扭矩限制已设置 (API待定): " << limit << std::endl;
    }
    return true;
}

bool NovaGripperCtrl::set_port_and_id(const std::string& port, int id)
{
    // 如果已经连接，先断开连接
    if (connected_) {
        disconnect();
    }
    
    // 更新串口路径和ID
    port_ = port;
    servo_id_ = id;
    //connect();
    //ping();
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已更新端口为: " << port_.c_str() << "，ID为: " << servo_id_ << std::endl;
    }
    return true;
}

void NovaGripperCtrl::disconnect()
{
    if (connected_) {
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在断开SCServo夹爪连接" << std::endl;
        }
        
        try {
            if (scservo_) {
                scservo_->end();  // 断开串口连接
            }
        } catch (const std::exception& e) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪断开连接期间发生异常: " << e.what() << std::endl;
            }
        } catch (...) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪断开连接期间发生未知异常" << std::endl;
            }
        }
        
        connected_ = false;
        if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo夹爪已断开连接" << std::endl;
        }
    }
}

int NovaGripperCtrl::get_min_position() const
{
    return min_position_;
}

int NovaGripperCtrl::get_max_position() const
{
    return max_position_;
}

int NovaGripperCtrl::get_open_position() const
{
    return get_max_position();
}

int NovaGripperCtrl::get_closed_position() const
{
    return get_min_position();
}

bool NovaGripperCtrl::is_open() const
{
    if (!connected_) return false;
    try {
        return get_current_position() <= get_open_position();
    } catch (const std::exception&) {
        return false;
    }
}

bool NovaGripperCtrl::is_closed() const
{
    if (!connected_) return false;
    try {
        return get_current_position() >= get_closed_position();
    } catch (const std::exception&) {
        return false;
    }
}

int NovaGripperCtrl::get_current_position() const
{
    if (!connected_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo夹爪未连接" << std::endl;
        }
        throw std::runtime_error("SCServo夹爪未连接");
    }
    
    if (!scservo_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo未初始化" << std::endl;
        }
        throw std::runtime_error("SCServo未初始化");
    }
    
    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在从舵机ID读取位置: " << servo_id_ << std::endl;
    }
    int present_position = scservo_->ReadPos(servo_id_);
    
    if (present_position < 0) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 从SCServo读取当前位置失败，结果: " << present_position << std::endl;
        }
        // 尝试再次读取
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        present_position = scservo_->ReadPos(servo_id_);
        if (present_position < 0) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第二次尝试也失败了，结果: " << present_position << std::endl;
            }
            throw std::runtime_error("Failed to read current position from SCServo");
        }
    }
    
    // 添加调试信息
    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 原始舵机位置: " << present_position << std::endl;
    }
    
    // Convert servo position to lo
    // cal position
    double local_pos = (present_position - min_servo_pos_)*1.0 /  (max_servo_pos_ - min_servo_pos_)* (max_position_ - min_position_);
    
    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 转换后位置: " << local_pos << std::endl;
    }
    return static_cast<int>(std::round(local_pos));
}

int NovaGripperCtrl::get_current_speed() const
{
    if (!connected_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo夹爪未连接" << std::endl;
        }
        throw std::runtime_error("SCServo夹爪未连接");
    }

    if (!scservo_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo未初始化" << std::endl;
        }
        throw std::runtime_error("SCServo未初始化");
    }

    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在从舵机ID读取速度: " << servo_id_ << std::endl;
    }
    int present_speed = scservo_->ReadSpeed(servo_id_);

    if (present_speed < 0) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 从SCServo读取当前速度失败，结果: " << present_speed << std::endl;
        }
        // 尝试再次读取
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        present_speed = scservo_->ReadSpeed(servo_id_);
        if (present_speed < 0) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第二次尝试也失败了，结果: " << present_speed << std::endl;
            }
            throw std::runtime_error("Failed to read current speed from SCServo");
        }
    }

    // Convert servo speed to local speed
    int local_speed = static_cast<int>((static_cast<double>(present_speed - min_servo_speed_) / 
                                      (max_servo_speed_ - min_servo_speed_)) * 
                                     (max_speed_ - min_speed_));

    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 当前速度: " << local_speed << " (原始: " << present_speed << ")" << std::endl;
    }
    return local_speed;
}

std::pair<bool, int> NovaGripperCtrl::move(int position, int speed, int force)
{
    if (!connected_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪未连接" << std::endl;
        }
        return std::make_pair(false, -1);
    }
    
    std::lock_guard<std::mutex> lock(command_lock_);
    
    // Validate parameters
    if (position > max_position_ || position < min_position_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 位置超出范围 [" << min_position_ << ", " << max_position_ << "]，给定值: " << position << std::endl;
        }
        return std::make_pair(false, -2);
    }
    
    if (speed < min_speed_ || speed > max_speed_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 速度超出范围 [" << min_speed_ << ", " << max_speed_ << "]，给定值: " << speed << std::endl;
        }
        return std::make_pair(false, -3);
    }
    
    // Convert to servo coordinates
    int local_pos = static_cast<int>((static_cast<double>(position) / (max_position_ - min_position_)) * 
                                   (max_servo_pos_ - min_servo_pos_) + min_servo_pos_);
    
    int local_speed = static_cast<int>((static_cast<double>(speed) / (max_speed_ - min_speed_)) * 
                                     (max_servo_speed_ - min_servo_speed_) + min_servo_speed_);
    
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在将SCServo移动到位置 " << position << " (舵机位置: " << local_pos << ")，速度 " << speed << " (舵机速度: " << local_speed << ")" << std::endl;
    }
    
    if (!scservo_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo未初始化" << std::endl;
        }
        return std::make_pair(false, -4);
    }
    
    // SCServo 移动命令
    GripperStatus gripper_status=get_status();
    int result = scservo_->WritePos(servo_id_, local_pos, 0, local_speed);
    if(gripper_status.position>-1 && local_speed>0)
    {
       int time_wait= abs(gripper_status.position-local_pos)*1.0*(max_servo_pos_ - min_servo_pos_)/(max_position_ - min_position_)/local_speed *1000+100;//等待时间us
       if (!disable_log_) {
           std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - wait_time: " << time_wait << " curStatePos:" << gripper_status.position << " SetPos:" << local_pos << std::endl;
       }
       usleep(time_wait);
    }
    gripper_status=get_status();
     if (!disable_log_) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪当前运动状态: " << gripper_status.moving << ")" << std::endl;
        }
    if (gripper_status.moving)
    {//计算的等待时间不准，大行程时需要额外的等待时间
        int consecutive_stops = 0;
        int max_iterations = 100;
        int iteration = 0;
        int time_step=50;
        while (iteration < max_iterations) 
        {
            usleep(time_step); // 等待50微秒
            GripperStatus status = get_status();
            if (status.moving == 0) {
                consecutive_stops++;
                time_step=10;//舵机静止后减小第二次反馈间隔
                if (consecutive_stops >= 2) {
                    break; // 连续两次moving为0，退出循环
            }
             } else {
            consecutive_stops = 0; // 重置计数器
            }
            iteration++;
        }
    }
    
    if (result != 1) {  // SCServo 成功返回 1
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪移动失败，结果: " << result << ")" << std::endl;
        }
        // 尝试再次发送命令
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        result = scservo_->WritePos(servo_id_, position, 0, local_speed);
        if (result != 1) {
            if (!disable_log_) {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第二次尝试也失败了，结果: " << result << std::endl;
            }
            return std::make_pair(false, result);
        }
    }
    
    gripper_status=get_status();
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 当前位置：" << gripper_status.position << " 当前速度：" << gripper_status.speed << " 当前负载：" << gripper_status.load << " 当前电压：" << gripper_status.voltage << " 当前温度：" << gripper_status.temperature << " 是否移动：" << gripper_status.moving << " 当前电流：" << gripper_status.current << std::endl;
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 夹爪已成功移动到位置 " << position << "，速度 " << speed << std::endl;
    }
    //scservo_->WritePos(servo_id_, 4000, 0, 1500);
    //usleep(754*1000);
    return std::make_pair(true, 0);
}

GripperStatus NovaGripperCtrl::get_status() const
{
    GripperStatus status;

    status.position = -1;
    status.speed = -1;
    status.load = -1;
    status.voltage = -1;
    status.temperature = -1;
    status.moving = -1;
    status.current = -1;

    if (!connected_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo夹爪未连接" << std::endl;
        }
        return status;
    }
    
    if (!scservo_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo未初始化" << std::endl;
        }
        return status;
    }
    
    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在从舵机ID读取状态: " << servo_id_ << std::endl;
    }
    
    // 使用FeedBack方法一次性读取所有数据
    if (scservo_->FeedBack(servo_id_) != -1) {
        #if 0
        status.position = scservo_->ReadPos(-1);
        status.speed = scservo_->ReadSpeed(-1);
        status.load = scservo_->ReadLoad(-1);
        status.voltage = scservo_->ReadVoltage(-1);
        status.temperature = scservo_->ReadTemper(-1);
        status.moving = scservo_->ReadMove(-1);
        status.current = scservo_->ReadCurrent(-1);
        #else
        status.position = scservo_->ReadPos(servo_id_);
        status.speed = scservo_->ReadSpeed(servo_id_);
        status.load = scservo_->ReadLoad(servo_id_);
        status.voltage = scservo_->ReadVoltage(servo_id_);
        status.temperature = scservo_->ReadTemper(servo_id_);
        status.moving = scservo_->ReadMove(servo_id_);
        status.current = scservo_->ReadCurrent(servo_id_);
        #endif

    } else {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 从SCServo读取状态失败" << std::endl;
        }
    }
    
    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 状态 - 位置: " << status.position << ", 速度: " << status.speed << ", 负载: " << status.load << ", 电压: " << status.voltage << ", 温度: " << status.temperature << ", 移动: " << status.moving << ", 电流: " << status.current << std::endl;
    }
    
    return status;
}

std::pair<bool, int> NovaGripperCtrl::reset_position(int speed, int force)
{
    return move(0, speed, force);
}

std::pair<bool, int> NovaGripperCtrl::move_by_abs_pos(int position, int speed, int force)
{
    if (!connected_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo夹爪未连接" << std::endl;
        }
        return std::make_pair(false, -1);
    }
    
    if (!scservo_) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - SCServo未初始化" << std::endl;
        }
        return std::make_pair(false, -1);
    }
    
    if (!disable_log_) {
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在将绝对位置: " << position << " 速度: " << speed << " 力度: " << force << " 写入舵机ID: " << servo_id_ << std::endl;
    }
    
    // 锁定命令执行
    std::lock_guard<std::mutex> lock(command_lock_);
    
    // 使用WritePos写入位置，直接使用传入的速度和力值
    int result = scservo_->WritePos(servo_id_, position, force, speed);
    
    // 等待一小段时间
    usleep(2*1000);
    
    if (result != 1) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 写入绝对位置 " << position << " 失败，结果: " << result << std::endl;
        }
        return std::make_pair(false, result);
    }
    
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功写入绝对位置: " << position << std::endl;
    }
    
    return std::make_pair(true, 0);
}

std::vector<std::map<std::string, int>> NovaGripperCtrl::scan_usb_ports_and_servos() {
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 开始扫描USB端口和舵机" << std::endl;
    
    std::vector<std::map<std::string, int>> result;
    std::vector<std::string> usb_ports = get_available_usb_ports();
    
    // 遍历每个端口，尝试连接并查找舵机
    for (const auto& port : usb_ports) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在扫描端口: " << port << std::endl;
        std::map<std::string, int> port_servos;
#if 0
        // 遍历每个端口，尝试连接并查找舵机
        for (int id = scan_id_min_; id <= scan_id_max_; ++id) { // 使用静态成员变量设置扫描范围
            NovaGripperCtrl temp_gripper(port, id, {0, 4096}); // ID从scan_id_min_开始测试
            temp_gripper.set_disable_log(true); // 要用临时实例的日志输出
            temp_gripper.set_port_and_id(port, id);
            temp_gripper.disconnect();
        }
#else
        // 使用get_connect_id接口实现ID扫描
        NovaGripperCtrl temp_gripper(port, 1, {0, 4096}); // 创建临时实例用于扫描
        temp_gripper.set_disable_log(true); // 禁用日志输出
        
        // 调用get_connect_id接口扫描指定ID范围内的连接舵机
        std::vector<int> connected_ids = temp_gripper.get_connect_id(scan_id_min_, scan_id_max_);
        
        // 处理扫描结果
        for (int id : connected_ids) {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 在端口 " << port << " 上发现ID为 " << id << " 的连接舵机" << std::endl;
            port_servos[port] = id;
        }
#endif
        // 如果在此端口找到了舵机，将其添加到结果中
        if (!port_servos.empty()) {
            result.push_back(port_servos);
        }
    }
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - USB端口和舵机扫描完成" << std::endl;
    return result;
}

std::vector<int> NovaGripperCtrl::get_connect_id(int scan_min, int scan_max) {
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 开始扫描指定ID范围内的连接舵机" << std::endl;
    
    std::vector<int> connected_ids;
    
    // 初始化串口连接
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在连接夹爪，端口: " << port_ << " 波特率: " << baudrate_ << std::endl;
    }
    
    // 检查串口设备是否存在
    if (access(port_.c_str(), F_OK) == -1) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 串口设备不存在: " << port_ << std::endl;
        }
        return connected_ids; // 返回空的结果
    }
    
    // 检查串口设备是否可读写
    if (access(port_.c_str(), R_OK | W_OK) == -1) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 串口无读写权限: " << port_ << std::endl;
        }
        return connected_ids; // 返回空的结果
    }
    
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在初始化SCServo，波特率: " << baudrate_ << " 端口: " << port_ << std::endl;
    }
    if (scservo_->begin(baudrate_, port_.c_str()) != true) {
        if (!disable_log_) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 打开串口失败: " << port_ << " 波特率: " << baudrate_ << std::endl;
        }
        return connected_ids; // 返回空的结果
    }
    
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功打开串口: " << port_ << std::endl;
    }
    
    // 等待一段时间让连接稳定
    if (!disable_log_) {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 等待连接稳定..." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 遍历指定ID范围内的每个ID
    for (int id = scan_min; id <= scan_max; ++id) 
    {
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在ping SCServo，ID: " << id << std::endl;
        servo_id_ = id;
        if (ping())
        {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 成功ping通SCServo，ID: " << servo_id_ << std::endl;
            connected_ids.push_back(id);
        }
    }
    std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 指定ID范围内的连接舵机扫描完成" << std::endl;
    return connected_ids;
}

// 静态成员函数实现
std::vector<std::string> NovaGripperCtrl::get_available_usb_ports() {
    std::vector<std::string> usb_ports;
    DIR* dir = opendir("/sys/class/tty/");
    if (!dir) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无法打开 /sys/class/tty/ 目录" << std::endl;
        return usb_ports;
    }    
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string tty_name = entry->d_name;
        // 筛选出可能的USB串口设备 (ttyUSB*, ttyACM*)
        if (tty_name.find("ttyUSB") == 0 || tty_name.find("ttyACM") == 0) {
            std::string device_path = "/dev/" + tty_name;
            usb_ports.push_back(device_path);
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 发现潜在的USB串口: " << device_path << std::endl;
        }
    }
    closedir(dir);
    
    if (usb_ports.empty()) {
        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 未找到USB串口" << std::endl;
        return usb_ports; // 返回空的结果
    }    
    
    // 对端口进行排序，确保扫描顺序一致
    std::sort(usb_ports.begin(), usb_ports.end());
    
    return usb_ports;
}

void NovaGripperCtrl::set_disable_log(bool disable)
{
    disable_log_ = disable;
}

bool NovaGripperCtrl::get_disable_log() const
{
    return disable_log_;
}

} // namespace nova_robot_ctrl