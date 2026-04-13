#ifndef AUDIO_COMMON__AUDIO_DEVICE_MANAGER_HPP
#define AUDIO_COMMON__AUDIO_DEVICE_MANAGER_HPP

#include <portaudio.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <custom_msgs_comm/msg/audio_device_status.hpp>
#include <memory>

// 添加符号导出宏
#ifdef _WIN32
  #ifdef AUDIO_COMMON_LIB_EXPORT
    #define AUDIO_COMMON_LIB_PUBLIC __declspec(dllexport)
  #else
    #define AUDIO_COMMON_LIB_PUBLIC __declspec(dllimport)
  #endif
#else
  #define AUDIO_COMMON_LIB_PUBLIC __attribute__((visibility("default")))
#endif

namespace audio_common {

class AUDIO_COMMON_LIB_PUBLIC AudioDeviceManager {
public:
    // 音频设备状态枚举
    enum AudioDeviceStatus {
        IDLE_STATUS = custom_msgs_comm::msg::AudioDeviceStatus::IDLE_STATUS,        // 空闲状态
        RECORDING_STATUS = custom_msgs_comm::msg::AudioDeviceStatus::RECORDING_STATUS,   // 录音状态
        PLAYING_STATUS = custom_msgs_comm::msg::AudioDeviceStatus::PLAYING_STATUS      // 播放状态
    };

    /**
     * @brief 获取音频设备管理器实例（单例模式）
     * @return AudioDeviceManager实例的共享指针
     */
    static std::shared_ptr<AudioDeviceManager> getInstance();

    /**
     * @brief 根据设备名称查找设备ID
     * @param device_name 设备名称，如"hw:2,0"
     * @param is_input_device 是否为输入设备（麦克风）
     * @return 设备ID，如果未找到返回-1
     */
    static int find_device_id_by_name(const std::string& device_name, bool is_input_device, rclcpp::Logger logger);

    /**
     * @brief 自动寻找可用的音频输入设备
     * @return 设备ID，如果未找到返回-1
     */
    static int find_available_input_device(rclcpp::Logger logger);

    /**
     * @brief 打印所有音频设备信息
     */
    static void print_all_devices(rclcpp::Logger logger);
    
    /**
     * @brief 获取设备详细信息，类似arecord --dump-hw-params命令的输出
     * @param device_name 设备名称，如"hw:0,0"
     * @param logger 日志记录器
     */
    static void print_device_details(const std::string& device_name, rclcpp::Logger logger);

    /**
     * @brief 通过系统命令检查设备是否实际连接
     * @param device_name 设备名称，如"hw:2,0"
     * @param is_input_device 是否为输入设备（麦克风）
     * @return 如果设备连接返回true，否则返回false
     */
    static bool is_device_connected(const std::string& device_name, bool is_input_device, rclcpp::Logger logger);

    /**
     * @brief 获取音频设备当前状态
     * @return 当前设备状态
     */
    uint8_t get_audio_device_status();

    /**
     * @brief 设置音频设备状态
     * @param status 要设置的状态
     * @param node ROS 2节点指针（用于初始化发布器）
     */
    void set_audio_device_status(uint8_t status, rclcpp::Node::SharedPtr node = nullptr);

private:
    /**
     * @brief 检查设备是否实际可用
     * @param device_id 设备ID
     * @param is_input_device 是否为输入设备（麦克风）
     * @return 如果设备可用返回true，否则返回false
     */
    static bool is_device_available(int device_id, bool is_input_device, rclcpp::Logger logger);
    
    /**
     * @brief 执行系统命令并获取输出
     * @param cmd 要执行的命令
     * @return 命令输出结果
     */
    static std::string exec_command(const std::string& cmd, rclcpp::Logger logger);
    
    /**
     * @brief 从设备名称中提取hw:x,y部分
     * @param device_name 设备名称
     * @return hw:x,y格式的设备名称，如果未找到返回空字符串
     */
    static std::string extract_hw_name(const std::string& device_name, rclcpp::Logger logger);

    /**
     * @brief 发布音频设备状态
     * @param status 要发布的状态
     */
    void publish_audio_device_status(uint8_t status);

    // 私有构造函数（单例模式）
    AudioDeviceManager();

    // 删除拷贝构造函数和赋值操作符，确保单例唯一性
    AudioDeviceManager(const AudioDeviceManager&) = delete;
    AudioDeviceManager& operator=(const AudioDeviceManager&) = delete;

    // 静态变量存储实例
    static std::shared_ptr<AudioDeviceManager> instance_;
    
    // 静态变量存储设备状态（所有实例共享）
    static uint8_t audio_device_status_;
    
    // 音频设备状态发布器
    rclcpp::Publisher<custom_msgs_comm::msg::AudioDeviceStatus>::SharedPtr audio_status_pub_;
    
    // Logger for logging messages
    rclcpp::Logger logger_;
};

} // namespace audio_common

#endif // AUDIO_COMMON__AUDIO_DEVICE_MANAGER_HPP