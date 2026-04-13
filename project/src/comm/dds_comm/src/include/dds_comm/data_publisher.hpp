#ifndef DDS_COMMUNICATION__DATA_PUBLISHER_HPP_
#define DDS_COMMUNICATION__DATA_PUBLISHER_HPP_

#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include "command_types.hpp"

namespace dds_comm {

/**
 * @class DataPublisher
 * @brief 数据发布管理器 - DDS通讯核心组件
 * 
 * 功能概述：
 * 1. 多种数据类型的发布管理 - 支持字符串、JSON、二进制和自定义数据类型
 * 2. 发布频率控制 - 可配置发布间隔，防止发布过快
 * 3. 数据格式验证 - JSON格式自动验证，二进制数据大小检查
 * 4. 发布统计信息 - 实时统计发布成功率、速率、错误信息
 * 5. 错误处理和重试机制 - 带指数退避的重试策略，最大重试次数可配置
 * 
 * 耦合关系：
 * - 依赖log_system进行日志记录
 * - 与ProjectManager协同工作，提供数据发布服务
 * - 使用nlohmann/json库进行JSON数据处理
 * 
 * 使用说明：
 * 1. 创建DataPublisher实例
 * 2. 使用createPublisher创建特定数据类型的发布者
 * 3. 调用相应的publish方法发布数据
 * 4. 通过getPublishStats获取发布统计信息
 * 
 * 线程安全：
 * - 使用std::mutex确保多线程环境下的线程安全
 * - 所有公共方法都进行了线程安全保护
 * 
 * 性能特点：
 * - 发布间隔控制避免系统过载
 * - 统计信息实时更新，内存占用可控
 * - 重试机制保证数据可靠性
 */

class DataPublisher {
public:
    /**
     * @brief 发布统计信息
     */
    struct PublishStats {
        std::chrono::system_clock::time_point start_time;   ///< 开始时间
        uint64_t total_published;                          ///< 总发布数量
        uint64_t successful_published;                     ///< 成功发布数量
        uint64_t failed_published;                         ///< 失败发布数量
        double average_publish_rate;                       ///< 平均发布速率
        std::chrono::milliseconds last_publish_duration;   ///< 最后发布耗时
        std::string last_error;                           ///< 最后错误信息
    };
    
    /**
     * @brief 发布配置
     */
    struct PublishConfig {
        std::string topic_name;                            ///< 话题名称
        int qos_profile;                                   ///< QoS配置
        double publish_rate;                               ///< 发布速率(Hz)
        std::chrono::milliseconds publish_interval;       ///< 发布间隔(毫秒)
        bool enable_stats;                                 ///< 是否启用统计
        int max_retries;                                   ///< 最大重试次数
        
        /**
         * @brief 默认构造函数，设置默认值
         */
        PublishConfig() 
            : qos_profile(10),
              publish_rate(10.0),
              publish_interval(std::chrono::milliseconds(100)),
              enable_stats(true),
              max_retries(3) {}
    };
    
    /**
     * @brief 构造函数
     * @param data_type 数据类型
     * @param config 发布配置
     */
    DataPublisher(const std::string& data_type = "", const PublishConfig& config = PublishConfig());
    
    /**
     * @brief 默认构造函数
     */
    DataPublisher();
    
    /**
     * @brief 析构函数
     */
    ~DataPublisher();
    
    /**
     * @brief 创建发布者
     * @param data_type 数据类型
     * @param config 发布配置
     * @return bool 创建是否成功
     */
    bool createPublisher(const std::string& data_type, const PublishConfig& config);
    
    /**
     * @brief 发布字符串数据
     * @param data_type 数据类型
     * @param data 字符串数据
     * @return bool 发布是否成功
     */
    bool publishString(const std::string& data_type, const std::string& data);
    
    /**
     * @brief 发布JSON数据
     * @param data_type 数据类型
     * @param json_data JSON数据
     * @return bool 发布是否成功
     */
    bool publishJson(const std::string& data_type, const std::string& json_data);
    
    /**
     * @brief 发布二进制数据
     * @param data_type 数据类型
     * @param data 二进制数据指针
     * @param size 数据大小
     * @return bool 发布是否成功
     */
    bool publishBinary(const std::string& data_type, const void* data, size_t size);
    
    /**
     * @brief 发布自定义数据
     * @param data_type 数据类型
     * @param data 自定义数据
     * @return bool 发布是否成功
     */
    template<typename T>
    bool publishCustom(const std::string& data_type, const T& data);
    
    /**
     * @brief 设置发布间隔
     * @param data_type 数据类型
     * @param interval 发布间隔
     * @return bool 设置是否成功
     */
    bool setPublishInterval(const std::string& data_type, 
                           std::chrono::milliseconds interval);
    
    /**
     * @brief 获取发布统计信息
     * @param data_type 数据类型
     * @return PublishStats 发布统计
     */
    PublishStats getPublishStats(const std::string& data_type) const;
    
    /**
     * @brief 获取所有发布统计
     * @return std::map<std::string, PublishStats> 所有统计
     */
    std::map<std::string, PublishStats> getAllPublishStats() const;
    
    /**
     * @brief 重置发布统计
     * @param data_type 数据类型
     * @return bool 重置是否成功
     */
    bool resetPublishStats(const std::string& data_type);
    
    /**
     * @brief 检查发布者是否存在
     * @param data_type 数据类型
     * @return bool 是否存在
     */
    bool hasPublisher(const std::string& data_type) const;
    
    /**
     * @brief 删除发布者
     * @param data_type 数据类型
     * @return bool 删除是否成功
     */
    bool removePublisher(const std::string& data_type);
    
    /**
     * @brief 获取发布状态
     * @param data_type 数据类型
     * @return std::string 发布状态
     */
    std::string getPublishStatus(const std::string& data_type) const;

private:
    /**
     * @brief 发布者信息
     */
    struct PublisherInfo {
        void* publisher;                                   ///< 发布者指针
        PublishConfig config;
        PublishStats stats;
        std::chrono::system_clock::time_point last_publish_time;
    };
    
    /**
     * @brief 更新发布统计
     * @param data_type 数据类型
     * @param success 是否成功
     * @param duration 发布耗时
     */
    void updatePublishStats(const std::string& data_type, bool success,
                           std::chrono::milliseconds duration);
    
    /**
     * @brief 检查发布间隔
     * @param data_type 数据类型
     * @return bool 是否可以发布
     */
    bool checkPublishInterval(const std::string& data_type) const;
    
    /**
     * @brief 序列化数据
     * @param data 原始数据
     * @return std::string 序列化数据
     */
    std::string serializeData(const std::string& data) const;
    
    void* node_;                                          ///< 节点指针
    mutable std::mutex mutex_;                           ///< 线程安全锁
    std::map<std::string, PublisherInfo> publishers_;    ///< 发布者映射
};

} // namespace dds_comm

#endif // DDS_COMMUNICATION__DATA_PUBLISHER_HPP_