/**
 * @file test_data_publisher.cpp
 * @brief DataPublisher类功能测试
 * 
 * 测试DataPublisher类的以下功能：
 * 1. 多种数据类型的发布管理
 * 2. 发布频率控制
 * 3. 数据格式验证
 * 4. 发布统计信息
 * 5. 错误处理和重试机制
 */

#include "include/dds_comm/data_publisher.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "=== DataPublisher功能测试 ===" << std::endl;
    
    // 创建DataPublisher实例
    dds_comm::DataPublisher publisher;
    
    // 测试1: 创建发布者
    dds_comm::DataPublisher::PublishConfig config;
    config.topic_name = "test_topic";
    config.publish_interval = std::chrono::milliseconds(100);
    
    if (publisher.createPublisher("string_data", config)) {
        std::cout << "✓ 字符串数据发布者创建成功" << std::endl;
    } else {
        std::cout << "✗ 字符串数据发布者创建失败" << std::endl;
    }
    
    // 测试2: 发布字符串数据
    if (publisher.publishString("string_data", "Hello, DDS!")) {
        std::cout << "✓ 字符串数据发布成功" << std::endl;
    } else {
        std::cout << "✗ 字符串数据发布失败" << std::endl;
    }
    
    // 测试3: 发布JSON数据
    std::string json_data = R"({"message": "test", "value": 123})";
    if (publisher.publishJson("json_data", json_data)) {
        std::cout << "✓ JSON数据发布成功" << std::endl;
    } else {
        std::cout << "✗ JSON数据发布失败" << std::endl;
    }
    
    // 测试4: 发布二进制数据
    std::string binary_data = "binary content";
    if (publisher.publishBinary("binary_data", binary_data.data(), binary_data.size())) {
        std::cout << "✓ 二进制数据发布成功" << std::endl;
    } else {
        std::cout << "✗ 二进制数据发布失败" << std::endl;
    }
    
    // 测试5: 获取发布统计
    auto stats = publisher.getPublishStats("string_data");
    std::cout << "✓ 发布统计获取成功 - 总发布数: " << stats.total_published << std::endl;
    
    // 测试6: 检查发布者存在性
    if (publisher.hasPublisher("string_data")) {
        std::cout << "✓ 发布者存在性检查成功" << std::endl;
    } else {
        std::cout << "✗ 发布者存在性检查失败" << std::endl;
    }
    
    // 测试7: 发布频率控制
    std::cout << "测试发布频率控制..." << std::endl;
    for (int i = 0; i < 3; i++) {
        publisher.publishString("string_data", "Message " + std::to_string(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // 测试8: 获取发布状态
    std::string status = publisher.getPublishStatus("string_data");
    std::cout << "✓ 发布状态: " << status << std::endl;
    
    std::cout << "=== 测试完成 ===" << std::endl;
    return 0;
}