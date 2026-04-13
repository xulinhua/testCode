#pragma once

#include "task_factory.h"
#include <string>
#include <vector>
#include <memory>
#include "bas_operate/file_operate.hpp"

// 算法库配置结构
struct AlgorithmLibraryConfig {
    std::string name;                        // 算法名称
    std::string library;                     // 库文件名
    bool enabled = true;                     // 是否启用
    int priority = 0;                        // 加载优先级
    std::string description;                 // 描述
};

// 算法加载器 - 自动加载所有已安装的算法库
// 负责发现和加载算法实现库，触发静态注册
class AlgorithmLoader {
public:
    AlgorithmLoader();
    ~AlgorithmLoader();

    // 加载所有已安装的算法库
    void loadAllAlgorithms();

    // 从配置文件加载算法库
    bool loadFromConfig(const std::string& config_path);

    // 自动发现算法库（备份机制）
    void autoDiscoverAlgorithms();

    // 从指定路径加载算法库
    void loadAlgorithmsFromPath(const std::string& library_path);

    // 获取已加载的算法库列表
    std::vector<std::string> getLoadedLibraries() const;

    // 单例模式访问
    static AlgorithmLoader& getInstance();

private:
    // 禁止拷贝
    AlgorithmLoader(const AlgorithmLoader&) = delete;
    AlgorithmLoader& operator=(const AlgorithmLoader&) = delete;

    // 私有实现
    struct Impl;
    std::unique_ptr<Impl> pImpl_;

    // 日志项目路径
    std::string log_path_;

    // 查找配置文件路径
    std::string findConfigFile();

    // 解析配置文件
    bool parseConfigFile(const std::string& config_path,
                        std::vector<AlgorithmLibraryConfig>& configs);

    // 查找库文件路径
    std::string findLibraryPath(const std::string& library_name);

    // 检查库是否已加载
    bool isLibraryLoaded(const std::string& library_path) const;
};
