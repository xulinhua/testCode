#include "algorithm_loader.h"
#include <dlfcn.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <set>
#include <cstdlib>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "log_system/log_macros.hpp"
#include "bas_operate/file_operate.hpp"

namespace fs = std::filesystem;

// 私有实现
struct AlgorithmLoader::Impl {
    std::vector<void*> loaded_handles_;           // 已加载的库句柄
    std::vector<std::string> loaded_library_paths_;  // 已加载的库路径
    bool initialized_ = false;
    bool auto_discovery_enabled_ = false;         // 是否启用自动发现（true: 作为备份模式）
};

AlgorithmLoader::AlgorithmLoader() 
    : pImpl_(std::make_unique<Impl>()), log_path_("") {
    // 初始化日志路径
    const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
    log_path_ = project_name;

    // 构造时自动加载所有算法
    loadAllAlgorithms();
}

AlgorithmLoader::~AlgorithmLoader() {
    // 析构时卸载所有库
    for (void* handle : pImpl_->loaded_handles_) {
        if (handle) {
            dlclose(handle);
        }
    }
}

AlgorithmLoader& AlgorithmLoader::getInstance() {
    static AlgorithmLoader instance;
    return instance;
}

void AlgorithmLoader::loadAllAlgorithms() {
    if (pImpl_->initialized_) {
        return;  // 避免重复加载
    }

    LOG_INFO(log_path_, "[AlgorithmLoader] 开始加载算法库...");

    bool load_success = false;
    bool has_config_file = false;

    // 步骤1: 尝试从配置文件加载
    std::string config_path = findConfigFile();
    has_config_file = !config_path.empty();
    
    if (has_config_file) {
        LOG_INFO(log_path_, "[AlgorithmLoader] 使用配置文件: %s", config_path.c_str());
        load_success = loadFromConfig(config_path);
    }

    // 步骤2: 根据配置决定是否使用自动发现
    if (!load_success) {
        if (pImpl_->auto_discovery_enabled_) {
            // 自动发现模式：配置失败或无配置文件时启用
            if (!has_config_file) {
                LOG_INFO(log_path_, "[AlgorithmLoader] 未找到配置文件，启用自动发现模式");
            } else {
                LOG_INFO(log_path_, "[AlgorithmLoader] 配置加载失败，回退到自动发现模式");
            }
            autoDiscoverAlgorithms();
            load_success = true;  // 自动发现视为成功（即使没找到库也完成了流程）
        } else if (has_config_file) {
            // 配置文件存在但加载失败，且禁用了自动发现
            LOG_ERROR(log_path_, "[AlgorithmLoader] ❌ 配置文件加载失败！");
            LOG_ERROR(log_path_, "[AlgorithmLoader] 请检查配置文件中的算法库路径是否正确");
            
            auto detection_algs = TaskFactory::getSupportedDetectionAlgorithms();
            if (detection_algs.empty()) {
                LOG_ERROR(log_path_, "[AlgorithmLoader] ❌ 未加载任何算法库，程序可能无法正常工作");
            }
        } else {
            // 无配置文件，且禁用了自动发现
            LOG_WARN(log_path_, "[AlgorithmLoader] 未找到配置文件，且禁用了自动发现");
        }
    }

    // 输出已注册的算法列表
    auto detection_algs = TaskFactory::getSupportedDetectionAlgorithms();
    std::string det_list;
    for (const auto& alg : detection_algs) {
        if (!det_list.empty()) det_list += ", ";
        det_list += alg;
    }
    LOG_INFO(log_path_, "[AlgorithmLoader] 已注册的检测算法: %s", det_list.c_str());
    
    auto obb_algs = TaskFactory::getSupportedOBBDetectionAlgorithms();
    std::string obb_list;
    for (const auto& alg : obb_algs) {
        if (!obb_list.empty()) obb_list += ", ";
        obb_list += alg;
    }
    LOG_INFO(log_path_, "[AlgorithmLoader] 已注册的OBB检测算法: %s", obb_list.c_str());
    
    auto seg_algs = TaskFactory::getSupportedSegmentationAlgorithms();
    std::string seg_list;
    for (const auto& alg : seg_algs) {
        if (!seg_list.empty()) seg_list += ", ";
        seg_list += alg;
    }
    LOG_INFO(log_path_, "[AlgorithmLoader] 已注册的分割算法: %s", seg_list.c_str());

    pImpl_->initialized_ = true;
    
    // 根据加载结果输出不同的完成日志
    if (load_success) {
        LOG_INFO(log_path_, "[AlgorithmLoader] 算法加载完成");
    } else {
        LOG_ERROR(log_path_, "[AlgorithmLoader] 算法加载失败，请检查配置");
    }
}

std::string AlgorithmLoader::findConfigFile() {
    std::vector<std::string> candidate_paths;

    // 1. 环境变量指定
    const char* env_config = std::getenv("DETECTION_ALGORITHM_CONFIG");
    if (env_config && strlen(env_config) > 0) {
        candidate_paths.push_back(std::string(env_config));
    }

    // 2. ROS2 包路径
    try {
        std::string package_share = ament_index_cpp::get_package_share_directory("vision_base");
        candidate_paths.push_back(package_share + "/config/algorithm_libraries.yaml");
    } catch (const std::exception& e) {
        LOG_WARN(log_path_, "[AlgorithmLoader] 无法获取 vision_base 包路径: %s", e.what());
    }

    // 3. 从 AMENT_PREFIX_PATH 推断
    const char* ament_prefix = std::getenv("AMENT_PREFIX_PATH");
    if (ament_prefix) {
        std::string prefix_path(ament_prefix);
        // AMENT_PREFIX_PATH 可能包含多个路径，取第一个
        size_t colon_pos = prefix_path.find(':');
        std::string first_prefix = (colon_pos != std::string::npos) 
            ? prefix_path.substr(0, colon_pos) 
            : prefix_path;
        candidate_paths.push_back(first_prefix + "/share/vision_base/config/algorithm_libraries.yaml");
    }

    // 查找第一个存在的配置文件
    for (const auto& path : candidate_paths) {
        if (!path.empty() && fs::exists(path)) {
            LOG_INFO(log_path_, "[AlgorithmLoader] 找到配置文件: %s", path.c_str());
            return path;
        }
    }

    LOG_WARN(log_path_, "[AlgorithmLoader] 未找到配置文件");
    return "";
}

bool AlgorithmLoader::loadFromConfig(const std::string& config_path) {
    std::vector<AlgorithmLibraryConfig> configs;
    
    if (!parseConfigFile(config_path, configs)) {
        LOG_ERROR(log_path_, "[AlgorithmLoader] 解析配置文件失败: %s", config_path.c_str());
        return false;
    }

    if (configs.empty()) {
        LOG_WARN(log_path_, "[AlgorithmLoader] 配置文件中无算法库配置");
        return false;
    }

    // 按优先级排序（高优先级先加载）
    std::sort(configs.begin(), configs.end(), 
        [](const AlgorithmLibraryConfig& a, const AlgorithmLibraryConfig& b) {
            return a.priority > b.priority;
        });

    int loaded_count = 0;
    int skipped_count = 0;

    // 加载配置的算法库
    for (const auto& config : configs) {
        if (!config.enabled) {
            LOG_INFO(log_path_, "[AlgorithmLoader] 跳过已禁用的算法: %s", config.name.c_str());
            skipped_count++;
            continue;
        }

        // 查找库文件
        std::string library_path = findLibraryPath(config.library);
        if (library_path.empty()) {
            LOG_WARN(log_path_, "[AlgorithmLoader] 未找到算法库: %s (%s)", 
                     config.name.c_str(), config.library.c_str());
            skipped_count++;
            continue;
        }

        // 加载库
        loadAlgorithmsFromPath(library_path);
        loaded_count++;
    }

    LOG_INFO(log_path_, "[AlgorithmLoader] 配置加载完成: 加载 %d 个，跳过 %d 个", 
             loaded_count, skipped_count);
    
    return loaded_count > 0;  // 至少加载一个才算成功
}

bool AlgorithmLoader::parseConfigFile(const std::string& config_path,
                                     std::vector<AlgorithmLibraryConfig>& configs) {
    try {
        YAML::Node root = YAML::LoadFile(config_path);
        
        if (!root["algorithm_libraries"]) {
            LOG_ERROR(log_path_, "[AlgorithmLoader] 配置文件缺少 algorithm_libraries 字段");
            return false;
        }

        // 解析各类算法
        auto parseCategory = [&](const std::string& category) {
            if (root["algorithm_libraries"][category]) {
                for (const auto& node : root["algorithm_libraries"][category]) {
                    AlgorithmLibraryConfig config;
                    
                    if (!node["name"] || !node["library"]) {
                        LOG_WARN(log_path_, "[AlgorithmLoader] 算法配置缺少 name 或 library 字段");
                        continue;
                    }
                    
                    config.name = node["name"].as<std::string>();
                    config.library = node["library"].as<std::string>();
                    
                    if (node["enabled"]) {
                        config.enabled = node["enabled"].as<bool>();
                    }
                    if (node["priority"]) {
                        config.priority = node["priority"].as<int>();
                    }
                    if (node["description"]) {
                        config.description = node["description"].as<std::string>();
                    }
                    
                    configs.push_back(config);
                }
            }
        };
        
        parseCategory("detection");
        parseCategory("obb_detection");
        parseCategory("segmentation");
        parseCategory("ocr");
        parseCategory("gesture");

        // 解析自动发现配置
        if (root["auto_discovery"]) {
            auto auto_disc = root["auto_discovery"];
            if (auto_disc["enabled"]) {
                pImpl_->auto_discovery_enabled_ = auto_disc["enabled"].as<bool>();
            }
        }
        
        LOG_INFO(log_path_, "[AlgorithmLoader] 解析到 %zu 个算法库配置", configs.size());
        return true;
        
    } catch (const YAML::Exception& e) {
        LOG_ERROR(log_path_, "[AlgorithmLoader] YAML 解析错误: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR(log_path_, "[AlgorithmLoader] 解析配置文件异常: %s", e.what());
        return false;
    }
}

std::string AlgorithmLoader::findLibraryPath(const std::string& library_name) {
    const char* ament_prefix = std::getenv("AMENT_PREFIX_PATH");
    if (!ament_prefix) {
        LOG_WARN(log_path_, "[AlgorithmLoader] 未找到 AMENT_PREFIX_PATH 环境变量");
        return "";
    }

    // 解析 AMENT_PREFIX_PATH（可能包含多个路径）
    std::string prefix_path(ament_prefix);
    std::vector<std::string> search_paths;
    
    // 分割多个路径
    std::stringstream ss(prefix_path);
    std::string path;
    while (std::getline(ss, path, ':')) {
        if (!path.empty()) {
            search_paths.push_back(path);
        }
    }

    // 在每个路径中搜索
    for (const auto& search_path : search_paths) {
        try {
            if (!fs::exists(search_path)) continue;
            
            for (const auto& entry : fs::recursive_directory_iterator(search_path)) {
                if (!entry.is_regular_file()) continue;
                
                if (entry.path().filename().string() == library_name) {
                    return entry.path().string();
                }
            }
        } catch (const fs::filesystem_error& e) {
            LOG_WARN(log_path_, "[AlgorithmLoader] 文件系统错误: %s", e.what());
        }
    }
    
    return "";
}

bool AlgorithmLoader::isLibraryLoaded(const std::string& library_path) const {
    for (const auto& loaded : pImpl_->loaded_library_paths_) {
        if (loaded == library_path) {
            return true;
        }
    }
    return false;
}

void AlgorithmLoader::autoDiscoverAlgorithms() {
    LOG_INFO(log_path_, "[AlgorithmLoader] 启用自动发现模式...");

    const char* ament_prefix = std::getenv("AMENT_PREFIX_PATH");
    std::string install_dir;
    
    if (ament_prefix) {
        std::string prefix_path(ament_prefix);
        size_t install_pos = prefix_path.find("/install/");
        
        if (install_pos != std::string::npos) {
            install_dir = prefix_path.substr(0, install_pos) + "/install";
        } else {
            // 如果没有找到 /install/，尝试使用第一个路径
            size_t colon_pos = prefix_path.find(':');
            if (colon_pos != std::string::npos) {
                install_dir = prefix_path.substr(0, colon_pos);
            } else {
                install_dir = prefix_path;
            }
        }
    }

    if (install_dir.empty() || !fs::exists(install_dir)) {
        LOG_WARN(log_path_, "[AlgorithmLoader] 无法确定安装目录: %s", install_dir.c_str());
        return;
    }

    LOG_INFO(log_path_, "[AlgorithmLoader] 搜索目录: %s", install_dir.c_str());

    int loaded_count = 0;
    
    try {
        for (const auto& entry : fs::recursive_directory_iterator(install_dir)) {
            if (!entry.is_regular_file()) continue;
            
            const std::string& path = entry.path().string();
            const std::string filename = entry.path().filename().string();
            
            // 检查是否匹配算法库前缀
            if (path.find(".so") == std::string::npos) continue;
            
            // 使用配置文件中的前缀列表
            std::vector<std::string> prefixes = {
                "libyolo_", "libscrfd_", "libppocr_", "libgesture_"
            };
            
            for (const auto& prefix : prefixes) {
                if (filename.find(prefix) == 0) {
                    loadAlgorithmsFromPath(path);
                    loaded_count++;
                    break;
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        LOG_ERROR(log_path_, "[AlgorithmLoader] 文件系统错误: %s", e.what());
    }

    LOG_INFO(log_path_, "[AlgorithmLoader] 自动发现完成，加载 %d 个算法库", loaded_count);
}

void AlgorithmLoader::loadAlgorithmsFromPath(const std::string& library_path) {
    if (isLibraryLoaded(library_path)) {
        LOG_INFO(log_path_, "[AlgorithmLoader] 库已加载，跳过: %s", library_path.c_str());
        return;
    }

    LOG_INFO(log_path_, "[AlgorithmLoader] 加载算法库: %s", library_path.c_str());

    // 加载共享库（使用RTLD_GLOBAL确保符号可见）
    void* handle = dlopen(library_path.c_str(), RTLD_LAZY | RTLD_GLOBAL);

    if (!handle) {
        const char* error = dlerror();
        LOG_WARN(log_path_, "[AlgorithmLoader] 加载失败 %s: %s", 
                 library_path.c_str(), error ? error : "unknown error");
        return;
    }

    pImpl_->loaded_handles_.push_back(handle);
    pImpl_->loaded_library_paths_.push_back(library_path);
    LOG_INFO(log_path_, "[AlgorithmLoader] 成功加载: %s", library_path.c_str());
}

std::vector<std::string> AlgorithmLoader::getLoadedLibraries() const {
    return pImpl_->loaded_library_paths_;
}
