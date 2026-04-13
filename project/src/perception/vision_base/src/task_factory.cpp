#include "task_factory.h"
#include "algorithm_loader.h"
#include <stdexcept>
#include "log_system/log_macros.hpp"
#include "bas_operate/file_operate.hpp"

// ============ 检测任务实现 ============

std::map<std::string, std::map<InferenceEngineType, TaskFactory::DetectionTaskCreator>>&
TaskFactory::getDetectionAlgorithmRegistry() {
    static std::map<std::string, std::map<InferenceEngineType, DetectionTaskCreator>> registry;
    return registry;
}

bool TaskFactory::registerDetectionAlgorithm(
    const std::string& algorithm_name,
    InferenceEngineType engine_type,
    DetectionTaskCreator creator) {
    if (algorithm_name.empty() || !creator) {
        return false;
    }
    getDetectionAlgorithmRegistry()[algorithm_name][engine_type] = creator;
    return true;
}

std::vector<std::string> TaskFactory::getSupportedDetectionAlgorithms() {
    std::vector<std::string> algorithms;
    for (const auto& pair : getDetectionAlgorithmRegistry()) {
        algorithms.push_back(pair.first);
    }
    return algorithms;
}

std::vector<InferenceEngineType> TaskFactory::getSupportedDetectionEngineTypes(const std::string& algorithm_name) {
    std::vector<InferenceEngineType> engine_types;
    auto& registry = getDetectionAlgorithmRegistry();
    auto it = registry.find(algorithm_name);
    if (it != registry.end()) {
        for (const auto& engine_pair : it->second) {
            engine_types.push_back(engine_pair.first);
        }
    }
    return engine_types;
}

std::unique_ptr<IDetectionTask> TaskFactory::createDetectionTask(
    const std::string& algorithm_name,
    const std::string& model_path,
    const std::string& config_path,
    const InferenceEngineConfig& engine_config) {

    // 获取日志路径
    const std::string log_path = basmodule::get_project_name_by_file_path(__FILE__);

    // 0. 触发算法库自动加载（单例模式，只加载一次）
    static auto& loader = AlgorithmLoader::getInstance();

    // 1. 从注册表中查找算法
    auto& registry = getDetectionAlgorithmRegistry();
    auto it = registry.find(algorithm_name);
    if (it == registry.end()) {
        throw std::runtime_error("Unsupported detection algorithm: " + algorithm_name +
                               "\nSupported algorithms: " +
                               getSupportedDetectionAlgorithmsList());
    }

    // 2. 根据引擎配置查找对应的实现
    InferenceEngineType requested_engine = engine_config.engine_type;

    // 如果是 AUTO，尝试使用第一个可用的引擎实现
    if (requested_engine == InferenceEngineType::AUTO) {
        if (it->second.empty()) {
            throw std::runtime_error("No implementation found for algorithm: " + algorithm_name);
        }
        // 默认使用第一个引擎实现
        requested_engine = it->second.begin()->first;
        LOG_INFO(log_path, "[TaskFactory] Auto-selected engine type for %s: %d",
                 algorithm_name.c_str(), static_cast<int>(requested_engine));
    }

    // 查找指定引擎的实现
    auto engine_it = it->second.find(requested_engine);
    if (engine_it == it->second.end()) {
        // 该算法不支持请求的引擎类型
        auto supported = getSupportedDetectionEngineTypes(algorithm_name);
        std::string supported_list;
        for (size_t i = 0; i < supported.size(); ++i) {
            if (i > 0) supported_list += ", ";
            supported_list += std::to_string(static_cast<int>(supported[i]));
        }
        throw std::runtime_error("Algorithm '" + algorithm_name + "' does not support engine type " +
                               std::to_string(static_cast<int>(requested_engine)) +
                               "\nSupported engine types: " + supported_list);
    }

    // 3. 使用对应的创建器创建算法实例
    auto task = engine_it->second(model_path, config_path, engine_config);
    if (!task) {
        throw std::runtime_error("Failed to create detection task");
    }

    // 4. 检查是否初始化成功
    if (!task->isInitialized()) {
        throw std::runtime_error("Detection task failed to initialize");
    }

    // 5. 加载配置文件(如果提供了)
    if (!config_path.empty() && !task->loadConfig(config_path)) {
        throw std::runtime_error("Failed to load config: " + config_path);
    }

    return task;
}

// 辅助函数：获取支持的检测算法列表（用于错误消息）
std::string TaskFactory::getSupportedDetectionAlgorithmsList() {
    auto algorithms = getSupportedDetectionAlgorithms();
    if (algorithms.empty()) {
        return "(none)";
    }

    std::string list;
    for (size_t i = 0; i < algorithms.size(); ++i) {
        if (i > 0) list += ", ";
        list += algorithms[i];
    }
    return list;
}

// ============ 分割任务实现 ============

std::map<std::string, std::map<InferenceEngineType, TaskFactory::SegmentationTaskCreator>>&
TaskFactory::getSegmentationAlgorithmRegistry() {
    static std::map<std::string, std::map<InferenceEngineType, SegmentationTaskCreator>> registry;
    return registry;
}

bool TaskFactory::registerSegmentationAlgorithm(
    const std::string& algorithm_name,
    InferenceEngineType engine_type,
    SegmentationTaskCreator creator) {
    if (algorithm_name.empty() || !creator) {
        return false;
    }
    getSegmentationAlgorithmRegistry()[algorithm_name][engine_type] = creator;
    return true;
}

std::vector<std::string> TaskFactory::getSupportedSegmentationAlgorithms() {
    std::vector<std::string> algorithms;
    for (const auto& pair : getSegmentationAlgorithmRegistry()) {
        algorithms.push_back(pair.first);
    }
    return algorithms;
}

std::vector<InferenceEngineType> TaskFactory::getSupportedSegmentationEngineTypes(const std::string& algorithm_name) {
    std::vector<InferenceEngineType> engine_types;
    auto& registry = getSegmentationAlgorithmRegistry();
    auto it = registry.find(algorithm_name);
    if (it != registry.end()) {
        for (const auto& engine_pair : it->second) {
            engine_types.push_back(engine_pair.first);
        }
    }
    return engine_types;
}

std::unique_ptr<ISegmentationTask> TaskFactory::createSegmentationTask(
    const std::string& algorithm_name,
    const std::string& model_path,
    const std::string& config_path,
    const InferenceEngineConfig& engine_config) {

    // 获取日志路径
    const std::string log_path = basmodule::get_project_name_by_file_path(__FILE__);

    // 0. 触发算法库自动加载（单例模式，只加载一次）
    static auto& loader = AlgorithmLoader::getInstance();

    // 1. 从注册表中查找算法
    auto& registry = getSegmentationAlgorithmRegistry();
    auto it = registry.find(algorithm_name);
    if (it == registry.end()) {
        throw std::runtime_error("Unsupported segmentation algorithm: " + algorithm_name +
                               "\nSupported algorithms: " +
                               getSupportedSegmentationAlgorithmsList());
    }

    // 2. 根据引擎配置查找对应的实现
    InferenceEngineType requested_engine = engine_config.engine_type;

    // 如果是 AUTO，尝试使用第一个可用的引擎实现
    if (requested_engine == InferenceEngineType::AUTO) {
        if (it->second.empty()) {
            throw std::runtime_error("No implementation found for algorithm: " + algorithm_name);
        }
        // 默认使用第一个引擎实现
        requested_engine = it->second.begin()->first;
        LOG_INFO(log_path, "[TaskFactory] Auto-selected engine type for %s: %d",
                 algorithm_name.c_str(), static_cast<int>(requested_engine));
    }

    // 查找指定引擎的实现
    auto engine_it = it->second.find(requested_engine);
    if (engine_it == it->second.end()) {
        // 该算法不支持请求的引擎类型
        auto supported = getSupportedSegmentationEngineTypes(algorithm_name);
        std::string supported_list;
        for (size_t i = 0; i < supported.size(); ++i) {
            if (i > 0) supported_list += ", ";
            supported_list += std::to_string(static_cast<int>(supported[i]));
        }
        throw std::runtime_error("Algorithm '" + algorithm_name + "' does not support engine type " +
                               std::to_string(static_cast<int>(requested_engine)) +
                               "\nSupported engine types: " + supported_list);
    }

    // 3. 使用对应的创建器创建算法实例
    auto task = engine_it->second(model_path, config_path, engine_config);
    if (!task) {
        throw std::runtime_error("Failed to create segmentation task");
    }

    // 4. 检查是否初始化成功
    if (!task->isInitialized()) {
        throw std::runtime_error("Segmentation task failed to initialize");
    }

    // 5. 加载配置文件(如果提供了)
    if (!config_path.empty() && !task->loadConfig(config_path)) {
        throw std::runtime_error("Failed to load config: " + config_path);
    }

    return task;
}

// 辅助函数：获取支持的分割算法列表（用于错误消息）
std::string TaskFactory::getSupportedSegmentationAlgorithmsList() {
    auto algorithms = getSupportedSegmentationAlgorithms();
    if (algorithms.empty()) {
        return "(none)";
    }

    std::string list;
    for (size_t i = 0; i < algorithms.size(); ++i) {
        if (i > 0) list += ", ";
        list += algorithms[i];
    }
    return list;
}

// ============ OBB检测任务实现 ============

std::map<std::string, std::map<InferenceEngineType, TaskFactory::OBBDetectionTaskCreator>>&
TaskFactory::getOBBDetectionAlgorithmRegistry() {
    static std::map<std::string, std::map<InferenceEngineType, OBBDetectionTaskCreator>> registry;
    return registry;
}

bool TaskFactory::registerOBBDetectionAlgorithm(
    const std::string& algorithm_name,
    InferenceEngineType engine_type,
    OBBDetectionTaskCreator creator) {
    if (algorithm_name.empty() || !creator) {
        return false;
    }
    getOBBDetectionAlgorithmRegistry()[algorithm_name][engine_type] = creator;
    return true;
}

std::vector<std::string> TaskFactory::getSupportedOBBDetectionAlgorithms() {
    std::vector<std::string> algorithms;
    for (const auto& pair : getOBBDetectionAlgorithmRegistry()) {
        algorithms.push_back(pair.first);
    }
    return algorithms;
}

std::vector<InferenceEngineType> TaskFactory::getSupportedOBBDetectionEngineTypes(const std::string& algorithm_name) {
    std::vector<InferenceEngineType> engine_types;
    auto& registry = getOBBDetectionAlgorithmRegistry();
    auto it = registry.find(algorithm_name);
    if (it != registry.end()) {
        for (const auto& engine_pair : it->second) {
            engine_types.push_back(engine_pair.first);
        }
    }
    return engine_types;
}

std::unique_ptr<IOBBDetectionTask> TaskFactory::createOBBDetectionTask(
    const std::string& algorithm_name,
    const std::string& model_path,
    const std::string& config_path,
    const InferenceEngineConfig& engine_config) {

    // 获取日志路径
    const std::string log_path = basmodule::get_project_name_by_file_path(__FILE__);

    // 0. 触发算法库自动加载（单例模式，只加载一次）
    static auto& loader = AlgorithmLoader::getInstance();

    // 1. 从注册表中查找算法
    auto& registry = getOBBDetectionAlgorithmRegistry();
    auto it = registry.find(algorithm_name);
    if (it == registry.end()) {
        throw std::runtime_error("Unsupported OBB detection algorithm: " + algorithm_name +
                               "\nSupported algorithms: " +
                               getSupportedOBBDetectionAlgorithmsList());
    }

    // 2. 根据引擎配置查找对应的实现
    InferenceEngineType requested_engine = engine_config.engine_type;

    // 如果是 AUTO，尝试使用第一个可用的引擎实现
    if (requested_engine == InferenceEngineType::AUTO) {
        if (it->second.empty()) {
            throw std::runtime_error("No implementation found for algorithm: " + algorithm_name);
        }
        // 默认使用第一个引擎实现
        requested_engine = it->second.begin()->first;
        LOG_INFO(log_path, "[TaskFactory] Auto-selected engine type for %s: %d",
                 algorithm_name.c_str(), static_cast<int>(requested_engine));
    }

    // 查找指定引擎的实现
    auto engine_it = it->second.find(requested_engine);
    if (engine_it == it->second.end()) {
        // 该算法不支持请求的引擎类型
        auto supported = getSupportedOBBDetectionEngineTypes(algorithm_name);
        std::string supported_list;
        for (size_t i = 0; i < supported.size(); ++i) {
            if (i > 0) supported_list += ", ";
            supported_list += std::to_string(static_cast<int>(supported[i]));
        }
        throw std::runtime_error("Algorithm '" + algorithm_name + "' does not support engine type " +
                               std::to_string(static_cast<int>(requested_engine)) +
                               "\nSupported engine types: " + supported_list);
    }

    // 3. 使用对应的创建器创建算法实例
    auto task = engine_it->second(model_path, config_path, engine_config);
    if (!task) {
        throw std::runtime_error("Failed to create OBB detection task");
    }

    // 4. 检查是否初始化成功
    if (!task->isInitialized()) {
        throw std::runtime_error("OBB detection task failed to initialize");
    }

    // 5. 加载配置文件(如果提供了)
    if (!config_path.empty() && !task->loadConfig(config_path)) {
        throw std::runtime_error("Failed to load config: " + config_path);
    }

    return task;
}

// 辅助函数：获取支持的OBB检测算法列表（用于错误消息）
std::string TaskFactory::getSupportedOBBDetectionAlgorithmsList() {
    auto algorithms = getSupportedOBBDetectionAlgorithms();
    if (algorithms.empty()) {
        return "(none)";
    }

    std::string list;
    for (size_t i = 0; i < algorithms.size(); ++i) {
        if (i > 0) list += ", ";
        list += algorithms[i];
    }
    return list;
}
