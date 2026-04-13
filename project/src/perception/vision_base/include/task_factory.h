#pragma once

#include "det_task_base.h"
#include "seg_task_base.h"
#include "obb_task_base.h"
#include <memory>
#include <string>
#include <map>
#include <functional>
#include <vector>

// 通用任务工厂类 - 用于创建不同类型的任务实例
// 算法库注册时声明支持的引擎类型，工厂根据配置选择合适的实现
class TaskFactory {
public:
    // ============ 检测任务 ============

    // 检测任务创建器类型（支持传递引擎配置）
    using DetectionTaskCreator = std::function<std::unique_ptr<IDetectionTask>(
        const std::string& model_path,
        const std::string& config_path,
        const InferenceEngineConfig& engine_config)>;

    // 注册检测算法（带支持的引擎类型）
    static bool registerDetectionAlgorithm(
        const std::string& algorithm_name,
        InferenceEngineType engine_type,
        DetectionTaskCreator creator);

    // 创建检测任务实例（自动加载算法库，根据引擎配置选择实现）
    static std::unique_ptr<IDetectionTask> createDetectionTask(
        const std::string& algorithm_name,
        const std::string& model_path,
        const std::string& config_path = "",
        const InferenceEngineConfig& engine_config = {});

    // 获取支持的检测算法列表
    static std::vector<std::string> getSupportedDetectionAlgorithms();

    // 获取指定检测算法支持的引擎类型
    static std::vector<InferenceEngineType> getSupportedDetectionEngineTypes(const std::string& algorithm_name);

    // ============ OBB检测任务 ============

    // OBB检测任务创建器类型
    using OBBDetectionTaskCreator = std::function<std::unique_ptr<IOBBDetectionTask>(
        const std::string& model_path,
        const std::string& config_path,
        const InferenceEngineConfig& engine_config)>;

    // 注册OBB检测算法
    static bool registerOBBDetectionAlgorithm(
        const std::string& algorithm_name,
        InferenceEngineType engine_type,
        OBBDetectionTaskCreator creator);

    // 创建OBB检测任务实例
    static std::unique_ptr<IOBBDetectionTask> createOBBDetectionTask(
        const std::string& algorithm_name,
        const std::string& model_path,
        const std::string& config_path = "",
        const InferenceEngineConfig& engine_config = {});

    // 获取支持的OBB检测算法列表
    static std::vector<std::string> getSupportedOBBDetectionAlgorithms();

    // 获取指定OBB检测算法支持的引擎类型
    static std::vector<InferenceEngineType> getSupportedOBBDetectionEngineTypes(const std::string& algorithm_name);

    // ============ 分割任务 ============

    // 分割任务创建器类型（支持传递引擎配置）
    using SegmentationTaskCreator = std::function<std::unique_ptr<ISegmentationTask>(
        const std::string& model_path,
        const std::string& config_path,
        const InferenceEngineConfig& engine_config)>;

    // 注册分割算法（带支持的引擎类型）
    static bool registerSegmentationAlgorithm(
        const std::string& algorithm_name,
        InferenceEngineType engine_type,
        SegmentationTaskCreator creator);

    // 创建分割任务实例（自动加载算法库，根据引擎配置选择实现）
    static std::unique_ptr<ISegmentationTask> createSegmentationTask(
        const std::string& algorithm_name,
        const std::string& model_path,
        const std::string& config_path = "",
        const InferenceEngineConfig& engine_config = {});

    // 获取支持的分割算法列表
    static std::vector<std::string> getSupportedSegmentationAlgorithms();

    // 获取指定分割算法支持的引擎类型
    static std::vector<InferenceEngineType> getSupportedSegmentationEngineTypes(const std::string& algorithm_name);

private:
    // 辅助函数：获取支持的算法列表（用于错误消息）
    static std::string getSupportedDetectionAlgorithmsList();
    static std::string getSupportedOBBDetectionAlgorithmsList();
    static std::string getSupportedSegmentationAlgorithmsList();

    // 检测算法注册表：算法名 -> (引擎类型 -> 创建器)
    static std::map<std::string, std::map<InferenceEngineType, DetectionTaskCreator>>& getDetectionAlgorithmRegistry();

    // OBB检测算法注册表
    static std::map<std::string, std::map<InferenceEngineType, OBBDetectionTaskCreator>>& getOBBDetectionAlgorithmRegistry();

    // 分割算法注册表：算法名 -> (引擎类型 -> 创建器)
    static std::map<std::string, std::map<InferenceEngineType, SegmentationTaskCreator>>& getSegmentationAlgorithmRegistry();

    TaskFactory() = default;  // 静态类,禁止实例化
};
