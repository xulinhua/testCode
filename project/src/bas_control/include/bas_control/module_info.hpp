#ifndef BAS_CONTROL_MODULE_INFO_HPP
#define BAS_CONTROL_MODULE_INFO_HPP

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <memory>
#include "bas_operate_ros/module_status.hpp"

// 场景名称宏定义 - 统一管理所有场景字符串
#define SCENE_NAME_UNKNOWN      "unknown"
#define SCENE_NAME_IDLE         "idle"
#define SCENE_NAME_NAVIGATION   "navigation"
#define SCENE_NAME_INTERACTION  "interaction"
#define SCENE_NAME_MANIPULATION "manipulation"
#define SCENE_NAME_CALIBRATION  "calibration"
#define SCENE_NAME_FULL         "full"
#define SCENE_NAME_CUSTOM       "custom"

namespace bas_control {

// 引用bas_operate_ros命名空间中的ModuleStatus
using ModuleStatus = basros::ModuleStatus;
using basros::moduleStatusToString;
using basros::stringToModuleStatus;
using basros::isStatusRunning;
using basros::isStatusStopped;
using basros::isStatusError;

/**
 * @brief模块类型枚举
 */
enum class ModuleType {
    UNKNOWN = 0,      ///< 未知类型
    CONFIG_SERVICE = 1,   ///<配置服务
    CAMERA_SERVICE = 2,   ///<相机服务
    DETECTION_SERVICE = 3, ///<检测服务
    INTERACTION_SERVICE = 4, ///< 交互服务
    SYSTEM_SERVICE = 5    ///<系统服务
};

/**
 * @brief 场景类型枚举
 * 
 * 预定义的核心场景类型，提供编译期类型安全。
 * 使用枚举类确保类型安全，避免字符串拼写错误。
 */
enum class SceneType {
    UNKNOWN = 0,      ///< 未知场景
    IDLE,             ///< 空闲场景 - 仅启动基础服务
    NAVIGATION,       ///< 导航场景 - 目标检测+文字识别
    INTERACTION,      ///< 交互场景 - 人脸+手势识别
    MANIPULATION,     ///< 操作场景 - 精确检测服务
    CALIBRATION,      ///< 标定场景 - 手眼标定服务
    FULL,             ///< 完整场景 - 全功能运行
    CUSTOM = 255      ///< 自定义场景标记 - 支持运行时扩展
};

/**
 * @brief 场景类型与字符串转换工具函数
 * @param type 场景类型枚举
 * @return 场景名称字符串
 */
inline std::string sceneTypeToString(SceneType type) 
{
    switch (type) 
    {
        case SceneType::UNKNOWN: return SCENE_NAME_UNKNOWN;
        case SceneType::IDLE: return SCENE_NAME_IDLE;
        case SceneType::NAVIGATION: return SCENE_NAME_NAVIGATION;
        case SceneType::INTERACTION: return SCENE_NAME_INTERACTION;
        case SceneType::MANIPULATION: return SCENE_NAME_MANIPULATION;
        case SceneType::CALIBRATION: return SCENE_NAME_CALIBRATION;
        case SceneType::FULL: return SCENE_NAME_FULL;
        case SceneType::CUSTOM: return SCENE_NAME_CUSTOM;
        default: return SCENE_NAME_UNKNOWN;
    }
}

/**
 * @brief 字符串转换为场景类型
 * @param name 场景名称字符串
 * @return 场景类型枚举，未匹配则返回 CUSTOM
 */
inline SceneType stringToSceneType(const std::string& name) {
    if (name == SCENE_NAME_IDLE) return SceneType::IDLE;
    if (name == SCENE_NAME_NAVIGATION) return SceneType::NAVIGATION;
    if (name == SCENE_NAME_INTERACTION) return SceneType::INTERACTION;
    if (name == SCENE_NAME_MANIPULATION) return SceneType::MANIPULATION;
    if (name == SCENE_NAME_CALIBRATION) return SceneType::CALIBRATION;
    if (name == SCENE_NAME_FULL) return SceneType::FULL;
    if (name == SCENE_NAME_UNKNOWN) return SceneType::UNKNOWN;
    return SceneType::CUSTOM;  // 未识别的名称视为自定义场景
}

/**
 * @brief 检查是否为预定义的核心场景
 * @param type 场景类型
 * @return true 如果是核心场景（非CUSTOM）
 */
inline bool isCoreScene(SceneType type) {
    return type != SceneType::UNKNOWN && type != SceneType::CUSTOM;
}

/**
 * @brief模块信息结构体
 */
struct ModuleInfo {
    // 标识与启动信息
    std::string name;                      ///< 模块唯一标识名称
    std::string executable_path;           ///< 可执行文件或启动命令路径
    std::vector<std::string> dependencies; ///< 所依赖的模块名列表
    ModuleType type;                       ///< 模块类型（如视觉、控制、规划）
    ModuleStatus status;                   ///< 当前状态，见ModuleStatus枚举
    int pid;                               ///<进程ID，-1表示未运行

    // 运行时与健康信息
    std::chrono::steady_clock::time_point start_time; ///< 本次启动的时间点
    std::chrono::steady_clock::time_point last_heartbeat; ///< 最后一次收到心跳的时间
    std::string error_message;           ///< 记录错误详情信息
    std::map<std::string, std::string> parameters; ///< 启动参数键值表，用于热更新
    
    //构造函数
    ModuleInfo() : type(ModuleType::UNKNOWN), status(ModuleStatus::UNKNOWN), 
                   pid(-1) {}
    
    ModuleInfo(const std::string& module_name) 
        : name(module_name), type(ModuleType::UNKNOWN), 
          status(ModuleStatus::UNKNOWN), pid(-1) {}
    
    /**
     * @brief检查模块是否正在运行
     * @return true if running, false otherwise
     */
    bool isRunning() const {
        return isStatusRunning(status);
    }
    
    /**
     * @brief检查模块是否已停止
     * @return true if stopped, false otherwise
     */
    bool isStopped() const {
        return isStatusStopped(status);
    }
    
    /**
     * @brief 检查模块是否处于错误状态
     * @return true if in error state, false otherwise
     */
    bool hasError() const {
        return isStatusError(status);
    }
    
    /**
     * @brief 获取状态字符串表示
     * @return状态字符串
     */
    std::string getStatusString() const {
        return moduleStatusToString(status);
    }
    
    /**
     * @brief 获取类型字符串表示
     * @return 类型字符串
     */
    std::string getTypeString() const 
    {
        switch (type) 
        {
            case ModuleType::UNKNOWN: return "UNKNOWN";
            case ModuleType::CONFIG_SERVICE: return "CONFIG_SERVICE";
            case ModuleType::CAMERA_SERVICE: return "CAMERA_SERVICE";
            case ModuleType::DETECTION_SERVICE: return "DETECTION_SERVICE";
            case ModuleType::INTERACTION_SERVICE: return "INTERACTION_SERVICE";
            case ModuleType::SYSTEM_SERVICE: return "SYSTEM_SERVICE";
            default: return "UNKNOWN";
        }
    }
};

/**
 * @brief系统资源状态结构体
 */
struct SystemResource {
    float cpu_usage;        ///< 整体CPU使用率 (%)
    float memory_usage;     ///< 整体内存使用率 (%)
    float gpu_usage;        ///< 整体GPU使用率 (%)
    float temperature;      ///< 系统温度 (°C)
    uint64_t available_memory; ///< 可用物理内存 (MB)
    uint64_t total_memory;    ///< 总物理内存 (MB)
    
    SystemResource() : cpu_usage(0.0f), memory_usage(0.0f), gpu_usage(0.0f), 
                       temperature(0.0f), available_memory(0), total_memory(0) {}
};

/**
 * @brief 场景信息结构体（混合模式）
 * 
 * 整合枚举类型安全与字符串扩展能力：
 * - 核心场景使用枚举类型，提供编译期类型安全
 * - 自定义场景使用字符串名称，支持运行时扩展
 * 
 * 设计优势：
 * 1. 类型安全：核心场景通过枚举比较，避免拼写错误
 * 2. 高性能：枚举比较为整数操作，无需字符串比较
 * 3. 可扩展：支持从配置文件动态加载自定义场景
 * 4. 向后兼容：提供字符串转换接口，保持接口一致性
 */
struct SceneInfo {
    SceneType type;           ///< 场景类型枚举（快速比较/switch）
    std::string custom_name;  ///< 自定义场景名称（仅当 type == CUSTOM 时有效）
    
    /**
     * @brief 默认构造函数
     */
    SceneInfo() : type(SceneType::UNKNOWN) {}
    
    /**
     * @brief 从枚举类型构造（核心场景）
     * @param scene_type 场景类型枚举
     */
    explicit SceneInfo(SceneType scene_type) : type(scene_type) {}
    
    /**
     * @brief 从字符串构造（自动识别核心/自定义场景）
     * @param name 场景名称字符串
     */
    explicit SceneInfo(const std::string& name) 
        : type(stringToSceneType(name)), custom_name(type == SceneType::CUSTOM ? name : "") {}
    
    /**
     * @brief 构造自定义场景
     * @param custom_scene_name 自定义场景名称
     * @param is_custom 显式标记为自定义场景
     */
    SceneInfo(const std::string& custom_scene_name, bool is_custom) 
        : type(SceneType::CUSTOM), custom_name(is_custom ? custom_scene_name : "") {}
    
    /**
     * @brief 获取场景名称字符串
     * @return 核心场景返回枚举对应名称，自定义场景返回自定义名称
     */
    std::string getName() const {
        if (type == SceneType::CUSTOM) {
            return custom_name;
        }
        return sceneTypeToString(type);
    }
    
    /**
     * @brief 检查是否为核心场景
     * @return true 如果是核心场景
     */
    bool isCoreScene() const {
        return bas_control::isCoreScene(type);
    }
    
    /**
     * @brief 检查是否为有效场景
     * @return true 如果不是 UNKNOWN
     */
    bool isValid() const {
        return type != SceneType::UNKNOWN;
    }
    
    /**
     * @brief 比较运算符
     */
    bool operator==(const SceneInfo& other) const {
        if (type != other.type) return false;
        if (type == SceneType::CUSTOM) {
            return custom_name == other.custom_name;
        }
        return true;
    }
    
    bool operator!=(const SceneInfo& other) const {
        return !(*this == other);
    }
    
    /**
     * @brief 小于运算符（用于 std::map 等容器）
     */
    bool operator<(const SceneInfo& other) const {
        if (type != other.type) {
            return static_cast<int>(type) < static_cast<int>(other.type);
        }
        if (type == SceneType::CUSTOM) {
            return custom_name < other.custom_name;
        }
        return false;
    }
};

/**
 * @brief系统状态结构体
 */
struct SystemStatus {
    ModuleStatus overall_status;           ///<系统整体状态（通常由关键模块状态聚合而来）
    SystemResource resource_usage;         ///<当前系统资源使用情况
    std::map<std::string, ModuleInfo> modules; ///<各模块状态的映射表，键为模块名
    std::chrono::steady_clock::time_point timestamp; ///<本次状态快照的生成时间
    SceneInfo current_scene;               ///< 当前活跃的场景信息（混合模式：枚举+字符串）
    std::vector<std::string> active_modules; ///<当前场景下处于活跃状态的模块名称列表
    
    SystemStatus() : overall_status(ModuleStatus::UNKNOWN) {
        timestamp = std::chrono::steady_clock::now();
    }
    
    /**
     * @brief 获取运行中的模块数量
     * @return运行中的模块数
     */
    size_t getRunningModuleCount() const 
    {
        size_t count = 0;
        for (const auto& pair : modules) 
        {
            if (pair.second.isRunning()) {
                count++;
            }
        }
        return count;
    }
    
    /**
     * @brief 检查系统是否健康
     * @return true if system is healthy, false otherwise
     */
    bool isHealthy() const 
    {
        return overall_status == ModuleStatus::RUNNING && 
               resource_usage.cpu_usage < 90.0f && 
               resource_usage.memory_usage < 85.0f;
    }
    
    /**
     * @brief 获取系统状态字符串
     * @return 状态字符串
     */
    std::string getStatusString() const {
        return moduleStatusToString(overall_status);
    }
};

/**
 * @brief场景配置结构体
 */
struct SceneConfig {
    SceneType type;                           ///< 场景类型枚举（与 SceneInfo 保持一致）
    std::string name;                           ///<场名称，如 "navigation", "interaction"
    std::vector<std::string> active_modules;    ///< 该场景下必须激活的模块集合
    std::map<std::string, std::map<std::string, std::string>> module_params; ///<模块专属参数，支持场景差异化配置
    std::string description;                    ///<场景描述信息
    
    SceneConfig() : type(SceneType::UNKNOWN) {}
    
    SceneConfig(const std::string& scene_name, const std::vector<std::string>& modules)
        : type(stringToSceneType(scene_name)), name(scene_name), active_modules(modules) {}
    
    /**
     * @brief 从枚举类型构造（核心场景）
     * @param scene_type 场景类型枚举
     * @param modules 激活的模块列表
     */
    SceneConfig(SceneType scene_type, const std::vector<std::string>& modules)
        : type(scene_type), name(sceneTypeToString(scene_type)), active_modules(modules) {}
    
    /**
     * @brief 获取场景信息
     * @return SceneInfo 对象
     */
    SceneInfo getSceneInfo() const {
        if (type == SceneType::CUSTOM) {
            return SceneInfo(name, true);
        }
        return SceneInfo(type);
    }
    
    /**
     * @brief 检查是否为核心场景
     * @return true 如果是核心场景
     */
    bool isCoreScene() const {
        return bas_control::isCoreScene(type);
    }
};

/**
 * @brief配置参数结构体
 */
struct ConfigParams {
    //系统行为参数
    std::string config_file_path;              ///<配置文件路径（用于热更新监控）
    std::string default_scene;                 ///<默认启动场景名称（从配置文件读取）
    int health_check_interval_ms;              ///<健康检查执行间隔（毫秒）
    int status_report_interval_ms;            ///<状态上报间隔（毫秒）
    int startup_timeout_ms;                    ///<单个模块启动超时时间（毫秒）
    int max_restart_attempts;                  ///<模块启动失败后最大重试次数
    
    //资源监控告警阈值
    float cpu_threshold_warning;               ///< CPU使用率警告阈值(%)
    float memory_threshold_warning;            ///<内存使用率警告阈值(%)
    float gpu_threshold_warning;               ///< GPU使用率警告阈值(%)
    float temperature_threshold_warning;      ///<温度警告警告阈值(°C)
    
    //核心配置：驱动模块启动与场景管理
    std::vector<std::string> startup_order;    ///<模块全局启动顺序列表（经拓扑排序后）
    std::map<std::string, std::vector<std::string>> dependencies; ///<模块依赖关系图
    std::map<std::string, SceneConfig> scenes;  ///<所有预定义场景的配置映射
    
    ConfigParams() 
        : default_scene("idle"),
          health_check_interval_ms(5000),
          status_report_interval_ms(1000),
          startup_timeout_ms(30000),
          max_restart_attempts(3),
          cpu_threshold_warning(80.0f),
          memory_threshold_warning(85.0f),
          gpu_threshold_warning(85.0f),
          temperature_threshold_warning(70.0f) {}
};

/**
 * @brief 打印配置参数信息（普通函数，方便复用）
 * @param config 配置参数结构体
 */
void printConfigParams(const ConfigParams& config);

} // namespace bas_control

#endif // BAS_CONTROL_MODULE_INFO_HPP