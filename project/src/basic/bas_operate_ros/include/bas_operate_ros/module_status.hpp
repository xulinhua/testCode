#ifndef BAS_OPERATE_ROS_MODULE_STATUS_HPP
#define BAS_OPERATE_ROS_MODULE_STATUS_HPP

#include <string>
#include <map>
#include <sstream>

#define STATUS_NAME_UNKNOWN        "UNKNOWN"
#define STATUS_NAME_STOPPED        "STOPPED"
#define STATUS_NAME_STARTING       "STARTING"
#define STATUS_NAME_RUNNING        "RUNNING"
#define STATUS_NAME_RUNNING_PAUSED "RUNNING_PAUSED"
#define STATUS_NAME_RELOADING      "RELOADING"
#define STATUS_NAME_STOPPING       "STOPPING"
#define STATUS_NAME_ERROR          "ERROR"
#define STATUS_NAME_CRASHED        "CRASHED"

namespace basros {

/**
 * @brief 模块状态枚举
 */
enum class ModuleStatus {
    UNKNOWN = 0,        ///< 未知状态（初始或无法获取的状态）
    STOPPED,            ///< 模块已完全停止
    STARTING,           ///< 模块启动指令已发出，正在初始化和等待就绪
    RUNNING,            ///< 模块正常运行，心跳正常
    RUNNING_PAUSED,     ///< 业务暂停（如采图停止），进程运行但业务功能暂停
    RELOADING,          ///< 模块正在热加载/重载（如模型更新、配置重载）
    STOPPING,           ///< 模块正在执行停止流程
    ERROR,              ///< 模块运行出错（如启动超时、业务逻辑错误）
    CRASHED             ///< 模块进程异常崩溃
};

/**
 * @brief 模块状态转换为字符串
 * @param status 模块状态枚举
 * @return 状态名称字符串
 */
inline std::string moduleStatusToString(ModuleStatus status) {
    switch (status) 
    {
        case ModuleStatus::UNKNOWN: return STATUS_NAME_UNKNOWN;
        case ModuleStatus::STOPPED: return STATUS_NAME_STOPPED;
        case ModuleStatus::STARTING: return STATUS_NAME_STARTING;
        case ModuleStatus::RUNNING: return STATUS_NAME_RUNNING;
        case ModuleStatus::RUNNING_PAUSED: return STATUS_NAME_RUNNING_PAUSED;
        case ModuleStatus::RELOADING: return STATUS_NAME_RELOADING;
        case ModuleStatus::STOPPING: return STATUS_NAME_STOPPING;
        case ModuleStatus::ERROR: return STATUS_NAME_ERROR;
        case ModuleStatus::CRASHED: return STATUS_NAME_CRASHED;
        default: return STATUS_NAME_UNKNOWN;
    }
}

/**
 * @brief 字符串转换为模块状态
 * @param status_str 状态名称字符串
 * @return 模块状态枚举
 */
inline ModuleStatus stringToModuleStatus(const std::string& status_str) {
    if (status_str == STATUS_NAME_STOPPED) return ModuleStatus::STOPPED;
    if (status_str == STATUS_NAME_STARTING) return ModuleStatus::STARTING;
    if (status_str == STATUS_NAME_RUNNING) return ModuleStatus::RUNNING;
    if (status_str == STATUS_NAME_RUNNING_PAUSED) return ModuleStatus::RUNNING_PAUSED;
    if (status_str == STATUS_NAME_RELOADING) return ModuleStatus::RELOADING;
    if (status_str == STATUS_NAME_STOPPING) return ModuleStatus::STOPPING;
    if (status_str == STATUS_NAME_ERROR) return ModuleStatus::ERROR;
    if (status_str == STATUS_NAME_CRASHED) return ModuleStatus::CRASHED;
    return ModuleStatus::UNKNOWN;
}

/**
 * @brief 检查状态是否为运行中
 * @param status 模块状态
 * @return true if running, false otherwise
 */
inline bool isStatusRunning(ModuleStatus status) {
    return status == ModuleStatus::RUNNING || 
           status == ModuleStatus::RUNNING_PAUSED ||
           status == ModuleStatus::RELOADING;
}

/**
 * @brief 检查状态是否为已停止
 * @param status 模块状态
 * @return true if stopped, false otherwise
 */
inline bool isStatusStopped(ModuleStatus status) {
    return status == ModuleStatus::STOPPED || status == ModuleStatus::UNKNOWN;
}

/**
 * @brief 检查状态是否为错误状态
 * @param status 模块状态
 * @return true if in error state, false otherwise
 */
inline bool isStatusError(ModuleStatus status) {
    return status == ModuleStatus::ERROR || status == ModuleStatus::CRASHED;
}

/**
 * @brief 模块状态信息结构体
 * 
 * 用于记录某个模块的状态和状态信息文本
 * 支持按相机ID区分的子节点状态管理
 */
struct ModuleStatusInfo {
    std::string module_name;          ///< 模块名称（如 "marker_detect_ros"）
    int cam_id;                       ///< 相机ID（-1表示主节点，>=0表示子节点）
    ModuleStatus status;              ///< 当前状态
    std::string status_msg;           ///< 状态信息文本（如 "相机数据流已停止"）
    
    /**
     * @brief 默认构造函数
     */
    ModuleStatusInfo() 
        : cam_id(-1), status(ModuleStatus::UNKNOWN) {}
    
    /**
     * @brief 构造函数
     * @param name 模块名称
     * @param cid 相机ID
     * @param stat 模块状态
     * @param msg 状态信息文本
     */
    ModuleStatusInfo(const std::string& name, int cid, ModuleStatus stat, const std::string& msg = "")
        : module_name(name), cam_id(cid), status(stat), status_msg(msg) {}
    
    /**
     * @brief 获取状态字符串表示
     * @return 状态字符串
     */
    std::string getStatusString() const {
        return moduleStatusToString(status);
    }
    
    /**
     * @brief 检查模块是否正在运行
     * @return true if running, false otherwise
     */
    bool isRunning() const {
        return isStatusRunning(status);
    }
    
    /**
     * @brief 检查模块是否已停止
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
};

/**
 * @brief 将ModuleStatusInfo序列化为JSON字符串
 * @param info 模块状态信息
 * @return JSON格式字符串
 * 
 * JSON格式:
 * {
 *   "module_name": "yolo_det",
 *   "cam_id": 0,
 *   "status": "RUNNING",
 *   "status_msg": "正常工作中"
 * }
 */
inline std::string moduleStatusInfoToJson(const ModuleStatusInfo& info) {
    std::ostringstream oss;
    oss << "{";
    oss << "\"module_name\":\"" << info.module_name << "\",";
    oss << "\"cam_id\":" << info.cam_id << ",";
    oss << "\"status\":\"" << moduleStatusToString(info.status) << "\",";
    oss << "\"status_msg\":\"" << info.status_msg << "\"";
    oss << "}";
    return oss.str();
}

/**
 * @brief 从 JSON 字符串反序列化 ModuleStatusInfo
 * @param json_str JSON格式字符串
 * @return 模块状态信息对象，解析失败时返回默认构造的对象
 * 
 * JSON格式:
 * {
 *   "module_name": "yolo_det",
 *   "cam_id": 0,
 *   "status": "RUNNING",
 *   "status_msg": "正常工作中"
 * }
 */
inline ModuleStatusInfo jsonToModuleStatusInfo(const std::string& json_str) {
    ModuleStatusInfo info;
    
    // 简单的JSON解析（不依赖外部JSON库）
    auto extractString = [&json_str](const std::string& key) -> std::string {
        std::string search_key = "\"" + key + "\":";
        size_t pos = json_str.find(search_key);
        if (pos == std::string::npos) return "";
        
        pos = json_str.find("\"", pos + search_key.length());
        if (pos == std::string::npos) return "";
        
        size_t end_pos = json_str.find("\"", pos + 1);
        if (end_pos == std::string::npos) return "";
        
        return json_str.substr(pos + 1, end_pos - pos - 1);
    };
    
    auto extractInt = [&json_str](const std::string& key) -> int {
        std::string search_key = "\"" + key + "\":";
        size_t pos = json_str.find(search_key);
        if (pos == std::string::npos) return -1;
        
        // 跳过冒号后的空格
        pos += search_key.length();
        while (pos < json_str.length() && (json_str[pos] == ' ' || json_str[pos] == '\t')) {
            pos++;
        }
        
        // 提取数字
        std::string num_str;
        while (pos < json_str.length() && (json_str[pos] >= '0' && json_str[pos] <= '9')) {
            num_str += json_str[pos];
            pos++;
        }
        
        return num_str.empty() ? -1 : std::stoi(num_str);
    };
    
    info.module_name = extractString("module_name");
    info.cam_id = extractInt("cam_id");
    info.status = stringToModuleStatus(extractString("status"));
    info.status_msg = extractString("status_msg");
    
    return info;
}

/**
 * @brief 模块状态信息映射表（按模块名组织）
 * 
 * 外层map的键为模块名称，内层map的键为相机ID
 * 例如: module_status_map["marker_detect_ros"][0] 表示marker_detect_ros模块的相机0状态
 */
using ModuleStatusInfoMap = std::map<std::string, std::map<int, ModuleStatusInfo>>;

} // namespace basros

#endif // BAS_OPERATE_ROS_MODULE_STATUS_HPP
