#ifndef DDS_COMMUNICATION__COMMAND_TYPES_HPP_
#define DDS_COMMUNICATION__COMMAND_TYPES_HPP_

#include <string>
#include <map>
#include <vector>

namespace dds_comm {

/**
 * @enum CommandType
 * @brief 命令类型枚举
 * 
 * 定义所有支持的命令类型，与项目类型独立
 */
enum class CommandType {
    UNKNOWN,        ///< 未知命令
    START,          ///< 启动命令
    STOP,           ///< 停止命令
    RESTART,        ///< 重启命令
    STATUS,         ///< 状态查询命令
    CONFIGURE,      ///< 配置命令
    PAUSE,          ///< 暂停命令
    RESUME,         ///< 恢复命令
    EMERGENCY_STOP  ///< 紧急停止命令
};

/**
 * @enum ProjectType
 * @brief 项目类型枚举
 * 
 * 定义所有支持的项目类型，与命令类型独立
 */
enum class ProjectType {
    UNKNOWN,            ///< 未知项目
    DDS_COMMUNICATION,  ///< DDS通信项目
    HAND_EYE_CALIB,     ///< 手眼标定项目
    CAMERA,             ///< 相机项目
    PCL2LASERSCAN,      ///< 点云转激光扫描项目
    YOLO_DET,           ///< 目标检测项目
    NAVIGATION,         ///< 导航项目
    SLAM,               ///< SLAM项目
    MOTION_CONTROL,     ///< 运动控制项目
    SENSOR_FUSION       ///< 传感器融合项目
};

/**
 * @struct CommandMessage
 * @brief 命令消息结构
 * 
 * 用于在DDS通信中传输的命令消息
 */
struct CommandMessage {
    CommandType command_type;    ///< 命令类型
    ProjectType project_type;    ///< 项目类型
    int64_t timestamp;           ///< 时间戳（毫秒）
    std::string source;          ///< 命令来源
    std::string data;            ///< 附加数据（JSON格式）
};

/**
 * @struct StatusMessage
 * @brief 状态消息结构
 * 
 * 用于在DDS通信中传输的状态消息
 */
struct StatusMessage {
    ProjectType project_type;    ///< 项目类型
    std::string status;          ///< 状态描述
    bool is_running;             ///< 是否运行中
    int uptime;                  ///< 运行时间（秒）
    std::string last_error;      ///< 最后错误信息
    int64_t timestamp;           ///< 时间戳（毫秒）
};

/**
 * @class CommandTypeConverter
 * @brief 命令类型转换器
 * 
 * 提供命令类型和字符串之间的转换功能
 */
class CommandTypeConverter {
public:
    /**
     * @brief 命令类型转字符串
     * @param type 命令类型
     * @return std::string 字符串表示
     */
    static std::string toString(CommandType type) {
        static const std::map<CommandType, std::string> command_map = {
            {CommandType::UNKNOWN, "UNKNOWN"},
            {CommandType::START, "START"},
            {CommandType::STOP, "STOP"},
            {CommandType::RESTART, "RESTART"},
            {CommandType::STATUS, "STATUS"},
            {CommandType::CONFIGURE, "CONFIGURE"},
            {CommandType::PAUSE, "PAUSE"},
            {CommandType::RESUME, "RESUME"},
            {CommandType::EMERGENCY_STOP, "EMERGENCY_STOP"}
        };
        
        auto it = command_map.find(type);
        return it != command_map.end() ? it->second : "UNKNOWN";
    }
    
    /**
     * @brief 字符串转命令类型
     * @param str 字符串
     * @return CommandType 命令类型
     */
    static CommandType fromCommandString(const std::string& str) {
        static const std::map<std::string, CommandType> command_map = {
            {"UNKNOWN", CommandType::UNKNOWN},
            {"START", CommandType::START},
            {"STOP", CommandType::STOP},
            {"RESTART", CommandType::RESTART},
            {"STATUS", CommandType::STATUS},
            {"CONFIGURE", CommandType::CONFIGURE},
            {"PAUSE", CommandType::PAUSE},
            {"RESUME", CommandType::RESUME},
            {"EMERGENCY_STOP", CommandType::EMERGENCY_STOP}
        };
        
        auto it = command_map.find(str);
        return it != command_map.end() ? it->second : CommandType::UNKNOWN;
    }
    
    /**
     * @brief 项目类型转字符串
     * @param type 项目类型
     * @return std::string 字符串表示
     */
    static std::string toString(ProjectType type) {
        static const std::map<ProjectType, std::string> project_map = {
            {ProjectType::UNKNOWN, "UNKNOWN"},
            {ProjectType::DDS_COMMUNICATION, "dds_comm"},
            {ProjectType::HAND_EYE_CALIB, "hand_eye_calib"},
            {ProjectType::CAMERA, "camera"},
            {ProjectType::PCL2LASERSCAN, "pcl2laser"},
            {ProjectType::YOLO_DET, "detection"},
            {ProjectType::NAVIGATION, "navigation"},
            {ProjectType::SLAM, "slam"},
            {ProjectType::MOTION_CONTROL, "motion_control"},
            {ProjectType::SENSOR_FUSION, "sensor_fusion"}
        };
        
        auto it = project_map.find(type);
        return it != project_map.end() ? it->second : "UNKNOWN";
    }
    
    /**
     * @brief 字符串转项目类型
     * @param str 字符串
     * @return ProjectType 项目类型
     */
    static ProjectType fromProjectString(const std::string& str) {
        static const std::map<std::string, ProjectType> project_map = {
            {"UNKNOWN", ProjectType::UNKNOWN},
            {"dds_comm", ProjectType::DDS_COMMUNICATION},
            {"hand_eye_calib", ProjectType::HAND_EYE_CALIB},
            {"camera", ProjectType::CAMERA},
            {"pcl2laser", ProjectType::PCL2LASERSCAN},
            {"detection", ProjectType::YOLO_DET},
            {"navigation", ProjectType::NAVIGATION},
            {"slam", ProjectType::SLAM},
            {"motion_control", ProjectType::MOTION_CONTROL},
            {"sensor_fusion", ProjectType::SENSOR_FUSION}
        };
        
        auto it = project_map.find(str);
        return it != project_map.end() ? it->second : ProjectType::UNKNOWN;
    }
    
    /**
     * @brief 验证命令类型字符串是否有效
     * @param str 命令类型字符串
     * @return bool 是否有效
     */
    static bool isValidCommandString(const std::string& str) {
        return fromCommandString(str) != CommandType::UNKNOWN;
    }
    
    /**
     * @brief 验证项目类型字符串是否有效
     * @param str 项目类型字符串
     * @return bool 是否有效
     */
    static bool isValidProjectString(const std::string& str) {
        return fromProjectString(str) != ProjectType::UNKNOWN;
    }
    
    /**
     * @brief 获取所有支持的命令类型字符串
     * @return std::vector<std::string> 命令类型字符串列表
     */
    static std::vector<std::string> getAllCommandStrings() {
        return {
            "START", "STOP", "RESTART", "STATUS", "CONFIGURE",
            "PAUSE", "RESUME", "EMERGENCY_STOP"
        };
    }
    
    /**
     * @brief 获取所有支持的项目类型字符串
     * @return std::vector<std::string> 项目类型字符串列表
     */
    static std::vector<std::string> getAllProjectStrings() {
        return {
            "dds_comm", "hand_eye_calib", "camera", "pcl2laser",
            "detection", "navigation", "slam", "motion_control", "sensor_fusion"
        };
    }
    
    /**
     * @brief 获取项目日志名称
     * @param project_type 项目类型
     * @return std::string 项目日志名称
     */
    static std::string getProjectLogName(ProjectType project_type) {
        return toString(project_type);
    }
};

/**
 * @brief 获取项目日志名称
 * @param project_type 项目类型
 * @return std::string 项目日志名称
 */
inline std::string getProjectLogName(ProjectType project_type) {
    return CommandTypeConverter::toString(project_type);
}

} // namespace dds_comm

#endif // DDS_COMMUNICATION__COMMAND_TYPES_HPP_