#ifndef SYS_CONFIG_STRUCT
#define SYS_CONFIG_STRUCT

#include <vector>
#include <string>
#include <map>
#include <opencv2/opencv.hpp>

// 宏定义移到命名空间外部
#define SYS_ENABLE_CAM_LIST "sys_enable_cam_list"
#define SYS_ENABLE_ARM_LIST "sys_enable_arm_list"
#define SYS_ARM_LIST        "sys_arm_list"
#define SYS_CAM_CALIB_LIST  "sys_cam_calib_list"

namespace SysConfig
{
 /* @brief 机器人手臂类型枚举
 * 
 * 定义系统中可用的机器人手臂类型及其ID
 */
enum class RobotArmType : uint8_t
{
  LEFT_ARM = 0,   ///< 左臂
  RIGHT_ARM = 1,  ///< 右臂
  ASSIT_ARM = 2   ///< 辅助手臂
};

// 彩色图像分辨率枚举类
enum class ColorResolution : uint8_t
{
    RGB_NATIVE = 0,         // 彩色传感器的原生最高分辨率
    RGB_1920x1080,          // 例如，1080P
    RGB_1280x720,           // 例如，720P
    RGB_640x480,            // 例如，480P
    // ... 其他彩色相机支持的分辨率
};

// 深度图像分辨率枚举类
enum class DepthResolution : uint8_t
{
    DEPTH_NATIVE = 0,       // 深度传感器的原生最高分辨率
    DEPTH_1280x720,         // 例如，Orbbec Gemini 2的高分辨率深度模式
    DEPTH_640x480,          // 最常用的深度分辨率
    DEPTH_320x240,          // 低分辨率，用于高速或低带宽场景
    // ... 其他深度相机支持的分辨率
};

// （可选）模式枚举类，用于高级设置
enum class AlignmentMode : uint8_t
{
    ALIGN_DISABLE = 0,      // 不对齐
    ALIGN_DEPTH_TO_COLOR,   // 将深度图对齐到彩色图坐标系[1,5](@ref)
    ALIGN_COLOR_TO_DEPTH,   // 将彩色图对齐到深度图坐标系[1](@ref)
};

// 系统中的机械臂配置信息类
class ArmConfigInfo
{
public:
    // 机械臂配置参数名称枚举
    enum class ParamName : uint8_t {
        ARM_ID = 0,
        IS_ENABLE,
        ROBOT_ARM_IP,
        USER_NAME, 
        PARA_MAX
    };

    bool is_enable;               // 是否启用
    uint8_t arm_id;               // 机械臂ID
    std::string robot_arm_ip;     // 机械臂IP地址
    std::string user_name;        // 用户名称
    
    // 构造函数
    ArmConfigInfo();
    ArmConfigInfo(const ArmConfigInfo& para);
    ArmConfigInfo& operator = (const ArmConfigInfo& para);
    virtual ~ArmConfigInfo();
    
    // 初始化函数
    void Init();
    void Rst();
    
    // 拷贝函数
    void CopyFrom(const ArmConfigInfo& para);  // 从para拷贝数据
    void CopyTo(ArmConfigInfo& para) const;    // 拷贝数据到para

    static const char* getParamNameString(ParamName paramName);  // 获取参数名称字符串
    static ParamName getParamNameEnum(const std::string& paramNameStr); // 根据参数名称字符串获取ParamName枚举值
};

typedef std::vector<ArmConfigInfo> ArmConfigInfoList;

// 系统中的相机配置信息类
class CamConfigInfo
{
public:
    // 相机配置参数名称枚举
    enum class ParamName : uint8_t {
        IS_ENABLE = 0,
        CAM_ID,
        SERIAL_NUMBER,
        USER_NAME,
        DEFAULT_COLOR_RESOLUTION,
        DEFAULT_DEPTH_RESOLUTION,
        ARM_IDS, 
        PARA_MAX
    };

    static const char* getParamNameString(ParamName paramName);// 获取参数名称字符串
    static ParamName getParamNameEnum(const std::string& paramNameStr); // 根据参数名称字符串获取ParamName枚举值
    
    bool is_enable;               // 是否启用
    uint8_t cam_id;               // 相机ID
    std::string serial_number;    // 序列号
    std::string user_name;        // 用户名称
    ArmConfigInfoList armInfoList;  // 当前相机配置的机械臂信息列表
    ColorResolution default_color_resolution;  // 当前相机的默认彩色分辨率
    DepthResolution default_depth_resolution;  // 当前相机的默认深度分辨率
    // 构造函数
    CamConfigInfo();
    CamConfigInfo(const CamConfigInfo& para);
    CamConfigInfo& operator = (const CamConfigInfo& para);
    virtual ~CamConfigInfo();
    
    // 初始化函数
    void Init();
    void Rst();
    
    // 根据arm_ids列表初始化armInfoList，确保每个元素的arm_id与arm_ids列表中的ID一一对应
    void initArmInfoListByArmIds(const std::vector<uint8_t>& arm_ids);
    
    // 拷贝函数
    void CopyFrom(const CamConfigInfo& para);  // 从para拷贝数据
    void CopyTo(CamConfigInfo& para) const;    // 拷贝数据到para
};

typedef std::vector<CamConfigInfo> CamConfigInfo1D;

/**
 * @brief 从机械臂配置信息列表中提取所有机械臂ID
 * 
 * 该函数遍历机械臂配置信息列表，提取每个机械臂的ID并组成一个新的ID列表返回。
 * 主要用于参数服务器存储和日志输出等场景。
 * 
 * @param arm_list 机械臂配置信息列表
 * @return 包含所有机械臂ID的向量
 */
std::vector<uint8_t> getArmIds(const ArmConfigInfoList& arm_list);

std::vector<uint8_t> getCamIds(const CamConfigInfo1D& cam_list);

/**
 * @brief 根据机械臂ID列表获取完整的机械臂配置信息列表
 * 
 * 该函数用于解决参数服务器存储限制问题：参数服务器只存储机械臂ID列表，
 * 而实际使用时需要完整的机械臂配置信息。通过传入机械臂ID列表和系统已加载
 * 的完整机械臂配置列表，返回对应的完整机械臂信息列表。
 * 
 * @param arm_ids 机械臂ID列表（来自相机配置的arm_info字段）
 * @param full_arm_list 系统中完整的机械臂配置信息列表
 * @return 对应ID的完整机械臂配置信息列表
 */
SysConfig::ArmConfigInfoList getArmInfoListByIds(const std::vector<uint8_t>& arm_ids, const SysConfig::ArmConfigInfoList& full_arm_list);

}  // namespace SysConfig

#endif