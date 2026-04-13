#ifndef HAND_EYE_CALIB_SYS_CALIB_DATA_MGR_HPP
#define HAND_EYE_CALIB_SYS_CALIB_DATA_MGR_HPP

#include "hand_eye_calib/calib_struct.hpp"
#include <map>
#include <mutex>
#include <memory>
#include <string>

// 普通接口函数声明
namespace handeyecalib 
{

/**
 * @brief 系统标定数据管理器类
 * 负责管理整个系统中所有相机的所有机械臂下的所有标定数据
 * 采用单例模式设计，确保全局唯一实例
 */
class SysCalibMgr {
public:
    /**
     * @brief 获取单例实例
     * @return 单例实例引用
     */
    static SysCalibMgr& getInstance();

    /**
     * @brief 删除拷贝构造函数，防止拷贝
     */
    SysCalibMgr(const SysCalibMgr&) = delete;

    /**
     * @brief 删除赋值操作符，防止赋值
     */
    SysCalibMgr& operator=(const SysCalibMgr&) = delete;

    /**
     * @brief 设置系统相机个数
     * @param sys_cam_num 系统相机个数
     */
    void setSysCamNum(uint8_t sys_cam_num);

    /**
     * @brief 获取系统相机个数
     * @return 系统相机个数
     */
    uint8_t getSysCamNum() const;

    /**
     * @brief 设置指定相机的标定数据
     * @param cam_id 相机ID
     * @param cam_calib_info 相机标定信息
     */
    void setCamCalibData(uint8_t cam_id, const CamCalibInfo& cam_calib_info);

    /**
     * @brief 获取指定相机的标定数据
     * @param cam_id 相机ID
     * @return 相机标定信息
     */
    CamCalibInfo getCamCalibData(uint8_t cam_id) const;

    /**
     * @brief 获取所有相机的标定数据列表
     * @return 相机标定数据列表
     */
    CamCalibInfoList getCamCalibDataList() const;

    /**
     * @brief 检查是否存在指定相机的标定数据
     * @param cam_id 相机ID
     * @return 是否存在
     */
    bool hasCamCalibData(uint8_t cam_id) const;

    /**
     * @brief 获取指定相机下指定机械臂的标定数据
     * @param cam_id 相机ID
     * @param arm_id 机械臂ID
     * @return 机械臂标定信息
     */
    ArmCalibInfo getArmCalibData(uint8_t cam_id, uint8_t arm_id) const;

    /**
     * @brief 检查是否存在指定相机下指定机械臂的标定数据
     * @param cam_id 相机ID
     * @param arm_id 机械臂ID
     * @return 是否存在
     */
    bool hasArmCalibData(uint8_t cam_id, uint8_t arm_id) const;

    /**
     * @brief 重置指定相机的标定数据
     * @param cam_id 相机ID
     */
    void resetCamCalibData(uint8_t cam_id);

    /**
     * @brief 重置指定相机下指定机械臂的标定数据
     * @param cam_id 相机ID
     * @param arm_id 机械臂ID
     */
    void resetArmCalibData(uint8_t cam_id, uint8_t arm_id);

    /**
     * @brief 加载指定相机和机械臂的偏移补偿参数
     * @param bas_config_data_path bas_config_data路径
     * @param cam_id 相机ID
     * @param arm_id 机械臂ID
     * @return 是否加载成功
     */
    bool loadTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id);

    /**
     * @brief 修改指定相机和机械臂的偏移补偿参数
     * @param bas_config_data_path bas_config_data路径
     * @param cam_id 相机ID
     * @param arm_id 机械臂ID
     * @param offset_compensation 偏移补偿参数 [x, y, z, rx, ry, rz]
     */
    bool saveTcpOffset(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const std::vector<double>& offset_compensation);

    /**
     * @brief 加载指定相机和机械臂的标定结果
     * @param bas_config_data_path bas_config_data路径
     * @param cam_id 相机ID
     * @param arm_id 机械臂ID
     * @param calib_res 标定结果
     * @return 是否加载成功
     */
    // 加载指定相机和机械臂的标定结果
    bool loadCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, CalibRes& calib_res);

    /**
     * @brief 保存指定相机和机械臂的标定结果
     * @param bas_config_data_path bas_config_data路径
     * @param cam_id 相机ID
     * @param arm_id 机械臂ID
     * @param calib_res 标定结果
     * @return 是否保存成功
     */
    // 保存指定相机和机械臂的标定结果
    bool saveCalibRes(const std::string& bas_config_data_path, const uint8_t cam_id, const uint8_t arm_id, const CalibRes& calib_res);
        
    /**
     * @brief 保存指定相机和机械臂的手眼标定数据（新接口）
     * @param bas_config_data_path bas_config_data路径
     * @param cam_id 相机ID
     * @return 是否保存成功
     */
    bool saveArmCalibInfo(const std::string& bas_config_data_path, const uint8_t cam_id, const ArmCalibInfo& arm_calib_info);

    /**
     * @brief 加载系统中指定相机列表下的指定机械臂对应的手眼标定数据
     * @param bas_config_data_path bas_config_data路径
     * @param cam_arm_id_list 相机ID和相机ID下配置的机械臂ID映射表(key值为相机ID)
     * @return 是否加载成功
     */
    bool loadCamCalibInfoList(const std::string& bas_config_data_path, const std::map<uint8_t, std::vector<uint8_t>>& cam_arm_id_list);

    /**
     * @brief 加载某个相机下的所有机械臂对应的手眼标定数据
     * @param bas_config_data_path bas_config_data路径
     * @param cam_id 相机ID
     * @return 是否加载成功
     */
    bool loadCamCalibInfo(const std::string& bas_config_data_path, uint8_t cam_id, const std::vector<uint8_t>& arm_id_list);

private:
    /**
     * @brief 私有构造函数
     */
    SysCalibMgr() = default;

    /**
     * @brief 私有析构函数
     */
    ~SysCalibMgr() = default;

    // 使用静态局部变量实现线程安全的单例模式
    static SysCalibMgr* instance_;
    static std::mutex mutex_;

    CamCalibInfoList cam_calib_data_list_; // 相机标定数据列表
    
    // 可变互斥锁，用于读操作
    mutable std::mutex calib_data_mutex_;
    
    // 系统相机个数
    uint8_t sys_cam_num_ = 255;
};

} // namespace handeyecalib

#endif // HAND_EYE_CALIB_SYS_CALIB_DATA_MGR_HPP