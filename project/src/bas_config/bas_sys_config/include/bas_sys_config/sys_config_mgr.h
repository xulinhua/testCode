/**
 * @file sys_config_mgr.h
 * @brief 系统配置管理器类
 * 
 * 该类负责从配置文件中读取相机配置信息，并提供相关的查询接口
 */
#ifndef BAS_SYS_CONFIG__SYS_CONFIG_MGR_H_
#define BAS_SYS_CONFIG__SYS_CONFIG_MGR_H_

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "bas_sys_config/sys_config_struct.hpp"
// 添加线程安全相关的头文件
#include <mutex>
// 包含手眼标定相关结构体定义
#include "hand_eye_calib/calib_struct.hpp"

// 添加手眼标定管理器的前置声明
namespace handeyecalib {
class SysCalibMgr;
class CamCalibInfo;
}

namespace SysConfig
{
/**
 * @brief 系统配置管理器类
 * 
 * 负责从bas_sys_config目录下读取cam_config.yaml中的相机个数等配置信息
 * 实现为线程安全的单例模式
 */
class SysConfigMgr
{
public:
  // 最小系统相机个数
  static const uint8_t MIN_SYS_CAM_NUM = 1;
  
  // 最大系统相机个数
  static const uint8_t MAX_SYS_CAM_NUM = 3;
  
  // 最小系统机械臂个数
  static const uint8_t MIN_SYS_ARM_NUM = 0;
  
  // 最大系统机械臂个数
  static const uint8_t MAX_SYS_ARM_NUM = 3;

  /**
   * @brief 获取单例实例
   * @return 单例实例的引用
   */
  static SysConfigMgr& getInstance();

  /**
   * @brief 删除拷贝构造函数，防止拷贝
   */
  SysConfigMgr(const SysConfigMgr&) = delete;

  /**
   * @brief 删除赋值操作符，防止赋值
   */
  SysConfigMgr& operator=(const SysConfigMgr&) = delete;

  /**
   * @brief 加载系统配置
   * @param sys_config_path 系统配置目录路径
   * @return 是否成功加载配置
   */
  bool loadSysConfigData(const std::string& sys_config_path);

  /**
   * @brief 获取系统配置的相机数量
   * @return 系统配置的相机数量
   */
  uint8_t getSysCamNum() const;

  /**
   * @brief 获取系统配置的机械臂数量
   * @return 系统配置的机械臂数量
   */
  uint8_t getSysArmNum() const;

  /**
   * @brief 获取系统启用的相机数量
   * @return 系统启用的相机数量
   */
  uint8_t getSysEnableCamNum() const;

  /**
   * @brief 获取系统启用的机械臂数量
   * @return 系统启用的机械臂数量
   */
  uint8_t getSysEnableArmNum() const;

  /**
   * @brief 获取系统启用的相机ID列表
   * @return 系统启用的相机ID列表
   */
  SysConfig::CamConfigInfo1D getSysEnableCamInfoList() const;

  /**
   * @brief 获取系统启用的机械臂信息列表
   * @return 系统启用的机械臂信息列表
   */
  SysConfig::ArmConfigInfoList getSysEnableArmInfoList() const;

  /**
   * @brief 获取系统相机配置信息列表
   * @return 系统相机配置信息列表
   */
  const SysConfig::CamConfigInfo1D& getSysCamInfoList() const { return sys_cam_info_list_; }

  /**
   * @brief 获取系统机械臂配置信息列表
   * @return 系统机械臂配置信息列表
   */
  const SysConfig::ArmConfigInfoList& getSysArmInfoList() const { return sys_arm_info_list_; }

  /**
   * @brief 检查系统配置是否已完全初始化
   * @return 是否初始化完成
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief 加载指定相机列表下的指定机械臂的手眼标定数据
   * @param bas_config_data_path bas_config_data路径
   * @param cam_arm_id_list 相机ID和相机ID下配置的机械臂ID映射表(key值为相机ID)
   * @return 是否加载成功
   */
  bool loadCamCalibInfoList(const std::string& bas_config_data_path, const std::map<uint8_t, std::vector<uint8_t>>& cam_arm_id_list);

  /**
   * @brief 获取指定相机的手眼标定数据
   * @param cam_id 相机ID
   * @return 相机标定信息
   */
  handeyecalib::CamCalibInfo getCamCalibData(uint8_t cam_id) const;

  /**
   * @brief 获取所有启用相机的手眼标定数据
   * @return 所有启用相机的标定数据列表
   */
  handeyecalib::CamCalibInfoList getCamCalibDataList() const;

private:
  /**
   * @brief 构造函数设为私有，防止直接实例化
   */
  SysConfigMgr();

  /**
   * @brief 析构函数设为私有，防止直接销毁
   */
  ~SysConfigMgr();

  /**
   * @brief 加载系统相机配置文件
   * @param sys_config_path 相机配置文件完整路径
   * @return 是否成功加载相机配置
   */
  bool loadSysCamConfig(const std::string& sys_config_path);

  /**
   * @brief 加载系统机械臂配置文件
   * @param sys_config_path 机械臂配置文件完整路径
   * @return 是否成功加载机械臂配置
   */
  bool loadSysArmConfig(const std::string& sys_config_path);

  /**
   * @brief 检查配置文件是否存在并加载YAML节点
   * @param sys_config_path 系统配置文件路径
   * @param config YAML节点引用，用于返回加载的配置
   * @return 是否成功加载配置文件
   */
  bool loadConfigFile(const std::string& sys_config_path, YAML::Node& config);
  
  /**
   * @brief 根据机械臂启用状态过滤相机配置中的机械臂信息
   * @param cam_info_list 相机配置信息列表的引用
   * @return 是否成功过滤
   */
  bool filterDisabledArmInfo(SysConfig::CamConfigInfo1D& cam_info_list);

  std::string sys_config_path_;  ///< 系统配置文件路径
  uint8_t cam_num_;         ///< 相机数量
  uint8_t arm_num_;         ///< 机械臂数量
  uint8_t enable_cam_num_;  ///< 启用的相机数量
  uint8_t enable_arm_num_;  ///< 启用的机械臂数量
  SysConfig::CamConfigInfo1D enable_sys_cam_list_;  ///< 启用的相机配置信息列表
  SysConfig::ArmConfigInfoList enable_sys_arm_list_;  ///< 启用的机械臂配置信息列表
  SysConfig::CamConfigInfo1D sys_cam_info_list_;  ///< 系统相机配置信息列表
  SysConfig::ArmConfigInfoList sys_arm_info_list_;  ///< 系统机械臂配置信息列表
  
  // 线程安全相关的静态成员
  static SysConfigMgr* instance_;
  static std::mutex mutex_;
  
  bool initialized_;  ///< 标识配置是否已完全初始化
  
  // 手眼标定管理器指针
  handeyecalib::SysCalibMgr* sys_calib_mgr_;
};

}  // namespace SysConfig

#endif  // BAS_SYS_CONFIG__SYS_CONFIG_MGR_H_