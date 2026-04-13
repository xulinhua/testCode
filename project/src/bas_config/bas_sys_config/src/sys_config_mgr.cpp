/**
 * @file sys_config_mgr.cpp
 * @brief 系统配置管理器类实现
 * 
 * 实现了从配置文件中读取相机配置信息的功能
 */
#include "bas_sys_config/sys_config_mgr.h"
#include "bas_sys_config/sys_config_struct.hpp"
#include "bas_sys_config/config_reflector.hpp"
#include <iostream>
#include <sstream>
#include <filesystem>
#include "log_system/log_macros.hpp"
#include "hand_eye_calib/sys_calib_data_mgr.hpp"// 包含手眼标定管理器头文件
#include "hand_eye_calib/calib_struct.hpp"  // 包含CamCalibInfoList定义
#include "hand_eye_calib/calib_reflector.hpp"  // 包含printLog_CamCalibInfoList函数声明
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bas_operate/bas_utils.hpp"  // 包含bas_utils工具函数
#include "bas_operate/file_operate.hpp"
#include "data_handler/yaml_operate.h"  // 包含YAML操作函数

namespace SysConfig
{
// 静态成员初始化
SysConfigMgr* SysConfigMgr::instance_ = nullptr;
std::mutex SysConfigMgr::mutex_;

SysConfigMgr::SysConfigMgr()
: sys_config_path_(""), cam_num_(0), arm_num_(0), enable_cam_num_(0), enable_arm_num_(0), initialized_(false), sys_calib_mgr_(nullptr)
{
  sys_cam_info_list_.clear();
  sys_arm_info_list_.clear();
  enable_sys_cam_list_.clear();
  enable_sys_arm_list_.clear();

  // 初始化手眼标定管理器单例实例
  sys_calib_mgr_ = &handeyecalib::SysCalibMgr::getInstance();
}

SysConfigMgr::~SysConfigMgr()
{
}

SysConfigMgr& SysConfigMgr::getInstance()
{
  // 双重检查锁定模式实现线程安全的单例
  if (instance_ == nullptr) 
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
      instance_ = new SysConfigMgr();
    }
  }
  return *instance_;
}

bool SysConfigMgr::loadSysConfigData(const std::string& sys_config_path)
{
  sys_config_path_ = sys_config_path;
  LOG_INFO("===== 开始加载系统配置: %s =====", sys_config_path.c_str());
  // 使用反射器模式加载系统配置
  try 
  {
    // 加载机械臂配置，要在加载相机配置之前加载，因为相机配置中需要使用机械臂的配置信息
    bool arm_success = loadSysArmConfig(sys_config_path + "/sys_arm_config.yaml");
    if (!arm_success) 
    {
      LOG_ERROR("❌ 加载机械臂配置失败");
      return false;
    }
    // 加载相机配置
    bool cam_success = loadSysCamConfig(sys_config_path + "/sys_cam_config.yaml");
    if (!cam_success) 
    {
      LOG_ERROR("❌ 加载相机配置失败");
      return false;
    }
    LOG_DEBUG("开始根据机械臂启用状态过滤相机配置中的机械臂信息...");
    filterDisabledArmInfo(enable_sys_cam_list_);// 根据机械臂启用状态过滤相机配置中的机械臂信息
    LOG_DEBUG("过滤完成");
    if (cam_success && arm_success) // 如果配置加载成功，则加载相机标定数据
    {
      LOG_INFO("系统配置加载成功，开始加载相机标定数据...");
      std::vector<uint8_t> enabled_cam_ids;// 收集启用的相机ID列表
      for (const auto& cam_info : enable_sys_cam_list_) 
      {
        enabled_cam_ids.push_back(cam_info.cam_id);
        LOG_DEBUG("添加启用的相机ID到标定数据加载列表: %d", static_cast<int>(cam_info.cam_id));
      }
      // 构造bas_config_data路径
      // bas_config_data路径应该在sys_config_path的上一级目录
      std::string bas_config_data_path = sys_config_path_;
      size_t pos = bas_config_data_path.find_last_of("/\\");
      if (pos != std::string::npos) {
        bas_config_data_path = bas_config_data_path.substr(0, pos);
      }
          
      // 确保路径末尾没有多余的斜杠
      while (!bas_config_data_path.empty() && 
             (bas_config_data_path.back() == '/' || bas_config_data_path.back() == '\\')) {
        bas_config_data_path.pop_back();
      } 
      LOG_INFO("准备加载系统启用的相机标定数据，bas_config_data路径: %s", bas_config_data_path.c_str());
      // 构建相机ID和机械臂ID映射表
      std::map<uint8_t, std::vector<uint8_t>> cam_arm_id_list;
      for (const auto& cam_info : enable_sys_cam_list_) 
      {
        std::vector<uint8_t> arm_ids;
        for (const auto& arm_info : cam_info.armInfoList) 
        {
          arm_ids.push_back(arm_info.arm_id);
        }
        cam_arm_id_list[cam_info.cam_id] = arm_ids;
      }
      // 调用加载相机标定数据列表的接口
      if (loadCamCalibInfoList(bas_config_data_path, cam_arm_id_list)) {
        LOG_INFO("✅ 相机标定数据加载成功");
      } else {
        LOG_WARN("⚠️ 相机标定数据加载失败");
      }
    } else {
      LOG_WARN("系统配置加载失败，跳过相机标定数据加载");
    }
    initialized_ = true;
    LOG_INFO("===== ✅ 系统配置加载完成 ===== \n");
    return true;
  } catch (const std::exception& e) {
    LOG_ERROR("加载系统配置时发生异常: %s", e.what());
    return false;
  }
}

bool SysConfigMgr::loadSysArmConfig(const std::string& sys_config_path)
{
  LOG_INFO("===== 开始加载系统机械臂配置: %s =====", sys_config_path.c_str());
  YAML::Node config;
  if (!loadConfigFile(sys_config_path, config))// 加载配置文件
  {
    LOG_ERROR("❌ 加载系统机械臂配置文件失败");
    return false;
  }

  try 
  {
    // 读取机械臂数量
    const YAML::Node& sys_arm_config = config["sys_arm_config"];
    if (!sys_arm_config) {
      LOG_ERROR("❌ 未找到 sys_arm_config 配置项");
      return false;
    }
    
    arm_num_ = sys_arm_config["arm_num"].as<uint8_t>();
    LOG_DEBUG("系统机械臂数量: %d", static_cast<int>(arm_num_));

    // 检查机械臂数量是否在有效范围内
    if (arm_num_ < MIN_SYS_ARM_NUM || arm_num_ > MAX_SYS_ARM_NUM) 
    {
      LOG_ERROR("机械臂数量 %d 超出有效范围 [%d, %d]", static_cast<int>(arm_num_), static_cast<int>(MIN_SYS_ARM_NUM), static_cast<int>(MAX_SYS_ARM_NUM));
      return false;
    }
    logsys::Level log_level = logsys::Level::INFO;
    logsys::Color color = logsys::Color::BLUE;
    sys_arm_info_list_.clear();// 清空机械臂配置列表
    enable_sys_arm_list_.clear(); 
    enable_arm_num_ = 0;// 启用的机械臂数量计数器
    for (uint8_t i = 0; i < arm_num_; i++) // 读取各个机械臂的配置信息
    {
      LOG_DEBUG("");// 添加空行，便于区分查看
      LOG_DEBUG("===== 开始读取机械臂 %d 配置信息 =====", static_cast<int>(i));
      std::string arm_key = "arm_" + std::to_string(i);
      const YAML::Node& arm_config = config[arm_key];
      if (!arm_config) 
      {
        LOG_WARN("未找到机械臂配置: %s", arm_key.c_str());
        LOG_DEBUG("===== 结束读取机械臂arm_id= %d 配置信息 =====", static_cast<int>(i));
        continue;
      }
      SysConfig::ArmConfigInfo arm_info;
      arm_info.Init(); // 初始化机械臂配置信息对象并设置默认值
      SysConfig::ArmConfigInfoReflector reflector(arm_info);// 使用反射器模式读取配置参数
      arm_info.arm_id = i;// 设置机械臂ID
      LOG_DEBUG("读取机械臂配置，ID: %d", static_cast<int>(arm_info.arm_id));
      if (LOG_ON(log_level))
      {
        LOG_OUT(log_level, "加载前机械臂配置参数列表及默认值:");
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
        reflector.printLog_saved_params(project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
      }
      const auto& params_saved = reflector.getParamsSaved(); // 只加载需要从文件读取的参数
      for (const auto& param_info : params_saved) 
      {
          datahandler::loadParamInfoFromYaml<datahandler::ParamInfo, datahandler::ParamType>(arm_config, param_info);// 使用反射器自动加载需要从文件读取的参数值
      }
      reflector.updateParamAftLoad();//在加载配置文件后更新参数
      if (LOG_ON(log_level))
      {
        LOG_OUT(log_level, "加载后机械臂配置参数列表及实际值:");
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
        reflector.printLog_saved_params(project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        reflector.printLog(project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
      }
      sys_arm_info_list_.push_back(arm_info);// 将机械臂配置信息添加到系统机械臂配置列表中
      if (arm_info.is_enable) // 如果机械臂启用，则添加到启用的机械臂配置列表中
      {
        enable_sys_arm_list_.push_back(arm_info);
        enable_arm_num_++;
      }
      LOG_INFO("===== 结束读取机械臂arm_id= %d 配置信息 ===== \n", static_cast<int>(i));
    }
    if (LOG_ON(logsys::Level::INFO))
    {
      std::vector<uint8_t> enable_arm_ids;// 收集启用的机械臂ID列表
      std::vector<uint8_t> disable_arm_ids;// 收集禁用的机械臂ID列表
      for (const auto& arm_info : sys_arm_info_list_) 
      {
        if (arm_info.is_enable) {
          enable_arm_ids.push_back(arm_info.arm_id);
        } else {
          disable_arm_ids.push_back(arm_info.arm_id);
        }
      }
      std::string enable_arm_str = basmodule::get_list_string(enable_arm_ids);// 输出启用的机械臂ID列表
      std::string disable_arm_str = basmodule::get_list_string(disable_arm_ids);// 输出禁用的机械臂ID列表
      LOG_INFO("===== ✅ 系统机械臂配置加载完成，共加载 %d 个机械臂配置，启用 %d 个 %s，禁用 %d 个 %s ===== \n", 
        static_cast<int>(sys_arm_info_list_.size()), static_cast<int>(enable_arm_ids.size()), 
        enable_arm_str.c_str(), static_cast<int>(disable_arm_ids.size()), disable_arm_str.c_str());
    }
    return true;
  } catch (const YAML::Exception& e) {
    LOG_ERROR("❌ 解析系统机械臂配置文件时出错: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    LOG_ERROR("❌ 加载系统机械臂配置时发生异常: %s", e.what());
    return false;
  }
}

bool SysConfigMgr::loadSysCamConfig(const std::string& sys_config_path)
{
  LOG_INFO("=====开始加载系统相机配置: %s=====", sys_config_path.c_str());
  YAML::Node config;
  if (!loadConfigFile(sys_config_path, config))// 加载配置文件
  {
    LOG_ERROR("❌ 加载系统相机配置文件失败");
    return false;
  }
  try 
  {
    
    const YAML::Node& sys_cam_config = config["sys_cam_config"];
    if (!sys_cam_config) {
      LOG_ERROR("❌ 未找到 sys_cam_config 配置项");
      return false;
    }
    cam_num_ = sys_cam_config["cam_num"].as<uint8_t>();// 读取相机数量
    LOG_DEBUG("系统相机数量: %d", static_cast<int>(cam_num_));
    if (cam_num_ < MIN_SYS_CAM_NUM || cam_num_ > MAX_SYS_CAM_NUM) // 检查相机数量是否在有效范围内 
    {
      LOG_ERROR("相机数量 %d 超出有效范围 [%d, %d]", static_cast<int>(cam_num_), static_cast<int>(MIN_SYS_CAM_NUM), static_cast<int>(MAX_SYS_CAM_NUM));
      return false;
    }
    // 清空相机配置列表
    sys_cam_info_list_.clear();
    enable_sys_cam_list_.clear();
    enable_cam_num_ = 0;// 启用的相机数量计数器
    for (uint8_t i = 0; i < cam_num_; i++) // 读取各个相机的配置信息
    {
      LOG_DEBUG("");// 添加空行，便于区分查看
      LOG_DEBUG("===== 开始读取相机 %d 配置信息 =====", static_cast<int>(i));
      std::string cam_key = "cam_" + std::to_string(i);
      const YAML::Node& cam_config = config[cam_key];
      if (!cam_config) 
      {
        LOG_WARN("未找到相机配置: %s", cam_key.c_str());
        LOG_DEBUG("===== 结束读取相机 %d 配置信息 =====", static_cast<int>(i));
        continue;
      }
      SysConfig::CamConfigInfo cam_info;
      cam_info.Init(); // 初始化相机配置信息对象并设置默认值
      SysConfig::CamConfigInfoReflector reflector(cam_info);// 使用反射器模式读取配置参数
      cam_info.cam_id = i;// 设置相机ID
      LOG_DEBUG("读取相机配置，ID: %d", static_cast<int>(cam_info.cam_id));
      logsys::Level log_level = logsys::Level::INFO;
      logsys::Color color = logsys::Color::BLUE;
      if (LOG_ON(log_level))
      {
        LOG_OUT(log_level, "加载前相机配置参数列表及默认值:");
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
        reflector.printLog_saved_params(project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
      }
      // 只加载需要从文件读取的参数
      const auto& params_saved = reflector.getParamsSaved(); 
      std::vector<uint8_t> arm_ids;
      for (const auto& param_info : params_saved) 
      {
          datahandler::loadParamInfoFromYaml<datahandler::ParamInfo, datahandler::ParamType>(cam_config, param_info);// 使用反射器自动加载需要从文件读取的参数值
          if (param_info.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_COLOR_RESOLUTION)){
            cam_info.default_color_resolution = datahandler::stringToColorResolution(std::any_cast<std::string>(param_info.value));
          } else if (param_info.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::DEFAULT_DEPTH_RESOLUTION)){
            cam_info.default_depth_resolution = datahandler::stringToDepthResolution(std::any_cast<std::string>(param_info.value));
          } else if (param_info.name == CamConfigInfo::getParamNameString(CamConfigInfo::ParamName::ARM_IDS)) 
          {
              arm_ids = std::any_cast<std::vector<uint8_t>>(param_info.value);
              cam_info.armInfoList = SysConfig::getArmInfoListByIds(arm_ids, sys_arm_info_list_);// 根据arm_ids获取完整的机械臂信息列表
              arm_ids = SysConfig::getArmIds(cam_info.armInfoList);
          }
      }
      reflector.updateParamAftLoad();//在加载配置文件后更新参数
      if (LOG_ON(log_level))
      {
        LOG_OUT(log_level, "加载后相机配置参数列表及实际值:");
        const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
        reflector.printLog_saved_params(project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        reflector.printLog(project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
      }
      std::string arm_ids_str = basmodule::get_list_string(arm_ids);// 输出当前相机管理的所有机械臂ID
      LOG_DEBUG("关联的机械臂ID: %s", arm_ids_str.c_str());
      sys_cam_info_list_.push_back(cam_info);// 将相机配置信息添加到系统相机配置列表中
      if (cam_info.is_enable) // 如果相机启用，则添加到启用的相机配置列表中
      {
        enable_sys_cam_list_.push_back(cam_info);
        enable_cam_num_++;
      }
      LOG_INFO("===== 结束读取相机 %d 配置信息 =====", static_cast<int>(i));
    }
    if (LOG_ON(logsys::Level::INFO))
    {
      std::vector<uint8_t> enable_cam_ids;// 收集启用的相机ID列表
      std::vector<uint8_t> disable_cam_ids;// 收集禁用的相机ID列表
      for (const auto& cam_info : sys_cam_info_list_) 
      {
        if (cam_info.is_enable) {
          enable_cam_ids.push_back(cam_info.cam_id);
        } else {
          disable_cam_ids.push_back(cam_info.cam_id);
        }
      }
      std::string enable_cam_str = basmodule::get_list_string(enable_cam_ids); // 启用的相机ID列表
      std::string disable_cam_str = basmodule::get_list_string(disable_cam_ids); // 禁用的相机ID列表
      LOG_INFO("✅ 系统相机配置加载完成，共加载 %d 个相机配置，启用 %d 个 %s，禁用 %d 个 %s", 
        static_cast<int>(sys_cam_info_list_.size()), static_cast<int>(enable_cam_ids.size()), 
        enable_cam_str.c_str(), static_cast<int>(disable_cam_ids.size()), disable_cam_str.c_str());
    }
    return true;
  } catch (const YAML::Exception& e) {
    LOG_ERROR("❌ 解析系统相机配置文件时出错: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    LOG_ERROR("❌ 加载系统相机配置时发生异常: %s", e.what());
    return false;
  }
}

bool SysConfigMgr::filterDisabledArmInfo(SysConfig::CamConfigInfo1D& cam_info_list)
{
  LOG_INFO("=====开始过滤禁用的机械臂信息，相机数量: %d=====", static_cast<int>(cam_info_list.size()));
  try 
  {
    for (auto& cam_info : cam_info_list) // 遍历所有相机配置信息
    {
      std::vector<uint8_t> before_filter_ids;// 获取过滤前的机械臂ID列表
      for (const auto& arm_info : cam_info.armInfoList) {
        before_filter_ids.push_back(arm_info.arm_id);
      }
      std::string before_filter_str = basmodule::get_list_string(before_filter_ids);
      LOG_DEBUG("处理相机 %d 的机械臂信息，过滤前机械臂ID列表: %s, 机械臂数量: %d", 
        static_cast<int>(cam_info.cam_id), before_filter_str.c_str(), static_cast<int>(cam_info.armInfoList.size()));
      std::vector<uint8_t> filtered_arm_ids;
      SysConfig::ArmConfigInfoList filtered_arm_list;// 创建新的机械臂信息列表，只保留启用的机械臂
      std::vector<uint8_t> removed_arm_ids;
      for (const auto& arm_info : cam_info.armInfoList) 
      {
        if (arm_info.is_enable) 
        {
          filtered_arm_ids.push_back(arm_info.arm_id);
          filtered_arm_list.push_back(arm_info);
        } 
        else {
          removed_arm_ids.push_back(arm_info.arm_id);
        }
      }
      std::string filtered_arm_ids_str = basmodule::get_list_string(filtered_arm_ids);
      std::string removed_arm_ids_str = basmodule::get_list_string(removed_arm_ids);
      LOG_DEBUG("保留启用的机械臂数量: %d %s, 过滤禁用的机械臂数量: %d %s", 
        static_cast<int>(filtered_arm_ids.size()), filtered_arm_ids_str.c_str(), static_cast<int>(removed_arm_ids.size()), removed_arm_ids_str.c_str());
      cam_info.armInfoList = filtered_arm_list;// 更新相机配置中的机械臂信息列表
      LOG_DEBUG("相机 %d 过滤后机械臂ID列表: %s, 机械臂数量: %d %s", 
        static_cast<int>(cam_info.cam_id), filtered_arm_ids_str.c_str(), static_cast<int>(cam_info.armInfoList.size()), filtered_arm_ids_str.c_str());
    }
    LOG_INFO("=====✅ 机械臂信息过滤完成===== \n");
    return true;
  } catch (const std::exception& e) {
    LOG_ERROR("❌ 过滤机械臂信息时出错: %s", e.what());
    return false;
  }
}

bool SysConfigMgr::loadConfigFile(const std::string& sys_config_path, YAML::Node& config)
{
  try 
  {
    if (!std::filesystem::exists(sys_config_path)) // 检查文件是否存在
    {
      LOG_ERROR("配置文件不存在: %s", sys_config_path.c_str());
      return false;
    }
    config = YAML::LoadFile(sys_config_path);
    return true;
  } catch (const YAML::Exception & e) {
    LOG_ERROR("加载配置文件出错: %s, 异常: %s", sys_config_path.c_str(), e.what());
    return false;
  }
}

bool SysConfigMgr::loadCamCalibInfoList(const std::string& bas_config_data_path, const std::map<uint8_t, std::vector<uint8_t>>& cam_arm_id_list)
{
  if (cam_arm_id_list.empty()) // 检查相机ID和机械臂ID映射表是否为空
  {
    LOG_ERROR("相机ID和机械臂ID映射表为空");
    return false;
  }
  std::string cam_arm_mapping_info = "准备加载系统启用的相机ID和机械臂ID映射表:\n";// 打印要加载的相机ID和机械臂ID映射信息
  if (!cam_arm_id_list.empty()) 
  {
    for (const auto& cam_arm_pair : cam_arm_id_list) 
    {
      uint8_t cam_id = cam_arm_pair.first;
      const std::vector<uint8_t>& arm_id_list = cam_arm_pair.second;
      std::string arm_ids_str = basmodule::get_list_string(arm_id_list);
      char buffer[256];
      snprintf(buffer, sizeof(buffer), "  相机cam_id=%2d: %s\n", static_cast<int>(cam_id), arm_ids_str.c_str());
      cam_arm_mapping_info += buffer;
    }
    LOG_DEBUG("%s", cam_arm_mapping_info.c_str());
  } else {
    cam_arm_mapping_info += "机械臂ID映射表为空";
    LOG_WARN("%s", cam_arm_mapping_info.c_str());
  }
  for (const auto& cam_arm_pair : cam_arm_id_list) // 检查相机ID映射表中的每个ID是否超出范围 
  {
    uint8_t cam_id = cam_arm_pair.first;
    if (cam_id >= cam_num_) 
    {
      LOG_ERROR("相机ID %d 超出系统相机个数 %d", static_cast<int>(cam_id), static_cast<int>(cam_num_));
      return false;
    }
  }
  if (sys_calib_mgr_ == nullptr) // 检查手眼标定管理器是否有效
  {
    LOG_ERROR("手眼标定管理器未初始化");
    return false;
  }
  try 
  {
    sys_calib_mgr_->setSysCamNum(cam_num_);// 设置系统相机个数到手眼标定管理器
    // 调用手眼标定管理器的加载接口加载指定相机列表下指定机械臂的标定数据
    if (sys_calib_mgr_->loadCamCalibInfoList(bas_config_data_path, cam_arm_id_list)) 
    {
      LOG_INFO("成功加载指定相机列表下指定机械臂的手眼标定数据");
      if (0)
      {
        handeyecalib::CamCalibInfoList cam_calib_list = sys_calib_mgr_->getCamCalibDataList();// 获取当前系统所有启用相机的手眼标定数据
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "📊 加载后相机手眼标定数据信息如下:");
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CamCalibInfoList(cam_calib_list, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);// 打印CamCalibInfoList的函数
        }
      }
      return true;
    } else {
      LOG_WARN("加载指定相机列表下指定机械臂的手眼标定数据失败");
      return false;
    }
  } catch (const std::exception& e) {
    LOG_ERROR("加载指定相机列表下指定机械臂的手眼标定数据时发生异常: %s", e.what());
    return false;
  }
}

handeyecalib::CamCalibInfo SysConfigMgr::getCamCalibData(uint8_t cam_id) const
{
  // 检查手眼标定管理器是否有效
  if (sys_calib_mgr_ == nullptr) 
  {
    LOG_ERROR("手眼标定管理器未初始化");
    return handeyecalib::CamCalibInfo(); // 返回默认构造的相机标定信息
  }
  if (cam_id >= cam_num_) // 检查相机ID是否在有效范围内
  {
    LOG_ERROR("相机ID %d 超出系统相机个数 %d", static_cast<int>(cam_id), static_cast<int>(cam_num_));
    return handeyecalib::CamCalibInfo(); // 返回默认构造的相机标定信息
  }
  try 
  {
    handeyecalib::CamCalibInfo cam_calib_info = sys_calib_mgr_->getCamCalibData(cam_id);// 从手眼标定管理器获取相机标定数据
    LOG_INFO("成功获取相机 %d 的标定数据", static_cast<int>(cam_id));
    return cam_calib_info;
  } catch (const std::exception& e) {
    LOG_ERROR("获取相机 %d 的标定数据时发生异常: %s", static_cast<int>(cam_id), e.what());
    return handeyecalib::CamCalibInfo(); // 返回默认构造的相机标定信息
  }
}

handeyecalib::CamCalibInfoList SysConfigMgr::getCamCalibDataList() const
{
  if (sys_calib_mgr_ == nullptr) // 检查手眼标定管理器是否有效
  {
    LOG_ERROR("手眼标定管理器未初始化");
    return handeyecalib::CamCalibInfoList(); // 返回空的相机标定信息列表
  }
  try 
  {
    handeyecalib::CamCalibInfoList cam_calib_list = sys_calib_mgr_->getCamCalibDataList();
    LOG_INFO("成功获取所有相机的标定数据，相机数量: %d", static_cast<int>(cam_calib_list.size()));
    return cam_calib_list;
  } catch (const std::exception& e) {
    LOG_ERROR("获取所有相机的标定数据时发生异常: %s", e.what());
    return handeyecalib::CamCalibInfoList(); // 返回空的相机标定信息列表
  }
}

uint8_t SysConfigMgr::getSysCamNum() const
{
  return cam_num_;
}

uint8_t SysConfigMgr::getSysArmNum() const
{
  return arm_num_;
}

uint8_t SysConfigMgr::getSysEnableCamNum() const
{
  return enable_cam_num_;
}

uint8_t SysConfigMgr::getSysEnableArmNum() const
{
  return enable_arm_num_;
}

SysConfig::CamConfigInfo1D SysConfigMgr::getSysEnableCamInfoList() const
{
  return enable_sys_cam_list_;
}

SysConfig::ArmConfigInfoList SysConfigMgr::getSysEnableArmInfoList() const
{
  return enable_sys_arm_list_;
}

}  // namespace SysConfig