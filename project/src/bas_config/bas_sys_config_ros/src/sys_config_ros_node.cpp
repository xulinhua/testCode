/**
 * @file sys_config_ros_node.cpp
 * @brief 系统配置ROS节点主程序
 * 
 * 该节点负责发布系统配置信息，并响应其他模块的服务请求
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "bas_sys_config/sys_config_mgr.h"
#include "sys_info_src/sys_info_server.h"
#include "bas_operate_ros/param_from_server.hpp"
#include "bas_operate_ros/param_utils.hpp"
// 添加自定义服务消息头文件
#include "custom_msgs_comm/srv/get_comm_info.hpp"
#include "custom_msgs_comm/msg/ros_comm_info.hpp"
#include "std_msgs/msg/u_int32.hpp"
// 添加log_system头文件
#include "log_system/log_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bas_operate/file_operate.hpp"
// 添加文件系统头文件用于路径规范化
#include <filesystem>
#include "bas_sys_config/sys_config_struct.hpp"
// 添加手眼标定管理器头文件
#include "hand_eye_calib/sys_calib_data_mgr.hpp"
#include "hand_eye_calib/calib_reflector.hpp"
// 添加相机标定信息服务器头文件
#include "calib_info_src/calib_info_server.h"
// 添加头部电机角度订阅所需头文件
#include "std_msgs/msg/float64_multi_array.hpp"
#include "hand_eye_calib/calib_utils.hpp"
// 添加模块状态发布所需头文件
#include "bas_operate_ros/module_status.hpp"
#include "std_msgs/msg/string.hpp"
using namespace RosComm;

class SysConfigNode : public rclcpp::Node
{
private:
  // 头部电机角度相关变量
  std::vector<double> current_head_angles_;           ///< 当前头部电机角度 [pitch, yaw]
  std::mutex head_angles_mutex_;                      ///< 头部角度互斥锁
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_motor_angles_sub_;  ///< 头部电机角度订阅器
  std::map<std::pair<uint8_t, uint8_t>, std::vector<double>> cam_arm_original_head_angles_;  ///< 每个相机-机械臂对的原始标定头部角度 [cam_id, arm_id] -> [pitch, yaw]
  
  // 模块状态发布相关变量
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_status_pub_;  ///< 模块状态发布器
  std::string module_name_;                           ///< 本模块名称
  basros::ModuleStatus current_status_;               ///< 当前模块状态
  std::mutex status_mutex_;                           ///< 状态互斥锁
public:
  SysConfigNode()
  : Node("sys_config_ros_node"), module_name_("bas_sys_config_ros"), current_status_(basros::ModuleStatus::UNKNOWN)
  {
 
    // 获取系统配置管理器单例实例
    sys_config_mgr_ = &SysConfig::SysConfigMgr::getInstance();
    
    // 初始化头部电机角度变量
    current_head_angles_.resize(2, 0.0);  // 默认为0
    
    // 初始化模块状态发布器
    initModuleStatusPublisher();
    
    // 发布启动中状态
    publishModuleStatus(basros::ModuleStatus::STARTING, "模块正在启动");
    
    // 获取install目录下的系统配置路径
    std::string install_path = basmodule::get_install_dir();
    // 打印当前install_path信息用于调试
    LOG_DEBUG("当前install_path: %s", install_path.c_str());
    // 修正配置文件路径，确保指向bas_sys_config包而不是当前包
    // 在ROS2环境中，所有包共享同一个install目录，所以我们需要向上回溯一层然后进入bas_sys_config
    std::string sys_config_path = install_path + "/bas_config_data/cam_config";
    // 规范化路径
    std::filesystem::path normalized_path(sys_config_path);
    sys_config_path = std::filesystem::weakly_canonical(normalized_path).string();
    
    LOG_DEBUG("系统配置路径: %s", sys_config_path.c_str());
    if (!sys_config_mgr_->loadSysConfigData(sys_config_path)) // 加载系统配置
    {
        LOG_ERROR("加载系统配置失败");
        publishModuleStatus(basros::ModuleStatus::ERROR, "加载系统配置失败");
        throw std::runtime_error("加载系统配置失败");
    }
    
    // 创建获取通信信息服务的服务端
    get_comm_info_service_ = this->create_service<custom_msgs_comm::srv::GetCommInfo>(
      "bas_sys_config_ros/get_comm_info",
      [this](const std::shared_ptr<custom_msgs_comm::srv::GetCommInfo::Request> request,
             std::shared_ptr<custom_msgs_comm::srv::GetCommInfo::Response> response) {
        responseGetCommInfo(request, response);
      });
    
    // 初始化每个相机-机械臂对的原始头部角度
    initializeOriginalHeadAngles();
    
    // 初始化头部电机角度订阅器
    initHeadMotorAngleSubscriber();
    
    LOG_INFO("系统配置节点初始化成功");
    
    // 发布运行中状态
    publishModuleStatus(basros::ModuleStatus::RUNNING, "模块初始化完成，正常运行中");
  }

  void setSysConfigDatToServer()
  {
    LOG_INFO("开始将系统配置数据设置到参数服务器");
    auto sys_cam_list = sys_config_mgr_->getSysEnableCamInfoList();// 获取启用的相机信息列表
    auto sys_arm_list = sys_config_mgr_->getSysEnableArmInfoList();// 获取启用的机械臂信息列表
    LOG_DEBUG("准备设置设备参数: %d个相机, %d个机械臂", static_cast<int>(sys_cam_list.size()), static_cast<int>(sys_arm_list.size())); 
    // 使用参数转换工具函数将设备信息列表转换为参数格式并直接设置到参数服务器
    bool arm_params_set = RosComm::setArmInfoListToServer(shared_from_this(), SYS_ENABLE_ARM_LIST, sys_arm_list);
    bool cam_params_set = RosComm::setCamInfoListToServer(shared_from_this(), SYS_ENABLE_CAM_LIST, sys_cam_list);
    if (!arm_params_set) // 检查机械臂参数是否设置成功
    {
        LOG_ERROR("设置机械臂参数失败");
        publishModuleStatus(basros::ModuleStatus::ERROR, "设置机械臂参数失败");
    }
    if (!cam_params_set) // 检查相机参数是否设置成功
    {
        LOG_ERROR("设置相机参数失败");
        publishModuleStatus(basros::ModuleStatus::ERROR, "设置相机参数失败");
    }
    if (arm_params_set && cam_params_set) // 判断机械臂参数和相机参数是否都设置成功
    {
        LOG_INFO("设备参数设置成功，开始设置手眼标定数据");
        handeyecalib::CamCalibInfoList cam_calib_list = sys_config_mgr_->getCamCalibDataList();// 获取当前系统所有启用相机的手眼标定数据
        logsys::Level log_level = logsys::Level::INFO;
        logsys::Color color = logsys::Color::BLUE;
        if (LOG_ON(log_level))
        {
            LOG_OUT(log_level, "准备设置手眼标定数据，共%d个相机", static_cast<int>(cam_calib_list.size()));
            const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);  
            printLog_CamCalibInfoList(cam_calib_list, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
        }
        bool calib_params_set = RosComm::setCamCalibInfoListToServer(shared_from_this(), cam_calib_list, SYS_CAM_CALIB_LIST);// 将手眼标定数据写入参数服务器
        if (calib_params_set) 
        {
            LOG_INFO("成功将手眼标定数据写入参数服务器，共%d个相机", static_cast<int>(cam_calib_list.size()));
            for (const auto& cam_calib_info : cam_calib_list) // 打印每个相机的标定信息
            {
                LOG_DEBUG("相机ID: %d, 关联机械臂数量: %zu", static_cast<int>(cam_calib_info.cam_id), cam_calib_info.arm_calib1D.size());
                for (const auto& arm_pair : cam_calib_info.arm_calib1D) 
                {
                    const handeyecalib::ArmCalibInfo& arm_calib_info = arm_pair.second;
                    LOG_DEBUG("  机械臂ID: %d, 标定方法: %s, 时间戳: %s", static_cast<int>(arm_calib_info.arm_id), 
                      arm_calib_info.calib_info.calib_method.c_str(), arm_calib_info.calib_info.timestamp.c_str());
                }
            }
        } else {
            LOG_ERROR("写入手眼标定数据到参数服务器失败");
            publishModuleStatus(basros::ModuleStatus::ERROR, "写入手眼标定数据失败");
        }
    } else {
        LOG_WARN("由于设备参数设置失败，跳过手眼标定数据写入");
    }    
    syncToParameterServer(shared_from_this(), *sys_config_mgr_);// 同步SysConfigMgr内部状态到参数服务器
    LOG_INFO("已完成将启用设备参数设置到参数服务器: %d个相机, %d个机械臂", static_cast<int>(sys_cam_list.size()), static_cast<int>(sys_arm_list.size()));
  }  
  /**
   * @brief 处理获取通信信息服务的请求
   * 
   * @param request 服务请求
   * @param response 服务响应
   */
  void responseGetCommInfo(const std::shared_ptr<custom_msgs_comm::srv::GetCommInfo::Request> request,
    std::shared_ptr<custom_msgs_comm::srv::GetCommInfo::Response> response)
  {
    LOG_DEBUG("收到获取通信信息服务请求");
    
    // 将请求中的参数转换为内部枚举类型
    basros::RosCommMsgType msg_type = static_cast<basros::RosCommMsgType>(request->msg_type);
    uint8_t cam_id = request->cam_id;
    uint8_t arm_id = request->arm_id;
    
    // 使用解析函数生成通信信息
    basros::RosCommInfo comm_info = basros::parseCommInfo(msg_type, cam_id, arm_id);
    
    // 填充响应消息
    //response->comm_info.comm_method = static_cast<uint8_t>(comm_info.comm_method);
    response->comm_info.cam_id = comm_info.cam_id;
    response->comm_info.arm_id = static_cast<uint8_t>(comm_info.arm_id);
    response->comm_info.name = comm_info.name;
    
    LOG_DEBUG("返回通信信息: %s", comm_info.name.c_str());
  }
  
  // 在节点完全构造后调用此方法来设置设备参数
  /**
   * @brief 将SysConfigMgr的内部状态同步到参数服务器
   * @param node ROS节点指针，用于访问参数服务器
   * @param config_mgr 配置管理器实例
   */
  void syncToParameterServer(rclcpp::Node::SharedPtr node, const SysConfig::SysConfigMgr& config_mgr)
  {
    // 同步初始化完成状态
    if (node->has_parameter("bas_sys_config_ros.init_completed")) {
      node->set_parameter(rclcpp::Parameter("bas_sys_config_ros.init_completed", config_mgr.isInitialized()));
    } else {
      node->declare_parameter("bas_sys_config_ros.init_completed", config_mgr.isInitialized());
    }
    LOG_DEBUG("已将SysConfigMgr内部状态同步到参数服务器: initialized_=%s", config_mgr.isInitialized() ? "true" : "false");
  }
  
  /**
   * @brief 初始化每个相机-机械臂对的原始头部角度
   */
  void initializeOriginalHeadAngles() 
  {
    LOG_DEBUG("开始初始化每个相机-机械臂对的原始头部角度");
    // 获取系统中所有启用的相机的手眼标定数据
    handeyecalib::CamCalibInfoList cam_calib_list = sys_config_mgr_->getCamCalibDataList();
    for (const auto& cam_calib_info : cam_calib_list) // 遍历每个相机的标定数据
    {
        for (const auto& arm_pair : cam_calib_info.arm_calib1D) // 遍历每个相机关联的机械臂标定数据
        {
            const handeyecalib::ArmCalibInfo& arm_calib_info = arm_pair.second;
            // 获取原始标定数据中的头部电机角度（基准角度）
            std::vector<double> original_head_angles = arm_calib_info.calib_info.calib_res.head_motor_angles;
            // 如果原始头部角度数据为空，使用默认值[0, 0]
            if (original_head_angles.size() < 2) {
                original_head_angles.resize(2, 0.0);
            }
            // 存储相机-机械臂对的原始头部角度
            std::pair<uint8_t, uint8_t> key = std::make_pair(cam_calib_info.cam_id, arm_calib_info.arm_id);
            cam_arm_original_head_angles_[key] = original_head_angles; 
            LOG_DEBUG("已初始化相机ID %d 机械臂ID %d 的原始头部角度: ", static_cast<int>(cam_calib_info.cam_id), static_cast<int>(arm_calib_info.arm_id));
            LOG_DEBUG("俯仰角: %.2f°, 偏航角: %.2f°", original_head_angles[0], original_head_angles[1], false);
        }
    }
    LOG_DEBUG("完成初始化每个相机-机械臂对的原始头部角度");
  }
  
  /**
   * @brief 初始化头部电机角度订阅器
   */
  void initHeadMotorAngleSubscriber() 
  {
    // 订阅头部电机角度话题
    std::string head_motor_angle_topic = "/head_motor_angles";  // 可以通过参数配置
    head_motor_angles_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        head_motor_angle_topic, 10,
        std::bind(&SysConfigNode::headMotorAngleCallback, this, std::placeholders::_1));
    LOG_DEBUG("头部电机角度订阅器初始化完成，话题: %s", head_motor_angle_topic.c_str());
  }
  
  /**
   * @brief 头部电机角度回调函数
   * @param msg 头部电机角度消息
   */
  void headMotorAngleCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
  {
    std::lock_guard<std::mutex> lock(head_angles_mutex_);
    
    // 检查数组大小是否足够
    if (msg->data.size() < 2) {
        LOG_WARN("头部电机角度数据不足，需要2个值，实际收到%zu个", msg->data.size());
        return;
    }
    
    // 检查角度数据是否与上一次相同，如果相同则不更新
    if (current_head_angles_.size() >= 2 && 
        std::abs(current_head_angles_[0] - msg->data[0]) < 1e-6 && 
        std::abs(current_head_angles_[1] - msg->data[1]) < 1e-6) {
        LOG_DEBUG("头部电机角度未变化，跳过更新 - 俯仰角: %.2f°, 偏航角: %.2f°", msg->data[0], msg->data[1]);
        return;
    }
    
    // 更新当前头部电机角度
    current_head_angles_.resize(2);
    current_head_angles_[0] = msg->data[0];  // 俯仰角
    current_head_angles_[1] = msg->data[1];  // 偏航角
    
    LOG_DEBUG("接收到头部电机角度 - 俯仰角: %.2f°, 偏航角: %.2f°", current_head_angles_[0], current_head_angles_[1]);
    
    // 计算动态标定矩阵并更新到参数服务器
    updateDynamicCalibrationMatrices();
  }
  
  /**
   * @brief 计算并更新动态标定矩阵到参数服务器
   */
  void updateDynamicCalibrationMatrices()
  {
    LOG_DEBUG("开始计算动态标定矩阵");
    
    // 从系统配置管理器获取最新的原始标定数据
    handeyecalib::CamCalibInfoList cam_calib_list = sys_config_mgr_->getCamCalibDataList();
    
    // 遍历每个相机的标定数据
    for (auto& cam_calib_info : cam_calib_list) 
    {
        // 遍历每个相机关联的机械臂标定数据
        for (auto& arm_pair : cam_calib_info.arm_calib1D) 
        {
            handeyecalib::ArmCalibInfo& arm_calib_info = arm_pair.second;
            
            // 获取该相机-机械臂对的原始标定头部角度
            std::pair<uint8_t, uint8_t> key = std::make_pair(cam_calib_info.cam_id, arm_calib_info.arm_id);
            auto it = cam_arm_original_head_angles_.find(key);
            
            std::vector<double> original_head_angles;
            if (it != cam_arm_original_head_angles_.end()) {
                original_head_angles = it->second;
            } else {
                // 如果找不到原始角度，使用标定结果中的head_motor_angles
                original_head_angles = arm_calib_info.calib_info.calib_res.head_motor_angles;
                // 如果仍然为空，使用默认值[0, 0]
                if (original_head_angles.size() < 2) {
                    original_head_angles.resize(2, 0.0);
                }
                
                // 同时更新映射，确保后续使用一致
                cam_arm_original_head_angles_[key] = original_head_angles;
                
                LOG_WARN("未找到相机ID %d 机械臂ID %d 的原始头部角度，使用标定结果中的值", static_cast<int>(cam_calib_info.cam_id), static_cast<int>(arm_calib_info.arm_id));
            }
            
            // 计算相对于原始标定基准角度的差异
            double pitch_diff = current_head_angles_[0] - original_head_angles[0];
            double yaw_diff = current_head_angles_[1] - original_head_angles[1];
            
            LOG_DEBUG("计算相对姿态变换 - 相对俯仰角: %.2f°, 相对偏航角: %.2f° (当前: [%.2f°, %.2f°], 原始基准: [%.2f°, %.2f°])", 
              pitch_diff, yaw_diff, current_head_angles_[0], current_head_angles_[1], original_head_angles[0], original_head_angles[1]);
            
            // 使用原始标定数据和相对于基准角度的差异计算动态标定矩阵
            handeyecalib::CalibRes dynamic_calib_res = handeyecalib::computeDynamicCalibration(
                arm_calib_info.calib_info.calib_res, 
                pitch_diff, 
                yaw_diff);
            
            // 更新动态标定结果到arm_calib_info
            arm_calib_info.calib_info.calib_res = dynamic_calib_res;
            LOG_DEBUG("已更新相机ID %d 机械臂ID %d 的动态标定矩阵", static_cast<int>(cam_calib_info.cam_id), static_cast<int>(arm_calib_info.arm_id));
        }
    }
    
    // 将更新后的标定数据写入参数服务器
    bool calib_params_set = RosComm::setCamCalibInfoListToServer(shared_from_this(), cam_calib_list, SYS_CAM_CALIB_LIST);
    if (calib_params_set) {
        LOG_INFO("动态标定数据已成功写入参数服务器");
    } else {
        LOG_ERROR("写入动态标定数据到参数服务器失败");
        publishModuleStatus(basros::ModuleStatus::ERROR, "写入动态标定数据失败");
    }
  }
  
  /**
   * @brief 初始化模块状态发布器
   * 
   * 按照统一的话题命名规范: /{module_name}/mdl_status_info
   * bas_sys_config_ros是系统级模块，cam_id为-1
   */
  void initModuleStatusPublisher()
  {
    // 话题名称格式: /{module_name}/mdl_status_info (系统级模块)
    std::string topic_name = "/" + module_name_ + "/mdl_status_info";
    
    // 创建发布器，使用可靠QoS确保消息送达
    module_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    
    LOG_INFO("模块状态发布器初始化完成，话题: %s", topic_name.c_str());
  }
  
  /**
   * @brief 发布模块状态
   * @param status 模块状态
   * @param status_msg 状态信息文本
   */
  void publishModuleStatus(basros::ModuleStatus status, const std::string& status_msg)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 更新当前状态
    current_status_ = status;
    
    // 构建ModuleStatusInfo结构体
    basros::ModuleStatusInfo status_info(module_name_, -1, status, status_msg);
    
    // 序列化为JSON字符串
    std::string json_str = basros::moduleStatusInfoToJson(status_info);
    
    // 创建并发布消息
    auto msg = std_msgs::msg::String();
    msg.data = json_str;
    module_status_pub_->publish(msg);
    
    LOG_INFO("发布模块状态: %s - %s", basros::moduleStatusToString(status).c_str(), status_msg.c_str());
  }
  
  SysConfig::SysConfigMgr* sys_config_mgr_;
  rclcpp::Service<custom_msgs_comm::srv::GetCommInfo>::SharedPtr get_comm_info_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<SysConfigNode>();
    LOG_INFO("sys_config_ros_node节点启动");
    node->setSysConfigDatToServer();
    LOG_INFO("已完成参数服务器数据设置，开始运行节点");
    rclcpp::spin(node);
    
    // 正常退出时发布停止状态
    node->publishModuleStatus(basros::ModuleStatus::STOPPED, "模块正常停止");
  } 
  catch (const std::exception& e) 
  {
    LOG_ERROR("sys_config_ros_node异常退出: %s", e.what());
    // 异常时无法通过node发布状态，因为node可能未创建成功
    rclcpp::shutdown();
    return 1;
  }
  catch (...) 
  {
    LOG_ERROR("sys_config_ros_node发生未知异常");
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}