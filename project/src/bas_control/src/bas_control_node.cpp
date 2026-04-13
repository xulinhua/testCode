/**
 * @file bas_control_node.cpp
 * @brief 统一部署控制层ROS节点主程序
 * 
 * 该节点作为视觉系统的中央控制台，负责：
 * 1. 系统启动/停止管理：提供ROS服务接口启动/停止整个视觉系统
 * 2. 状态监控和资源管理：实时发布系统状态和资源使用情况
 * 3. 任务调度和场景切换：根据任务需求动态切换场景，激活/停用相应模块
 * 4. 配置热更新和模型管理：支持运行时配置和模型文件更新
 * 5. 模块状态订阅：订阅所有模块的状态话题，实时监控各模块运行状态
 * 
 * 主要ROS接口：
 * 1. 服务接口：
 *    - /bas/start_system (std_srvs/srv/Trigger) - 启动系统
 *    - /bas/stop_system (std_srvs/srv/Trigger) - 停止系统
 *    - /bas/get_status (std_srvs/srv/Trigger) - 获取系统状态
 *    - /bas/switch_scene (custom_msgs_comm/srv/SwitchScene) - 切换场景
 *    - /bas/get_scenes (std_srvs/srv/Trigger) - 获取可用场景列表
 *    - /bas/health_check (std_srvs/srv/SetBool) - 健康检查
 * 
 * 2. 话题接口：
 *    - /bas/system_status (std_msgs/msg/String) - 系统状态发布（1Hz）
 *    - /bas/system_info (std_msgs/msg/String) - 系统信息发布（0.2Hz）
 * 
 * 3. 模块状态订阅：
 *    - 按相机区分的模块话题格式：/cam{cam_id}/{module_name}/mdl_status_info
 *    - 系统级模块话题格式：/{module_name}/mdl_status_info
 * 
 * 使用方式：
 * 1. 启动节点：ros2 run bas_control bas_control_node
 * 2. 通过ROS服务控制系统启动/停止
 * 3. 通过场景切换服务调整视觉功能配置
 * 4. 订阅状态话题监控系统运行状况
 * 
 * 依赖组件：
 * - SystemMgr：系统管理器，协调各组件工作
 * - LaunchMgr：启动管理器，管理模块生命周期
 * - StatusMonitor：状态监控器，监控系统和资源状态
 * - TaskScheduler：任务调度器，管理场景切换
 * - ConfigHotUpdater：配置热更新器，支持运行时更新
 */

#include <memory>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "custom_msgs_comm/srv/switch_scene.hpp"
#include <filesystem>
#include <regex>
#include <yaml-cpp/yaml.h>

#include "bas_control/system_mgr.hpp"
#include "bas_control/module_info.hpp"
#include "bas_operate_ros/module_status.hpp"
#include "bas_operate/file_operate.hpp"
#include "log_system/log_macros.hpp"

using namespace bas_control;

/**
 * @brief 模块状态订阅器信息结构体
 */
struct ModuleStatusSubscription {
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription; ///< ROS订阅器
    std::string module_name;       ///< 模块名称
    int cam_id;                    ///< 相机ID（-1表示系统级模块）
    std::string topic_name;        ///< 话题名称
};

class BasControlNode : public rclcpp::Node {
public:
    explicit BasControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bas_control_node", options), 
          sys_cam_num_(1),
          status_refresh_interval_ms_(1000)
    {
        
        LOG_INFO("正在初始化控制节点...");
        
        // 获取参数
        std::string config_file;
        std::string package_share_directory;
        try {
            package_share_directory = ament_index_cpp::get_package_share_directory("bas_control");
        } catch (const std::exception& e) {
            LOG_ERROR("无法获取bas_control包的共享目录: %s", e.what());
            throw;
        }
        
        std::string default_config_file = package_share_directory + "/config/bas_control.yaml";
        this->declare_parameter("config_file", default_config_file);
        this->get_parameter("config_file", config_file);
        
        // 加载模块状态订阅配置
        loadStatusSubscriptionConfig(config_file);
        
        // 读取系统相机个数
        loadSystemCameraCount();
        
        // 创建系统管理器
        system_mgr_ = std::make_unique<SystemMgr>(config_file);
        
        // 初始化系统
        if (!system_mgr_->initialize()) {
            LOG_ERROR("系统管理器初始化失败");
            throw std::runtime_error("系统初始化失败");
        }
        
        // 创建话题和服务
        createTopicsAndServices();
        
        // 创建模块状态订阅器
        createModuleStatusSubscriptions();
        
        // 注册回调函数
        registerCallbacks();
        
        LOG_INFO("控制节点初始化成功");
    }
    
    ~BasControlNode() 
    {
        LOG_INFO("正在关闭控制节点...");
        // 清空订阅器
        module_status_subscriptions_.clear();
        if (system_mgr_) {
            system_mgr_->stop();
        }
    }
    
    /**
     * @brief 启动系统
     */
    bool startSystem() 
    {
        LOG_INFO("正在启动视觉系统...");
        
        if (!system_mgr_->start()) {
            LOG_ERROR("视觉系统启动失败");
            return false;
        }
        
        system_running_ = true;
        LOG_INFO("视觉系统启动成功");
        return true;
    }
    
    /**
     * @brief 停止系统
     */
    bool stopSystem() 
    {
        LOG_INFO("正在停止视觉系统...");
        
        if (!system_mgr_->stop()) {
            LOG_ERROR("视觉系统停止失败");
            return false;
        }
        
        system_running_ = false;
        LOG_INFO("视觉系统停止成功");
        return true;
    }

private:
    /**
     * @brief 加载模块状态订阅配置
     * @param config_file 配置文件路径
     */
    void loadStatusSubscriptionConfig(const std::string& config_file)
    {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
                
            if (config["module_status_subscription"]) {
                const auto& sub_config = config["module_status_subscription"];
                    
                // 读取状态刷新周期
                status_refresh_interval_ms_ = basmodule::get_param_from_yaml<int>(
                    sub_config, "status_refresh_interval_ms", 1000);
                LOG_INFO("状态刷新周期：%d ms", status_refresh_interval_ms_);
                    
                // 读取相机配置文件路径
                sys_cam_config_path_ = basmodule::get_param_from_yaml<std::string>(
                    sub_config, "sys_cam_config_path", "bas_config/bas_config_data/cam_config/sys_cam_config.yaml");
                LOG_INFO("系统相机配置路径：%s", sys_cam_config_path_.c_str());
                    
            } else {
                LOG_WARN("配置文件中未找到 module_status_subscription 配置项，使用默认配置");
            }
        } catch (const std::exception& e) {
            LOG_WARN("加载模块状态订阅配置失败：%s，使用默认配置", e.what());
        }
    }
    
    /**
     * @brief 初始化默认模块列表
     */
    void initDefaultModuleLists()
    {
        camera_based_modules_ = {
            "yolo_det",
            "face_det",
            "hand_gesture_rec",
            "ppocr",
            "yolo_obb_det",
            "marker_detect_ros",
            "hand_eye_calib_ros"
        };

        system_level_modules_ = {
            "bas_sys_config_ros",
            "cam_mgr_ros"
        };

        LOG_INFO("按相机区分的模块列表: 共 %zu 个", camera_based_modules_.size());
        LOG_INFO("系统级模块列表: 共 %zu 个", system_level_modules_.size());
    }

    /**
     * @brief 读取系统相机个数
     */
    void loadSystemCameraCount()
    {
        LOG_INFO("===== 开始读取系统相机个数 =====");
        
        // 获取 install 目录路径
        std::string install_path = basmodule::get_install_dir();
        
        // 构建相机配置文件的完整路径
        std::filesystem::path config_path(install_path);
        config_path = config_path / sys_cam_config_path_;
        
        // 规范化路径
        std::filesystem::path normalized_path = std::filesystem::weakly_canonical(config_path);
        std::string full_config_path = normalized_path.string();
        
        LOG_INFO("相机配置文件路径：%s", full_config_path.c_str());
        
        // 检查文件是否存在
        if (!std::filesystem::exists(full_config_path)) {
            LOG_ERROR("相机配置文件不存在：%s", full_config_path.c_str());
            LOG_WARN("使用默认相机个数：3");
            sys_cam_num_ = 3;
            return;
        }
        
        try {
            // 直接读取 YAML 配置文件获取相机个数
            YAML::Node config = YAML::LoadFile(full_config_path);
            
            // 从 sys_cam_config 节点读取 cam_num
            if (config["sys_cam_config"] && config["sys_cam_config"]["cam_num"]) {
                sys_cam_num_ = config["sys_cam_config"]["cam_num"].as<int>();
                LOG_INFO("✅ 系统相机个数：%d", sys_cam_num_);
            } else {
                LOG_WARN("配置文件中未找到 cam_num 字段，使用默认相机个数：3");
                sys_cam_num_ = 3;
            }
            
        } catch (const std::exception& e) {
            LOG_ERROR("读取系统相机个数失败：%s，使用默认值：1", e.what());
            sys_cam_num_ = 1;
        }
        
        initDefaultModuleLists();
        
        LOG_INFO("===== 完成读取系统相机个数 =====");
    }
    
    /**
     * @brief 创建模块状态订阅器
     */
    void createModuleStatusSubscriptions()
    {
        LOG_INFO("===== 开始创建模块状态订阅器 =====");
        
        int total_subscriptions = 0;
        
        // 为按相机区分的模块创建订阅器
        for (const auto& module_name : camera_based_modules_) {
            for (int cam_id = 0; cam_id < sys_cam_num_; ++cam_id) {
                std::string topic_name = "/cam" + std::to_string(cam_id) + "/" + module_name + "/mdl_status_info";
                createSingleSubscription(module_name, cam_id, topic_name);
                total_subscriptions++;
            }
        }
        
        // 为系统级模块创建订阅器
        for (const auto& module_name : system_level_modules_) 
        {
            std::string topic_name = "/" + module_name + "/mdl_status_info";
            createSingleSubscription(module_name, -1, topic_name);
            total_subscriptions++;
        }
        
        LOG_INFO("✅ 已创建 %d 个模块状态订阅器", total_subscriptions);
        LOG_INFO("===== 完成创建模块状态订阅器 =====");
    }
    
    /**
     * @brief 创建单个模块状态订阅器
     * @param module_name 模块名称
     * @param cam_id 相机ID（-1表示系统级模块）
     * @param topic_name 话题名称
     */
    void createSingleSubscription(const std::string& module_name, int cam_id, const std::string& topic_name)
    {
        ModuleStatusSubscription sub_info;
        sub_info.module_name = module_name;
        sub_info.cam_id = cam_id;
        sub_info.topic_name = topic_name;
        
        // 创建订阅器
        sub_info.subscription = this->create_subscription<std_msgs::msg::String>(
            topic_name,
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this, module_name, cam_id, topic_name](const std_msgs::msg::String::SharedPtr msg) {
                this->moduleStatusCallback(msg, module_name, cam_id, topic_name);
            });
        
        module_status_subscriptions_.push_back(std::move(sub_info));
        
        LOG_INFO("创建订阅器: %s (模块: %s, 相机ID: %d)", topic_name.c_str(), module_name.c_str(), cam_id);
    }
    
    /**
     * @brief 模块状态回调函数
     * @param msg 状态消息
     * @param module_name 模块名称
     * @param cam_id 相机ID
     * @param topic_name 话题名称
     */
    void moduleStatusCallback(const std_msgs::msg::String::SharedPtr msg, 
        const std::string& module_name, int cam_id, const std::string& topic_name)
    {
        try 
        {
            // 从JSON字符串解析状态信息
            basros::ModuleStatusInfo status_info = basros::jsonToModuleStatusInfo(msg->data);
            
            // 更新状态管理器
            if (system_mgr_) {
                system_mgr_->updateModuleCamStatus(module_name, cam_id, status_info);
            }
            
            LOG_DEBUG("收到模块状态: 模块=%s, 相机ID=%d, 状态=%s, 消息=%s", 
                     module_name.c_str(), cam_id, 
                     status_info.getStatusString().c_str(),
                     status_info.status_msg.c_str());
            
        } catch (const std::exception& e) {
            LOG_WARN("解析模块状态消息失败: %s, 话题: %s", e.what(), topic_name.c_str());
        }
    }
    
    /**
     * @brief 创建话题和服务接口
     */
    void createTopicsAndServices() 
    {
        //系统状态话题
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("/bas/system_status", 10);
        
        //系统信息话题
        info_publisher_ = this->create_publisher<std_msgs::msg::String>("/bas/system_info", 10);
        
        //系统状态服务
        status_service_ = this->create_service<std_srvs::srv::Trigger>("/bas/get_status",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                this->getStatusService(request, response);
            });
        
        //启动系统服务
        start_service_ = this->create_service<std_srvs::srv::Trigger>("/bas/start_system",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                this->startSystemService(request, response);
            });
        
        //停止系统服务
        stop_service_ = this->create_service<std_srvs::srv::Trigger>("/bas/stop_system",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                this->stopSystemService(request, response);
            });
        
        //场景切换服务
        scene_service_ = this->create_service<custom_msgs_comm::srv::SwitchScene>("/bas/switch_scene",
            [this](const std::shared_ptr<custom_msgs_comm::srv::SwitchScene::Request> request,
                   std::shared_ptr<custom_msgs_comm::srv::SwitchScene::Response> response) {
                this->switchSceneService(request, response);
            });
        
        //获取场景列表服务
        get_scenes_service_ = this->create_service<std_srvs::srv::Trigger>("/bas/get_scenes",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                this->getScenesService(request, response);
            });
        
        //系统健康检查服务
        health_service_ = this->create_service<std_srvs::srv::SetBool>("/bas/health_check",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                this->healthCheckService(request, response);
            });
        
        //定时发布状态
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(status_refresh_interval_ms_),
            [this]() {
                this->publishStatus();
            });
        
        //定时发布系统信息
        info_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000), // 5秒
            [this]() {
                this->publishSystemInfo();
            });
    }
    
    /**
     * @brief 注册回调函数
     */
    void registerCallbacks() 
    {
        //注册系统状态回调
        system_mgr_->registerSystemStatusCallback(
            [this](const SystemStatus& status) {
                this->onSystemStatusUpdate(status);
            });
        
        //注册场景切换回调
        system_mgr_->registerSceneCallback(
            [this](const std::string& scene_name, const std::vector<std::string>& active_modules) {
                this->onSceneSwitch(scene_name, active_modules);
            });
        
        //注册模块状态回调
        system_mgr_->registerModuleStatusCallback(
            [this](const ModuleInfo& module_info) {
                this->onModuleStatusUpdate(module_info);
            });
    }
    
    /**
     * @brief获取状态服务回调
     */
    void getStatusService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        try 
        {
            auto status = system_mgr_->getSystemStatus();
            response->success = true;
            response->message = "System Status: " + status.getStatusString() + 
                               ", Running Modules: " + std::to_string(status.getRunningModuleCount());
        } 
        catch (const std::exception& e) 
        {
            response->success = false;
            response->message = "Failed to get system status: " + std::string(e.what());
            LOG_ERROR("状态服务错误: %s", e.what());
        }
    }
    
    /**
     * @brief 启动系统服务回调
     */
    void startSystemService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try 
        {
            if (startSystem()) 
            {
                response->success = true;
                response->message = "系统启动成功";
            } 
            else
            {
                response->success = false;
                response->message = "系统启动失败";
            }
        } 
        catch (const std::exception& e) 
        {
            response->success = false;
            response->message = "系统启动异常: " + std::string(e.what());
            LOG_ERROR("启动服务错误: %s", e.what());
        }
    }
    
    /**
     * @brief停止系统服务回调
     */
    void stopSystemService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        try 
        {
            if (stopSystem()) 
            {
                response->success = true;
                response->message = "系统停止成功，即将退出进程";
                LOG_INFO("系统已停止，将在500ms后退出进程...");
                
                // 延迟退出进程，确保ROS响应成功发送
                std::thread exit_thread([this]() {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    LOG_INFO("退出 bas_control_node 进程");
                    rclcpp::shutdown();  // 触发进程退出
                });
                exit_thread.detach();
            } 
            else 
            {
                response->success = false;
                response->message = "系统停止失败";
            }
        } 
        catch (const std::exception& e) 
        {
            response->success = false;
            response->message = "系统停止异常: " + std::string(e.what());
            LOG_ERROR("停止服务错误: %s", e.what());
        }
    }
    
    /**
     * @brief 场景切换服务回调
     */
    void switchSceneService(const std::shared_ptr<custom_msgs_comm::srv::SwitchScene::Request> request, 
        std::shared_ptr<custom_msgs_comm::srv::SwitchScene::Response> response) 
    {
        try 
        {
            std::string scene_name = request->scene_name;
            if (scene_name.empty()) 
            {
                response->success = false;
                response->message = "场景名称为空";
                return;
            }
            
            // 将字符串转换为SceneType
            SceneType scene_type = stringToSceneType(scene_name);
            if (system_mgr_->switchScene(scene_type)) 
            {
                response->success = true;
                response->message = "场景已切换到: " + scene_name;
            } 
            else 
            {
                response->success = false;
                response->message = "场景切换失败: " + scene_name;
            }
        } 
        catch (const std::exception& e) 
        {
            response->success = false;
            response->message = "场景切换异常: " + std::string(e.what());
            LOG_ERROR("场景服务错误: %s", e.what());
        }
    }
    
    /**
     * @brief 获取场景列表服务回调
     */
    void getScenesService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        try 
        {
            auto scenes = system_mgr_->getAvailableScenes();
            std::string scene_list = "Available scenes: ";
            for (const auto& scene : scenes) {
                scene_list += sceneTypeToString(scene) + " ";
            }
            response->success = true;
            response->message = scene_list;
        } 
        catch (const std::exception& e) 
        {
            response->success = false;
            response->message = "获取场景失败: " + std::string(e.what());
            LOG_ERROR("获取场景服务错误: %s", e.what());
        }
    }
    
    /**
     * @brief 健康检查服务回调
     */
    void healthCheckService(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
    {
        try 
        {
            bool healthy = system_mgr_->isSystemHealthy();
            response->success = true;
            response->message = healthy ? "系统健康" : "系统存在问题";
        } 
        catch (const std::exception& e) 
        {
            response->success = false;
            response->message = "健康检查失败: " + std::string(e.what());
            LOG_ERROR("健康检查服务错误: %s", e.what());
        }
    }
    
    /**
     * @brief 发布系统状态
     */
    void publishStatus() 
    {
        try 
        {
            auto status = system_mgr_->getSystemStatus();
            auto msg = std::make_unique<std_msgs::msg::String>();
            msg->data = "Status: " + status.getStatusString() + 
                       ", Scene: " + sceneTypeToString(system_mgr_->getCurrentScene()) +
                       ", Modules: " + std::to_string(status.getRunningModuleCount());
            status_publisher_->publish(std::move(msg));
        } catch (const std::exception& e) {
            LOG_ERROR("发布状态失败: %s", e.what());
        }
    }
    
    /**
     * @brief 发布系统信息
     */
    void publishSystemInfo() 
    {
        try 
        {
            auto info = system_mgr_->getSystemInfo();
            auto msg = std::make_unique<std_msgs::msg::String>();
            std::string info_str = "System Info: ";
            for (const auto& pair : info) {
                info_str += pair.first + "=" + pair.second + " ";
            }
            msg->data = info_str;
            info_publisher_->publish(std::move(msg));
        } catch (const std::exception& e) {
            LOG_ERROR("发布系统信息失败: %s", e.what());
        }
    }
    
    /**
     * @brief 系统状态更新回调
     */
    void onSystemStatusUpdate(const SystemStatus& status) 
    {
        LOG_DEBUG("系统状态已更新: %s", status.getStatusString().c_str());
    }
    
    /**
     * @brief 场景切换回调
     */
    void onSceneSwitch(const std::string& scene_name, const std::vector<std::string>& active_modules) 
    {
        LOG_INFO("场景已切换到: %s, 活动模块数: %zu", scene_name.c_str(), active_modules.size());
    }
    
    /**
     * @brief 模块状态更新回调
     */
    void onModuleStatusUpdate(const ModuleInfo& module_info) 
    {
        LOG_DEBUG("模块 %s 状态: %s", module_info.name.c_str(), module_info.getStatusString().c_str());
    }

private:
    // 系统管理器
    std::unique_ptr<SystemMgr> system_mgr_;               ///< 系统管理器
    bool system_running_ = false;                         ///< 系统运行状态

    // 相机配置
    int sys_cam_num_;                                     ///< 系统相机个数
    std::string sys_cam_config_path_;                     ///< 系统相机配置文件路径

    // 模块状态订阅配置
    int status_refresh_interval_ms_;                      ///< 状态刷新周期（毫秒）
    std::vector<std::string> camera_based_modules_;       ///< 按相机区分的模块列表
    std::vector<std::string> system_level_modules_;       ///< 系统级模块列表
    std::vector<ModuleStatusSubscription> module_status_subscriptions_; ///< 模块状态订阅器列表

    // 话题发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;  ///< 状态发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_publisher_;     ///< 信息发布器

    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;     ///< 状态服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;       ///< 启动服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;        ///< 停止服务
    rclcpp::Service<custom_msgs_comm::srv::SwitchScene>::SharedPtr scene_service_;       ///< 场景服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_scenes_service_;   ///< 获取场景服务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr health_service_;      ///< 健康检查服务

    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;               ///< 状态定时器
    rclcpp::TimerBase::SharedPtr info_timer_;                 ///< 信息定时器
};

// 信号处理函数
void signalHandler(int signal) 
{
    rclcpp::shutdown();
}

int main(int argc, char* argv[]) 
{
    LOG_INIT("bas_control");
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 初始化ROS
    rclcpp::init(argc, argv);
    
    try 
    {
        // 创建节点
        auto node = std::make_shared<BasControlNode>();
        
        // 启动系统（默认场景已在TaskScheduler初始化时设置）
        node->startSystem();
        
        // 运行节点
        rclcpp::spin(node);
        
        // 停止系统
        node->stopSystem();
        
    } catch (const std::exception& e) {
        LOG_ERROR("节点初始化失败: %s", e.what());
        return 1;
    }
    
    // 清理
    rclcpp::shutdown();
    return 0;
}