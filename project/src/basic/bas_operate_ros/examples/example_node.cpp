#include <rclcpp/rclcpp.hpp>
#include "data_handler/param_reflector.hpp"
#include "bas_operate_ros/param_to_server.hpp"
#include "bas_operate_ros/param_from_server.hpp"
#include "bas_operate_ros/param_utils.hpp"
#include "log_system/log_macros.hpp"
#include "bas_operate/file_operate.hpp"

// 示例配置类，继承自ConfigReflector
using namespace basros;
class ExampleConfig : public datahandler::ConfigReflector {
public:
    // 示例参数
    bool enable_feature = true;
    int32_t max_count = 100;
    double threshold = 0.5;
    std::string device_name = "camera";
    std::vector<int32_t> id_list = {1, 2, 3, 4, 5};
    std::vector<double> values = {1.1, 2.2, 3.3};

    ExampleConfig() 
    {
        // 注册参数
        registerParam("enable_feature", enable_feature);
        registerParam("max_count", max_count);
        registerParam("threshold", threshold);
        registerParam("device_name", device_name);
        registerParam("id_list", id_list);
        registerParam("values", values);
    }

    // 实现纯虚函数
    const std::vector<datahandler::ParamInfo>& getParamsSaved() const override
    {
        return params_;
    }
};

class ExampleNode : public rclcpp::Node {
public:
    ExampleNode() : Node("example_node") 
    {
        // 创建配置实例
        config_ = std::make_shared<ExampleConfig>();
        
        // 将bas_operate参数转换为ROS参数
        auto bas_paras = config_->getParamsSaved();
        std::vector<rclcpp::Parameter> ros_paras;
        const std::string prefix = "test_config";
        bool bRet = basros::paraInfoToRos(bas_paras, prefix, ros_paras);
        if (!bRet) {
            LOG_ERROR("参数转换失败");
        } 
        else 
        {
            bool bRet = basros::paraInfoToServer(this, ros_paras);// 一次性原子性设置所有参数
            if (!bRet) {
                LOG_ERROR("参数原子性设置失败");
            }
            else 
            {
                logsys::Level log_level = logsys::Level::INFO;
                logsys::Color color = logsys::Color::BLUE;
                LOG_OUT(log_level, "完成参数原子性设置，共设置 %d 个参数：", static_cast<int>(ros_paras.size()));
                const std::string project_name = basmodule::get_project_name_by_file_path(__FILE__);
                if (!project_name.empty()) {
                    bRet = basros::printLog_rosParam(ros_paras, project_name, (int)log_level, (int)color, __FILE__, __FUNCTION__, __LINE__);
                }
                else {
                    LOG_ERROR("项目名称为空");
                }
            }
        }
    }

private:
    std::shared_ptr<ExampleConfig> config_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExampleNode>());
    rclcpp::shutdown();
    return 0;
}