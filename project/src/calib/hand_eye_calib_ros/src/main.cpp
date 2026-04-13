#include "hand_eye_calib_ros/hand_eye_calib_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>

int main(int argc, char * argv[]) {
    printf("Starting hand_eye_calib_node...\n");

    // 构造新的命令行参数，包含配置文件路径
    std::vector<std::string> new_argv_strings;
    std::vector<char*> new_argv;
    // 复制原始参数
    for (int i = 1; i < argc; i++) {
        new_argv_strings.push_back(std::string(argv[i]));
    }

    std::string default_config_file_path = ament_index_cpp::get_package_share_directory("hand_eye_calib_ros") + "/config" + "/hand_eye_calib_ros.yaml";

    if (argc <= 1)
        new_argv_strings.push_back("--ros-args");
    new_argv_strings.push_back("--params-file");
    new_argv_strings.push_back(default_config_file_path);
    std::cout << "使用默认配置文件: " << default_config_file_path << std::endl;
  
    // 转换为char*数组
    new_argv.reserve(new_argv_strings.size() + 1);
    for (const auto& arg : new_argv_strings) {
        std::cout << "参数配置: " << arg << std::endl;
        new_argv.push_back(const_cast<char*>(arg.c_str()));
    }
    new_argv.push_back(nullptr); // 确保以nullptr结尾
  
    // 初始化ROS 2
    rclcpp::init(new_argv.size() - 1, new_argv.data());

    printf("ROS 2 initialized.\n");
    
    auto node = std::make_shared<handeyecalib_ros::HandEyeCalibNode>();
    printf("Node created.\n");
    
    rclcpp::spin(node);
    printf("Node spinning finished.\n");

    // 正常退出时发布停止状态
    node->publishModuleStatus(basros::ModuleStatus::STOPPED, "模块正常停止");
    
    rclcpp::shutdown();
    printf("ROS 2 shutdown.\n");
    return 0;
}