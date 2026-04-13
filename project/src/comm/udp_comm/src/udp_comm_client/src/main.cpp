#include "udp_comm_client/udp_client_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv)
{
  // 构造新的命令行参数，包含配置文件路径
  std::vector<std::string> new_argv_strings;
  std::vector<char*> new_argv;
  
  // 复制原始参数
  for (int i = 1; i < argc; i++) 
  {
    new_argv_strings.push_back(std::string(argv[i]));
  }
  std::string default_config_file_path = ament_index_cpp::get_package_share_directory("udp_comm_client") + "/config" + "/udp_client_params.yaml";
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
  //rclcpp::init(argc, argv);
  
  auto node = std::make_shared<udp_comm_client::UDPClientNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}