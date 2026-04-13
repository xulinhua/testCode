#include "cam_mgr_ros/cam_mgr.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto camera_manager = std::make_shared<cam_mgr_ros::CamMgrRos>();
    
    rclcpp::spin(camera_manager);
    
    rclcpp::shutdown();
    return 0;
}
