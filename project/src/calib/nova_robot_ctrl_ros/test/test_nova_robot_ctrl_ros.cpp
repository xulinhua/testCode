#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "nova_robot_ctrl_ros/nova_robot_ctrl_node.hpp"

// 测试NovaRobotCtrlNode类的基本功能
TEST(NovaRobotCtrlRosTest, TestNodeCreation) {
    // 初始化rclcpp
    auto node = std::make_shared<nova_robot_ctrl_ros::NovaRobotCtrlNode>();
    
    // 检查节点是否创建成功
    ASSERT_NE(node, nullptr);
    
    // 检查节点名称
    EXPECT_EQ(node->get_name(), std::string("nova_robot_ctrl_node"));
}

// 测试参数声明
TEST(NovaRobotCtrlRosTest, TestParameterDeclaration) {
    auto node = std::make_shared<nova_robot_ctrl_ros::NovaRobotCtrlNode>();
    
    // 检查参数是否正确声明
    EXPECT_TRUE(node->has_parameter("robot_ip"));
    EXPECT_TRUE(node->has_parameter("robot_target_pos_topic"));
    EXPECT_TRUE(node->has_parameter("robot_pose_topic"));
    EXPECT_TRUE(node->has_parameter("robot_run_state_topic"));
    EXPECT_TRUE(node->has_parameter("robot_control_service"));
    EXPECT_TRUE(node->has_parameter("timer_period_ms"));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}