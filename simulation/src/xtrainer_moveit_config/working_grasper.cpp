#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <chrono>

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "working_grasper",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("working_grasper");

    RCLCPP_INFO(logger, "Creating ROS node and publishers...");

    // Create arm trajectory publisher
    auto arm_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/l_arm_controller/joint_trajectory", 10);

    // Create gripper publisher
    auto gripper_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/l_gripper_controller/commands", 10);

    RCLCPP_INFO(logger, "Working Grasper initialized");

    // Function to control gripper
    auto control_gripper = [gripper_publisher, logger](double position) {
        RCLCPP_INFO(logger, "Setting gripper position to %.3f", position);

        auto command = std_msgs::msg::Float64MultiArray();
        command.data = {position, position};  // R1-7 and R1-8 gripper joints
        gripper_publisher->publish(command);

        // Wait for gripper movement
        std::this_thread::sleep_for(std::chrono::seconds(1));
    };

    // Function to move arm to specific position
    auto move_arm_to_position = [arm_publisher, logger](const std::vector<double>& joint_positions, const std::string& position_name) {
        RCLCPP_INFO(logger, "Moving arm to %s position", position_name.c_str());

        auto trajectory = trajectory_msgs::msg::JointTrajectory();
        trajectory.joint_names = {"R1-1", "R1-2", "R1-3", "R1-4", "R1-5", "R1-6"};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = joint_positions;
        point.time_from_start = rclcpp::Duration::from_seconds(3.0);

        trajectory.points.push_back(point);
        arm_publisher->publish(trajectory);

        // Wait for arm movement
        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(logger, "Arm movement to %s completed", position_name.c_str());
    };

    // Define joint positions for different poses
    // These are example values - you may need to adjust them for your robot
    std::vector<double> home_position = {-1.5078, 0.0, -1.5078, 0.0, 1.5078, 1.5078};
    std::vector<double> grasp_position = {-0.5, 0.3, -0.8, 0.5, 0.8, 0.0};  // Example grasp position
    std::vector<double> lift_position = {-0.5, 0.3, -0.8, 0.5, 0.8, 0.0};    // Same as grasp but lifted

    // Wait for user to press Enter before starting grasp sequence
    RCLCPP_INFO(logger, "Press Enter to start grasp sequence...");
    std::cin.get();

    RCLCPP_INFO(logger, "Starting grasp sequence");

    // Step 1: Move to home position first (safer)
    RCLCPP_INFO(logger, "Moving to home position");
    move_arm_to_position(home_position, "home");

    // Step 2: Open gripper
    RCLCPP_INFO(logger, "Opening gripper");
    control_gripper(0.04);  // Open position
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Step 3: Move to grasp position
    RCLCPP_INFO(logger, "Moving to grasp position");
    move_arm_to_position(grasp_position, "grasp");

    // Step 4: Close gripper
    RCLCPP_INFO(logger, "Closing gripper");
    control_gripper(0.0);  // Close position
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Step 5: Lift object (move arm up)
    RCLCPP_INFO(logger, "Lifting object");
    // Modify the grasp position to lift (adjust joint 4 or 5 for upward movement)
    std::vector<double> lifted_position = grasp_position;
    lifted_position[3] += 0.3;  // Lift by adjusting joint 4
    move_arm_to_position(lifted_position, "lifted");

    // Step 6: Move back to home
    RCLCPP_INFO(logger, "Moving back to home position");
    move_arm_to_position(home_position, "home");

    RCLCPP_INFO(logger, "Grasp sequence completed");

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}