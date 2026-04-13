#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/planner_interface_description.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <thread>
#include <chrono>
#include <std_msgs/msg/float64_multi_array.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "fixed_position_grasper",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&]() {
        executor.spin();
    });

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("fixed_position_grasper");

    // Create gripper publisher
    auto gripper_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/l_gripper_controller/commands", 10);

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
  

    try {
        RCLCPP_INFO(logger, "Creating MoveGroup interface for 'l_arm'...");
        auto move_group_interface = MoveGroupInterface(node, "l_arm");

    // Function to control gripper
    auto control_gripper = [gripper_publisher, logger](double position) {
        auto command = std_msgs::msg::Float64MultiArray();
        command.data = {position, position};  // R1-7 and R1-8 gripper joints
        gripper_publisher->publish(command);

        // Wait for gripper movement
        std::this_thread::sleep_for(std::chrono::seconds(1));
    };

    // Function for hybrid Cartesian/regular motion planning
    auto plan_and_execute_motion = [logger](moveit::planning_interface::MoveGroupInterface& move_group,
                                          const geometry_msgs::msg::Pose& target_pose,
                                          double eef_step = 0.01,
                                          double success_threshold = 0.8) -> bool {

        // Get current pose as starting point
        auto current_pose = move_group.getCurrentPose();

        RCLCPP_INFO(logger, "Starting hybrid motion planning to target: z=%.3f", target_pose.position.z);

        // Try Cartesian path first for straight-line movement
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose.pose);  // Start pose
        waypoints.push_back(target_pose);        // Target pose

        // Store original velocity/acceleration scaling (assume default values)
        double orig_vel_scale = 1.0;  // Default full speed
        double orig_acc_scale = 1.0;  // Default full acceleration

        // Set conservative parameters for Cartesian planning
        move_group.setMaxVelocityScalingFactor(0.3);
        move_group.setMaxAccelerationScalingFactor(0.2);

        // Try Cartesian path planning
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        bool success = false;

        if (fraction >= success_threshold) {  // Cartesian path succeeded
            RCLCPP_INFO(logger, "Cartesian path planned successfully (%.2f%%), executing...", fraction * 100.0);
            move_group.execute(trajectory);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            success = true;
        } else {  // Fall back to regular planning
            RCLCPP_WARN(logger, "Cartesian path failed (%.2f%%), trying regular planning...", fraction * 100.0);

            // Restore original scaling factors
            move_group.setMaxVelocityScalingFactor(orig_vel_scale);
            move_group.setMaxAccelerationScalingFactor(orig_acc_scale);

            // Set pose target and plan
            move_group.setPoseTarget(target_pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group.plan(plan)) {
                RCLCPP_INFO(logger, "Regular motion plan successful, executing...");
                move_group.execute(plan);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                success = true;
            } else {
                RCLCPP_ERROR(logger, "Both Cartesian and regular planning failed!");
                success = false;
            }
        }

        return success;
    };

    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(1);

    RCLCPP_INFO(logger, "Starting grasp sequence");

    // Step 1: Open gripper
    RCLCPP_INFO(logger, "Opening gripper");
    control_gripper(0);  // Open position

    // Step 2: Move to grasp position
    RCLCPP_INFO(logger, "Moving to grasp position");

    // Debug: Check MoveGroupInterface state
    RCLCPP_INFO(logger, "MoveGroupInterface initialized, checking state...");

    // Try to get current pose with detailed error info
    RCLCPP_INFO(logger, "Attempting to get current pose...");
    auto current_pose = move_group_interface.getCurrentPose();

    // Set a target Pose
    auto const target_pose = [current_pose] {
        geometry_msgs::msg::Pose msg;
        msg.orientation = current_pose.pose.orientation;
        msg.position.x = 0.5;     // x in meters
        msg.position.y = -0.11;     // y in meters
        msg.position.z = 0.2;     // z in meters
        return msg;
    }();

    // Use the hybrid motion planning function
    bool success = plan_and_execute_motion(move_group_interface, target_pose);

    if (!success) {
        RCLCPP_ERROR(logger, "Motion planning failed!");
        rclcpp::shutdown();
        return 1;
    }

    // Step 3: Close gripper
    RCLCPP_INFO(logger, "Closing gripper");
    control_gripper(0.035);  // Close position

    // Step 4: Lift object
    RCLCPP_INFO(logger, "Lifting object");
    auto lift_pose = current_pose;

    // Use the hybrid motion planning function for lifting
    bool lift_success = plan_and_execute_motion(move_group_interface, lift_pose.pose);

    if (!lift_success) {
        RCLCPP_ERROR(logger, "Lift planning failed!");
    }

    RCLCPP_INFO(logger, "Grasp sequence completed");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Exception occurred: %s", e.what());
        RCLCPP_ERROR(logger, "Make sure MoveIt is running properly");
        return 1;
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}