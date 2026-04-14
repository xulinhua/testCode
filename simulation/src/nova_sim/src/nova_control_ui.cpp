#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
const std::vector<std::string> kJointOrder = {
  "J1_1_joint", "J1_2_joint", "J1_3_joint", "J1_4_joint", "J1_5_joint", "J1_6_joint", "J1_7_joint", "J1_8_joint",
  "J2_1_joint", "J2_2_joint", "J2_3_joint", "J2_4_joint", "J2_5_joint", "J2_6_joint", "J2_7_joint", "J2_8_joint",
  "J3_1_joint", "J3_2_joint", "J3_3_joint", "J3_4_joint", "J3_5_joint", "J3_6_joint",
  "J4_1_joint", "J4_2_joint", "J4_3_joint", "J4_4_joint", "J4_5_joint", "J4_6_joint"};
}  // namespace

class NovaControlUiCpp : public rclcpp::Node
{
public:
  NovaControlUiCpp()
  : Node("nova_control_ui_cpp")
  {
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 10);
    arm_id_pub_ = create_publisher<std_msgs::msg::Int32>("/nova_arm_id", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/nova_target_pose", 10);
    gripper_pub_ = create_publisher<std_msgs::msg::String>("/nova_gripper_goal", 10);
  }

  void run()
  {
    print_help();
    while (rclcpp::ok()) {
      std::cout << "\n[UI] select mode: 1) all joints  2) pose  3) gripper  q) quit\n> " << std::flush;
      std::string cmd;
      if (!std::getline(std::cin, cmd)) {
        break;
      }
      if (cmd == "q" || cmd == "quit") {
        break;
      }
      if (cmd == "1") {
        handle_joint_mode();
      } else if (cmd == "2") {
        handle_pose_mode();
      } else if (cmd == "3") {
        handle_gripper_mode();
      } else {
        std::cout << "[UI] unknown command\n";
      }
    }
  }

private:
  void print_help() const
  {
    std::cout << "Nova Control UI (C++)\n";
    std::cout << "- Joint mode: publish 28 values to /arm_controller/commands\n";
    std::cout << "- Pose mode: publish /nova_arm_id + /nova_target_pose\n";
    std::cout << "- Gripper mode: publish /nova_arm_id + /nova_gripper_goal\n";
  }

  bool parse_doubles_line(const std::string & line, std::vector<double> & out) const
  {
    std::stringstream ss(line);
    out.clear();
    double v = 0.0;
    while (ss >> v) {
      out.push_back(v);
    }
    return !out.empty();
  }

  void handle_joint_mode()
  {
    std::cout << "[UI] input 28 joint values separated by spaces.\n";
    std::cout << "[UI] shortcut: input 'zero' for all zeros.\n> " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
      return;
    }

    std_msgs::msg::Float64MultiArray msg;
    if (line == "zero") {
      msg.data.assign(kJointOrder.size(), 0.0);
    } else {
      std::vector<double> vals;
      if (!parse_doubles_line(line, vals) || vals.size() != kJointOrder.size()) {
        std::cout << "[UI] need exactly 28 values, got " << vals.size() << "\n";
        return;
      }
      msg.data = vals;
    }
    cmd_pub_->publish(msg);
    std::cout << "[UI] published /arm_controller/commands\n";
  }

  void handle_pose_mode()
  {
    std::cout << "[UI] arm_id (0/1/2/3): " << std::flush;
    int arm_id = 0;
    if (!(std::cin >> arm_id)) {
      clear_stdin();
      std::cout << "[UI] invalid arm_id\n";
      return;
    }
    if (arm_id < 0 || arm_id > 3) {
      clear_stdin();
      std::cout << "[UI] arm_id out of range\n";
      return;
    }

    std::cout << "[UI] input x y z qx qy qz qw: " << std::flush;
    double x, y, z, qx, qy, qz, qw;
    if (!(std::cin >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      clear_stdin();
      std::cout << "[UI] invalid pose numbers\n";
      return;
    }
    clear_stdin();

    std_msgs::msg::Int32 id_msg;
    id_msg.data = arm_id;
    arm_id_pub_->publish(id_msg);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;
    pose_pub_->publish(pose);

    std::cout << "[UI] published pose goal for arm_id=" << arm_id << "\n";
  }

  void handle_gripper_mode()
  {
    std::cout << "[UI] arm_id (0/1): " << std::flush;
    int arm_id = 0;
    if (!(std::cin >> arm_id)) {
      clear_stdin();
      std::cout << "[UI] invalid arm_id\n";
      return;
    }
    if (arm_id < 0 || arm_id > 1) {
      clear_stdin();
      std::cout << "[UI] gripper only supports arm_id 0/1\n";
      return;
    }

    std::cout << "[UI] mode: open / close / width\n> " << std::flush;
    clear_stdin();
    std::string mode;
    if (!std::getline(std::cin, mode)) {
      return;
    }

    std_msgs::msg::Int32 id_msg;
    id_msg.data = arm_id;
    arm_id_pub_->publish(id_msg);

    std_msgs::msg::String gmsg;
    if (mode == "open" || mode == "close") {
      gmsg.data = mode;
    } else if (mode == "width") {
      std::cout << "[UI] width in meters (0~0.1): " << std::flush;
      double width = 0.05;
      if (!(std::cin >> width)) {
        clear_stdin();
        std::cout << "[UI] invalid width\n";
        return;
      }
      clear_stdin();
      gmsg.data = "width:" + std::to_string(width);
    } else {
      std::cout << "[UI] unsupported mode\n";
      return;
    }
    gripper_pub_->publish(gmsg);
    std::cout << "[UI] published gripper command for arm_id=" << arm_id << "\n";
  }

  void clear_stdin()
  {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_id_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NovaControlUiCpp>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
