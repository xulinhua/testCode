#include "ros_robot_workbench/kinematics/arm_kinematics_moveit.hpp"

#include <chrono>
#include <cmath>

#include "moveit_msgs/msg/position_ik_request.hpp"
#include "rclcpp/parameter_client.hpp"

namespace ros_robot_workbench::kinematics
{
namespace
{

int NormQuat(geometry_msgs::msg::Pose & p)
{
  const double & qx = p.orientation.x;
  const double & qy = p.orientation.y;
  const double & qz = p.orientation.z;
  const double & qw = p.orientation.w;
  const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n < 1e-10) {
    return -1;
  }
  p.orientation.x = qx / n;
  p.orientation.y = qy / n;
  p.orientation.z = qz / n;
  p.orientation.w = qw / n;
  return 0;
}

}  // namespace

bool CallMoveitIk(
  const rclcpp::Node::SharedPtr & node, const MoveitIkRequest & req,
  std::vector<std::string> * joint_names_out, std::vector<double> * positions_out, std::string * err)
{
  if (!node) {
    if (err) {
      *err = "rclcpp::Node 为空";
    }
    return false;
  }
  auto client = node->create_client<moveit_msgs::srv::GetPositionIK>(req.service);
  if (!client->wait_for_service(std::chrono::milliseconds(500))) {
    if (err) {
      *err = "服务不可用: " + req.service;
    }
    return false;
  }

  auto ik_req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  moveit_msgs::msg::PositionIKRequest & ikr = ik_req->ik_request;
  ikr.group_name = req.group_name;
  ikr.ik_link_name = req.ik_link_name;
  ikr.avoid_collisions = req.avoid_collisions;
  ikr.timeout.sec = req.timeout_ms / 1000;
  ikr.timeout.nanosec = (req.timeout_ms % 1000) * 1000000;

  ikr.pose_stamped.header.stamp = node->now();
  ikr.pose_stamped.header.frame_id = req.frame_id;
  ikr.pose_stamped.pose = req.desired_pose;
  if (NormQuat(ikr.pose_stamped.pose) < 0) {
    if (err) {
      *err = "四元数长度过小";
    }
    return false;
  }

  sensor_msgs::msg::JointState & js = ikr.robot_state.joint_state;
  js.name.clear();
  js.position.clear();
  for (const auto & kv : req.seed) {
    js.name.push_back(kv.first);
    js.position.push_back(kv.second);
  }
  if (js.name.empty()) {
    if (err) {
      *err = "seed 关节为空，请填写关节初值(名称+角度)";
    }
    return false;
  }

  auto future = client->async_send_request(ik_req);
  const rclcpp::FutureReturnCode rc = rclcpp::spin_until_future_complete(
    node, future, std::chrono::milliseconds(req.timeout_ms + 200));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    if (err) {
      *err = "IK 服务调用超时或失败";
    }
    return false;
  }

  const auto res = future.get();
  if (!res) {
    if (err) {
      *err = "无响应";
    }
    return false;
  }
  if (res->error_code.val != 1) {
    if (err) {
      *err = "MoveIt IK 失败, error_code=" + std::to_string(res->error_code.val);
    }
    return false;
  }
  if (joint_names_out) {
    *joint_names_out = res->solution.joint_state.name;
  }
  if (positions_out) {
    *positions_out = res->solution.joint_state.position;
  }
  return true;
}

bool QueryMoveitKinematicsSolverPlugin(
  const rclcpp::Node::SharedPtr & node, const std::string & move_group_node_name,
  const std::string & group_name, std::string * plugin_class, std::string * err)
{
  if (!node) {
    if (err) {
      *err = "rclcpp::Node 为空";
    }
    return false;
  }
  if (group_name.empty()) {
    if (err) {
      *err = "group_name 为空";
    }
    return false;
  }
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, move_group_node_name);
  if (!param_client->wait_for_service(std::chrono::milliseconds(1200))) {
    if (err) {
      *err = "无法连接参数服务: " + move_group_node_name;
    }
    return false;
  }
  const std::string key =
    "robot_description_kinematics." + group_name + ".kinematics_solver";
  if (!param_client->has_parameter(key)) {
    if (err) {
      *err = "未找到参数: " + key;
    }
    return false;
  }
  const auto vals = param_client->get_parameters({key});
  if (vals.empty() || vals[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    if (err) {
      *err = "参数类型异常: " + key;
    }
    return false;
  }
  if (plugin_class) {
    *plugin_class = vals[0].as_string();
  }
  return true;
}

}  // namespace ros_robot_workbench::kinematics
