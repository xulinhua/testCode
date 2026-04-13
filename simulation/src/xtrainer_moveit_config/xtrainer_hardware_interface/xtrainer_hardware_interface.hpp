#ifndef XTRAINER_HARDWARE_INTERFACE_HPP
#define XTRAINER_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <memory>

class DobotNova2;
class DobotGripper;

namespace xtrainer_moveit_config
{
class XTrainerHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XTrainerHardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 存储关节位置、速度状态和命令
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  std::shared_ptr<DobotNova2> left_arm_;
  std::shared_ptr<DobotNova2> right_arm_;

  std::shared_ptr<DobotGripper> left_gripper_;
  std::shared_ptr<DobotGripper> right_gripper_;
};

}  // namespace xtrainer_moveit_config

#endif