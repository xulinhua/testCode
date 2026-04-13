#include "xtrainer_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "DobotNova2.h"
#include "DobotGripper.h"

#ifndef PI
#define PI 3.1415926 
#endif

namespace xtrainer_moveit_config
{

hardware_interface::CallbackReturn XTrainerHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_arm_ = std::make_shared<DobotNova2>();
  right_arm_ = std::make_shared<DobotNova2>();

  auto l_gripper_port = info_.hardware_parameters["left_gripper_port"];
  auto r_gripper_port = info_.hardware_parameters["right_gripper_port"];
  left_gripper_ = std::make_shared<DobotGripper>(l_gripper_port, 21, std::vector<int>({2048, 3052}));
  right_gripper_ = std::make_shared<DobotGripper>(r_gripper_port, 22, std::vector<int>({2048, 3052}));
  
  // 初始化关节数组 (16个关节: R1-1到R1-8, R2-1到R2-8)
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
XTrainerHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
XTrainerHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn XTrainerHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
    auto l_arm_ip = info_.hardware_parameters["left_arm_ip"];
    auto r_arm_ip = info_.hardware_parameters["right_arm_ip"];

    bool init_ret1 = left_arm_->Open(l_arm_ip);
    bool init_ret2  = right_arm_->Open(r_arm_ip);
    bool init_ret3  = left_gripper_->connect();
    bool init_ret4  = right_gripper_->connect();
    bool init_ret = init_ret1 && init_ret2 && init_ret3 && init_ret4;
    if(!init_ret){
        if(init_ret1) left_arm_->Close();
        if(init_ret2) right_arm_->Close();
        if(init_ret3) left_gripper_->disconnect();
        if(init_ret4) right_gripper_->disconnect();
        return hardware_interface::CallbackReturn::ERROR;
    }
    left_arm_->EnableRobot();
    right_arm_->EnableRobot();
    std::this_thread::sleep_for(std::chrono::seconds(1));

      auto lpushedInfo = left_arm_->GetPushedInfo();
  auto lGripperPos = left_gripper_->getCurrentPosition();
  auto lGripperSpeed = 0/* left_gripper_->getCurrentSpeed() */;
  auto rPushedInfo = right_arm_->GetPushedInfo();
  auto rGripperPos = right_gripper_->getCurrentPosition();
  auto rGripperSpeed = 0/* right_gripper_->getCurrentSpeed() */;
  //位置
  for(int i = 0, len = lpushedInfo.q_actual.size(); i < len; i++){
    hw_positions_.at(i) = lpushedInfo.q_actual.at(i)  * PI / 180.0;
  }
  hw_positions_.at(6) = 0.035 * (255 - lGripperPos) / 255.;
  hw_positions_.at(7) = 0.035 * (255 - lGripperPos) / 255.;

  for(int i = 0, len = rPushedInfo.q_actual.size(); i < len; i++){
    hw_positions_.at(8 + i) = rPushedInfo.q_actual.at(i)  * PI / 180.0;
  }

  hw_positions_.at(14) = 0.035 * (255 - rGripperPos) / 255.;
  hw_positions_.at(15) = 0.035 * (255 - rGripperPos) / 255.;

  hw_commands_ = hw_positions_;
    RCLCPP_INFO(rclcpp::get_logger("XTrainerHardwareInterface"), "Hardware interface activated successfully");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XTrainerHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
    left_arm_->Close();
    right_arm_->Close();
    left_gripper_->disconnect();
    right_gripper_->disconnect();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type XTrainerHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO: 实现从实际硬件读取关节状态
  // 这里应该添加您的硬件通信代码
  auto lpushedInfo = left_arm_->GetPushedInfo();
  auto lGripperPos = left_gripper_->getCurrentPosition();
  auto lGripperSpeed = 0/* left_gripper_->getCurrentSpeed() */;
  auto rPushedInfo = right_arm_->GetPushedInfo();
  auto rGripperPos = right_gripper_->getCurrentPosition();
  auto rGripperSpeed = 0/* right_gripper_->getCurrentSpeed() */;
  //位置
  for(int i = 0, len = lpushedInfo.q_actual.size(); i < len; i++){
    hw_positions_.at(i) = lpushedInfo.q_actual.at(i)  * PI / 180.0;
  }
  hw_positions_.at(6) = 0.035 * (255 - lGripperPos) / 255.;
  hw_positions_.at(7) = 0.035 * (255 - lGripperPos) / 255.;

  for(int i = 0, len = rPushedInfo.q_actual.size(); i < len; i++){
    hw_positions_.at(8 + i) = rPushedInfo.q_actual.at(i)  * PI / 180.0;
  }

  hw_positions_.at(14) = 0.035 * (255 - rGripperPos) / 255.;
  hw_positions_.at(15) = 0.035 * (255 - rGripperPos) / 255.;

  //速度
  for(int i = 0, len = lpushedInfo.q_d_actual.size(); i < len; i++){
    hw_velocities_.at(i) = lpushedInfo.q_d_actual.at(i)  * PI / 180.0;
  }
  hw_velocities_.at(6) = lGripperSpeed;
  hw_velocities_.at(7) = lGripperSpeed;

  for(int i = 0, len = rPushedInfo.q_d_actual.size(); i < len; i++){
    hw_velocities_.at(8 + i) = rPushedInfo.q_d_actual.at(i) * PI / 180.0;
  }

  hw_velocities_.at(14) = rGripperSpeed;
  hw_velocities_.at(15) = rGripperSpeed;
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XTrainerHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO: 实现向实际硬件发送控制命令
  // 这里应该添加您的硬件控制代码
  // RCLCPP_INFO(rclcpp::get_logger("XTrainerHardwareInterface"), 
  //             "hw_commands_: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
  //             hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3],
  //             hw_commands_[4], hw_commands_[5], hw_commands_[6], hw_commands_[7],
  //             hw_commands_[8], hw_commands_[9], hw_commands_[10], hw_commands_[11],
  //             hw_commands_[12], hw_commands_[13], hw_commands_[14], hw_commands_[15]);
  {
      auto jonits = std::vector<double>(hw_commands_.begin(), hw_commands_.begin() + 6);
      for(auto& j : jonits){
        j = j * 180.0 / PI; 
      }
      
      left_arm_->ServoJ(jonits);  
  }
  
  left_gripper_->move((0.035 - hw_commands_.at(6)) / 0.035 * 255 ,50, 50);

  {
      auto jonits = std::vector<double>(hw_commands_.begin() + 8, hw_commands_.begin() + 8 + 6);
      for(auto& j : jonits){
        j = j * 180.0 / PI; 
      }
      
      right_arm_->ServoJ(jonits);  
  }

  right_gripper_->move((0.035 - hw_commands_.at(14)) / 0.035 * 255 ,50, 50);

  return hardware_interface::return_type::OK;
}

}  // namespace xtrainer_moveit_config

PLUGINLIB_EXPORT_CLASS(
  xtrainer_moveit_config::XTrainerHardwareInterface, 
  hardware_interface::SystemInterface)