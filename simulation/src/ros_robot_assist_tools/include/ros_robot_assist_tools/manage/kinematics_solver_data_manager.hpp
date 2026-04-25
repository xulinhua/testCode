#ifndef ROS_ROBOT_ASSIST_TOOLS__MANAGE__KINEMATICS_SOLVER_DATA_MANAGER_HPP_
#define ROS_ROBOT_ASSIST_TOOLS__MANAGE__KINEMATICS_SOLVER_DATA_MANAGER_HPP_

#include <string>

#include "ros_robot_assist_tools/manage/feature_data_manager_base.hpp"

namespace ros_robot_assist_tools::manage
{

class KinematicsSolverDataManager : public FeatureDataManagerBase
{
public:
  KinematicsSolverDataManager();

  void EnsureDefaults() override;

  void SetArmBackend(const std::string & v);
  std::string GetArmBackend() const;
  void SetArmUrdfPath(const std::string & v);
  std::string GetArmUrdfPath() const;
  void SetArmBaseLink(const std::string & v);
  std::string GetArmBaseLink() const;
  void SetArmTipLink(const std::string & v);
  std::string GetArmTipLink() const;
  void SetMoveitGroup(const std::string & v);
  std::string GetMoveitGroup() const;
  void SetMoveitIkLink(const std::string & v);
  std::string GetMoveitIkLink() const;
  void SetMoveitService(const std::string & v);
  std::string GetMoveitService() const;
  void SetMoveitFrameId(const std::string & v);
  std::string GetMoveitFrameId() const;
  void SetMoveitNodeName(const std::string & v);
  std::string GetMoveitNodeName() const;
  void SetMoveitSeedText(const std::string & v);
  std::string GetMoveitSeedText() const;

  void SetDiffTrackM(double v);
  double GetDiffTrackM() const;
  void SetDiffWheelRadiusM(double v);
  double GetDiffWheelRadiusM() const;

  void SetAckWheelbaseM(double v);
  double GetAckWheelbaseM() const;
};

}  // namespace ros_robot_assist_tools::manage

#endif  // ROS_ROBOT_ASSIST_TOOLS__MANAGE__KINEMATICS_SOLVER_DATA_MANAGER_HPP_
