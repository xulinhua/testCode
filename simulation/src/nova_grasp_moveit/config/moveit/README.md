# MoveIt2 配置（独立内嵌于 nova_grasp_moveit，供 /compute_ik）
#
# - nova_robot.urdf  : 运动学树（已去 mesh，避免依赖其它包的 mesh）
# - nova_robot.srdf  : 规划组 l_arm / r_arm / ...
# - kinematics.yaml  : KDL IK
# - joint_limits.yaml / ompl_planning.yaml / moveit_controllers.yaml
