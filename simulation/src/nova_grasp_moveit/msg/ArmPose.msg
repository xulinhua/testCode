# 目标机械臂：0=J1/l_arm，1=J2/r_arm。
int32 arm_id
# MoveIt 腕部目标；pose.header.frame_id 通常为 base_link，目标 link 为 J1_6/J2_6。
geometry_msgs/PoseStamped pose
