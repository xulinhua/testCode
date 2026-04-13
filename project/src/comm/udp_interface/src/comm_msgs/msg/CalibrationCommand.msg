# 标定指令
uint8 command_type  # 指令类型：0=开始标定, 1=移动到下一个点, 2=结束标定
uint8 CALIB_START=0       # 开始标定
uint8 CALIB_NEXT_POINT=1  # 移动到下一个点
uint8 CALIB_FINISHED=2    # 结束标定
float32[] robot_pose  # 机械手当前位姿(x,y,z,roll,pitch,yaw)

