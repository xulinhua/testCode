# 标定响应
uint8 response_type  # 响应类型：0=识别成功, 1=识别失败, 2=标定完成
uint8 RES_SUCCESS=0       # 识别成功
uint8 RES_FAILURE=1       # 识别失败
uint8 RES_COMPLETED_SUCCESS=2    # 标定成功完成（收集到足够点且计算出矩阵）
uint8 RES_COMPLETED_FAILED=3     # 标定失败完成（例如点不够等）

float32[] aruco_pixel_pose  # Aruco码像素坐标(u,v)
float32[] robot_pose  # 机械手位姿(x,y,z,roll,pitch,yaw)