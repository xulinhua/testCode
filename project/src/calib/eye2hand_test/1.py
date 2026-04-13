import numpy as np
import cv2
import cv2.aruco as aruco
import time
import pyrealsense2 as rs

# 相机内参和畸变参数（将在第一帧时获取）
mtx = None
dist = None

# 初始化RealSense相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

# 启动相机
profile = pipeline.start(config)

# 标志位，用于确保只在第一帧获取相机内参
intrinsics_acquired = False

font = cv2.FONT_HERSHEY_SIMPLEX

# 新版ArUco API设置 (5x5字典，ID: 0，尺寸: 100mm)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)  # 使用5x5_100字典
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# 标记实际大小（单位：米），aruco码边长实际大小
marker_length = 0.1  # 100mm = 0.1m

while True:
    # 获取RealSense帧
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    
    if not color_frame:
        continue
    
    # 将RealSense帧转换为numpy数组
    frame = np.asanyarray(color_frame.get_data())
    
    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 在第一帧获取相机内参和畸变系数
    if not intrinsics_acquired:
        # 获取相机内参
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        
        # 构建3x3内参矩阵
        mtx = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        
        # 畸变系数
        dist = np.array(intrinsics.coeffs)
        
        # 打印相机内参和畸变系数
        print("相机内参矩阵:")
        print(mtx)
        print("\n畸变系数:")
        print(dist)
        
        intrinsics_acquired = True

    # 检测标记（新版API）
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    if ids is not None:
        # 估计每个标记的位姿（新版API）
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)

        # 为每个检测到的标记绘制坐标系和显示信息
        for i in range(len(ids)):
            # 绘制标记坐标系
            cv2.drawFrameAxes(frame, mtx, dist, rvec[i], tvec[i], marker_length)

            # 绘制标记边界（新版API）
            aruco.drawDetectedMarkers(frame, corners)

            # 计算到相机的距离（欧氏距离）
            distance = np.linalg.norm(tvec[i])

            # 显示信息（每个标记单独显示）
            text_y = 40 + i * 100  # 垂直间距
            cv2.putText(frame, f"ID: {ids[i][0]}", (20, text_y), font, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"Position (m): X:{tvec[i][0][0]:.3f} Y:{tvec[i][0][1]:.3f} Z:{tvec[i][0][2]:.3f}",
                        (20, text_y + 25), font, 0.6, (0, 255, 0), 1)
            #cv2.putText(frame, f"Rotation (rad): X:{rvec[i][0][0]:.3f} Y:{rvec[i][0][1]:.3f} Z:{rvec[i][0][2]:.3f}",
            #            (20, text_y + 50), font, 0.6, (0, 255, 0), 1)
            cv2.putText(frame, f"Rotation (deg): X:{np.degrees(rvec[i][0][0]):.3f} Y:{np.degrees(rvec[i][0][1]):.3f} Z:{np.degrees(rvec[i][0][2]):.3f}",
                        (20, text_y + 50), font, 0.6, (0, 255, 0), 1)
            print(f"ID: {ids[i][0]}")
            print(f"Position (m): X:{tvec[i][0][0]:.3f} Y:{tvec[i][0][1]:.3f} Z:{tvec[i][0][2]:.3f}")
            #print(f"Rotation (rad): X:{rvec[i][0][0]:.3f} Y:{rvec[i][0][1]:.3f} Z:{rvec[i][0][2]:.3f}")
            print(f"Rotation (deg): X:{np.degrees(rvec[i][0][0]):.3f} Y:{np.degrees(rvec[i][0][1]):.3f} Z:{np.degrees(rvec[i][0][2]):.3f}")
            cv2.putText(frame, f"Distance: {distance:.3f}m",
                        (20, text_y + 75), font, 0.6, (255, 0, 0), 2)

            # 在标记旁边显示ID（可选）
            center = corners[i][0].mean(axis=0).astype(int)
            cv2.putText(frame, str(ids[i][0]), (center[0], center[1]), font, 0.8, (0, 0, 255), 2)
    else:
        cv2.putText(frame, "No markers detected", (20, 40), font, 1, (0, 0, 255), 2)

    # 显示帧
    cv2.imshow("ArUco Marker Detection", frame)

    # 键盘控制
    key = cv2.waitKey(1)
    if key == 27:  # ESC退出
        break
    elif key == ord('s'):  # 按's'保存当前帧
        timestamp = int(time.time())
        cv2.imwrite(f"aruco_detection_{timestamp}.jpg", frame)
        print(f"Frame saved as aruco_detection_{timestamp}.jpg")

# 释放资源
pipeline.stop()
cv2.destroyAllWindows()
