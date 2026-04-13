import numpy as np
import cv2
import time
from camsdk_realsense import CamSDK_Realsense
from aruco_detector import Aruco_Detector


def main():
    # 创建各个模块的实例
    camera_sdk = CamSDK_Realsense()
    
    # 设置是否保存相机内参（新增功能）
    camera_sdk.set_save_intrinsics(True)
    
    # 设置是否自动保存帧（新增功能）
    camera_sdk.set_auto_save_frames(False)  # 默认不自动保存，可通过按键手动保存
    
    aruco_detector = Aruco_Detector(marker_length=0.1)  # 100mm = 0.1m
    
    try:
        # 初始化相机
        pipeline, profile = camera_sdk.initialize_camera(width=1280, height=720, fps=30)
        
        # 获取相机内参（如果初始化时未自动获取）
        if not camera_sdk.is_intrinsics_acquired():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame:
                mtx, dist = camera_sdk.acquire_cam_intrinsics(color_frame)
        
        # 检查是否成功获取相机内参
        cam_params = camera_sdk.get_cam_intrinsics()
        if cam_params[0] is None:
            print("错误：无法获取相机内参")
            return
            
        mtx, dist, _ = cam_params
                    
        # 设置相机内参
        aruco_detector.set_camera_intrinsics(mtx, dist)
            
        while True:
            # 获取RealSense帧
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame:
                continue
            
            # 将RealSense帧转换为numpy数组
            frame = np.asanyarray(color_frame.get_data())
            
            # 使用新封装的接口检测并处理Aruco标记
            # 传入参数：是否绘制渲染图=True，是否打印输出信息=True
            result = aruco_detector.detect_and_process_markers(frame, depth_frame, draw_results=True, print_results=True)
            markers_info = result['markers_info']
            
            # 获取处理后的帧（如果需要绘制结果）
            if 'frame' in result:
                frame = result['frame']
            
            # 如果启用了自动保存帧功能，则保存帧
            if camera_sdk.save_frames_enabled:
                try:
                    filename = camera_sdk.save_frame(frame, format=camera_sdk.default_save_format)
                    print(f"自动保存帧: {filename}")
                except Exception as e:
                    print(f"自动保存帧失败: {e}")
            
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
                
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 释放资源
        camera_sdk.stop_camera()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()