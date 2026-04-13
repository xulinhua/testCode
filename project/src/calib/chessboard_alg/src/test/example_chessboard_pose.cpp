#include <opencv2/opencv.hpp>
#include <iostream>
#include "chessboard_alg/chessboard_pose_detector.hpp"

int main()
{
    // 创建棋盘格位姿检测器实例
    // 8x6 棋盘格，每个方格2.5cm
    chessboard_alg::ChessboardPoseDetector detector(cv::Size(8, 6), 0.025f);

    // 设置相机内参 (示例参数，请根据实际相机进行调整)
    // 这里使用典型的相机内参矩阵
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        640.0, 0.0, 320.0,
        0.0, 640.0, 240.0,
        0.0, 0.0, 1.0);

    // 畸变系数 (示例参数)
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    // 设置相机内参
    detector.setCameraIntrinsics(camera_matrix, dist_coeffs);

    // 打开摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }

    // 设置摄像头分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    std::cout << "棋盘格位姿检测演示" << std::endl;
    std::cout << "将棋盘格放置在相机视野中..." << std::endl;
    std::cout << "按 'q' 键退出" << std::endl;

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法获取视频帧" << std::endl;
            break;
        }

        // 检测并估计棋盘格位姿
        chessboard_alg::MarkerInfo marker_info;
        bool found = detector.detectAndEstimatePose(
            frame,           // 输入图像
            marker_info,     // 输出位姿信息
            false            // 不打印详细信息到控制台
        );

        // 显示结果
        cv::imshow("Chessboard Pose Detection", frame);

        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q') {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}