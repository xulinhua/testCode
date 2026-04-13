#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include "chessboard_alg/chessboard_pose_detector.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc != 2) {
        std::cerr << "用法: " << argv[0] << " <标定图像文件夹路径>" << std::endl;
        return -1;
    }

    std::string images_path = argv[1];
    
    // 检查路径是否存在
    if (!fs::exists(images_path) || !fs::is_directory(images_path)) {
        std::cerr << "错误: 无效的图像文件夹路径" << std::endl;
        return -1;
    }

    // 创建棋盘格位姿检测器实例
    // 10x7 棋盘格，每个方格2.5cm
    chessboard_alg::ChessboardPoseDetector detector(cv::Size(10, 7), 0.024125f);

    // 加载标定图像
    std::vector<cv::Mat> calibration_images;
    std::vector<std::string> image_files;

    std::cout << "正在加载标定图像..." << std::endl;
    
    // 遍历文件夹中的图像文件
    for (const auto& entry : fs::directory_iterator(images_path)) {
        std::string filename = entry.path().string();
        
        // 检查文件扩展名
        std::string ext = filename.substr(filename.find_last_of(".") + 1);
        if (ext == "jpg" || ext == "jpeg" || ext == "png" || ext == "bmp") {
            cv::Mat img = cv::imread(filename);
            if (!img.empty()) {
                calibration_images.push_back(img);
                image_files.push_back(filename);
                std::cout << "加载图像: " << filename << std::endl;
            } else {
                std::cerr << "无法加载图像: " << filename << std::endl;
            }
        }
    }

    if (calibration_images.empty()) {
        std::cerr << "错误: 未找到有效图像" << std::endl;
        return -1;
    }

    std::cout << "共加载 " << calibration_images.size() << " 张标定图像" << std::endl;

    // 进行相机标定
    cv::Mat camera_matrix, dist_coeffs;
    std::cout << "\n开始相机标定..." << std::endl;
    
    bool success = detector.calibrateCameraIntrinsics(calibration_images, camera_matrix, dist_coeffs);

    if (success) {
        std::cout << "\n相机标定成功！" << std::endl;
        std::cout << "====================================" << std::endl;
        std::cout << "相机内参矩阵:" << std::endl;
        std::cout << camera_matrix << std::endl;
        std::cout << "\n畸变系数:" << std::endl;
        std::cout << dist_coeffs << std::endl;
        std::cout << "====================================" << std::endl;

        // 保存标定结果
        std::string calibration_file = "camera_calibration.yml";
        cv::FileStorage fs(calibration_file, cv::FileStorage::WRITE);
        if (fs.isOpened()) {
            fs << "camera_matrix" << camera_matrix;
            fs << "dist_coeffs" << dist_coeffs;
            fs << "board_size" << cv::Size(10, 7);
            fs << "square_size" << 0.025f;
            fs << "image_count" << (int)calibration_images.size();
            fs.release();
            std::cout << "\n标定结果已保存到: " << calibration_file << std::endl;
        } else {
            std::cerr << "无法保存标定结果" << std::endl;
        }

        // 可选：使用标定结果进行测试
        std::cout << "\n使用标定结果测试..." << std::endl;
        detector.setCameraIntrinsics(camera_matrix, dist_coeffs);

        // 测试第一张图像
        if (!calibration_images.empty()) {
            cv::Mat test_frame = calibration_images[0].clone();
            auto result = detector.detectAndProcessMarkers(test_frame, nullptr, true, true);
            
            if (result.found)
		    {
			    std::cout << "检测到棋盘格" << std::endl;
			    // 可视化结果
			    test_frame = result.processed_frame.clone();
		    }
            std::string test_result = "calibration_test_result.jpg";
            cv::imwrite(test_result, test_frame);
            std::cout << "测试结果已保存到: " << test_result << std::endl;
        }

    } else {
        std::cerr << "相机标定失败！" << std::endl;
        return -1;
    }

    std::cout << "\n标定完成！" << std::endl;
    return 0;
}