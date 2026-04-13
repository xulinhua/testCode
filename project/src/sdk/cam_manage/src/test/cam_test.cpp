#include "../include/cam_manage/cam_manage.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// 相机图像获取和显示线程函数
void camera_thread_func(CameraManager& cam, short cam_id, const std::string& window_name) {
    std::cout << "Starting thread for camera ID: " << cam_id << std::endl;
    
    while (true) {
        cv::Mat* img_color = nullptr;
        RtnType rtn = cam.get_one_frame_color(cam_id, img_color);
        
        if (rtn == RtnType::RTN_SUCCESS && img_color != nullptr) {
            // 在图像上绘制尺寸信息
            std::string size_info = std::to_string(img_color->cols) + "x" + std::to_string(img_color->rows);
            
            // 设置文字位置（距离左边和顶部各20像素）
            cv::Point text_position(20, 40);
            
            // 绘制黑色背景矩形以增强文字可读性
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(size_info, cv::FONT_HERSHEY_SIMPLEX, 1.2, 2, &baseline);
            cv::rectangle(*img_color, 
                         cv::Point(text_position.x - 5, text_position.y - text_size.height - 5),
                         cv::Point(text_position.x + text_size.width + 5, text_position.y + 5),
                         cv::Scalar(0, 0, 0), -1);
            
            // 绘制白色文字
            cv::putText(*img_color, size_info, text_position, 
                       cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2);
            
            std::cout << "cam_id" << cam_id << " Image size: " << size_info << std::endl;
            cv::imshow(window_name, *img_color);
            cv::waitKey(10);
            delete img_color;
            img_color = nullptr;
        }
        
        // 检查按键退出
        if (cv::waitKey(1) == 27) { // ESC键退出
            break;
        }
        
        // 短暂休眠以控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    
    std::cout << "Exiting thread for camera ID: " << cam_id << std::endl;
}

int main()
{
    std::cout << "----------main start------------" << std::endl;
    CameraManager &cam = CameraManager::get_instance();
    
    // 1. 遍历所有相机设备
    CamDevInfoList cam_list;
    cam.get_all_devices(cam_list);
    
    std::cout << "Found " << cam_list.size() << " cameras" << std::endl;
    
    // 2. 为每个相机生成配置
    CamConfigInfo1D configs;
    
    for (size_t i = 0; i < cam_list.size(); ++i) {
        CamConfigInfo cam_config;
        cam_config.cam_id = static_cast<short>(i);
        cam_config.serial_number = cam_list[i].serial_number;
        cam_config.cam_type = cam_list[i].cam_type;
        cam_config.cam_usr_name = cam_list[i].device_name;
        cam_config.cam_index = static_cast<int>(i);
        cam_config.enable_color_stream = true;
        cam_config.enable_depth_stream = false;
        cam_config.color_para.width = 1280;
        cam_config.color_para.height = 720;
        cam_config.color_para.fps = 30;
        cam_config.depth_para.width = 848;
        cam_config.depth_para.height = 480;
        cam_config.depth_para.fps = 30;
        
        configs.push_back(cam_config);
        std::cout << "Generated config for camera " << i << ": " << cam_list[i].device_name 
                  << " (SN: " << cam_list[i].serial_number << ")" << std::endl;
    }
    
    // 3. 初始化所有相机
    std::cout << "Initializing all cameras" << std::endl;
    RtnType init_result = cam.init_all_camera(configs);
    if (init_result != RtnType::RTN_SUCCESS) {
        std::cerr << "Failed to initialize cameras" << std::endl;
        return -1;
    }
    
    // 4. 获取所有已开启流的相机内参
    std::cout << "Getting camera intrinsics" << std::endl;
    for (size_t i = 0; i < configs.size(); ++i) {
        short cam_id = static_cast<short>(i);
        bool color_enabled = false;
        bool depth_enabled = false;
        
        // 检查流是否已启用
        cam.get_stream_enable(cam_id, CamStreamType::STREAM_COLOR, color_enabled);
        cam.get_stream_enable(cam_id, CamStreamType::STREAM_DEPTH, depth_enabled);
        
        std::cout << "Camera " << cam_id << " streams - Color: " << (color_enabled ? "ON" : "OFF") 
                  << ", Depth: " << (depth_enabled ? "ON" : "OFF") << std::endl;
        
        CamIntrinsics intrinsics;
        if (color_enabled) {
            RtnType rtn = cam.get_cam_intrinsics(cam_id, CamStreamType::STREAM_COLOR, intrinsics);
            if (rtn == RtnType::RTN_SUCCESS) {
                std::cout << "Camera " << cam_id << " color intrinsics: " 
                          << intrinsics.width << "x" << intrinsics.height 
                          << " fx=" << intrinsics.fx << " fy=" << intrinsics.fy 
                          << " cx=" << intrinsics.cx << " cy=" << intrinsics.cy << std::endl;
            }
        }
        
        if (depth_enabled) {
            RtnType rtn = cam.get_cam_intrinsics(cam_id, CamStreamType::STREAM_DEPTH, intrinsics);
            if (rtn == RtnType::RTN_SUCCESS) {
                std::cout << "Camera " << cam_id << " depth intrinsics: " 
                          << intrinsics.width << "x" << intrinsics.height 
                          << " fx=" << intrinsics.fx << " fy=" << intrinsics.fy 
                          << " cx=" << intrinsics.cx << " cy=" << intrinsics.cy << std::endl;
            }
        }
    }
    
    // 5. 保存所有相机的内参
    std::cout << "Saving camera intrinsics" << std::endl;
    RtnType save_result = cam.save_all_cam_intrinsics("camera_intrinsics.json");
    if (save_result == RtnType::RTN_SUCCESS) {
        std::cout << "Successfully saved all camera intrinsics" << std::endl;
    } else {
        std::cerr << "Failed to save camera intrinsics" << std::endl;
    }
    
    // 6. 创建多线程处理每个相机的图像获取和显示
    std::vector<std::thread> threads;
    std::vector<std::string> window_names;
    
    for (size_t i = 0; i < configs.size(); ++i) {
        short cam_id = static_cast<short>(i);
        std::string window_name = "Camera_" + std::to_string(cam_id) + "_" + configs[i].cam_usr_name;
        window_names.push_back(window_name);
        
        // 创建窗口
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        
        // 创建线程
        threads.emplace_back(camera_thread_func, std::ref(cam), cam_id, window_name);
    }
    
    std::cout << "Started " << threads.size() << " threads for image capture and display" << std::endl;
    std::cout << "Press ESC in any window to exit" << std::endl;
    
    // 等待所有线程完成
    for (auto& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }
    
    // 清理窗口
    for (const auto& name : window_names) {
        cv::destroyWindow(name);
    }
    
    std::cout << "----------main end------------" << std::endl;
    return 0;
}