#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "stereo_calib/stereo_calib.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <vector>

namespace fs = std::filesystem;

void loadConfig(const std::string& config_path,
                std::string& left_folder,
                std::string& right_folder,
                std::string& calib_file,
                stereo_calib::StereoCalibParams& params,
                stereo_calib::DisparityParams& disp_params)
{
    YAML::Node config = YAML::LoadFile(config_path);
    
    if (config["stereo_calib_parameters"]) {
        auto node = config["stereo_calib_parameters"];
        
        if (node["board_size"]) {
            params.board_size = cv::Size(node["board_size"][0].as<int>(),
                                         node["board_size"][1].as<int>());
        }
        if (node["square_size"]) {
            params.square_size = node["square_size"].as<float>();
        }
        if (node["image_width"] && node["image_height"]) {
            params.image_size = cv::Size(node["image_width"].as<int>(),
                                         node["image_height"].as<int>());
        }
        if (node["left_image_folder"]) {
            left_folder = node["left_image_folder"].as<std::string>();
        }
        if (node["right_image_folder"]) {
            right_folder = node["right_image_folder"].as<std::string>();
        }
        if (node["calibration_file"]) {
            calib_file = node["calibration_file"].as<std::string>();
        }
        
        if (node["disparity_params"]) {
            auto disp_node = node["disparity_params"];
            if (disp_node["num_disparities"])
                disp_params.num_disparities = disp_node["num_disparities"].as<int>();
            if (disp_node["block_size"])
                disp_params.block_size = disp_node["block_size"].as<int>();
            if (disp_node["pre_filter_cap"])
                disp_params.pre_filter_cap = disp_node["pre_filter_cap"].as<int>();
            if (disp_node["min_disparity"])
                disp_params.min_disparity = disp_node["min_disparity"].as<int>();
            if (disp_node["texture_threshold"])
                disp_params.texture_threshold = disp_node["texture_threshold"].as<int>();
            if (disp_node["uniqueness_ratio"])
                disp_params.uniqueness_ratio = disp_node["uniqueness_ratio"].as<int>();
            if (disp_node["speckle_window_size"])
                disp_params.speckle_window_size = disp_node["speckle_window_size"].as<int>();
            if (disp_node["speckle_range"])
                disp_params.speckle_range = disp_node["speckle_range"].as<int>();
            if (disp_node["disp12_max_diff"])
                disp_params.disp12_max_diff = disp_node["disp12_max_diff"].as<int>();
            if (disp_node["use_sgbm"])
                disp_params.use_sgbm = disp_node["use_sgbm"].as<bool>();
        }
    }
}

int main(int argc, char** argv)
{
    std::string config_path = "/home/user/code/Dev/src/calib/stereo_calib/src/config/stereo_calib_params.yaml";
    
    if (argc > 1) {
        config_path = argv[1];
    }
    
    std::string left_folder, right_folder, calib_file;
    stereo_calib::StereoCalibParams params;
    stereo_calib::DisparityParams disp_params;
    
    loadConfig(config_path, left_folder, right_folder, calib_file, params, disp_params);
    
    stereo_calib::StereoCalib stereo(params);
    stereo.setDisparityParams(disp_params);
    
    std::cout << "双目标定示例程序" << std::endl;
    std::cout << "配置文件: " << config_path << std::endl;
    std::cout << "左图文件夹: " << left_folder << std::endl;
    std::cout << "右图文件夹: " << right_folder << std::endl;
    std::cout << "标定结果文件: " << calib_file << std::endl;
    
    bool use_existing_calib = false;
    std::ifstream calib_check(calib_file);
    if (calib_check.good()) {
        std::cout << "\n发现已有标定文件，是否使用？(y/n): ";
        char input;
        std::cin >> input;
        if (input == 'y' || input == 'Y') {
            if (stereo.loadCalibrationFromYaml(calib_file)) {
                use_existing_calib = true;
                std::cout << "成功加载标定参数" << std::endl;
            }
        }
    }
    
    if (!use_existing_calib) {
        std::cout << "\n开始双目标定..." << std::endl;
        if (stereo.calibrateFromFolders(left_folder, right_folder)) {
            stereo.printCalibrationInfo();
            stereo.saveCalibrationToYaml(calib_file);
            std::cout << "标定结果已保存到: " << calib_file << std::endl;
        } else {
            std::cerr << "标定失败！" << std::endl;
            return -1;
        }
    }
    
    cv::namedWindow("Rectified Stereo", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Disparity", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
    
    std::cout << "\n按 'q' 退出，按 's' 保存当前结果" << std::endl;
    
    std::vector<std::string> left_files, right_files;
    
    for (const auto& entry : fs::directory_iterator(left_folder)) {
        if (entry.path().extension() == ".png" ||
            entry.path().extension() == ".jpg" ||
            entry.path().extension() == ".jpeg") {
            left_files.push_back(entry.path().string());
        }
    }
    for (const auto& entry : fs::directory_iterator(right_folder)) {
        if (entry.path().extension() == ".png" ||
            entry.path().extension() == ".jpg" ||
            entry.path().extension() == ".jpeg") {
            right_files.push_back(entry.path().string());
        }
    }
    
    std::sort(left_files.begin(), left_files.end());
    std::sort(right_files.begin(), right_files.end());
    
    int img_idx = 0;
    size_t total_images = std::min(left_files.size(), right_files.size());
    
    cv::Mat disparity_vis, depth_vis;
    
    while (true) {
        if (static_cast<size_t>(img_idx) < total_images) {
            cv::Mat left_img = cv::imread(left_files[img_idx]);
            cv::Mat right_img = cv::imread(right_files[img_idx]);
            
            if (!left_img.empty() && !right_img.empty()) {
                stereo_calib::StereoOutput output = stereo.processStereoImages(left_img, right_img, false);
                
                depth_vis = stereo.visualizeDepth(output.depth_map);
                
                cv::normalize(output.disparity, disparity_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
                cv::applyColorMap(disparity_vis, disparity_vis, cv::COLORMAP_JET);
                
                cv::Mat stereo_rect_vis;
                cv::hconcat(output.rectified_left, output.rectified_right, stereo_rect_vis);
                
                int line_step = 30;
                for (int y = 0; y < stereo_rect_vis.rows; y += line_step) {
                    cv::line(stereo_rect_vis, cv::Point(0, y), cv::Point(stereo_rect_vis.cols, y), 
                             cv::Scalar(0, 255, 0), 1);
                }
                
                cv::imshow("Rectified Stereo", stereo_rect_vis);
                cv::imshow("Disparity", disparity_vis);
                cv::imshow("Depth", depth_vis);
                
                std::cout << "\r处理图像: " << img_idx + 1 << "/" << total_images 
                          << " | 按 'n' 下一张, 'p' 上一张, 'q' 退出" << std::flush;
            }
        }
        
        char key = cv::waitKey(100) & 0xFF;
        
        if (key == 'q') {
            break;
        } else if (key == 'n' || key == ' ') {
            img_idx = std::min(img_idx + 1, static_cast<int>(total_images) - 1);
        } else if (key == 'p') {
            img_idx = std::max(img_idx - 1, 0);
        } else if (key == 's') {
            std::string save_prefix = "stereo_result_" + std::to_string(img_idx);
            cv::imwrite(save_prefix + "_disparity.png", disparity_vis);
            cv::imwrite(save_prefix + "_depth.png", depth_vis);
            std::cout << "\n结果已保存: " << save_prefix << std::endl;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}
