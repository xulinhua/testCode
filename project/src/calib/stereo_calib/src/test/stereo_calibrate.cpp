#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "stereo_calib/stereo_calib.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>

void loadConfig(const std::string& config_path,
                std::string& left_folder,
                std::string& right_folder,
                std::string& calib_file,
                stereo_calib::StereoCalibParams& params)
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
    
    loadConfig(config_path, left_folder, right_folder, calib_file, params);
    
    stereo_calib::StereoCalib stereo(params);
    
    std::cout << "双目标定程序" << std::endl;
    std::cout << "====================" << std::endl;
    std::cout << "棋盘格尺寸: " << params.board_size.width << " x " << params.board_size.height << std::endl;
    std::cout << "方格大小: " << params.square_size << " m" << std::endl;
    std::cout << "左图文件夹: " << left_folder << std::endl;
    std::cout << "右图文件夹: " << right_folder << std::endl;
    std::cout << "====================" << std::endl;
    
    std::cout << "\n开始双目标定..." << std::endl;
    
    if (stereo.calibrateFromFolders(left_folder, right_folder)) {
        stereo.printCalibrationInfo();
        
        std::filesystem::path calib_path(calib_file);
        std::filesystem::path calib_dir = calib_path.parent_path();
        
        if (!std::filesystem::exists(calib_dir)) {
            std::filesystem::create_directories(calib_dir);
        }
        
        if (stereo.saveCalibrationToYaml(calib_file)) {
            std::cout << "\n标定成功！结果已保存到: " << calib_file << std::endl;
        } else {
            std::cerr << "\n保存标定结果失败！" << std::endl;
            return -1;
        }
    } else {
        std::cerr << "\n标定失败！请检查图像文件夹路径和棋盘格参数。" << std::endl;
        return -1;
    }
    
    return 0;
}
