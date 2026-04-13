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
                std::string& output_folder,
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
        if (node["output_folder"]) {
            output_folder = node["output_folder"].as<std::string>();
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

bool savePointCloudToPLY(const std::string& filepath, const cv::Mat& points, const cv::Mat& colors = cv::Mat())
{
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        return false;
    }
    
    int valid_points = 0;
    for (int y = 0; y < points.rows; ++y) {
        for (int x = 0; x < points.cols; ++x) {
            cv::Vec3f pt = points.at<cv::Vec3f>(y, x);
            if (std::isfinite(pt[0]) && std::isfinite(pt[1]) && std::isfinite(pt[2]) &&
                pt[2] > 0 && pt[2] < 10.0f) {
                valid_points++;
            }
        }
    }
    
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << valid_points << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    if (!colors.empty()) {
        ofs << "property uchar red\n";
        ofs << "property uchar green\n";
        ofs << "property uchar blue\n";
    }
    ofs << "end_header\n";
    
    for (int y = 0; y < points.rows; ++y) {
        for (int x = 0; x < points.cols; ++x) {
            cv::Vec3f pt = points.at<cv::Vec3f>(y, x);
            if (std::isfinite(pt[0]) && std::isfinite(pt[1]) && std::isfinite(pt[2]) &&
                pt[2] > 0 && pt[2] < 10.0f) {
                ofs << pt[0] << " " << pt[1] << " " << pt[2];
                if (!colors.empty()) {
                    cv::Vec3b c = colors.at<cv::Vec3b>(y, x);
                    ofs << " " << (int)c[2] << " " << (int)c[1] << " " << (int)c[0];
                }
                ofs << "\n";
            }
        }
    }
    
    ofs.close();
    return true;
}

int main(int argc, char** argv)
{
    std::string config_path = "/home/user/code/Dev/src/calib/stereo_calib/src/config/stereo_calib_params.yaml";
    
    if (argc > 1) {
        config_path = argv[1];
    }
    
    std::string left_folder, right_folder, calib_file, output_folder;
    stereo_calib::StereoCalibParams params;
    stereo_calib::DisparityParams disp_params;
    
    loadConfig(config_path, left_folder, right_folder, calib_file, output_folder, params, disp_params);
    
    std::cout << "========================================" << std::endl;
    std::cout << "双目标定测试程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "配置文件: " << config_path << std::endl;
    std::cout << "棋盘格尺寸: " << params.board_size.width << " x " << params.board_size.height << std::endl;
    std::cout << "方格大小: " << params.square_size << " m" << std::endl;
    std::cout << "图像分辨率: " << params.image_size.width << " x " << params.image_size.height << std::endl;
    std::cout << "左图文件夹: " << left_folder << std::endl;
    std::cout << "右图文件夹: " << right_folder << std::endl;
    std::cout << "标定结果文件: " << calib_file << std::endl;
    std::cout << "输出文件夹: " << output_folder << std::endl;
    std::cout << "========================================" << std::endl;
    
    stereo_calib::StereoCalib stereo(params);
    stereo.setDisparityParams(disp_params);
    
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
            
            fs::path calib_path(calib_file);
            fs::path calib_dir = calib_path.parent_path();
            if (!fs::exists(calib_dir)) {
                fs::create_directories(calib_dir);
            }
            stereo.saveCalibrationToYaml(calib_file);
            std::cout << "标定结果已保存到: " << calib_file << std::endl;
        } else {
            std::cerr << "标定失败！" << std::endl;
            return -1;
        }
    }
    
    std::cout << "\n创建输出目录..." << std::endl;
    fs::path output_path(output_folder);
    fs::path rect_left_path = output_path / "rectified_left";
    fs::path rect_right_path = output_path / "rectified_right";
    fs::path disparity_path = output_path / "disparity";
    fs::path depth_path = output_path / "depth";
    fs::path pointcloud_path = output_path / "pointcloud";
    
    fs::create_directories(rect_left_path);
    fs::create_directories(rect_right_path);
    fs::create_directories(disparity_path);
    fs::create_directories(depth_path);
    fs::create_directories(pointcloud_path);
    
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
    
    size_t total_images = std::min(left_files.size(), right_files.size());
    std::cout << "\n找到 " << total_images << " 对图像，开始处理..." << std::endl;
    
    for (size_t idx = 0; idx < total_images; ++idx) {
        cv::Mat left_img = cv::imread(left_files[idx]);
        cv::Mat right_img = cv::imread(right_files[idx]);
        
        if (left_img.empty() || right_img.empty()) {
            std::cout << "跳过无效图像对: " << idx << std::endl;
            continue;
        }
        
        std::cout << "\r处理图像 " << idx + 1 << "/" << total_images << ": " 
                  << fs::path(left_files[idx]).stem().string() << std::flush;
        
        stereo_calib::StereoOutput output = stereo.processStereoImages(left_img, right_img, true);
        
        std::string base_name = fs::path(left_files[idx]).stem().string();
        
        cv::imwrite((rect_left_path / (base_name + ".png")).string(), output.rectified_left);
        cv::imwrite((rect_right_path / (base_name + ".png")).string(), output.rectified_right);
        
        cv::Mat disparity_vis = stereo.visualizeDisparity(output.disparity);
        cv::imwrite((disparity_path / (base_name + ".png")).string(), disparity_vis);
        
        cv::Mat depth_vis = stereo.visualizeDepth(output.depth_map);
        cv::imwrite((depth_path / (base_name + ".png")).string(), depth_vis);
        
        savePointCloudToPLY((pointcloud_path / (base_name + ".ply")).string(), 
                            output.point_cloud, output.rectified_left);
    }
    
    std::cout << "\n\n========================================" << std::endl;
    std::cout << "处理完成！" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "输出结果保存在: " << output_folder << std::endl;
    std::cout << "  - 校正左图: " << rect_left_path << std::endl;
    std::cout << "  - 校正右图: " << rect_right_path << std::endl;
    std::cout << "  - 视差图: " << disparity_path << std::endl;
    std::cout << "  - 深度图: " << depth_path << std::endl;
    std::cout << "  - 点云: " << pointcloud_path << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
