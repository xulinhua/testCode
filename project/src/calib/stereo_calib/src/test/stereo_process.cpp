#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "stereo_calib/stereo_calib.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <algorithm>
#include <vector>

namespace fs = std::filesystem;

struct ConfigPaths {
    std::string left_folder;
    std::string right_folder;
    std::string calib_file;
    std::string output_folder;
    stereo_calib::DisparityParams disp_params;
    bool valid = false;
};

ConfigPaths loadConfig(const std::string& config_path)
{
    ConfigPaths paths;
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["stereo_calib_parameters"]) {
            auto node = config["stereo_calib_parameters"];
            
            if (node["left_image_folder"])
                paths.left_folder = node["left_image_folder"].as<std::string>();
            if (node["right_image_folder"])
                paths.right_folder = node["right_image_folder"].as<std::string>();
            if (node["calibration_file"])
                paths.calib_file = node["calibration_file"].as<std::string>();
            if (node["output_folder"])
                paths.output_folder = node["output_folder"].as<std::string>();
            
            if (node["disparity_params"]) {
                auto disp_node = node["disparity_params"];
                if (disp_node["num_disparities"])
                    paths.disp_params.num_disparities = disp_node["num_disparities"].as<int>();
                if (disp_node["block_size"])
                    paths.disp_params.block_size = disp_node["block_size"].as<int>();
                if (disp_node["pre_filter_cap"])
                    paths.disp_params.pre_filter_cap = disp_node["pre_filter_cap"].as<int>();
                if (disp_node["min_disparity"])
                    paths.disp_params.min_disparity = disp_node["min_disparity"].as<int>();
                if (disp_node["texture_threshold"])
                    paths.disp_params.texture_threshold = disp_node["texture_threshold"].as<int>();
                if (disp_node["uniqueness_ratio"])
                    paths.disp_params.uniqueness_ratio = disp_node["uniqueness_ratio"].as<int>();
                if (disp_node["speckle_window_size"])
                    paths.disp_params.speckle_window_size = disp_node["speckle_window_size"].as<int>();
                if (disp_node["speckle_range"])
                    paths.disp_params.speckle_range = disp_node["speckle_range"].as<int>();
                if (disp_node["disp12_max_diff"])
                    paths.disp_params.disp12_max_diff = disp_node["disp12_max_diff"].as<int>();
                if (disp_node["use_sgbm"])
                    paths.disp_params.use_sgbm = disp_node["use_sgbm"].as<bool>();
            }
            paths.valid = true;
        }
    } catch (const std::exception& e) {
        std::cerr << "警告: 无法加载配置文件: " << e.what() << std::endl;
    }
    return paths;
}

bool savePointCloudToPLY(const std::string& filepath, const cv::Mat& points, const cv::Mat& colors = cv::Mat())
{
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        return false;
    }
    
    // 统计有效点数
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
    
    // 写入PLY头
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
    
    // 写入点云数据
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

void printUsage(const char* program_name)
{
    std::cout << "用法: " << program_name << " [选项]\n"
              << "选项:\n"
              << "  --config <file>     配置文件路径 (包含所有路径和参数)\n"
              << "  --calib <file>      标定结果文件路径 (覆盖配置文件)\n"
              << "  --left <folder>     左图文件夹路径 (覆盖配置文件)\n"
              << "  --right <folder>    右图文件夹路径 (覆盖配置文件)\n"
              << "  --output <folder>   输出文件夹路径 (覆盖配置文件)\n"
              << "  --left-img <file>   单张左图路径 (替代文件夹模式)\n"
              << "  --right-img <file>  单张右图路径 (替代文件夹模式)\n"
              << "  --help              显示此帮助信息\n"
              << "\n示例:\n"
              << "  # 使用配置文件 (推荐)\n"
              << "  " << program_name << " --config stereo_calib_params.yaml\n"
              << "\n  # 覆盖部分配置\n"
              << "  " << program_name << " --config params.yaml --output ./my_output\n"
              << "\n  # 完全手动指定\n"
              << "  " << program_name << " --calib calib.yaml --left left/ --right right/ --output output/\n";
}

int main(int argc, char** argv)
{
    // 默认配置文件路径
    std::string config_file = "/home/user/code/Dev/src/calib/stereo_calib/src/config/stereo_calib_params.yaml";
    std::string calib_file;
    std::string left_folder, right_folder;
    std::string left_img, right_img;
    std::string output_folder;
    
    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--calib" && i + 1 < argc) {
            calib_file = argv[++i];
        } else if (arg == "--left" && i + 1 < argc) {
            left_folder = argv[++i];
        } else if (arg == "--right" && i + 1 < argc) {
            right_folder = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_folder = argv[++i];
        } else if (arg == "--left-img" && i + 1 < argc) {
            left_img = argv[++i];
        } else if (arg == "--right-img" && i + 1 < argc) {
            right_img = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        }
    }
    
    // 加载配置文件
    ConfigPaths config = loadConfig(config_file);
    
    // 命令行参数覆盖配置文件
    if (calib_file.empty()) calib_file = config.calib_file;
    if (left_folder.empty()) left_folder = config.left_folder;
    if (right_folder.empty()) right_folder = config.right_folder;
    if (output_folder.empty()) output_folder = config.output_folder;
    
    // 验证必需参数
    if (calib_file.empty()) {
        std::cerr << "错误: 必须指定标定文件 (--calib 或在配置文件中)" << std::endl;
        printUsage(argv[0]);
        return -1;
    }
    
    bool single_image_mode = false;
    if (!left_img.empty() && !right_img.empty()) {
        single_image_mode = true;
    } else if (left_folder.empty() || right_folder.empty()) {
        std::cerr << "错误: 必须指定左右图像路径" << std::endl;
        printUsage(argv[0]);
        return -1;
    }
    
    if (output_folder.empty()) {
        output_folder = "./output";
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "双目立体处理程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "配置文件: " << config_file << std::endl;
    std::cout << "标定文件: " << calib_file << std::endl;
    if (single_image_mode) {
        std::cout << "左图: " << left_img << std::endl;
        std::cout << "右图: " << right_img << std::endl;
    } else {
        std::cout << "左图文件夹: " << left_folder << std::endl;
        std::cout << "右图文件夹: " << right_folder << std::endl;
    }
    std::cout << "输出文件夹: " << output_folder << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 加载标定结果
    stereo_calib::StereoCalibParams params;
    stereo_calib::StereoCalib stereo(params);
    
    // 设置视差参数
    stereo.setDisparityParams(config.disp_params);
    
    if (!stereo.loadCalibrationFromYaml(calib_file)) {
        std::cerr << "错误: 无法加载标定文件" << std::endl;
        return -1;
    }
    
    stereo.printCalibrationInfo();
    
    // 创建输出目录
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
    
    // 处理图像
    if (single_image_mode) {
        // 单张图像模式
        cv::Mat left = cv::imread(left_img);
        cv::Mat right = cv::imread(right_img);
        
        if (left.empty() || right.empty()) {
            std::cerr << "错误: 无法读取图像" << std::endl;
            return -1;
        }
        
        std::cout << "\n处理图像..." << std::endl;
        
        stereo_calib::StereoOutput output = stereo.processStereoImages(left, right, true);
        
        std::string base_name = fs::path(left_img).stem().string();
        
        cv::imwrite((rect_left_path / (base_name + ".png")).string(), output.rectified_left);
        cv::imwrite((rect_right_path / (base_name + ".png")).string(), output.rectified_right);
        
        cv::Mat disparity_vis = stereo.visualizeDisparity(output.disparity);
        cv::imwrite((disparity_path / (base_name + ".png")).string(), disparity_vis);
        
        cv::Mat depth_vis = stereo.visualizeDepth(output.depth_map);
        cv::imwrite((depth_path / (base_name + ".png")).string(), depth_vis);
        
        savePointCloudToPLY((pointcloud_path / (base_name + ".ply")).string(), 
                            output.point_cloud, output.rectified_left);
        
        std::cout << "处理完成: " << base_name << std::endl;
        
    } else {
        // 文件夹模式
        std::vector<std::string> left_files, right_files;
        
        for (const auto& entry : fs::directory_iterator(left_folder)) {
            if (entry.path().extension() == ".png" ||
                entry.path().extension() == ".jpg" ||
                entry.path().extension() == ".jpeg" ||
                entry.path().extension() == ".bmp") {
                left_files.push_back(entry.path().string());
            }
        }
        for (const auto& entry : fs::directory_iterator(right_folder)) {
            if (entry.path().extension() == ".png" ||
                entry.path().extension() == ".jpg" ||
                entry.path().extension() == ".jpeg" ||
                entry.path().extension() == ".bmp") {
                right_files.push_back(entry.path().string());
            }
        }
        
        std::sort(left_files.begin(), left_files.end());
        std::sort(right_files.begin(), right_files.end());
        
        size_t total_images = std::min(left_files.size(), right_files.size());
        std::cout << "\n找到 " << total_images << " 对图像，开始处理..." << std::endl;
        
        int processed = 0;
        for (size_t idx = 0; idx < total_images; ++idx) {
            cv::Mat left = cv::imread(left_files[idx]);
            cv::Mat right = cv::imread(right_files[idx]);
            
            if (left.empty() || right.empty()) {
                continue;
            }
            
            stereo_calib::StereoOutput output = stereo.processStereoImages(left, right, true);
            
            std::string base_name = fs::path(left_files[idx]).stem().string();
            
            cv::imwrite((rect_left_path / (base_name + ".png")).string(), output.rectified_left);
            cv::imwrite((rect_right_path / (base_name + ".png")).string(), output.rectified_right);
            
            cv::Mat disparity_vis = stereo.visualizeDisparity(output.disparity);
            cv::imwrite((disparity_path / (base_name + ".png")).string(), disparity_vis);
            
            cv::Mat depth_vis = stereo.visualizeDepth(output.depth_map);
            cv::imwrite((depth_path / (base_name + ".png")).string(), depth_vis);
            
            savePointCloudToPLY((pointcloud_path / (base_name + ".ply")).string(), 
                                output.point_cloud, output.rectified_left);
            
            processed++;
            std::cout << "\r处理进度: " << processed << "/" << total_images << std::flush;
        }
        
        std::cout << "\n处理完成 " << processed << " 对图像" << std::endl;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "输出结果保存在: " << output_folder << std::endl;
    std::cout << "  - 校正左图: " << rect_left_path << std::endl;
    std::cout << "  - 校正右图: " << rect_right_path << std::endl;
    std::cout << "  - 视差图: " << disparity_path << std::endl;
    std::cout << "  - 深度图: " << depth_path << std::endl;
    std::cout << "  - 点云: " << pointcloud_path << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
