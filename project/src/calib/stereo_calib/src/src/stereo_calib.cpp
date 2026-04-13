#include "../include/stereo_calib/stereo_calib.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace stereo_calib
{

// ==================== 构造函数 ====================

StereoCalib::StereoCalib()
: term_criteria_(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001),
  is_calibrated_(false)
{
  // 初始化BM立体匹配器
  stereo_bm_ = cv::StereoBM::create(disparity_params_.num_disparities, disparity_params_.block_size);
  
  // 初始化SGBM立体匹配器，参数说明：
  // - minDisparity: 最小视差
  // - numDisparities: 视差搜索范围
  // - blockSize: 匹配块大小
  // - P1, P2: 平滑参数，控制视差平滑度
  // - disp12MaxDiff: 左右一致性检查阈值
  // - preFilterCap: 预滤波截断值
  // - uniquenessRatio: 唯一性比率
  // - speckleWindowSize/Range: 斑点滤波参数
  // - mode: SGBM_3WAY 使用三路动态规划
  stereo_sgbm_ = cv::StereoSGBM::create(
    disparity_params_.min_disparity,
    disparity_params_.num_disparities,
    disparity_params_.block_size,
    8 * disparity_params_.block_size * disparity_params_.block_size,  // P1: 8*blockSize^2
    32 * disparity_params_.block_size * disparity_params_.block_size, // P2: 32*blockSize^2
    disparity_params_.disp12_max_diff,
    disparity_params_.pre_filter_cap,
    disparity_params_.uniqueness_ratio,
    disparity_params_.speckle_window_size,
    disparity_params_.speckle_range,
    cv::StereoSGBM::MODE_SGBM_3WAY
  );
}

StereoCalib::StereoCalib(const StereoCalibParams& params)
: params_(params),
  term_criteria_(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001),
  is_calibrated_(false)
{
  stereo_bm_ = cv::StereoBM::create(disparity_params_.num_disparities, disparity_params_.block_size);
  stereo_sgbm_ = cv::StereoSGBM::create(
    disparity_params_.min_disparity,
    disparity_params_.num_disparities,
    disparity_params_.block_size,
    8 * disparity_params_.block_size * disparity_params_.block_size,
    32 * disparity_params_.block_size * disparity_params_.block_size,
    disparity_params_.disp12_max_diff,
    disparity_params_.pre_filter_cap,
    disparity_params_.uniqueness_ratio,
    disparity_params_.speckle_window_size,
    disparity_params_.speckle_range,
    cv::StereoSGBM::MODE_SGBM_3WAY
  );
}

// ==================== 参数设置/获取 ====================

void StereoCalib::setBoardSize(const cv::Size& board_size)
{
  params_.board_size = board_size;
}

void StereoCalib::setSquareSize(float square_size)
{
  params_.square_size = square_size;
}

void StereoCalib::setImageSize(const cv::Size& image_size)
{
  params_.image_size = image_size;
}

cv::Size StereoCalib::getBoardSize() const
{
  return params_.board_size;
}

float StereoCalib::getSquareSize() const
{
  return params_.square_size;
}

cv::Size StereoCalib::getImageSize() const
{
  return params_.image_size;
}

// ==================== 私有辅助函数 ====================

std::vector<cv::Point3f> StereoCalib::generateObjectPoints() const
{
  // 生成棋盘格角点的3D世界坐标
  // 假设棋盘格在Z=0平面上，按行列顺序排列
  std::vector<cv::Point3f> obj_points;
  for (int i = 0; i < params_.board_size.height; ++i) {
    for (int j = 0; j < params_.board_size.width; ++j) {
      obj_points.push_back(cv::Point3f(
        static_cast<float>(j * params_.square_size),  // X坐标
        static_cast<float>(i * params_.square_size),  // Y坐标
        0.0f                                           // Z坐标(平面)
      ));
    }
  }
  return obj_points;
}

bool StereoCalib::detectChessboardCorners(
  const cv::Mat& image,
  std::vector<cv::Point2f>& corners)
{
  if (image.empty()) {
    return false;
  }

  // 转换为灰度图像
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  // 检测棋盘格角点
  // 使用自适应阈值、归一化图像和快速检查优化检测
  bool found = cv::findChessboardCorners(
    gray, params_.board_size, corners,
    cv::CALIB_CB_ADAPTIVE_THRESH |  // 自适应阈值
    cv::CALIB_CB_NORMALIZE_IMAGE |  // 归一化图像
    cv::CALIB_CB_FAST_CHECK         // 快速检查
  );

  // 如果找到角点，使用亚像素精度优化角点位置
  if (found) {
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), term_criteria_);
  }

  return found;
}

// ==================== 双目标定核心函数 ====================

bool StereoCalib::calibrateFromImages(
  const std::vector<cv::Mat>& left_images,
  const std::vector<cv::Mat>& right_images)
{
  // 输入验证
  if (left_images.empty() || right_images.empty()) {
    std::cerr << "标定失败：输入图像集为空" << std::endl;
    return false;
  }

  if (left_images.size() != right_images.size()) {
    std::cerr << "标定失败：左右图像数量不匹配" << std::endl;
    return false;
  }

  // 准备存储角点数据
  std::vector<std::vector<cv::Point3f>> object_points;      // 3D世界坐标点
  std::vector<std::vector<cv::Point2f>> image_points_left;  // 左图像角点
  std::vector<std::vector<cv::Point2f>> image_points_right; // 右图像角点

  // 生成棋盘格的3D世界坐标模板
  std::vector<cv::Point3f> obj_points = generateObjectPoints();

  cv::Size img_size;
  size_t valid_pairs = 0;

  // 遍历所有图像对，检测棋盘格角点
  for (size_t i = 0; i < left_images.size(); ++i) {
    std::vector<cv::Point2f> corners_left, corners_right;
    
    // 分别检测左右图像中的棋盘格角点
    bool found_left = detectChessboardCorners(left_images[i], corners_left);
    bool found_right = detectChessboardCorners(right_images[i], corners_right);

    // 只有两张图像都检测到角点才有效
    if (found_left && found_right) {
      object_points.push_back(obj_points);
      image_points_left.push_back(corners_left);
      image_points_right.push_back(corners_right);
      
      if (img_size.width == 0) {
        img_size = left_images[i].size();
      }
      valid_pairs++;
    }
  }

  // 至少需要3对有效图像才能进行标定
  if (valid_pairs < 3) {
    std::cerr << "有效棋盘格图像对数量不足，至少需要3对" << std::endl;
    return false;
  }

  std::cout << "检测到 " << valid_pairs << " 对有效棋盘格图像" << std::endl;

  // 初始化相机内参矩阵和畸变系数
  // 给定合理的初始估计值，帮助标定收敛
  double cx = img_size.width / 2.0;   // 光心x坐标(图像中心)
  double cy = img_size.height / 2.0;  // 光心y坐标(图像中心)
  double fx = cx + cy;  // 初始焦距估计，根据图像尺寸调整
  double fy = fx;  // 左右相机焦距相同
  
  intrinsics_.camera_matrix_left = (cv::Mat_<double>(3, 3) << 
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1);
  intrinsics_.camera_matrix_right = (cv::Mat_<double>(3, 3) << 
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1);
  intrinsics_.dist_coeffs_left = cv::Mat::zeros(5, 1, CV_64F);
  intrinsics_.dist_coeffs_right = cv::Mat::zeros(5, 1, CV_64F);

  // 执行双目标定
  // 计算左右相机的内参、畸变系数，以及相对位姿(R, T)
  // 注意: 如果已有准确的单独标定的内参，可使用 CALIB_FIX_INTRINSIC 固定内参
  // 这里我们同时优化内参和外参，获得更好的结果
  int flags = cv::CALIB_FIX_ASPECT_RATIO |  // 固定 fx/fy 比例
              cv::CALIB_ZERO_TANGENT_DIST |   // 切向畸变设为0
              cv::CALIB_USE_INTRINSIC_GUESS/* | // 使用初始猜测值
              cv::CALIB_SAME_FOCAL_LENGTH*/;    // 左右相机焦距相同
  double rms = cv::stereoCalibrate(
    object_points,
    image_points_left,
    image_points_right,
    intrinsics_.camera_matrix_left,
    intrinsics_.dist_coeffs_left,
    intrinsics_.camera_matrix_right,
    intrinsics_.dist_coeffs_right,
    img_size,
    intrinsics_.R,   // 右相机相对左相机的旋转矩阵
    intrinsics_.T,   // 右相机相对左相机的平移向量
    intrinsics_.E,   // 本质矩阵
    intrinsics_.F,   // 基础矩阵
    flags,
    term_criteria_
  );

  std::cout << "双目标定完成，重投影误差: " << rms << std::endl;

  // 计算立体校正参数
  // 将左右图像投影到同一平面，使极线水平对齐
  cv::stereoRectify(
    intrinsics_.camera_matrix_left,
    intrinsics_.dist_coeffs_left,
    intrinsics_.camera_matrix_right,
    intrinsics_.dist_coeffs_right,
    img_size,
    intrinsics_.R,
    intrinsics_.T,
    intrinsics_.R1,  // 左相机校正旋转矩阵
    intrinsics_.R2,  // 右相机校正旋转矩阵
    intrinsics_.P1,  // 左相机投影矩阵
    intrinsics_.P2,  // 右相机投影矩阵
    intrinsics_.Q,   // 视差到深度映射矩阵
    cv::CALIB_ZERO_DISPARITY,  // 主点相同
    0,               // alpha=0: 裁剪无效区域
    img_size,
    &intrinsics_.valid_roi_left,
    &intrinsics_.valid_roi_right
  );

  // 计算基线距离: baseline = -P2(0,3) / P2(0,0)
  intrinsics_.baseline = static_cast<double>(-intrinsics_.P2.at<double>(0, 3) / intrinsics_.P2.at<double>(0, 0));
  intrinsics_.is_calibrated = true;
  is_calibrated_ = true;
  params_.image_size = img_size;

  // 初始化校正映射表
  initRectifyMaps();

  return true;
}

bool StereoCalib::calibrateFromFolders(
  const std::string& left_folder,
  const std::string& right_folder)
{
  std::vector<cv::Mat> left_images, right_images;
  std::vector<std::string> left_files, right_files;

  // 遍历左相机文件夹，收集图像文件路径
  for (const auto& entry : std::filesystem::directory_iterator(left_folder)) {
    if (entry.path().extension() == ".png" ||
        entry.path().extension() == ".jpg" ||
        entry.path().extension() == ".jpeg" ||
        entry.path().extension() == ".bmp") {
      left_files.push_back(entry.path().string());
    }
  }

  // 遍历右相机文件夹，收集图像文件路径
  for (const auto& entry : std::filesystem::directory_iterator(right_folder)) {
    if (entry.path().extension() == ".png" ||
        entry.path().extension() == ".jpg" ||
        entry.path().extension() == ".jpeg" ||
        entry.path().extension() == ".bmp") {
      right_files.push_back(entry.path().string());
    }
  }

  // 按文件名排序，确保左右图像配对正确
  std::sort(left_files.begin(), left_files.end());
  std::sort(right_files.begin(), right_files.end());

  size_t num_pairs = std::min(left_files.size(), right_files.size());

  // 加载图像
  for (size_t i = 0; i < num_pairs; ++i) {
    std::cout << left_files[i] << std::endl;
    std::cout << right_files[i] << std::endl;

    cv::Mat left_img = cv::imread(left_files[i]);
    cv::Mat right_img = cv::imread(right_files[i]);

    if (!left_img.empty() && !right_img.empty()) {
      left_images.push_back(left_img);
      right_images.push_back(right_img);
    }
  }

  std::cout << "从文件夹加载了 " << left_images.size() << " 对图像" << std::endl;

  return calibrateFromImages(left_images, right_images);
}

// ==================== 标定参数存取 ====================

void StereoCalib::setCameraIntrinsics(const StereoCameraIntrinsics& intrinsics)
{
  intrinsics_ = intrinsics;
  is_calibrated_ = intrinsics.is_calibrated;
  if (is_calibrated_) {
    initRectifyMaps();
  }
}

StereoCameraIntrinsics StereoCalib::getCameraIntrinsics() const
{
  return intrinsics_;
}

bool StereoCalib::loadCalibration(const std::string& filepath)
{
  return loadCalibrationFromYaml(filepath);
}

bool StereoCalib::saveCalibration(const std::string& filepath) const
{
  return saveCalibrationToYaml(filepath);
}

bool StereoCalib::loadCalibrationFromYaml(const std::string& filepath)
{
  try {
    YAML::Node config = YAML::LoadFile(filepath);

    // 从YAML加载矩阵的辅助函数
    auto load_mat = [&config](const std::string& key, cv::Mat& mat) {
      if (config[key]) {
        std::vector<double> data = config[key].as<std::vector<double>>();
        int rows = config[key + "_rows"].as<int>();
        int cols = config[key + "_cols"].as<int>();
        mat = cv::Mat(rows, cols, CV_64F, data.data()).clone();
      }
    };

    // 加载所有标定参数
    load_mat("camera_matrix_left", intrinsics_.camera_matrix_left);
    load_mat("dist_coeffs_left", intrinsics_.dist_coeffs_left);
    load_mat("camera_matrix_right", intrinsics_.camera_matrix_right);
    load_mat("dist_coeffs_right", intrinsics_.dist_coeffs_right);
    load_mat("R", intrinsics_.R);
    load_mat("T", intrinsics_.T);
    load_mat("E", intrinsics_.E);
    load_mat("F", intrinsics_.F);
    load_mat("R1", intrinsics_.R1);
    load_mat("R2", intrinsics_.R2);
    load_mat("P1", intrinsics_.P1);
    load_mat("P2", intrinsics_.P2);
    load_mat("Q", intrinsics_.Q);

    if (config["baseline"]) {
      intrinsics_.baseline = config["baseline"].as<double>();
    }

    if (config["image_width"] && config["image_height"]) {
      params_.image_size = cv::Size(
        config["image_width"].as<int>(),
        config["image_height"].as<int>()
      );
    }

    intrinsics_.is_calibrated = true;
    is_calibrated_ = true;

    // 加载后初始化校正映射
    initRectifyMaps();

    std::cout << "双目标定参数加载成功: " << filepath << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "加载标定参数失败: " << e.what() << std::endl;
    return false;
  }
}

bool StereoCalib::saveCalibrationToYaml(const std::string& filepath) const
{
  try {
    YAML::Emitter out;

    // 将矩阵写入YAML的辅助函数
    auto emit_mat = [&out](const std::string& key, const cv::Mat& mat) {
      out << YAML::Key << key << YAML::Value << YAML::BeginSeq;
      for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
          out << mat.at<double>(i, j);
        }
      }
      out << YAML::EndSeq;
      out << YAML::Key << (key + "_rows") << YAML::Value << mat.rows;
      out << YAML::Key << (key + "_cols") << YAML::Value << mat.cols;
    };

    out << YAML::BeginMap;
    
    out << YAML::Key << "image_width" << YAML::Value << params_.image_size.width;
    out << YAML::Key << "image_height" << YAML::Value << params_.image_size.height;
    
    emit_mat("camera_matrix_left", intrinsics_.camera_matrix_left);
    emit_mat("dist_coeffs_left", intrinsics_.dist_coeffs_left);
    emit_mat("camera_matrix_right", intrinsics_.camera_matrix_right);
    emit_mat("dist_coeffs_right", intrinsics_.dist_coeffs_right);
    emit_mat("R", intrinsics_.R);
    emit_mat("T", intrinsics_.T);
    emit_mat("E", intrinsics_.E);
    emit_mat("F", intrinsics_.F);
    emit_mat("R1", intrinsics_.R1);
    emit_mat("R2", intrinsics_.R2);
    emit_mat("P1", intrinsics_.P1);
    emit_mat("P2", intrinsics_.P2);
    emit_mat("Q", intrinsics_.Q);
    
    out << YAML::Key << "baseline" << YAML::Value << intrinsics_.baseline;
    
    out << YAML::EndMap;

    std::ofstream fout(filepath);
    fout << out.c_str();
    fout.close();

    std::cout << "双目标定参数保存成功: " << filepath << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "保存标定参数失败: " << e.what() << std::endl;
    return false;
  }
}

// ==================== 图像校正 ====================

void StereoCalib::initRectifyMaps()
{
  if (!is_calibrated_) {
    std::cerr << "相机未标定，无法初始化校正映射" << std::endl;
    return;
  }

  // 计算左相机的校正映射表
  // map1: 目标像素对应的源图像坐标
  // map2: 插值系数(用于双线性插值)
  cv::initUndistortRectifyMap(
    intrinsics_.camera_matrix_left,
    intrinsics_.dist_coeffs_left,
    intrinsics_.R1,
    intrinsics_.P1,
    params_.image_size,
    CV_16SC2,  // 使用16位有符号2通道格式，节省内存
    rectify_maps_.map1_left,
    rectify_maps_.map2_left
  );

  // 计算右相机的校正映射表
  cv::initUndistortRectifyMap(
    intrinsics_.camera_matrix_right,
    intrinsics_.dist_coeffs_right,
    intrinsics_.R2,
    intrinsics_.P2,
    params_.image_size,
    CV_16SC2,
    rectify_maps_.map1_right,
    rectify_maps_.map2_right
  );

  rectify_maps_.is_initialized = true;
  std::cout << "校正映射初始化完成" << std::endl;
}

bool StereoCalib::isRectifyMapsInitialized() const
{
  return rectify_maps_.is_initialized;
}

cv::Mat StereoCalib::rectifyImage(const cv::Mat& image, bool is_left) const
{
  if (!rectify_maps_.is_initialized) {
    std::cerr << "校正映射未初始化" << std::endl;
    return image.clone();
  }

  cv::Mat rectified;
  // 使用remap进行图像校正
  // 将原始图像像素映射到校正后的位置
  if (is_left) {
    cv::remap(image, rectified, rectify_maps_.map1_left, rectify_maps_.map2_left, cv::INTER_LINEAR);
  } else {
    cv::remap(image, rectified, rectify_maps_.map1_right, rectify_maps_.map2_right, cv::INTER_LINEAR);
  }
  return rectified;
}

void StereoCalib::rectifyStereoImages(
  const cv::Mat& left_image,
  const cv::Mat& right_image,
  cv::Mat& rectified_left,
  cv::Mat& rectified_right) const
{
  rectified_left = rectifyImage(left_image, true);
  rectified_right = rectifyImage(right_image, false);
}

// ==================== 视差计算 ====================

void StereoCalib::setDisparityParams(const DisparityParams& params)
{
  disparity_params_ = params;
  
  // 更新BM匹配器参数
  stereo_bm_->setNumDisparities(disparity_params_.num_disparities);
  stereo_bm_->setBlockSize(disparity_params_.block_size);
  stereo_bm_->setPreFilterCap(disparity_params_.pre_filter_cap);
  stereo_bm_->setPreFilterSize(disparity_params_.pre_filter_size);
  stereo_bm_->setTextureThreshold(disparity_params_.texture_threshold);
  stereo_bm_->setUniquenessRatio(disparity_params_.uniqueness_ratio);
  stereo_bm_->setSpeckleWindowSize(disparity_params_.speckle_window_size);
  stereo_bm_->setSpeckleRange(disparity_params_.speckle_range);

  // 更新SGBM匹配器参数
  stereo_sgbm_->setMinDisparity(disparity_params_.min_disparity);
  stereo_sgbm_->setNumDisparities(disparity_params_.num_disparities);
  stereo_sgbm_->setBlockSize(disparity_params_.block_size);
  stereo_sgbm_->setPreFilterCap(disparity_params_.pre_filter_cap);
  stereo_sgbm_->setUniquenessRatio(disparity_params_.uniqueness_ratio);
  stereo_sgbm_->setSpeckleWindowSize(disparity_params_.speckle_window_size);
  stereo_sgbm_->setSpeckleRange(disparity_params_.speckle_range);
  stereo_sgbm_->setDisp12MaxDiff(disparity_params_.disp12_max_diff);
}

DisparityParams StereoCalib::getDisparityParams() const
{
  return disparity_params_;
}

cv::Mat StereoCalib::computeDisparity(
  const cv::Mat& left_image,
  const cv::Mat& right_image)
{
  // 根据配置选择视差计算算法
  if (disparity_params_.use_sgbm) {
    return computeDisparitySGBM(left_image, right_image);
  } else {
    return computeDisparityBM(left_image, right_image);
  }
}

cv::Mat StereoCalib::computeDisparitySGBM(
  const cv::Mat& left_image,
  const cv::Mat& right_image)
{
  // 转换为灰度图像
  cv::Mat left_gray, right_gray;
  
  if (left_image.channels() == 3) {
    cv::cvtColor(left_image, left_gray, cv::COLOR_BGR2GRAY);
  } else {
    left_gray = left_image;
  }
  
  if (right_image.channels() == 3) {
    cv::cvtColor(right_image, right_gray, cv::COLOR_BGR2GRAY);
  } else {
    right_gray = right_image;
  }

  // 计算视差图
  // SGBM使用半全局匹配算法，精度更高但速度较慢
  cv::Mat disparity;
  stereo_sgbm_->compute(left_gray, right_gray, disparity);
  
  // 视差值以16倍存储，需要除以16得到真实视差
  disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);
  
  return disparity;
}

cv::Mat StereoCalib::computeDisparityBM(
  const cv::Mat& left_image,
  const cv::Mat& right_image)
{
  // 转换为灰度图像
  cv::Mat left_gray, right_gray;
  
  if (left_image.channels() == 3) {
    cv::cvtColor(left_image, left_gray, cv::COLOR_BGR2GRAY);
  } else {
    left_gray = left_image;
  }
  
  if (right_image.channels() == 3) {
    cv::cvtColor(right_image, right_gray, cv::COLOR_BGR2GRAY);
  } else {
    right_gray = right_image;
  }

  // 计算视差图
  // BM使用块匹配算法，速度快但精度较低
  cv::Mat disparity;
  stereo_bm_->compute(left_gray, right_gray, disparity);
  
  // 视差值以16倍存储，需要除以16得到真实视差
  disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);
  
  return disparity;
}

// ==================== 深度估计与3D重建 ====================

cv::Mat StereoCalib::disparityToDepth(const cv::Mat& disparity) const
{
  if (!is_calibrated_) {
    std::cerr << "相机未标定，无法计算深度" << std::endl;
    return cv::Mat();
  }

  cv::Mat depth_map = cv::Mat::zeros(disparity.size(), CV_32F);
  
  // 获取焦距和基线距离
  double fx = intrinsics_.P1.at<double>(0, 0);
  double baseline = intrinsics_.baseline;

  // 深度计算公式: depth = (fx * baseline) / disparity
  // 视差越大，距离越近；视差越小，距离越远
  for (int y = 0; y < disparity.rows; ++y) {
    for (int x = 0; x < disparity.cols; ++x) {
      float d = disparity.at<float>(y, x);
      if (d > 0) {
        depth_map.at<float>(y, x) = static_cast<float>(fx * baseline / d);
      }
    }
  }

  return depth_map;
}

cv::Mat StereoCalib::depthToPointCloud(
  const cv::Mat& depth_map,
  const cv::Mat& left_image) const
{
  if (!is_calibrated_) {
    std::cerr << "相机未标定，无法生成点云" << std::endl;
    return cv::Mat();
  }

  cv::Mat points_3d;
  // 使用Q矩阵将深度图重投影到3D空间
  // Q矩阵包含了从视差/深度到3D坐标的映射关系
  cv::reprojectImageTo3D(depth_map, points_3d, intrinsics_.Q, true);

  return points_3d;
}

cv::Mat StereoCalib::disparityToPointCloud(
  const cv::Mat& disparity,
  const cv::Mat& left_image) const
{
  // 先转换为深度图，再生成点云
  cv::Mat depth_map = disparityToDepth(disparity);
  return depthToPointCloud(depth_map, left_image);
}

// ==================== 完整处理流程 ====================

StereoOutput StereoCalib::processStereoImages(
  const cv::Mat& left_image,
  const cv::Mat& right_image,
  bool compute_point_cloud)
{
  StereoOutput output;

  if (!is_calibrated_) {
    std::cerr << "相机未标定" << std::endl;
    output.is_valid = false;
    return output;
  }

  // 步骤1: 图像校正
  // 消除畸变，使极线水平对齐
  rectifyStereoImages(left_image, right_image, output.rectified_left, output.rectified_right);

  // 步骤2: 计算视差图
  // 在校正后的图像上计算立体匹配
  output.disparity = computeDisparity(output.rectified_left, output.rectified_right);

  // 步骤3: 视差图可视化
  output.disparity_normalized = visualizeDisparity(output.disparity);

  // 步骤4: 计算深度图
  // 从视差转换为真实距离
  output.depth_map = disparityToDepth(output.disparity);

  // 步骤5: 生成3D点云(可选)
  if (compute_point_cloud) {
    output.point_cloud = disparityToPointCloud(output.disparity, output.rectified_left);
  }

  output.is_valid = true;
  return output;
}

// ==================== 单点三角测量 ====================

bool StereoCalib::projectPointTo3D(
  const cv::Point2f& left_point,
  const cv::Point2f& right_point,
  cv::Point3f& point_3d) const
{
  if (!is_calibrated_) {
    return false;
  }

  // 计算视差: d = x_left - x_right
  float disparity = left_point.x - right_point.x;
  
  if (disparity <= 0) {
    return false;
  }

  // 从投影矩阵获取相机参数
  double fx = intrinsics_.P1.at<double>(0, 0);
  double fy = intrinsics_.P1.at<double>(1, 1);
  double cx = intrinsics_.P1.at<double>(0, 2);  // 主点x坐标
  double cy = intrinsics_.P1.at<double>(1, 2);  // 主点y坐标
  double baseline = intrinsics_.baseline;

  // 三角测量计算3D坐标
  // Z = fx * baseline / d
  // X = (x - cx) * Z / fx
  // Y = (y - cy) * Z / fy
  double z = fx * baseline / disparity;
  double x = (left_point.x - cx) * z / fx;
  double y = (left_point.y - cy) * z / fy;

  point_3d = cv::Point3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  
  return true;
}

// ==================== 可视化函数 ====================

cv::Mat StereoCalib::visualizeDisparity(const cv::Mat& disparity) const
{
  cv::Mat disparity_normalized;
  
  // 找到视差范围
  double min_val, max_val;
  cv::minMaxLoc(disparity, &min_val, &max_val);
  
  // 归一化到0-255并转换为8位图像
  disparity.convertTo(disparity_normalized, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
  
  // 应用伪彩色映射，便于观察
  cv::applyColorMap(disparity_normalized, disparity_normalized, cv::COLORMAP_JET);
  
  return disparity_normalized;
}

cv::Mat StereoCalib::visualizeDepth(const cv::Mat& depth_map) const
{
  cv::Mat depth_normalized;
  
  double min_val, max_val;
  cv::minMaxLoc(depth_map, &min_val, &max_val);
  
  depth_map.convertTo(depth_normalized, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
  
  cv::applyColorMap(depth_normalized, depth_normalized, cv::COLORMAP_JET);
  
  return depth_normalized;
}

// ==================== 信息打印 ====================

void StereoCalib::printCalibrationInfo() const
{
  std::cout << "=== Stereo Calibration Info ===" << std::endl;
  std::cout << "Image Size: " << params_.image_size.width << " x " << params_.image_size.height << std::endl;
  std::cout << "Board Size: " << params_.board_size.width << " x " << params_.board_size.height << std::endl;
  std::cout << "Square Size: " << params_.square_size << " m" << std::endl;
  std::cout << "Baseline: " << intrinsics_.baseline << " m" << std::endl;
  std::cout << "Calibrated: " << (is_calibrated_ ? "Yes" : "No") << std::endl;
  
  if (is_calibrated_) {
    std::cout << "\nLeft Camera Matrix:\n" << intrinsics_.camera_matrix_left << std::endl;
    std::cout << "\nLeft Distortion Coefficients:\n" << intrinsics_.dist_coeffs_left.t() << std::endl;
    std::cout << "\nRight Camera Matrix:\n" << intrinsics_.camera_matrix_right << std::endl;
    std::cout << "\nRight Distortion Coefficients:\n" << intrinsics_.dist_coeffs_right.t() << std::endl;
    std::cout << "\nRotation Matrix (R):\n" << intrinsics_.R << std::endl;
    std::cout << "\nTranslation Vector (T):\n" << intrinsics_.T << std::endl;
  }
  
  std::cout << "===============================" << std::endl;
}

}  // namespace stereo_calib
