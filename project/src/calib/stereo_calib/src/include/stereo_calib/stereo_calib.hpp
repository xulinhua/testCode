#ifndef STEREO_CALIB__STEREO_CALIB_HPP_
#define STEREO_CALIB__STEREO_CALIB_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace stereo_calib
{

/**
 * @brief 双目标定参数配置结构体
 * 
 * 用于存储棋盘格标定板和图像的基本参数配置
 */
struct StereoCalibParams
{
  cv::Size board_size;    ///< 棋盘格内角点数量 (宽度, 高度)
  float square_size;      ///< 棋盘格方格边长，单位：米
  cv::Size image_size;    ///< 图像分辨率 (宽度, 高度)
  
  StereoCalibParams()
  : board_size(9, 6), square_size(0.025f), image_size(640, 480)
  {}
};

/**
 * @brief 双目相机内参和外参结构体
 * 
 * 存储双目相机标定后的所有参数，包括左右相机内参、
 * 相对位姿关系、校正参数和投影矩阵等
 */
struct StereoCameraIntrinsics
{
  cv::Mat camera_matrix_left;   ///< 左相机内参矩阵 (3x3)
  cv::Mat dist_coeffs_left;     ///< 左相机畸变系数 (5x1): [k1, k2, p1, p2, k3]
  cv::Mat camera_matrix_right;  ///< 右相机内参矩阵 (3x3)
  cv::Mat dist_coeffs_right;    ///< 右相机畸变系数 (5x1)
  
  cv::Mat R;    ///< 右相机相对于左相机的旋转矩阵 (3x3)
  cv::Mat T;    ///< 右相机相对于左相机的平移向量 (3x1)
  cv::Mat E;    ///< 本质矩阵 (Essential Matrix, 3x3)
  cv::Mat F;    ///< 基础矩阵 (Fundamental Matrix, 3x3)
  
  cv::Mat R1;   ///< 左相机校正旋转矩阵 (3x3)
  cv::Mat R2;   ///< 右相机校正旋转矩阵 (3x3)
  cv::Mat P1;   ///< 左相机投影矩阵 (3x4)
  cv::Mat P2;   ///< 右相机投影矩阵 (3x4)
  cv::Mat Q;    ///< 视差到深度映射矩阵 (4x4)
  
  cv::Rect valid_roi_left;   ///< 左相机校正后的有效区域
  cv::Rect valid_roi_right;  ///< 右相机校正后的有效区域
  
  double baseline;         ///< 双目基线距离，单位：米
  bool is_calibrated;      ///< 是否已完成标定
  
  StereoCameraIntrinsics() : baseline(0.0), is_calibrated(false) {}
};

/**
 * @brief 立体校正映射表结构体
 * 
 * 存储图像校正所需的映射表，用于快速校正图像
 */
struct StereoRectifyMaps
{
  cv::Mat map1_left;   ///< 左相机校正映射表1 (坐标映射)
  cv::Mat map2_left;   ///< 左相机校正映射表2 (插值系数)
  cv::Mat map1_right;  ///< 右相机校正映射表1 (坐标映射)
  cv::Mat map2_right;  ///< 右相机校正映射表2 (插值系数)
  bool is_initialized; ///< 映射表是否已初始化
  
  StereoRectifyMaps() : is_initialized(false) {}
};

/**
 * @brief 视差计算参数结构体
 * 
 * 配置BM/SGBM立体匹配算法的各项参数
 */
struct DisparityParams
{
  int num_disparities;      ///< 视差搜索范围，必须为16的倍数
  int block_size;           ///< 匹配块大小，奇数，范围[5,21]
  int pre_filter_cap;       ///< 预滤波截断值，范围[1,63]
  int pre_filter_size;      ///< 预滤波窗口大小，奇数，范围[5,255]
  int min_disparity;        ///< 最小视差值
  int texture_threshold;    ///< 纹理阈值，低于此值的区域不计算视差
  int uniqueness_ratio;     ///< 唯一性比率，范围[0,100]
  int speckle_window_size;  ///< 斑点滤波窗口大小
  int speckle_range;        ///< 斑点滤波视差变化阈值
  int disp12_max_diff;      ///< 左右一致性检查最大差异
  bool use_sgbm;            ///< true: 使用SGBM算法, false: 使用BM算法
  
  DisparityParams()
  : num_disparities(64),
    block_size(11),
    pre_filter_cap(31),
    pre_filter_size(9),
    min_disparity(0),
    texture_threshold(10),
    uniqueness_ratio(15),
    speckle_window_size(100),
    speckle_range(32),
    disp12_max_diff(1),
    use_sgbm(true)
  {}
};

/**
 * @brief 双目处理输出结果结构体
 * 
 * 包含双目图像处理后的所有输出数据
 */
struct StereoOutput
{
  cv::Mat disparity;           ///< 视差图 (CV_32F，单位：像素)
  cv::Mat disparity_normalized;///< 归一化彩色视差图 (用于可视化)
  cv::Mat depth_map;           ///< 深度图 (CV_32F，单位：米)
  cv::Mat point_cloud;         ///< 3D点云 (CV_32FC3，每个像素对应一个3D点)
  cv::Mat rectified_left;      ///< 校正后的左相机图像
  cv::Mat rectified_right;     ///< 校正后的右相机图像
  bool is_valid;               ///< 处理结果是否有效
  
  StereoOutput() : is_valid(false) {}
};

/**
 * @brief 双目相机标定类
 * 
 * 提供完整的双目相机标定、图像校正、视差计算、深度估计和3D重建功能
 * 
 * 典型使用流程：
 * @code
 *   // 1. 创建标定对象并配置参数
 *   stereo_calib::StereoCalibParams params;
 *   params.board_size = cv::Size(6, 9);
 *   params.square_size = 0.04f;
 *   stereo_calib::StereoCalib stereo(params);
 *   
 *   // 2. 执行标定
 *   stereo.calibrateFromFolders(left_folder, right_folder);
 *   
 *   // 3. 处理图像
 *   StereoOutput output = stereo.processStereoImages(left_img, right_img);
 * @endcode
 */
class StereoCalib
{
public:
  /**
   * @brief 默认构造函数
   */
  explicit StereoCalib();
  
  /**
   * @brief 带参数的构造函数
   * @param params 标定参数配置
   */
  explicit StereoCalib(const StereoCalibParams& params);
  
  ~StereoCalib() = default;

  void setBoardSize(const cv::Size& board_size);
  void setSquareSize(float square_size);
  void setImageSize(const cv::Size& image_size);
  
  cv::Size getBoardSize() const;
  float getSquareSize() const;
  cv::Size getImageSize() const;

  /**
   * @brief 从图像向量执行双目标定
   * @param left_images 左相机图像集合
   * @param right_images 右相机图像集合
   * @return true-标定成功，false-标定失败
   */
  bool calibrateFromImages(
    const std::vector<cv::Mat>& left_images,
    const std::vector<cv::Mat>& right_images);

  /**
   * @brief 从文件夹加载图像并执行双目标定
   * @param left_folder 左相机图像文件夹路径
   * @param right_folder 右相机图像文件夹路径
   * @return true-标定成功，false-标定失败
   */
  bool calibrateFromFolders(
    const std::string& left_folder,
    const std::string& right_folder);

  void setCameraIntrinsics(const StereoCameraIntrinsics& intrinsics);
  StereoCameraIntrinsics getCameraIntrinsics() const;

  bool loadCalibration(const std::string& filepath);
  bool saveCalibration(const std::string& filepath) const;
  bool loadCalibrationFromYaml(const std::string& filepath);
  bool saveCalibrationToYaml(const std::string& filepath) const;

  /**
   * @brief 初始化图像校正映射表
   * 
   * 根据标定结果计算图像校正所需的映射表，
   * 必须在标定完成后调用
   */
  void initRectifyMaps();
  
  bool isRectifyMapsInitialized() const;

  /**
   * @brief 校正单张图像
   * @param image 输入图像
   * @param is_left true-左相机图像，false-右相机图像
   * @return 校正后的图像
   */
  cv::Mat rectifyImage(const cv::Mat& image, bool is_left) const;
  
  /**
   * @brief 校正立体图像对
   * @param left_image 左相机图像
   * @param right_image 右相机图像
   * @param rectified_left 输出校正后的左相机图像
   * @param rectified_right 输出校正后的右相机图像
   */
  void rectifyStereoImages(
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    cv::Mat& rectified_left,
    cv::Mat& rectified_right) const;

  void setDisparityParams(const DisparityParams& params);
  DisparityParams getDisparityParams() const;

  /**
   * @brief 计算视差图
   * @param left_image 校正后的左相机图像
   * @param right_image 校正后的右相机图像
   * @return 视差图 (CV_32F)
   */
  cv::Mat computeDisparity(
    const cv::Mat& left_image,
    const cv::Mat& right_image);

  /**
   * @brief 使用SGBM算法计算视差图
   * @param left_image 校正后的左相机图像
   * @param right_image 校正后的右相机图像
   * @return 视差图 (CV_32F)
   */
  cv::Mat computeDisparitySGBM(
    const cv::Mat& left_image,
    const cv::Mat& right_image);

  /**
   * @brief 使用BM算法计算视差图
   * @param left_image 校正后的左相机图像
   * @param right_image 校正后的右相机图像
   * @return 视差图 (CV_32F)
   */
  cv::Mat computeDisparityBM(
    const cv::Mat& left_image,
    const cv::Mat& right_image);

  /**
   * @brief 视差图转深度图
   * @param disparity 视差图 (CV_32F)
   * @return 深度图 (CV_32F，单位：米)
   * 
   * 深度计算公式: depth = (fx * baseline) / disparity
   */
  cv::Mat disparityToDepth(const cv::Mat& disparity) const;

  /**
   * @brief 深度图转3D点云
   * @param depth_map 深度图
   * @param left_image 左相机图像 (可选，用于获取颜色信息)
   * @return 3D点云 (CV_32FC3)
   */
  cv::Mat depthToPointCloud(
    const cv::Mat& depth_map,
    const cv::Mat& left_image = cv::Mat()) const;

  /**
   * @brief 视差图转3D点云
   * @param disparity 视差图
   * @param left_image 左相机图像 (可选，用于获取颜色信息)
   * @return 3D点云 (CV_32FC3)
   */
  cv::Mat disparityToPointCloud(
    const cv::Mat& disparity,
    const cv::Mat& left_image = cv::Mat()) const;

  /**
   * @brief 完整的双目图像处理流程
   * @param left_image 左相机原始图像
   * @param right_image 右相机原始图像
   * @param compute_point_cloud 是否计算点云
   * @return 处理结果结构体
   * 
   * 处理流程：图像校正 -> 视差计算 -> 深度估计 -> 3D重建
   */
  StereoOutput processStereoImages(
    const cv::Mat& left_image,
    const cv::Mat& right_image,
    bool compute_point_cloud = true);

  /**
   * @brief 将匹配的左右图像点投影到3D空间
   * @param left_point 左图像上的2D点
   * @param right_point 右图像上的对应2D点
   * @param point_3d 输出3D点坐标
   * @return true-投影成功，false-投影失败
   * 
   * 三角测量原理：
   * - 视差 d = left_point.x - right_point.x
   * - 深度 Z = fx * baseline / d
   * - X = (left_point.x - cx) * Z / fx
   * - Y = (left_point.y - cy) * Z / fy
   */
  bool projectPointTo3D(
    const cv::Point2f& left_point,
    const cv::Point2f& right_point,
    cv::Point3f& point_3d) const;

  /**
   * @brief 可视化视差图
   * @param disparity 视差图
   * @return 彩色可视化的视差图
   */
  cv::Mat visualizeDisparity(const cv::Mat& disparity) const;
  
  /**
   * @brief 可视化深度图
   * @param depth_map 深度图
   * @return 彩色可视化的深度图
   */
  cv::Mat visualizeDepth(const cv::Mat& depth_map) const;

  void printCalibrationInfo() const;

private:
  bool detectChessboardCorners(
    const cv::Mat& image,
    std::vector<cv::Point2f>& corners);

  void computeReprojectionErrors(
    const std::vector<std::vector<cv::Point3f>>& object_points,
    const std::vector<std::vector<cv::Point2f>>& image_points_left,
    const std::vector<std::vector<cv::Point2f>>& image_points_right,
    double& rms_left,
    double& rms_right,
    double& rms_stereo);

  std::vector<cv::Point3f> generateObjectPoints() const;

private:
  StereoCalibParams params_;           ///< 标定参数
  StereoCameraIntrinsics intrinsics_;  ///< 相机内参外参
  StereoRectifyMaps rectify_maps_;     ///< 校正映射表
  DisparityParams disparity_params_;   ///< 视差计算参数
  cv::Ptr<cv::StereoBM> stereo_bm_;    ///< BM立体匹配器
  cv::Ptr<cv::StereoSGBM> stereo_sgbm_;///< SGBM立体匹配器
  cv::TermCriteria term_criteria_;     ///< 迭代终止条件
  bool is_calibrated_;                 ///< 是否已标定
};

}  // namespace stereo_calib

#endif  // STEREO_CALIB__STEREO_CALIB_HPP_
