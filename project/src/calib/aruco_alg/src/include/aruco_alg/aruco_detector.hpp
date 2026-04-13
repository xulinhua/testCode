#ifndef ARUCO_ALG__ARUCO_DETECTOR_HPP_
#define ARUCO_ALG__ARUCO_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>
#include <memory>

// 包含共用数据结构和基类
#include "comm_alg/marker_detect_base.hpp"

namespace aruco_alg
{

// 使用共用的数据结构
typedef comm_alg::MarkerInfo MarkerInfo;
typedef comm_alg::CameraIntrinsics CameraIntrinsics;
typedef comm_alg::DetectionResult DetectionResult;

struct ArucoDetectorInfo
{
    ArucoDetectorInfo(double marker_length, cv::aruco::PredefinedDictionaryType aruco_dict_type)
    {
      marker_length_ = marker_length;
      aruco_dict_type_ = aruco_dict_type;
      aruco_dict_ = cv::makePtr<cv::aruco::Dictionary>(
      cv::aruco::getPredefinedDictionary(aruco_dict_type));
      parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();
  
      // Optimize detection parameters for better performance
      parameters_->minCornerDistanceRate = 0.05;
      parameters_->minDistanceToBorder = 5;
      parameters_->minMarkerPerimeterRate = 0.1;
      parameters_->maxMarkerPerimeterRate = 4.0;
      parameters_->polygonalApproxAccuracyRate = 0.05;
      parameters_->minOtsuStdDev = 5.0;
      parameters_->errorCorrectionRate = 0.6;
  
      detector_ = cv::makePtr<cv::aruco::ArucoDetector>(*aruco_dict_, *parameters_);
    }
    double marker_length_;
    cv::aruco::PredefinedDictionaryType aruco_dict_type_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Ptr<cv::aruco::ArucoDetector> detector_;
};
typedef std::vector<ArucoDetectorInfo> ArucoDetectorInfoList;

class ArucoDetector : public comm_alg::MarkerDetectorBase
{
public:
  /**
   * @brief Constructor for ArucoDetector
   * 
   * @param marker_length Actual size of the marker in meters (default: 0.1m)
   * @param aruco_dict_type ArUco dictionary type (default: DICT_5X5_100)
   */
  explicit ArucoDetector(
    std::vector<double> marker_length = {0.1},
    std::vector<int> aruco_dict_type = {5});

  /**
   * @brief Destructor
   */
  ~ArucoDetector() = default;

  /**
   * @brief Detect ArUco markers in the given frame
   * 
   * @param frame Input image frame
   * @return std::vector<std::vector<cv::Point2f>> Detected marker corners
   * @return std::vector<int> Detected marker IDs
   * @return std::vector<std::vector<cv::Point2f>> Rejected image points
   */
bool detectMarkers(const cv::Ptr<cv::aruco::ArucoDetector> detector,
                  const cv::Mat& image, 
                  std::vector<std::vector<cv::Point2f>>& corners, 
                  std::vector<int>& ids,
                  std::vector<std::vector<cv::Point2f>>& rejected);

  /**
   * @brief Calculate the world coordinates of the marker corners
   * 
   * @param marker_length Length of the marker side
   * @param rotation_matrix Rotation matrix of the marker
   * @param tvec Translation vector of the marker
   * @return std::vector<cv::Point3f> World coordinates of the marker corners
   */
  std::vector<cv::Point3f> calculateCornerWorldCoordinates(
    double marker_length, const cv::Mat& rotation_matrix, const cv::Vec3d& tvec) const;

  /**
   * @brief Get the 2D center point of an ArUco marker
   * 
   * @param corners Detected marker corners
   * @param marker_index Index of the marker (default: 0)
   * @return cv::Point2f Center point in pixel coordinates
   */
  cv::Point2f getMarkerCenterPixel(
    const std::vector<std::vector<cv::Point2f>> & corners,
    size_t marker_index = 0) const;

  /**
   * @brief Get complete pose information using PCA method
   * 
   * @param corners Detected marker corners
   * @param depth_frame Depth frame data
   * @param intrinsics Camera intrinsics object
   * @param marker_index Index of the marker (default: 0)
   * @return MarkerInfo Complete pose information
   */
  MarkerInfo getMarkerResultPca(
    const std::vector<std::vector<cv::Point2f>> & corners,
    const cv::Mat & depth_frame,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs,
    size_t marker_index) const;

  /**
   * @brief Get complete pose information using PnP algorithm
   * 
   * @param corners Detected marker corners
   * @param ids Detected marker IDs
   * @param camera_matrix Camera intrinsic matrix
   * @param dist_coeffs Distortion coefficients
   * @param target_id Target marker ID (optional)
   * @return MarkerInfo Complete pose information
   */
  MarkerInfo getMarkerResultPnP(
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs,
    float marker_length,
    int target_id = -1) const;

  /**
   * @brief Get detection results for multiple ArUco markers
   * 
   * @param corners Detected marker corners
   * @param ids Detected marker IDs
   * @param camera_matrix Camera intrinsic matrix
   * @param dist_coeffs Distortion coefficients
   * @param depth_frame Depth frame data (optional)
   * @param intrinsics Camera intrinsics object (optional)
   * @return std::vector<MarkerInfo> List of marker information
   */
  std::vector<MarkerInfo> getArucoResults(
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs,
    float marker_length, 
    void * depth_frame = nullptr,
    void * intrinsics = nullptr
  ) const;

  /**
   * @brief Draw detection results on the frame
   * 
   * @param frame Input image frame
   * @param markers_info List of marker information
   * @param corners Detected marker corners
   * @param ids Detected marker IDs
   * @param camera_matrix Camera intrinsic matrix
   * @param dist_coeffs Distortion coefficients
   * @return cv::Mat Frame with drawn results
   */
  cv::Mat drawArucoResults(
    const cv::Mat & frame,
    const std::vector<MarkerInfo> & markers_info,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs) const;

  /**
   * @brief Print marker information to console
   * 
   * @param markers_info List of marker information
   */
  void printArucoResults(const std::vector<MarkerInfo> & markers_info) const;

  /**
   * @brief Detect and process ArUco markers (complete pipeline)
   * 
   * @param frame Input image frame
   * @param draw_results Whether to draw results on frame (default: true)
   * @param print_results Whether to print results to console (default: true)
   * @return DetectionResult Complete detection result
   */
  DetectionResult detectAndProcessMarkers(
    const cv::Mat & frame,
    void * depth_frame = nullptr,
    bool draw_results = true,
    bool print_results = true) override;

  // Setter methods for configuration
  void setEnableScaling(bool enable);
  void setScaleFactor(double factor);
  void setForcePnP(bool force_pnp);
  void setPrintDebugInfo(bool print_debug);

  // Getter methods
  double getMarkerLength() const;
  bool isEnableScaling() const;
  double getScaleFactor() const;
  bool isForcePnP() const;
  bool isPrintDebugInfo() const;

private:
  ArucoDetectorInfoList detector_info_list_;

  bool enable_scaling_;
  double scale_factor_;
  bool force_pnp_;
  bool print_debug_info_;

  /**
   * @brief Perform PCA analysis on point cloud
   */
  cv::Vec3f performPCA(const std::vector<cv::Point3f> & point_cloud) const;
};

}  // namespace aruco_alg

#endif  // ARUCO_ALG__ARUCO_DETECTOR_HPP_