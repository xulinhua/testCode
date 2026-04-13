#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "aruco_alg/aruco_detector.hpp"

class ArucoDemoNode
{
public:
  ArucoDemoNode();
  ~ArucoDemoNode() = default;

  /**
   * @brief Run the demo application
   */
  void run();

public:
  std::unique_ptr<aruco_alg::ArucoDetector> detector_;
  cv::VideoCapture camera_;
  
  // Camera parameters (modify these according to your camera)
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  
  // Configuration
  int camera_id_;
  bool use_real_sense_;
  bool display_results_;
  bool print_console_;
  double marker_length_;
  bool enable_scaling_;
  double scale_factor_;

  /**
   * @brief Initialize camera and detector
   */
  void initialize();

  /**
   * @brief Load camera calibration data
   */
  void loadCameraCalibration();

  /**
   * @brief Process video from regular camera
   */
  void processRegularCamera();

  /**
   * @brief Print usage information
   */
  void printUsage();

  /**
   * @brief Parse command line arguments
   */
  void parseArguments(int argc, char * argv[]);
};

ArucoDemoNode::ArucoDemoNode()
: camera_id_(0),
  use_real_sense_(false),
  display_results_(true),
  print_console_(true),
  marker_length_(0.1),
  enable_scaling_(false),
  scale_factor_(1.0)
{
  detector_ = std::make_unique<aruco_alg::ArucoDetector>(std::vector<double>{marker_length_}, std::vector<int>{5});
}

void ArucoDemoNode::initialize()
{
  std::cout << "=== ArUco Marker Detection Demo ===" << std::endl;
  std::cout << "Camera ID: " << camera_id_ << std::endl;
  std::cout << "Use RealSense: " << (use_real_sense_ ? "Yes" : "No") << std::endl;
  std::cout << "Display Results: " << (display_results_ ? "Yes" : "No") << std::endl;
  std::cout << "Print Console: " << (print_console_ ? "Yes" : "No") << std::endl;
  std::cout << "Marker Length: " << marker_length_ << "m" << std::endl;
  std::cout << "Enable Scaling: " << (enable_scaling_ ? "Yes" : "No") << std::endl;
  if (enable_scaling_) {
    std::cout << "Scale Factor: " << scale_factor_ << std::endl;
  }
  std::cout << "======================================" << std::endl;

  // Load camera calibration
  loadCameraCalibration();

  // Set camera intrinsics for the detector
  if (!camera_matrix_.empty() && !dist_coeffs_.empty()) {
    detector_->setCameraIntrinsics(camera_matrix_, dist_coeffs_);
  } else {
    std::cout << "Warning: No camera calibration available, using default parameters" << std::endl;
    // Use default camera parameters (modify these according to your camera)
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
      800, 0, 320,
      0, 800, 240,
      0, 0, 1);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    detector_->setCameraIntrinsics(camera_matrix_, dist_coeffs_);
  }

  // Configure detector
  detector_->setEnableScaling(enable_scaling_);
  detector_->setScaleFactor(scale_factor_);
  detector_->setPrintDebugInfo(false);
}

void ArucoDemoNode::loadCameraCalibration()
{
  // Try to load calibration from file
  std::string calibration_file = "camera_calibration.yaml";
  cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
  
  if (fs.isOpened()) {
    std::cout << "Loading camera calibration from: " << calibration_file << std::endl;
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;
    fs.release();
    
    if (!camera_matrix_.empty() && !dist_coeffs_.empty()) {
      std::cout << "Camera calibration loaded successfully" << std::endl;
      std::cout << "Camera Matrix:" << std::endl << camera_matrix_ << std::endl;
      std::cout << "Distortion Coefficients:" << std::endl << dist_coeffs_ << std::endl;
    }
  } else {
    std::cout << "Camera calibration file not found: " << calibration_file << std::endl;
    std::cout << "Using default camera parameters" << std::endl;
  }
}

void ArucoDemoNode::processRegularCamera()
{
  if (!camera_.open(camera_id_)) {
    throw std::runtime_error("Failed to open camera with ID: " + std::to_string(camera_id_));
  }

  std::cout << "Camera opened successfully. Press 'q' to quit, 's' to save frame." << std::endl;

  cv::Mat frame;
  bool running = true;

  while (running) {
    camera_ >> frame;
    
    if (frame.empty()) {
      std::cout << "Failed to capture frame from camera" << std::endl;
      break;
    }

    try {
      // Detect and process ArUco markers
      auto result = detector_->detectAndProcessMarkers(
        frame, nullptr, display_results_, print_console_);

      // Display results if enabled and frame is available
      if (display_results_ && !result.processed_frame.empty()) {
        cv::imshow("ArUco Detection Demo", result.processed_frame);
      }

      // Handle keyboard input
      int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) {  // 'q' or ESC
        running = false;
      } else if (key == 's') {  // 's' to save frame
        std::string filename = "aruco_detection_" + 
          std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) + ".jpg";
        cv::imwrite(filename, result.processed_frame);
        std::cout << "Frame saved: " << filename << std::endl;
      }

    } catch (const std::exception & e) {
      std::cout << "Error during processing: " << e.what() << std::endl;
    }
  }

  camera_.release();
  cv::destroyAllWindows();
}

void ArucoDemoNode::printUsage()
{
  std::cout << "Usage: aruco_alg_demo_node [options]" << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --camera-id <id>     Camera device ID (default: 0)" << std::endl;
  std::cout << "  --realsense          Use RealSense camera (default: false)" << std::endl;
  std::cout << "  --no-display         Disable display window" << std::endl;
  std::cout << "  --no-console         Disable console output" << std::endl;
  std::cout << "  --marker-length <m>  Marker length in meters (default: 0.1)" << std::endl;
  std::cout << "  --enable-scaling     Enable image scaling for performance" << std::endl;
  std::cout << "  --scale-factor <f>   Image scaling factor (default: 0.5)" << std::endl;
  std::cout << "  --help, -h           Show this help message" << std::endl;
}

void ArucoDemoNode::parseArguments(int argc, char * argv[])
{
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    
    if (arg == "--help" || arg == "-h") {
      printUsage();
      exit(0);
    } else if (arg == "--camera-id" && i + 1 < argc) {
      camera_id_ = std::atoi(argv[++i]);
    } else if (arg == "--realsense") {
      use_real_sense_ = true;
    } else if (arg == "--no-display") {
      display_results_ = false;
    } else if (arg == "--no-console") {
      print_console_ = false;
    } else if (arg == "--marker-length" && i + 1 < argc) {
      marker_length_ = std::stod(argv[++i]);
      detector_ = std::make_unique<aruco_alg::ArucoDetector>(std::vector<double>{marker_length_}, std::vector<int>{5});
    } else if (arg == "--enable-scaling") {
      enable_scaling_ = true;
    } else if (arg == "--scale-factor" && i + 1 < argc) {
      scale_factor_ = std::stod(argv[++i]);
    }
  }
}

void ArucoDemoNode::run()
{
  try {
    initialize();
    
    if (use_real_sense_) {
      std::cout << "Error: RealSense support not compiled in" << std::endl;
      std::cout << "Please install RealSense SDK and recompile" << std::endl;
    } else {
      processRegularCamera();
    }
    
  } catch (const std::exception & e) {
    std::cout << "Error: " << e.what() << std::endl;
    printUsage();
  }
}

int main(int argc, char * argv[])
{
  try {
    ArucoDemoNode demo;
    demo.parseArguments(argc, argv);
    demo.run();
  } catch (const std::exception & e) {
    std::cout << "Fatal error: " << e.what() << std::endl;
    return -1;
  }
  
  return 0;
}