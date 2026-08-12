#include "hs_calib_suite/core/detectors/chessboard_detector.hpp"
#include "hs_calib_suite/core/targets/chessboard_target.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"
#include "hs_calib_suite/core/io/export_camera_yaml.hpp"
#include "hs_calib_suite/core/registry/registry.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;

namespace {

/// \brief 按针孔模型把棋盘画进图像（用于可检测的合成数据）
cv::Mat render_chessboard_view(
    int squares_x, int squares_y, double square_m, const cv::Mat &K,
    const cv::Mat &rvec, const cv::Mat &tvec, int width, int height) {
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(48, 48, 48));
  for (int y = 0; y < squares_y + 1; ++y) {
    for (int x = 0; x < squares_x + 1; ++x) {
      if ((x + y) % 2 != 0) {
        continue;
      }
      std::vector<cv::Point3f> sq = {
          {static_cast<float>(x * square_m), static_cast<float>(y * square_m), 0.f},
          {static_cast<float>((x + 1) * square_m), static_cast<float>(y * square_m), 0.f},
          {static_cast<float>((x + 1) * square_m), static_cast<float>((y + 1) * square_m), 0.f},
          {static_cast<float>(x * square_m), static_cast<float>((y + 1) * square_m), 0.f},
      };
      std::vector<cv::Point2f> proj;
      cv::projectPoints(sq, rvec, tvec, K, cv::Mat(), proj);
      std::vector<cv::Point> poly;
      bool ok = true;
      for (const auto &p : proj) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
          ok = false;
          break;
        }
        poly.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));
      }
      if (ok) {
        cv::fillConvexPoly(img, poly, cv::Scalar(0, 0, 0));
      }
    }
  }
  // 白格边线，提升角点对比
  for (int y = 0; y <= squares_y + 1; ++y) {
    for (int x = 0; x <= squares_x + 1; ++x) {
      if ((x + y) % 2 == 0) {
        continue;
      }
      if (x >= squares_x + 1 || y >= squares_y + 1) {
        continue;
      }
      std::vector<cv::Point3f> sq = {
          {static_cast<float>(x * square_m), static_cast<float>(y * square_m), 0.f},
          {static_cast<float>((x + 1) * square_m), static_cast<float>(y * square_m), 0.f},
          {static_cast<float>((x + 1) * square_m), static_cast<float>((y + 1) * square_m), 0.f},
          {static_cast<float>(x * square_m), static_cast<float>((y + 1) * square_m), 0.f},
      };
      std::vector<cv::Point2f> proj;
      cv::projectPoints(sq, rvec, tvec, K, cv::Mat(), proj);
      std::vector<cv::Point> poly;
      for (const auto &p : proj) {
        poly.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));
      }
      cv::fillConvexPoly(img, poly, cv::Scalar(245, 245, 245));
    }
  }
  return img;
}

}  // namespace

int main(int argc, char **argv) {
  const int sx = 9;
  const int sy = 6;
  const double square_m = 0.025;
  const int width = 800;
  const int height = 600;
  const fs::path out_dir =
      (argc > 1) ? fs::path(argv[1]) : fs::temp_directory_path() / "hs_calib_chessboard";
  fs::create_directories(out_dir);

  cv::Mat K = (cv::Mat_<double>(3, 3) << 700.0, 0.0, 400.0, 0.0, 700.0, 300.0, 0.0, 0.0, 1.0);

  // 多姿态：绕轴小角度 + 距离变化
  struct Pose {
    double rx, ry, rz, tx, ty, tz;
  };
  const Pose poses[] = {
      {0.15, -0.25, 0.05, -0.08, -0.05, 0.45},
      {-0.20, 0.18, -0.08, -0.05, -0.06, 0.50},
      {0.05, 0.10, 0.20, -0.10, -0.04, 0.42},
      {-0.12, -0.15, 0.12, -0.06, -0.08, 0.55},
      {0.22, 0.08, -0.15, -0.09, -0.03, 0.48},
      {-0.08, 0.22, 0.05, -0.07, -0.07, 0.52},
      {0.10, -0.18, -0.10, -0.04, -0.05, 0.46},
      {-0.18, -0.05, 0.18, -0.11, -0.06, 0.58},
      {0.00, 0.00, 0.00, -0.09, -0.06, 0.50},
      {0.16, 0.14, 0.08, -0.05, -0.04, 0.44},
  };

  std::vector<fs::path> paths;
  for (int i = 0; i < 10; ++i) {
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << poses[i].rx, poses[i].ry, poses[i].rz);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << poses[i].tx, poses[i].ty, poses[i].tz);
    cv::Mat img = render_chessboard_view(sx, sy, square_m, K, rvec, tvec, width, height);
    // 轻微模糊，更接近真实成像
    cv::GaussianBlur(img, img, cv::Size(3, 3), 0.6);
    const fs::path p = out_dir / ("board_" + std::to_string(i) + ".png");
    cv::imwrite(p.string(), img);
    paths.push_back(p);
  }

  hs_calib::core::ChessboardTarget target(sx, sy, square_m);
  hs_calib::core::ChessboardDetector detector(target);
  hs_calib::core::ObservationBatch batch;
  for (const auto &p : paths) {
    cv::Mat bgr = cv::imread(p.string(), cv::IMREAD_COLOR);
    auto corrs = detector.detect(hs_calib::core::mat_as_image_frame(bgr, "bgr8"));
    if (corrs.empty()) {
      std::cerr << "detect fail: " << p << "\n";
      continue;
    }
    hs_calib::core::Observation obs;
    obs.source_path = p.string();
    obs.image_width = bgr.cols;
    obs.image_height = bgr.rows;
    obs.correspondences = corrs;
    batch.items.push_back(obs);
  }
  std::cout << "detected views: " << batch.items.size() << "\n";
  if (batch.items.size() < 3) {
    return 2;
  }

  auto cal = hs_calib::core::CalibratorRegistry::instance().create("cam_intrinsics");
  std::map<std::string, std::string> cfg = {
      {"squares_x", "9"},
      {"squares_y", "6"},
      {"square_length", "0.025"},
  };
  const auto result = cal->calibrate(batch, cfg);
  std::cout << hs_calib::core::format_intrinsics_text(result);
  if (!result.success) {
    return 3;
  }

  const double fx = std::stod(result.intrinsics_meta.at("fx"));
  const double fy = std::stod(result.intrinsics_meta.at("fy"));
  const double cx = std::stod(result.intrinsics_meta.at("cx"));
  const double cy = std::stod(result.intrinsics_meta.at("cy"));
  const double rms = result.metrics.at("reprojection_rmse");
  if (rms > 1.0 || std::abs(fx - 700.0) > 80.0 || std::abs(fy - 700.0) > 80.0 ||
      std::abs(cx - 400.0) > 40.0 || std::abs(cy - 300.0) > 40.0)
  {
    std::cerr << "intrinsics out of tolerance vs ground-truth K\n";
    return 5;
  }

  const fs::path yaml = out_dir / "camera_intrinsics.yaml";
  if (!hs_calib::core::export_camera_yaml(result, yaml.string())) {
    std::cerr << "export failed\n";
    return 4;
  }
  std::cout << "exported: " << yaml << "\n";
  std::cout << "images: " << out_dir << "\n";
  std::cout << "SMOKE OK\n";
  return 0;
}
