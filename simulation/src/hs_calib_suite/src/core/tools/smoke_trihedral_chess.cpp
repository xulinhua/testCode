#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "hs_calib_suite/core/detectors/trihedral_chess_detector.hpp"
#include "hs_calib_suite/core/util/cv_bridge_local.hpp"

namespace fs = std::filesystem;

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  const std::string dir =
      argc > 1 ? argv[1]
               : "/home/hs/testCode/simulation/hs_calib_trihedral_oneshot_20260812_165511/images";
  const bool use_fast = (argc > 2 && std::string(argv[2]) == "fast");
  hs_calib::core::TrihedralTarget target(8, 8, 0.025);
  hs_calib::core::TrihedralChessDetector det(target);
  const auto speed = use_fast ? hs_calib::core::TrihedralChessDetectSpeed::Fast
                              : hs_calib::core::TrihedralChessDetectSpeed::Thorough;

  std::vector<fs::path> imgs;
  for (const auto &e : fs::directory_iterator(dir)) {
    if (!e.is_regular_file()) {
      continue;
    }
    const auto stem = e.path().stem().string();
    if (stem.size() == 2 &&
        stem.find_first_not_of("0123456789") == std::string::npos) {
      imgs.push_back(e.path());
    }
  }
  std::sort(imgs.begin(), imgs.end());

  int sum_pts = 0;
  int sum_faces = 0;
  int n3 = 0;
  double sum_ms = 0.0;
  for (const auto &p : imgs) {
    cv::Mat bgr = cv::imread(p.string(), cv::IMREAD_COLOR);
    if (bgr.empty()) {
      continue;
    }
    const hs_calib::core::ImageFrame frame =
        hs_calib::core::mat_as_image_frame(bgr);
    int faces = 0;
    const auto t0 = std::chrono::steady_clock::now();
    const auto corr = det.detect_merged(frame, &faces, speed);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    const int n = static_cast<int>(corr.image_points.rows());
    sum_pts += n;
    sum_faces += faces;
    sum_ms += ms;
    if (faces >= 3) {
      ++n3;
    }
    std::printf(
        "%s  pts=%3d  faces=%d  %.0fms\n", p.filename().c_str(), n, faces, ms);
  }
  std::printf(
      "--- mode=%s avg_pts=%.1f avg_faces=%.2f 3faces=%d/%zu avg_ms=%.0f\n",
      use_fast ? "fast" : "thorough",
      imgs.empty() ? 0.0 : sum_pts / static_cast<double>(imgs.size()),
      imgs.empty() ? 0.0 : sum_faces / static_cast<double>(imgs.size()), n3,
      imgs.size(),
      imgs.empty() ? 0.0 : sum_ms / static_cast<double>(imgs.size()));
  return 0;
}
