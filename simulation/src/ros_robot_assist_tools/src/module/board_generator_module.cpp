#include "ros_robot_assist_tools/module/board_generator_module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

#include <QFileInfo>

#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace ros_robot_assist_tools::ui
{
namespace
{

cv::aruco::PredefinedDictionaryType DictTypeFromIndex(int idx)
{
  static const std::vector<cv::aruco::PredefinedDictionaryType> dicts = {
    cv::aruco::DICT_4X4_50, cv::aruco::DICT_4X4_100, cv::aruco::DICT_4X4_250, cv::aruco::DICT_4X4_1000,
    cv::aruco::DICT_5X5_50, cv::aruco::DICT_5X5_100, cv::aruco::DICT_5X5_250, cv::aruco::DICT_5X5_1000,
    cv::aruco::DICT_6X6_50, cv::aruco::DICT_6X6_100, cv::aruco::DICT_6X6_250, cv::aruco::DICT_6X6_1000,
    cv::aruco::DICT_7X7_50, cv::aruco::DICT_7X7_100, cv::aruco::DICT_7X7_250, cv::aruco::DICT_7X7_1000,
    cv::aruco::DICT_ARUCO_ORIGINAL, cv::aruco::DICT_APRILTAG_16h5, cv::aruco::DICT_APRILTAG_25h9,
    cv::aruco::DICT_APRILTAG_36h10, cv::aruco::DICT_APRILTAG_36h11
  };
  if (idx < 0 || idx >= static_cast<int>(dicts.size())) { return cv::aruco::DICT_4X4_50; }
  return dicts[idx];
}

}  // namespace

bool GenerateCalibrationBoard(const BoardGeneratorParams & p, cv::Mat * out_image, QString * err_msg)
{
  try {
    if (out_image == nullptr) {
      if (err_msg != nullptr) { *err_msg = "out_image is null"; }
      return false;
    }
    const int type = p.board_type;
    if (type == 0) {
      const int marker_px = std::max(8, p.marker_size);
      int border_side = 0;
      if (p.aruco_white_border && p.aruco_border_px > 0) {
        border_side = std::min(2048, p.aruco_border_px);
      }
      const int out_wh = marker_px + 2 * border_side;
      out_image->release();
      out_image->create(out_wh, out_wh, CV_8UC1);
      out_image->setTo(cv::Scalar(255));
      cv::aruco::Dictionary dict_obj = cv::aruco::getPredefinedDictionary(DictTypeFromIndex(p.dict_index));
      cv::Mat marker_img;
      dict_obj.generateImageMarker(p.marker_id, marker_px, marker_img);
      marker_img.copyTo((*out_image)(cv::Rect(border_side, border_side, marker_px, marker_px)));
      return true;
    }

    if (type == 5) {
      // AprilGrid（calib.io / Kalibr 打印样式，DICT_APRILTAG_36h11）:
      //   tagSize = Tag 检测方格边到边 [mm]
      //   tagSpacing = 空白 / tagSize（0.3 → 12mm Tag 时空白 3.6mm）
      // 布局：Tag 间为白缝；每个白缝十字交叉处以及外圈，放置 gap×gap 的小黑方格。
      // 7×7 Tag → 8×8 个小黑方格。
      const int rows = std::max(2, p.rows);
      const int cols = std::max(2, p.cols);
      const double board_w_mm = std::max(1.0, p.board_width_mm);
      const double board_h_mm = std::max(1.0, p.board_height_mm);
      const double tag_mm = std::max(0.1, p.tag_size_mm);
      const double spacing = std::max(0.05, std::min(2.0, p.tag_spacing));
      const double gap_mm = tag_mm * spacing;
      const int start_marker_id = std::max(0, p.start_id);

      const double pattern_w_mm = (cols + 1) * gap_mm + cols * tag_mm;
      const double pattern_h_mm = (rows + 1) * gap_mm + rows * tag_mm;
      const double margin_mm = tag_mm;
      const double need_w_mm = pattern_w_mm + 2.0 * margin_mm;
      const double need_h_mm = pattern_h_mm + 2.0 * margin_mm;
      const double fit_scale = std::min(1.0, std::min(board_w_mm / need_w_mm, board_h_mm / need_h_mm));
      const double tag_draw_mm = tag_mm * fit_scale;
      const double gap_draw_mm = gap_mm * fit_scale;

      const double dpi = 300.0;
      const double ppm_phys = dpi / 25.4;
      const int export_cap = std::max(512, p.export_pixel_size);
      const double ppm_cap = std::min(
        static_cast<double>(export_cap) / board_w_mm,
        static_cast<double>(export_cap) / board_h_mm);
      const double ppm = std::min(ppm_phys, ppm_cap);

      const int canvas_w = std::max(64, static_cast<int>(std::round(board_w_mm * ppm)));
      const int canvas_h = std::max(64, static_cast<int>(std::round(board_h_mm * ppm)));
      out_image->release();
      out_image->create(canvas_h, canvas_w, CV_8UC1);
      out_image->setTo(cv::Scalar(255));

      const int tag_px = std::max(16, static_cast<int>(std::round(tag_draw_mm * ppm)));
      const int gap_px = std::max(2, static_cast<int>(std::round(gap_draw_mm * ppm)));
      const int step_px = tag_px + gap_px;
      const int pattern_w_px = (cols + 1) * gap_px + cols * tag_px;
      const int pattern_h_px = (rows + 1) * gap_px + rows * tag_px;
      const int origin_x = (canvas_w - pattern_w_px) / 2;
      const int origin_y = (canvas_h - pattern_h_px) / 2;

      // 小黑方格：外圈 + Tag 间白缝十字交叉点，(rows+1)×(cols+1) 个
      for (int gr = 0; gr <= rows; ++gr) {
        for (int gc = 0; gc <= cols; ++gc) {
          cv::rectangle(
            *out_image,
            cv::Rect(origin_x + gc * step_px, origin_y + gr * step_px, gap_px, gap_px),
            cv::Scalar(0), cv::FILLED);
        }
      }

      cv::aruco::Dictionary dict_obj =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
      const int dict_size = std::max(1, dict_obj.bytesList.rows);
      if (start_marker_id + rows * cols > dict_size) {
        if (err_msg != nullptr) {
          *err_msg = QString("AprilTag 36h11 字典只有 %1 个 ID，当前需要 ID [%2 .. %3)")
                       .arg(dict_size)
                       .arg(start_marker_id)
                       .arg(start_marker_id + rows * cols);
        }
        return false;
      }

      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          const int id = start_marker_id + r * cols + c;
          cv::Mat tag_img;
          dict_obj.generateImageMarker(id, tag_px, tag_img);
          const int x = origin_x + gap_px + c * step_px;
          const int y = origin_y + gap_px + r * step_px;
          tag_img.copyTo((*out_image)(cv::Rect(x, y, tag_px, tag_px)));
        }
      }

      return true;
    }

    const int size = std::max(128, p.marker_size);
    const int rows = std::max(2, p.rows);
    const int cols = std::max(2, p.cols);
    const double board_w_mm = std::max(1.0, p.board_width_mm);
    const double board_h_mm = std::max(1.0, p.board_height_mm);
    const double cell_mm = std::max(1.0, p.cell_size_mm);
    const double circle_mm = std::max(0.1, p.circle_diameter_mm);
    const int start_marker_id = std::max(0, p.start_id);
    const double marker_ratio_value = std::max(0.1, std::min(0.95, p.marker_ratio));

    out_image->release();
    out_image->create(size, size, CV_8UC1);
    out_image->setTo(cv::Scalar(255));

    const int canvas_margin_px = std::max(8, size / 40);
    const double ppm = std::min(
      static_cast<double>(size - 2 * canvas_margin_px) / board_w_mm,
      static_cast<double>(size - 2 * canvas_margin_px) / board_h_mm);
    const int board_w_px = std::max(2, static_cast<int>(std::round(board_w_mm * ppm)));
    const int board_h_px = std::max(2, static_cast<int>(std::round(board_h_mm * ppm)));
    const int board_x = (size - board_w_px) / 2;
    const int board_y = (size - board_h_px) / 2;
    cv::rectangle(*out_image, cv::Rect(board_x, board_y, board_w_px, board_h_px), cv::Scalar(255), cv::FILLED);

    if (type == 1) {
    const int cell = std::max(2, static_cast<int>(std::round(cell_mm * ppm)));
    const int grid_w = cols * cell;
    const int grid_h = rows * cell;
    const int off_x = board_x + (board_w_px - grid_w) / 2;
    const int off_y = board_y + (board_h_px - grid_h) / 2;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        if ((r + c) % 2 == 0) {
          cv::rectangle(*out_image, cv::Rect(off_x + c * cell, off_y + r * cell, cell, cell), cv::Scalar(0), cv::FILLED);
        }
      }
    }
    } else if (type == 2 || type == 3) {
      const double desired_step_x = std::max(1.0, cell_mm * ppm);
      const double desired_step_y = std::max(1.0, cell_mm * ppm);
      const double max_step_x_sym = (cols > 1) ? (static_cast<double>(board_w_px) / static_cast<double>(cols - 1)) : desired_step_x;
      const double max_step_x_asym = (cols > 1) ? (static_cast<double>(board_w_px) / (static_cast<double>(cols) - 0.5)) : desired_step_x;
      const double max_step_y = (rows > 1) ? (static_cast<double>(board_h_px) / static_cast<double>(rows - 1)) : desired_step_y;
      const double step_x = std::max(1.0, std::min(desired_step_x, (type == 3) ? max_step_x_asym : max_step_x_sym));
      const double step_y = std::max(1.0, std::min(desired_step_y, max_step_y));

      const double radius_px_by_mm = (circle_mm * 0.5) * ppm;
      const double radius_px_by_pitch = 0.28 * std::min(step_x, step_y);
      const int radius = std::max(2, static_cast<int>(std::round(std::min(radius_px_by_mm, radius_px_by_pitch))));

      const double grid_w = (cols - 1) * step_x + ((type == 3) ? (0.5 * step_x) : 0.0);
      const double grid_h = (rows - 1) * step_y;
      const double start_x = board_x + (board_w_px - grid_w) * 0.5;
      const double start_y = board_y + (board_h_px - grid_h) * 0.5;
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          const double offset = (type == 3 && (r % 2 == 1)) ? (0.5 * step_x) : 0.0;
          const int cx = static_cast<int>(std::round(start_x + c * step_x + offset));
          const int cy = static_cast<int>(std::round(start_y + r * step_y));
          if (cx >= board_x + radius && cx < board_x + board_w_px - radius &&
              cy >= board_y + radius && cy < board_y + board_h_px - radius) {
            cv::circle(*out_image, cv::Point(cx, cy), radius, cv::Scalar(0), cv::FILLED);
          }
        }
      }
    } else if (type == 4) {
      cv::aruco::Dictionary dict_obj = cv::aruco::getPredefinedDictionary(DictTypeFromIndex(p.dict_index));
      const int cell_px_target = std::max(8, static_cast<int>(std::round(cell_mm * ppm)));
      const int cell_px_w_fit = std::max(8, board_w_px / cols);
      const int cell_px_h_fit = std::max(8, board_h_px / rows);
      const int cell_px = std::min(cell_px_target, std::min(cell_px_w_fit, cell_px_h_fit));
      const int grid_w = cols * cell_px;
      const int grid_h = rows * cell_px;
      const int off_x = board_x + (board_w_px - grid_w) / 2;
      const int off_y = board_y + (board_h_px - grid_h) / 2;
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          const int x = off_x + c * cell_px;
          const int y = off_y + r * cell_px;
          const bool black_cell = ((r + c) % 2 == 0);
          cv::rectangle(*out_image, cv::Rect(x, y, cell_px, cell_px), black_cell ? cv::Scalar(0) : cv::Scalar(255), cv::FILLED);
        }
      }
      const int dict_size = std::max(1, dict_obj.bytesList.rows);
      const int min_marker_px = std::max(8, dict_obj.markerSize + 2);
      const int marker_px = std::max(min_marker_px, std::min(cell_px - 2, static_cast<int>(std::round(cell_px * marker_ratio_value))));
      int marker_idx = 0;
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          if ((r + c) % 2 == 0) continue;
          const int id = (start_marker_id + marker_idx) % dict_size;
          marker_idx++;
          cv::Mat marker_img;
          dict_obj.generateImageMarker(id, marker_px, marker_img);
          const int cell_x = off_x + c * cell_px;
          const int cell_y = off_y + r * cell_px;
          const int mx = cell_x + (cell_px - marker_px) / 2;
          const int my = cell_y + (cell_px - marker_px) / 2;
          marker_img.copyTo((*out_image)(cv::Rect(mx, my, marker_px, marker_px)));
        }
      }
    } else {
      if (err_msg != nullptr) { *err_msg = QString("unknown board_type=%1").arg(type); }
      return false;
    }
    return true;
  } catch (const std::exception & e) {
    if (err_msg != nullptr) { *err_msg = e.what(); }
    return false;
  }
}

bool ExportCalibrationBoardImage(const cv::Mat & image, const QString & path, QString * err_msg)
{
  try {
    if (path.trimmed().isEmpty()) {
      if (err_msg != nullptr) { *err_msg = "empty output path"; }
      return false;
    }
    if (image.empty()) {
      if (err_msg != nullptr) { *err_msg = "empty image"; }
      return false;
    }
    if (!cv::imwrite(path.toStdString(), image)) {
      if (err_msg != nullptr) { *err_msg = "imwrite failed"; }
      return false;
    }
    return true;
  } catch (const std::exception & e) {
    if (err_msg != nullptr) { *err_msg = e.what(); }
    return false;
  }
}

bool ExportCalibrationBoardDae(const cv::Mat & image, const QString & dae_path, QString * err_msg)
{
  try {
    if (dae_path.trimmed().isEmpty()) {
      if (err_msg != nullptr) { *err_msg = "empty dae path"; }
      return false;
    }
    if (image.empty()) {
      if (err_msg != nullptr) { *err_msg = "empty image"; }
      return false;
    }
    QString normalized_dae = dae_path;
    if (!normalized_dae.endsWith(".dae", Qt::CaseInsensitive)) {
      normalized_dae += ".dae";
    }
    const QFileInfo dae_info(normalized_dae);
    const QString texture = dae_info.path() + "/" + dae_info.completeBaseName() + ".png";
    if (!cv::imwrite(texture.toStdString(), image)) {
      if (err_msg != nullptr) { *err_msg = "save texture failed"; }
      return false;
    }
    std::ofstream out(normalized_dae.toStdString());
    if (!out.is_open()) {
      if (err_msg != nullptr) { *err_msg = "open dae failed"; }
      return false;
    }
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n";
    out << "  <asset><contributor><author>ros_robot_assist_tools</author></contributor></asset>\n";
    out << "  <library_images><image id=\"aruco_tex\"><init_from>" << QFileInfo(texture).fileName().toStdString()
        << "</init_from></image></library_images>\n";
    out << "  <scene><instance_visual_scene url=\"#Scene\"/></scene>\n";
    out << "</COLLADA>\n";
    out.close();
    return true;
  } catch (const std::exception & e) {
    if (err_msg != nullptr) { *err_msg = e.what(); }
    return false;
  }
}

}  // namespace ros_robot_assist_tools::ui
