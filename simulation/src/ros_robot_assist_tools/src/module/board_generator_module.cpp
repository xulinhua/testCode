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
    const int size = std::max(128, p.marker_size);
    const int rows = std::max(2, p.rows);
    const int cols = std::max(2, p.cols);
    const double board_w_mm = std::max(1.0, p.board_width_mm);
    const double board_h_mm = std::max(1.0, p.board_height_mm);
    const double cell_mm = std::max(1.0, p.cell_size_mm);
    const double circle_mm = std::max(0.1, p.circle_diameter_mm);
    const double tag_mm = std::max(0.1, p.tag_size_mm);
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

    const int type = p.board_type;
    if (type == 0) {
    cv::aruco::Dictionary dict_obj = cv::aruco::getPredefinedDictionary(DictTypeFromIndex(p.dict_index));
    const int marker_px = std::max(20, std::min(board_w_px, board_h_px));
    cv::Mat marker_img;
    dict_obj.generateImageMarker(p.marker_id, marker_px, marker_img);
    const int mx = board_x + (board_w_px - marker_px) / 2;
    const int my = board_y + (board_h_px - marker_px) / 2;
    marker_img.copyTo((*out_image)(cv::Rect(mx, my, marker_px, marker_px)));
    } else if (type == 1) {
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
      cv::aruco::Dictionary dict_obj = cv::aruco::getPredefinedDictionary(DictTypeFromIndex(p.dict_index));
      const int dict_size = std::max(1, dict_obj.bytesList.rows);
      const int pitch_x = std::max(8, board_w_px / cols);
      const int pitch_y = std::max(8, board_h_px / rows);
      const int total_w = cols * pitch_x;
      const int total_h = rows * pitch_y;
      const int start_x = board_x + (board_w_px - total_w) / 2;
      const int start_y = board_y + (board_h_px - total_h) / 2;
      const int min_tag_px = std::max(8, dict_obj.markerSize + 2);
      const int tag_px_from_mm = std::max(min_tag_px, static_cast<int>(std::round(tag_mm * ppm)));
      const int cell_inner_px = std::max(8, std::min(pitch_x, pitch_y) - 2);
      const int tag_px = std::max(min_tag_px, std::min(tag_px_from_mm, static_cast<int>(std::round(cell_inner_px * 0.72))));
      const int cell_border = std::max(1, static_cast<int>(std::round(std::min(pitch_x, pitch_y) * 0.03)));
      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          const int cell_x = start_x + c * pitch_x;
          const int cell_y = start_y + r * pitch_y;
          cv::rectangle(*out_image, cv::Rect(cell_x, cell_y, pitch_x, pitch_y), cv::Scalar(0), cell_border);
          int id = (start_marker_id + r * cols + c) % dict_size;
          cv::Mat tag_img;
          dict_obj.generateImageMarker(id, tag_px, tag_img);
          const int x = cell_x + (pitch_x - tag_px) / 2;
          const int y = cell_y + (pitch_y - tag_px) / 2;
          if (x >= board_x && y >= board_y && x + tag_px <= board_x + board_w_px && y + tag_px <= board_y + board_h_px) {
            tag_img.copyTo((*out_image)(cv::Rect(x, y, tag_px, tag_px)));
          }
        }
      }
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
