#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace hs_calib {
namespace core {

/// \brief 标定板类型识别的一条假设（按 score 降序）
struct BoardTypeHypothesis {
  std::string type_id;     ///< chessboard / charuco / aruco / …
  double score = 0.0;      ///< 归一化置信度 [0,1]
  int feature_count = 0;   ///< 检出特征数（角点或码数）
  std::string dict_hint;   ///< 建议字典（若适用）
  std::string note;        ///< 人类可读补充
};

/// \brief 类型识别输入（尺寸一律可忽略，内部用占位几何）
struct BoardTypeIdentifyOptions {
  /// \brief 优先尝试的字典名；空则用内置短列表
  std::vector<std::string> dictionary_hints;
  /// \brief 要试探的类型；空则默认平面靶 + 三面靶
  std::vector<std::string> candidate_types;
  /// \brief 字典短扫上限（含 hints；≤0 表示不限制）
  int max_dictionary_scan = 8;
  /// \brief 是否在 overlay 上画 Top-1 叠加
  bool draw_overlay = true;
};

/// \brief 类型识别输出
struct BoardTypeIdentifyResult {
  std::vector<BoardTypeHypothesis> ranked;
  cv::Mat overlay_bgr;  ///< 可为空
  std::string message;  ///< 失败原因或摘要
  bool ok() const { return !ranked.empty() && ranked.front().score > 0.05; }
};

/// \brief 多类型轻量试探：只判「是什么板」，不要求真实物理尺寸
class BoardTypeIdentifier {
public:
  /// \brief 对 BGR 图做类型识别
  BoardTypeIdentifyResult identify(
      const cv::Mat &bgr,
      const BoardTypeIdentifyOptions &options = {}) const;
};

}  // namespace core
}  // namespace hs_calib
