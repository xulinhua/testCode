#include "hs_calib_suite/gui/session/session_controller.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QMetaObject>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sstream>
#include <thread>

#include <QImage>

#include "hs_calib_suite/core/calibrators/stereo_intrinsics_calibrator.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/board_frame_metrics.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_capture_filter.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_config_params.hpp"
#include "hs_calib_suite/core/calibrators/intrinsics/intrinsics_reprojection.hpp"
#include "hs_calib_suite/gui/bridges/stereo_image_loader.hpp"

namespace hs_calib {
namespace gui {

namespace {

void copy_meta_prefixed(
    const core::CalibrationResult &src,
    const std::string &prefix,
    core::CalibrationResult *dst) {
  for (const auto &kv : src.intrinsics_meta) {
    dst->intrinsics_meta[prefix + kv.first] = kv.second;
  }
  for (const auto &kv : src.metrics) {
    dst->metrics[prefix + kv.first] = kv.second;
  }
}

const core::Observation *find_obs_by_path(
    const core::ObservationBatch &batch, const std::string &path) {
  for (const auto &obs : batch.items) {
    if (obs.source_path == path) {
      return &obs;
    }
  }
  return nullptr;
}

core::ObservationBatch filter_side_batch(
    const core::ObservationBatch &in, const std::string &side) {
  core::ObservationBatch out;
  out.notes = in.notes;
  for (const auto &obs : in.items) {
    std::string tag = obs.frame_id;
    if (tag != "left" && tag != "right") {
      if (obs.source_path.rfind("left:", 0) == 0) {
        tag = "left";
      } else if (obs.source_path.rfind("right:", 0) == 0) {
        tag = "right";
      }
    }
    if (tag == side) {
      out.items.push_back(obs);
    }
  }
  return out;
}

bool kd_from_calib_meta(
    const std::map<std::string, std::string> &meta,
    const std::string &prefix,
    cv::Mat *K,
    cv::Mat *D) {
  if (K == nullptr || D == nullptr) {
    return false;
  }
  auto get = [&](const char *k) -> std::string {
    const std::string key = prefix + k;
    const auto it = meta.find(key);
    return it != meta.end() ? it->second : std::string();
  };
  const std::string fx = get("fx");
  const std::string fy = get("fy");
  const std::string cx = get("cx");
  const std::string cy = get("cy");
  if (fx.empty() || fy.empty() || cx.empty() || cy.empty()) {
    return false;
  }
  *K = cv::Mat::eye(3, 3, CV_64F);
  K->at<double>(0, 0) = std::stod(fx);
  K->at<double>(1, 1) = std::stod(fy);
  K->at<double>(0, 2) = std::stod(cx);
  K->at<double>(1, 2) = std::stod(cy);
  int dist_n = 5;
  const std::string dist_n_str = get("dist_n");
  if (!dist_n_str.empty()) {
    dist_n = std::max(4, std::stoi(dist_n_str));
  }
  *D = cv::Mat::zeros(dist_n, 1, CV_64F);
  auto d_at = [&](int i) {
    const std::string v = get(
        i == 0
            ? "k1"
            : (i == 1 ? "k2"
                      : (i == 2 ? "p1"
                                : (i == 3 ? "p2"
                                          : (i == 4 ? "k3"
                                                    : (i == 5 ? "k4" : "k5"))))));
    return v.empty() ? 0.0 : std::stod(v);
  };
  for (int i = 0; i < dist_n; ++i) {
    D->at<double>(i, 0) = d_at(i);
  }
  return true;
}

void stereo_side_batches(
    const SessionController *session,
    core::ObservationBatch *left_batch,
    core::ObservationBatch *right_batch) {
  if (session == nullptr || left_batch == nullptr || right_batch == nullptr) {
    return;
  }
  if (session->uses_tier4_intrinsics()) {
    *left_batch = session->intrinsics_state_for_side(QStringLiteral("left")).training_batch();
    *right_batch = session->intrinsics_state_for_side(QStringLiteral("right")).training_batch();
    return;
  }
  const core::ObservationBatch all = session->batch();
  *left_batch = filter_side_batch(all, "left");
  *right_batch = filter_side_batch(all, "right");
}

}  // namespace

int SessionController::stereo_rectify_pair_count() const {
  if (!stereo_pairs_.empty()) {
    return stereo_pair_count();
  }
  const int loaded = stereo_loaded_pair_count();
  if (loaded > 0) {
    return loaded;
  }
  if (!stereo_bag_reader_.empty()) {
    return stereo_bag_reader_.size();
  }
  return 0;
}

bool SessionController::uses_stereo_dual_session() const {
  return is_stereo_intrinsics();
}

QString SessionController::stereo_capture_mode() const {
  const auto it = solve_options_.find("stereo_capture_mode");
  if (it != solve_options_.end() && !it->second.empty()) {
    return QString::fromStdString(it->second);
  }
  const auto side_it = solve_options_.find("stereo_side");
  if (side_it != solve_options_.end()) {
  const std::string &s = side_it->second;
  if (s == "right" || s == "RIGHT" || s == "R") {
    return QStringLiteral("right");
  }
  if (s == "left" || s == "LEFT" || s == "L") {
    return QStringLiteral("left");
  }
  }
  return QStringLiteral("paired");
}

core::IntrinsicsSessionState &SessionController::intrinsics_state_for_side(
    const QString &side) {
  return side == QStringLiteral("right") ? intrinsics_right_state_
                                         : intrinsics_left_state_;
}

const core::IntrinsicsSessionState &SessionController::intrinsics_state_for_side(
    const QString &side) const {
  return side == QStringLiteral("right") ? intrinsics_right_state_
                                         : intrinsics_left_state_;
}

int SessionController::stereo_left_sample_count() const {
  if (!uses_stereo_dual_session()) {
    return 0;
  }
  if (uses_tier4_intrinsics()) {
    return intrinsics_left_state_.collector().training_count();
  }
  int n = 0;
  for (const auto &obs : batch_.items) {
    if (obs.frame_id == "left" || obs.source_path.rfind("left:", 0) == 0) {
      ++n;
    }
  }
  return n;
}

int SessionController::stereo_right_sample_count() const {
  if (!uses_stereo_dual_session()) {
    return 0;
  }
  if (uses_tier4_intrinsics()) {
    return intrinsics_right_state_.collector().training_count();
  }
  int n = 0;
  for (const auto &obs : batch_.items) {
    if (obs.frame_id == "right" || obs.source_path.rfind("right:", 0) == 0) {
      ++n;
    }
  }
  return n;
}

void SessionController::set_live_stereo_bgr(
    const cv::Mat &left, const cv::Mat &right, int64_t sync_delta_ms) {
  live_left_bgr_ = left.empty() ? cv::Mat() : left.clone();
  live_right_bgr_ = right.empty() ? cv::Mat() : right.clone();
  last_stereo_sync_delta_ms_ = sync_delta_ms;
}

void SessionController::set_live_stereo_bgr(
    cv::Mat &&left, cv::Mat &&right, int64_t sync_delta_ms) {
  live_left_bgr_ = std::move(left);
  live_right_bgr_ = std::move(right);
  last_stereo_sync_delta_ms_ = sync_delta_ms;
}

void SessionController::set_stereo_ros_topics(
    const QString &left_topic, const QString &right_topic) {
  set_ros_topic_name(left_topic);
  solve_options_["left_image_topic"] = left_topic.toStdString();
  solve_options_["right_image_topic"] = right_topic.toStdString();
  if (!solve_options_.count("stereo_max_sync_ms")) {
    solve_options_["stereo_max_sync_ms"] = "30";
  }
}

void SessionController::set_stereo_pair_index(int index) {
  const int n = stereo_rectify_pair_count();
  if (n <= 0) {
    stereo_pair_index_ = -1;
    emit current_changed();
    return;
  }
  if (index < 0) {
    index = 0;
  }
  if (index >= n) {
    index = n - 1;
  }
  stereo_pair_index_ = index;
  if (source_mode_ != SourceMode::RosTopic && !stereo_left_paths_.isEmpty() &&
      !stereo_right_paths_.isEmpty()) {
    current_index_ = index;
  }
  has_detection_ = false;
  stereo_left_detect_.has = false;
  stereo_right_detect_.has = false;

  live_left_bgr_.release();
  live_right_bgr_.release();
  last_stereo_sync_delta_ms_ = -1;

  cv::Mat left;
  cv::Mat right;
  if (load_stereo_pair_bgr(index, &left, &right)) {
    live_left_bgr_ = std::move(left);
    live_right_bgr_ = std::move(right);
    if (index >= 0 && index < static_cast<int>(stereo_pairs_.size())) {
      last_stereo_sync_delta_ms_ = stereo_pairs_.at(static_cast<size_t>(index)).timestamp_delta_ms;
    } else if (source_mode_ == SourceMode::Offline) {
      last_stereo_sync_delta_ms_ = 0;
    }
  }

  emit current_changed();
}

int SessionController::load_stereo_image_dir(
    const QString &root_or_left_dir, const QString &right_dir) {
  clear_loaded_source_data();
  stereo_pairs_.clear();
  next_stereo_pair_id_ = 1;
  stereo_left_paths_.clear();
  stereo_right_paths_.clear();
  stereo_pair_index_ = -1;
  intrinsics_left_state_.reset();
  intrinsics_right_state_.reset();

  const auto scan = StereoImageLoader::scan(root_or_left_dir, right_dir);
  stereo_left_paths_ = scan.left_paths;
  stereo_right_paths_ = scan.right_paths;
  if (!scan.valid()) {
    emit images_changed();
    emit observations_changed();
    emit result_changed();
    return 0;
  }
  image_paths_ = stereo_left_paths_;
  set_stereo_pair_index(0);
  set_source_mode(SourceMode::Offline);
  emit images_changed();
  emit observations_changed();
  emit result_changed();
  return scan.pair_count();
}

QImage SessionController::last_stereo_left_preview() const {
  return stereo_left_detect_.preview;
}

QImage SessionController::last_stereo_right_preview() const {
  return stereo_right_detect_.preview;
}

bool SessionController::stereo_left_has_detection() const {
  return stereo_left_detect_.has;
}

bool SessionController::stereo_right_has_detection() const {
  return stereo_right_detect_.has;
}

void SessionController::configure_stereo_intrinsics_states() {
  if (!uses_stereo_dual_session() || !uses_tier4_intrinsics()) {
    return;
  }
  core::merge_tier4_intrinsics_defaults(&solve_options_);
  if (source_mode_ == SourceMode::RosTopic) {
    solve_options_["collector_filter_speed"] = "false";
  }
  core::IntrinsicsProfile profile =
      core::profile_from_config_map(solve_options_);
  const auto it_solver = solve_options_.find("intrinsics_solver");
  if (it_solver != solve_options_.end()) {
    const std::string s = it_solver->second;
    profile.solver = (s == "ceres" || s == "Ceres")
                         ? core::IntrinsicsSolverKind::Ceres
                         : core::IntrinsicsSolverKind::OpenCV;
  }
  const auto collector_params =
      core::collector_params_from_config(solve_options_, profile);
  const bool offline =
      source_mode_ == SourceMode::Offline || source_mode_ == SourceMode::RosBag;
  intrinsics_left_state_.configure(profile, collector_params, solve_config_map());
  intrinsics_right_state_.configure(profile, collector_params, solve_config_map());
  intrinsics_left_state_.set_offline_source(offline);
  intrinsics_right_state_.set_offline_source(offline);
}

bool SessionController::detect_on_bgr(
    const cv::Mat &bgr, bool fast, QImage *preview_out, QString *error_out) {
  const cv::Mat saved = live_bgr_;
  live_bgr_ = bgr.clone();
  const bool ok = detect_current(preview_out, error_out, fast);
  live_bgr_ = saved;
  return ok;
}

bool SessionController::add_side_observation(
    const std::string &side,
    const core::Correspondence &corr,
    int width,
    int height,
    const QString &path,
    QString *error_out) {
  if (corr.image_points.rows() < 6) {
    if (error_out) {
      *error_out = QStringLiteral("有效角点不足");
    }
    return false;
  }
  std::string stored_path = path.toStdString();
  if (stored_path.find(side + ":") != 0) {
    stored_path = side + ":" + stored_path;
  }

  core::Observation obs;
  obs.source_path = stored_path;
  obs.frame_id = side;
  obs.image_width = width;
  obs.image_height = height;
  obs.correspondences = {corr};

  if (uses_intrinsics_capture_filter()) {
    const core::IntrinsicsProfile profile =
        core::profile_from_config_map(solve_options_);
    auto &state = intrinsics_state_for_side(QString::fromStdString(side));
    cv::Mat K = state.provisional_model().valid
        ? state.provisional_model().camera_matrix
        : core::make_initial_camera_matrix(width, height);
    cv::Mat D = state.provisional_model().valid
        ? state.provisional_model().dist_coeffs
        : core::make_initial_dist_coeffs(profile.rational_coeffs);
    core::IntrinsicsView view;
    view.object_points.reserve(static_cast<size_t>(corr.image_points.rows()));
    view.image_points.reserve(static_cast<size_t>(corr.image_points.rows()));
    for (int r = 0; r < corr.image_points.rows(); ++r) {
      view.object_points.emplace_back(
          static_cast<float>(corr.object_points(r, 0)),
          static_cast<float>(corr.object_points(r, 1)),
          static_cast<float>(corr.object_points(r, 2)));
      view.image_points.emplace_back(
          static_cast<float>(corr.image_points(r, 0)),
          static_cast<float>(corr.image_points(r, 1)));
    }
    core::fill_view_fingerprint(&view, width, height);
    cv::Mat rv, tv;
    if (core::solve_board_pose(view, K, D, &rv, &tv)) {
      const auto stats = core::compute_reprojection_stats(view, K, D, rv, tv);
      obs.has_board_pose = true;
      for (int i = 0; i < 3; ++i) {
        obs.board_rvec(i) = rv.at<double>(i, 0);
        obs.board_tvec(i) = tv.at<double>(i, 0);
      }
      obs.board_reproj_rms = stats.rms;
      obs.board_reproj_max = stats.max;
      obs.board_center_x_norm = view.centroid_x;
      obs.board_center_y_norm = view.centroid_y;
      obs.board_tilt_deg = view.tilt_deg;
    }
  }

  if (uses_stereo_dual_session() && uses_tier4_intrinsics()) {
    core::BoardFrameFingerprint fp =
        core::fingerprint_from_correspondence(corr, width, height);
    if (obs.has_board_pose) {
      fp.has_pose = true;
      fp.position_m = cv::Vec3d(
          obs.board_tvec.x(), obs.board_tvec.y(), obs.board_tvec.z());
      fp.tilt_deg = obs.board_tilt_deg;
    }
    auto &state = intrinsics_state_for_side(QString::fromStdString(side));
    core::IntrinsicsSampleSplit split = core::IntrinsicsSampleSplit::Training;
    const auto reason = state.try_capture(std::move(obs), fp, 0.0, &split);
    if (reason != core::CollectorRejectReason::Accepted) {
      if (error_out) {
        *error_out = QString::fromStdString(
            core::collector_reject_reason_text(reason));
      }
      return false;
    }
    return true;
  }

  batch_.items.push_back(std::move(obs));
  return true;
}

bool SessionController::capture_paired_observation(QString *error_out) {
  if (!uses_stereo_dual_session()) {
    if (error_out) {
      *error_out = QStringLiteral("非双目内参任务");
    }
    return false;
  }
  if (!stereo_left_detect_.has || !stereo_right_detect_.has) {
    if (error_out) {
      *error_out = QStringLiteral("请先对左右目均检测成功");
    }
    return false;
  }
  if (source_mode_ == SourceMode::RosTopic && last_stereo_sync_delta_ms_ < 0) {
    if (error_out) {
      *error_out = QStringLiteral("无有效时间同步信息");
    }
    return false;
  }
  const int max_dt =
      solve_options_.count("stereo_max_sync_ms")
          ? std::max(1, std::stoi(solve_options_.at("stereo_max_sync_ms")))
          : 30;
  if (source_mode_ == SourceMode::RosTopic &&
      last_stereo_sync_delta_ms_ > max_dt) {
    if (error_out) {
      *error_out = QStringLiteral("左右帧 Δt=%1ms 超过阈值 %2ms")
                        .arg(last_stereo_sync_delta_ms_)
                        .arg(max_dt);
    }
    return false;
  }

  QString left_path;
  QString right_path;
  QString left_cache;
  QString right_cache;
  if (source_mode_ == SourceMode::RosTopic) {
    const QString topic_l =
        solve_options_.count("left_image_topic")
            ? QString::fromStdString(solve_options_.at("left_image_topic"))
            : ros_topic_name_;
    const QString topic_r =
        solve_options_.count("right_image_topic")
            ? QString::fromStdString(solve_options_.at("right_image_topic"))
            : QString();
    const QString ros_left =
        QStringLiteral("ros://%1#%2").arg(topic_l).arg(++live_seq_);
    const QString ros_right =
        QStringLiteral("ros://%1#%2").arg(topic_r).arg(live_seq_);
    left_cache =
        save_capture_original(stereo_left_detect_.bgr, static_cast<int>(stereo_pairs_.size()) * 2);
    right_cache = save_capture_original(
        stereo_right_detect_.bgr, static_cast<int>(stereo_pairs_.size()) * 2 + 1);
    if (left_cache.isEmpty() && !live_left_bgr_.empty()) {
      left_cache =
          save_capture_original(live_left_bgr_, static_cast<int>(stereo_pairs_.size()) * 2);
    }
    if (right_cache.isEmpty() && !live_right_bgr_.empty()) {
      right_cache = save_capture_original(
          live_right_bgr_, static_cast<int>(stereo_pairs_.size()) * 2 + 1);
    }
    left_path = left_cache.isEmpty() ? ros_left : left_cache;
    right_path = right_cache.isEmpty() ? ros_right : right_cache;
  } else if (stereo_pair_index_ >= 0 &&
             stereo_pair_index_ < stereo_left_paths_.size() &&
             stereo_pair_index_ < stereo_right_paths_.size()) {
    left_path = stereo_left_paths_.at(stereo_pair_index_);
    right_path = stereo_right_paths_.at(stereo_pair_index_);
  } else {
    left_path = QStringLiteral("pair#%1").arg(next_stereo_pair_id_);
    right_path = left_path;
  }

  if (!add_side_observation(
          "left", stereo_left_detect_.corr, stereo_left_detect_.width,
          stereo_left_detect_.height, left_path, error_out)) {
    return false;
  }
  const std::string left_stored = std::string("left:") + left_path.toStdString();
  if (!add_side_observation(
          "right", stereo_right_detect_.corr, stereo_right_detect_.width,
          stereo_right_detect_.height, right_path, error_out)) {
    return false;
  }
  const std::string right_stored = std::string("right:") + right_path.toStdString();

  StereoPairRecord rec;
  rec.pair_id = next_stereo_pair_id_++;
  rec.left_source_path = left_stored;
  rec.right_source_path = right_stored;
  if (!left_cache.isEmpty()) {
    rec.left_image_path = left_cache.toStdString();
  } else if (!left_path.isEmpty() && !left_path.startsWith(QStringLiteral("ros://"))) {
    rec.left_image_path = left_path.toStdString();
  }
  if (!right_cache.isEmpty()) {
    rec.right_image_path = right_cache.toStdString();
  } else if (!right_path.isEmpty() && !right_path.startsWith(QStringLiteral("ros://"))) {
    rec.right_image_path = right_path.toStdString();
  }
  rec.timestamp_delta_ms = last_stereo_sync_delta_ms_;
  stereo_pairs_.push_back(rec);

  stereo_left_detect_.has = false;
  stereo_right_detect_.has = false;
  if (uses_tier4_intrinsics()) {
    batch_ = intrinsics_left_state_.training_batch();
    for (const auto &obs : intrinsics_right_state_.training_batch().items) {
      batch_.items.push_back(obs);
    }
    emit intrinsics_state_changed();
  }
  emit observations_changed();
  return true;
}

void SessionController::request_stereo_detect(bool fast) {
  if (!uses_stereo_dual_session() || stereo_detect_busy_.exchange(true)) {
    return;
  }
  const uint64_t epoch = stereo_detect_epoch_.load();
  cv::Mat left = std::move(live_left_bgr_);
  cv::Mat right = std::move(live_right_bgr_);
  if (left.empty() || right.empty()) {
    if (!stereo_bag_reader_.empty() && stereo_pair_index_ >= 0 &&
        stereo_pair_index_ < stereo_bag_reader_.size()) {
      const auto &pair = stereo_bag_reader_.pair(stereo_pair_index_);
      if (left.empty()) {
        left = pair.left_bgr.clone();
      }
      if (right.empty()) {
        right = pair.right_bgr.clone();
      }
    } else if (
        stereo_pair_index_ >= 0 && stereo_pair_index_ < stereo_left_paths_.size() &&
        stereo_pair_index_ < stereo_right_paths_.size()) {
      const QString lp = stereo_left_paths_.at(stereo_pair_index_);
      const QString rp = stereo_right_paths_.at(stereo_pair_index_);
      if (left.empty() && !lp.startsWith(QStringLiteral("bag://"))) {
        left = cv::imread(lp.toStdString(), cv::IMREAD_COLOR);
      }
      if (right.empty() && !rp.startsWith(QStringLiteral("bag://"))) {
        right = cv::imread(rp.toStdString(), cv::IMREAD_COLOR);
      }
    }
  }
  emit detect_started();
  std::thread([this, left = std::move(left), right = std::move(right), fast, epoch]() {
    QString err;
    bool ok = true;
    StereoSideDetect ldet;
    StereoSideDetect rdet;
    QImage lprev;
    QImage rprev;
    if (left.empty() || right.empty()) {
      ok = false;
      err = QStringLiteral("左右目无有效图像");
    } else {
      SessionController *self = this;
      ok = self->detect_on_bgr(left, fast, &lprev, &err);
      if (ok) {
        ldet.has = true;
        ldet.bgr = left.clone();
        ldet.corr = self->current_corr_;
        ldet.width = self->detect_width_;
        ldet.height = self->detect_height_;
        ldet.confidence = self->last_confidence_;
        ldet.preview = lprev;
        ldet.fp = core::fingerprint_from_correspondence(
            ldet.corr, ldet.width, ldet.height);
      }
      QString rerr;
      const bool rok = self->detect_on_bgr(right, fast, &rprev, &rerr);
      ok = ok && rok;
      if (!rok && err.isEmpty()) {
        err = rerr;
      }
      if (rok) {
        rdet.has = true;
        rdet.bgr = right.clone();
        rdet.corr = self->current_corr_;
        rdet.width = self->detect_width_;
        rdet.height = self->detect_height_;
        rdet.confidence = self->last_confidence_;
        rdet.preview = rprev;
        rdet.fp = core::fingerprint_from_correspondence(
            rdet.corr, rdet.width, rdet.height);
      }
    }
    QMetaObject::invokeMethod(
        this,
        [this, ok, err, ldet, rdet, epoch]() {
          if (epoch != stereo_detect_epoch_.load()) {
            stereo_detect_busy_.store(false);
            return;
          }
          stereo_left_detect_ = ldet;
          stereo_right_detect_ = rdet;
          if (!ldet.bgr.empty()) {
            live_left_bgr_ = ldet.bgr.clone();
          }
          if (!rdet.bgr.empty()) {
            live_right_bgr_ = rdet.bgr.clone();
          }
          stereo_detect_busy_.store(false);
          emit stereo_detect_finished(ok, err);
          emit detect_finished(ok, err);
        },
        Qt::QueuedConnection);
  }).detach();
}

void SessionController::merge_stereo_calib_results(
    const core::CalibrationResult &left_r,
    const core::CalibrationResult &right_r,
    core::CalibrationResult *out) {
  *out = {};
  if (left_r.success) {
    copy_meta_prefixed(left_r, "left_", out);
  }
  if (right_r.success) {
    copy_meta_prefixed(right_r, "right_", out);
  }
  out->success = left_r.success && right_r.success;
  out->intrinsics_meta["stereo_mode"] = "separate";
  if (left_r.success) {
    out->metrics["left_reprojection_rmse"] =
        left_r.metrics.count("reprojection_rmse")
            ? left_r.metrics.at("reprojection_rmse")
            : 0.0;
  }
  if (right_r.success) {
    out->metrics["right_reprojection_rmse"] =
        right_r.metrics.count("reprojection_rmse")
            ? right_r.metrics.at("reprojection_rmse")
            : 0.0;
  }
  out->metrics["num_views_left"] = static_cast<double>(stereo_left_sample_count());
  out->metrics["num_views_right"] = static_cast<double>(stereo_right_sample_count());
  out->metrics["num_pairs"] = static_cast<double>(stereo_pairs_.size());
  std::ostringstream msg;
  msg << "stereo_intrinsics L=" << stereo_left_sample_count()
      << " R=" << stereo_right_sample_count()
      << " pairs=" << stereo_pairs_.size();
  out->message = msg.str();
}

bool SessionController::append_stereo_rectified_meta(core::CalibrationResult *result) {
  if (result == nullptr || stereo_pairs_.size() < 3) {
    return false;
  }

  cv::Mat K1;
  cv::Mat D1;
  cv::Mat K2;
  cv::Mat D2;
  if (uses_tier4_intrinsics()) {
    K1 = intrinsics_left_state_.has_calibrated_model()
        ? intrinsics_left_state_.calibrated_K()
        : intrinsics_left_state_.provisional_model().camera_matrix;
    D1 = intrinsics_left_state_.has_calibrated_model()
        ? intrinsics_left_state_.calibrated_D()
        : intrinsics_left_state_.provisional_model().dist_coeffs;
    K2 = intrinsics_right_state_.has_calibrated_model()
        ? intrinsics_right_state_.calibrated_K()
        : intrinsics_right_state_.provisional_model().camera_matrix;
    D2 = intrinsics_right_state_.has_calibrated_model()
        ? intrinsics_right_state_.calibrated_D()
        : intrinsics_right_state_.provisional_model().dist_coeffs;
  }
  if (K1.empty() || K2.empty()) {
    if (!kd_from_calib_meta(result->intrinsics_meta, "left_", &K1, &D1) ||
        !kd_from_calib_meta(result->intrinsics_meta, "right_", &K2, &D2)) {
      return false;
    }
  }
  if (D1.empty()) {
    D1 = cv::Mat::zeros(5, 1, CV_64F);
  }
  if (D2.empty()) {
    D2 = cv::Mat::zeros(5, 1, CV_64F);
  }

  core::ObservationBatch left_batch;
  core::ObservationBatch right_batch;
  stereo_side_batches(this, &left_batch, &right_batch);

  std::vector<std::vector<cv::Point3f>> obj_pts;
  std::vector<std::vector<cv::Point2f>> img_l;
  std::vector<std::vector<cv::Point2f>> img_r;
  int image_w = 0;
  int image_h = 0;

  for (const auto &pair : stereo_pairs_) {
    const core::Observation *ol = find_obs_by_path(left_batch, pair.left_source_path);
    const core::Observation *or_obs =
        find_obs_by_path(right_batch, pair.right_source_path);
    if (ol == nullptr || or_obs == nullptr || ol->correspondences.empty() ||
        or_obs->correspondences.empty()) {
      continue;
    }
    const auto &cl = ol->correspondences.front();
    const auto &cr = or_obs->correspondences.front();
    if (cl.image_points.rows() != cr.image_points.rows() ||
        cl.image_points.rows() < 6) {
      continue;
    }
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> pl;
    std::vector<cv::Point2f> pr;
    for (int i = 0; i < cl.image_points.rows(); ++i) {
      obj.emplace_back(
          static_cast<float>(cl.object_points(i, 0)),
          static_cast<float>(cl.object_points(i, 1)),
          static_cast<float>(cl.object_points(i, 2)));
      pl.emplace_back(
          static_cast<float>(cl.image_points(i, 0)),
          static_cast<float>(cl.image_points(i, 1)));
      pr.emplace_back(
          static_cast<float>(cr.image_points(i, 0)),
          static_cast<float>(cr.image_points(i, 1)));
    }
    obj_pts.push_back(std::move(obj));
    img_l.push_back(std::move(pl));
    img_r.push_back(std::move(pr));
    image_w = std::max(image_w, ol->image_width);
    image_h = std::max(image_h, ol->image_height);
  }
  if (obj_pts.size() < 3) {
    return false;
  }
  if (image_w <= 0 || image_h <= 0) {
    image_w = 1280;
    image_h = 720;
  }

  cv::Mat R, T, E, F;
  const int flags =
      solve_options_.count("stereo_joint_refine") &&
              solve_options_.at("stereo_joint_refine") == "true"
          ? 0
          : cv::CALIB_FIX_INTRINSIC;
  double rms = 0.0;
  try {
    rms = cv::stereoCalibrate(
        obj_pts, img_l, img_r, K1, D1, K2, D2, cv::Size(image_w, image_h), R, T,
        E, F, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6));
  } catch (const cv::Exception &) {
    return false;
  }

  cv::Mat R1, R2, P1, P2, Q;
  try {
    cv::stereoRectify(
        K1, D1, K2, D2, cv::Size(image_w, image_h), R, T, R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, -1, cv::Size(image_w, image_h));
  } catch (const cv::Exception &) {
    return false;
  }

  auto mat_csv = [](const cv::Mat &m) {
    std::ostringstream os;
    for (int r = 0; r < m.rows; ++r) {
      for (int c = 0; c < m.cols; ++c) {
        if (r > 0 || c > 0) {
          os << (c == 0 ? ";" : ",");
        }
        os << m.at<double>(r, c);
      }
    }
    return os.str();
  };

  result->intrinsics_meta["stereo_rectified"] = "true";
  result->intrinsics_meta["left_image_width"] = std::to_string(image_w);
  result->intrinsics_meta["left_image_height"] = std::to_string(image_h);
  result->intrinsics_meta["R1"] = mat_csv(R1);
  result->intrinsics_meta["R2"] = mat_csv(R2);
  result->intrinsics_meta["P1"] = mat_csv(P1);
  result->intrinsics_meta["P2"] = mat_csv(P2);
  result->intrinsics_meta["Q"] = mat_csv(Q);
  result->intrinsics_meta["stereo_R"] = mat_csv(R);
  result->intrinsics_meta["stereo_T"] = mat_csv(T);
  result->metrics["stereo_rms"] = rms;
  result->metrics["baseline_m"] = cv::norm(T);
  result->metrics["rectified_pairs"] = static_cast<double>(obj_pts.size());
  return true;
}

bool SessionController::solve_stereo_intrinsics(QString *error_out) {
  if (!uses_stereo_dual_session()) {
    if (error_out) {
      *error_out = QStringLiteral("内部错误：非双目内参任务");
    }
    return false;
  }
  if (!validate_solve_preconditions(error_out)) {
    return false;
  }
  if (stereo_left_sample_count() < min_views_ || stereo_right_sample_count() < min_views_) {
    if (error_out) {
      *error_out = QStringLiteral("左右目训练样本均不足（各需 ≥%1）").arg(min_views_);
    }
    return false;
  }

  core::CalibrationResult left_r;
  core::CalibrationResult right_r;
  if (uses_tier4_intrinsics()) {
    std::string err_l;
    std::string err_r;
    const bool ok_l = intrinsics_left_state_.calibrate(&err_l);
    left_r = intrinsics_left_state_.last_result();
    const bool ok_r = intrinsics_right_state_.calibrate(&err_r);
    right_r = intrinsics_right_state_.last_result();
    if (!ok_l || !ok_r) {
      if (error_out) {
        *error_out = QStringLiteral("分侧标定失败：L=%1 · R=%2")
                          .arg(QString::fromStdString(err_l))
                          .arg(QString::fromStdString(err_r));
      }
      last_result_ = left_r.success ? left_r : right_r;
      emit result_changed();
      return false;
    }
  } else {
    core::StereoIntrinsicsCalibrator cal;
    last_result_ = cal.calibrate(batch_, solve_config_map());
    if (!last_result_.success) {
      emit result_changed();
      if (error_out) {
        *error_out = QString::fromStdString(last_result_.message);
      }
      return false;
    }
    last_result_.metrics["num_pairs"] = static_cast<double>(stereo_pairs_.size());
    const bool gen_rect =
        !solve_options_.count("generate_stereo_rectified") ||
        solve_options_.at("generate_stereo_rectified") != "false";
    if (gen_rect) {
      if (!append_stereo_rectified_meta(&last_result_)) {
        last_result_.intrinsics_meta["stereo_rectify_note"] =
            "stereoCalibrate failed (need >=3 paired views with matching corners)";
      } else {
        rebuild_stereo_rectify_maps();
      }
    }
    emit result_changed();
    return true;
  }

  merge_stereo_calib_results(left_r, right_r, &last_result_);
  const bool gen_rect =
      !solve_options_.count("generate_stereo_rectified") ||
      solve_options_.at("generate_stereo_rectified") != "false";
  if (gen_rect) {
    if (!append_stereo_rectified_meta(&last_result_)) {
      last_result_.intrinsics_meta["stereo_rectify_note"] =
          "stereoCalibrate failed (need >=3 paired views with matching corners)";
    } else {
      rebuild_stereo_rectify_maps();
    }
  }
  emit result_changed();
  emit intrinsics_state_changed();
  if (!last_result_.success && error_out) {
    *error_out = QString::fromStdString(last_result_.message);
  }
  return last_result_.success;
}

namespace {

QImage mat_bgr_to_qimage_local(const cv::Mat &bgr) {
  if (bgr.empty()) {
    return {};
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return QImage(
             rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
             QImage::Format_RGB888)
      .copy();
}

cv::Mat parse_mat_csv(const std::string &csv, int rows, int cols) {
  cv::Mat m(rows, cols, CV_64F);
  std::stringstream ss(csv);
  std::string row_str;
  int r = 0;
  while (std::getline(ss, row_str, ';') && r < rows) {
    std::stringstream rs(row_str);
    std::string cell;
    int c = 0;
    while (std::getline(rs, cell, ',') && c < cols) {
      m.at<double>(r, c++) = std::stod(cell);
    }
    ++r;
  }
  return m;
}

void draw_epipolar_lines(cv::Mat *bgr, int step_px = 40) {
  if (bgr == nullptr || bgr->empty()) {
    return;
  }
  for (int y = 0; y < bgr->rows; y += step_px) {
    cv::line(
        *bgr, cv::Point(0, y), cv::Point(bgr->cols - 1, y), cv::Scalar(0, 220, 0),
        1, cv::LINE_AA);
  }
}

}  // namespace

int SessionController::load_stereo_rosbag(
    const QString &bag_uri,
    const QString &left_topic,
    const QString &right_topic,
    int max_pairs,
    QString *error_out) {
  const int max_dt =
      solve_options_.count("stereo_max_sync_ms")
          ? std::max(1, std::stoi(solve_options_.at("stereo_max_sync_ms")))
          : 30;
  RosBagStereoFrameReader reader;
  const int n = reader.open(
      bag_uri, left_topic, right_topic, max_pairs, max_dt, error_out);
  if (n <= 0) {
    return 0;
  }
  return apply_loaded_stereo_bag(std::move(reader), left_topic, right_topic);
}

int SessionController::apply_loaded_stereo_bag(
    RosBagStereoFrameReader reader, const QString &left_topic, const QString &right_topic) {
  clear_loaded_source_data();
  stereo_pairs_.clear();
  next_stereo_pair_id_ = 1;
  intrinsics_left_state_.reset();
  intrinsics_right_state_.reset();

  stereo_bag_reader_ = std::move(reader);
  const int n = stereo_bag_reader_.size();
  if (n <= 0) {
    stereo_bag_reader_.clear();
    emit images_changed();
    emit observations_changed();
    emit result_changed();
    return 0;
  }

  stereo_left_paths_.reserve(n);
  stereo_right_paths_.reserve(n);
  for (int i = 0; i < n; ++i) {
    stereo_left_paths_.push_back(stereo_bag_reader_.left_label(i));
    stereo_right_paths_.push_back(stereo_bag_reader_.right_label(i));
  }
  image_paths_ = stereo_left_paths_;
  stereo_pair_index_ = 0;
  current_index_ = 0;
  set_ros_topic_name(left_topic);
  solve_options_["left_image_topic"] = left_topic.toStdString();
  solve_options_["right_image_topic"] = right_topic.toStdString();
  set_source_mode(SourceMode::RosBag);
  configure_stereo_intrinsics_states();

  const auto &pair0 = stereo_bag_reader_.pair(0);
  live_left_bgr_ = pair0.left_bgr.clone();
  live_right_bgr_ = pair0.right_bgr.clone();
  last_stereo_sync_delta_ms_ = pair0.delta_ms;

  emit images_changed();
  emit current_changed();
  emit observations_changed();
  emit result_changed();
  return n;
}

bool SessionController::try_auto_capture_paired(
    double min_confidence, double min_diversity, QString *error_out) {
  if (!uses_stereo_dual_session()) {
    if (error_out) {
      *error_out = QStringLiteral("非双目内参任务");
    }
    return false;
  }
  if (!stereo_left_detect_.has || !stereo_right_detect_.has) {
    if (error_out) {
      *error_out = QStringLiteral("左右目未同时检出");
    }
    return false;
  }
  if (stereo_left_detect_.confidence < min_confidence ||
      stereo_right_detect_.confidence < min_confidence) {
    if (error_out) {
      *error_out = QStringLiteral("置信度不足");
    }
    return false;
  }
  const int max_dt =
      solve_options_.count("stereo_max_sync_ms")
          ? std::max(1, std::stoi(solve_options_.at("stereo_max_sync_ms")))
          : 30;
  if (source_mode_ == SourceMode::RosTopic &&
      (last_stereo_sync_delta_ms_ < 0 || last_stereo_sync_delta_ms_ > max_dt)) {
    if (error_out) {
      *error_out = QStringLiteral("左右帧未同步（Δt=%1ms）").arg(last_stereo_sync_delta_ms_);
    }
    return false;
  }
  if (!uses_tier4_intrinsics()) {
    ViewFingerprint lfp;
    lfp.area_ratio = stereo_left_detect_.fp.normalized_size;
    lfp.cx = stereo_left_detect_.fp.centroid_x;
    lfp.cy = stereo_left_detect_.fp.centroid_y;
    lfp.tilt_deg = stereo_left_detect_.fp.tilt_deg;
    ViewFingerprint rfp;
    rfp.area_ratio = stereo_right_detect_.fp.normalized_size;
    rfp.cx = stereo_right_detect_.fp.centroid_x;
    rfp.cy = stereo_right_detect_.fp.centroid_y;
    rfp.tilt_deg = stereo_right_detect_.fp.tilt_deg;
    if (!is_diverse_enough(lfp, min_diversity) ||
        !is_diverse_enough(rfp, min_diversity)) {
      if (error_out) {
        *error_out = QStringLiteral("与已采集姿态过于相似");
      }
      return false;
    }
  }
  return capture_paired_observation(error_out);
}

bool SessionController::has_stereo_rectified() const {
  return last_result_.success && last_result_.intrinsics_meta.count("stereo_rectified") &&
         last_result_.intrinsics_meta.at("stereo_rectified") == "true";
}

bool SessionController::ensure_stereo_rectification() {
  if (!is_stereo_intrinsics() || !last_result_.success) {
    return has_stereo_rectified() && has_stereo_rectify_maps_;
  }
  if (!has_stereo_rectified()) {
    const bool gen_rect =
        !solve_options_.count("generate_stereo_rectified") ||
        solve_options_.at("generate_stereo_rectified") != "false";
    if (!gen_rect || stereo_pairs_.size() < 3) {
      return false;
    }
    if (!append_stereo_rectified_meta(&last_result_)) {
      return false;
    }
    emit result_changed();
  }
  backfill_stereo_pair_image_paths();
  if (!has_stereo_rectify_maps_) {
    rebuild_stereo_rectify_maps();
  }
  return has_stereo_rectified() && has_stereo_rectify_maps_;
}

void SessionController::backfill_stereo_pair_image_paths() {
  if (!capture_cache_dir_.isEmpty()) {
    for (size_t i = 0; i < stereo_pairs_.size(); ++i) {
      auto &rec = stereo_pairs_[i];
      const int base = static_cast<int>(i) * 2;
      if (rec.left_image_path.empty()) {
        const QString p = QDir(capture_cache_dir_).filePath(
            QStringLiteral("%1.png").arg(base, 4, 10, QChar('0')));
        if (QFile::exists(p)) {
          rec.left_image_path = p.toStdString();
        }
      }
      if (rec.right_image_path.empty()) {
        const QString p = QDir(capture_cache_dir_).filePath(
            QStringLiteral("%1.png").arg(base + 1, 4, 10, QChar('0')));
        if (QFile::exists(p)) {
          rec.right_image_path = p.toStdString();
        }
      }
    }
  }
  for (size_t i = 0; i < stereo_pairs_.size(); ++i) {
    auto &rec = stereo_pairs_[i];
    if (!rec.left_image_path.empty() && !rec.right_image_path.empty()) {
      continue;
    }
    const QString lp =
        resolve_obs_source_path(QString::fromStdString(rec.left_source_path));
    const QString rp =
        resolve_obs_source_path(QString::fromStdString(rec.right_source_path));
    if (rec.left_image_path.empty() && !lp.isEmpty() && QFile::exists(lp)) {
      rec.left_image_path = lp.toStdString();
    }
    if (rec.right_image_path.empty() && !rp.isEmpty() && QFile::exists(rp)) {
      rec.right_image_path = rp.toStdString();
    }
  }
}

bool SessionController::load_stereo_pair_bgr(
    int index, cv::Mat *left, cv::Mat *right) const {
  if (left == nullptr || right == nullptr || index < 0) {
    return false;
  }
  left->release();
  right->release();

  if (index < static_cast<int>(stereo_pairs_.size())) {
    const auto &rec = stereo_pairs_.at(static_cast<size_t>(index));
    QString lp;
    QString rp;
    if (!rec.left_image_path.empty()) {
      lp = QString::fromStdString(rec.left_image_path);
    } else {
      lp = resolve_obs_source_path(QString::fromStdString(rec.left_source_path));
    }
    if (!rec.right_image_path.empty()) {
      rp = QString::fromStdString(rec.right_image_path);
    } else {
      rp = resolve_obs_source_path(QString::fromStdString(rec.right_source_path));
    }
    if (!lp.isEmpty()) {
      *left = cv::imread(lp.toStdString(), cv::IMREAD_COLOR);
    }
    if (!rp.isEmpty()) {
      *right = cv::imread(rp.toStdString(), cv::IMREAD_COLOR);
    }
  }

  if ((left->empty() || right->empty()) && source_mode_ == SourceMode::RosBag &&
      !stereo_bag_reader_.empty() && index < stereo_bag_reader_.size()) {
    const auto &pair = stereo_bag_reader_.pair(index);
    if (left->empty()) {
      *left = pair.left_bgr.clone();
    }
    if (right->empty()) {
      *right = pair.right_bgr.clone();
    }
  }

  if ((left->empty() || right->empty()) && index >= 0 &&
      index < stereo_left_paths_.size() && index < stereo_right_paths_.size()) {
    const QString lp = stereo_left_paths_.at(index);
    const QString rp = stereo_right_paths_.at(index);
    if (left->empty() && !lp.startsWith(QStringLiteral("bag://"))) {
      *left = cv::imread(lp.toStdString(), cv::IMREAD_COLOR);
    }
    if (right->empty() && !rp.startsWith(QStringLiteral("bag://"))) {
      *right = cv::imread(rp.toStdString(), cv::IMREAD_COLOR);
    }
  }

  if (index == stereo_pair_index_) {
    if (left->empty() && !live_left_bgr_.empty()) {
      *left = live_left_bgr_.clone();
    }
    if (right->empty() && !live_right_bgr_.empty()) {
      *right = live_right_bgr_.clone();
    }
  }

  return !left->empty() && !right->empty();
}

void SessionController::rebuild_stereo_rectify_maps() {
  has_stereo_rectify_maps_ = false;
  stereo_map1_x_.release();
  stereo_map1_y_.release();
  stereo_map2_x_.release();
  stereo_map2_y_.release();
  if (!has_stereo_rectified()) {
    return;
  }
  const auto it_r1 = last_result_.intrinsics_meta.find("R1");
  const auto it_p1 = last_result_.intrinsics_meta.find("P1");
  if (it_r1 == last_result_.intrinsics_meta.end() ||
      it_p1 == last_result_.intrinsics_meta.end()) {
    return;
  }

  cv::Mat K1;
  cv::Mat D1;
  cv::Mat K2;
  cv::Mat D2;
  if (uses_tier4_intrinsics()) {
    K1 = intrinsics_left_state_.has_calibrated_model()
        ? intrinsics_left_state_.calibrated_K()
        : intrinsics_left_state_.provisional_model().camera_matrix;
    D1 = intrinsics_left_state_.has_calibrated_model()
        ? intrinsics_left_state_.calibrated_D()
        : intrinsics_left_state_.provisional_model().dist_coeffs;
    K2 = intrinsics_right_state_.has_calibrated_model()
        ? intrinsics_right_state_.calibrated_K()
        : intrinsics_right_state_.provisional_model().camera_matrix;
    D2 = intrinsics_right_state_.has_calibrated_model()
        ? intrinsics_right_state_.calibrated_D()
        : intrinsics_right_state_.provisional_model().dist_coeffs;
  }
  if (K1.empty() || K2.empty()) {
    if (!kd_from_calib_meta(last_result_.intrinsics_meta, "left_", &K1, &D1) ||
        !kd_from_calib_meta(last_result_.intrinsics_meta, "right_", &K2, &D2)) {
      return;
    }
  }
  if (D1.empty()) {
    D1 = cv::Mat::zeros(5, 1, CV_64F);
  }
  if (D2.empty()) {
    D2 = cv::Mat::zeros(5, 1, CV_64F);
  }

  const cv::Mat R1 = parse_mat_csv(it_r1->second, 3, 3);
  const cv::Mat R2 = parse_mat_csv(last_result_.intrinsics_meta.at("R2"), 3, 3);
  const cv::Mat P1 = parse_mat_csv(it_p1->second, 3, 4);
  const cv::Mat P2 = parse_mat_csv(last_result_.intrinsics_meta.at("P2"), 3, 4);

  int w = 0;
  int h = 0;
  if (last_result_.intrinsics_meta.count("left_image_width")) {
    w = std::stoi(last_result_.intrinsics_meta.at("left_image_width"));
    h = std::stoi(last_result_.intrinsics_meta.at("left_image_height"));
  }
  if (w <= 0 || h <= 0) {
    const auto it_w = last_result_.intrinsics_meta.find("left_image_width");
    const auto it_h = last_result_.intrinsics_meta.find("left_image_height");
    if (it_w != last_result_.intrinsics_meta.end() &&
        it_h != last_result_.intrinsics_meta.end()) {
      w = std::stoi(it_w->second);
      h = std::stoi(it_h->second);
    }
  }
  if (w <= 0 || h <= 0) {
    core::ObservationBatch left_batch;
    core::ObservationBatch right_batch;
    stereo_side_batches(this, &left_batch, &right_batch);
    const core::ObservationBatch *batch =
        !left_batch.items.empty() ? &left_batch : &right_batch;
    if (batch != nullptr && !batch->items.empty()) {
      w = batch->items.front().image_width;
      h = batch->items.front().image_height;
    }
  }
  if (w <= 0 || h <= 0) {
    w = 1280;
    h = 720;
  }
  stereo_rect_size_ = cv::Size(w, h);

  try {
    cv::initUndistortRectifyMap(
        K1, D1, R1, P1, stereo_rect_size_, CV_32FC1, stereo_map1_x_, stereo_map1_y_);
    cv::initUndistortRectifyMap(
        K2, D2, R2, P2, stereo_rect_size_, CV_32FC1, stereo_map2_x_, stereo_map2_y_);
    has_stereo_rectify_maps_ = true;
  } catch (const cv::Exception &) {
    has_stereo_rectify_maps_ = false;
  }
}

bool SessionController::stereo_rectified_preview(
    QImage *left_out, QImage *right_out) const {
  if (left_out == nullptr || right_out == nullptr || !has_stereo_rectify_maps_) {
    return false;
  }
  cv::Mat left;
  cv::Mat right;
  const int idx = stereo_pair_index_ >= 0 ? stereo_pair_index_ : 0;
  if (!load_stereo_pair_bgr(idx, &left, &right)) {
    return false;
  }
  cv::Mat rl;
  cv::Mat rr;
  try {
    cv::remap(left, rl, stereo_map1_x_, stereo_map1_y_, cv::INTER_LINEAR);
    cv::remap(right, rr, stereo_map2_x_, stereo_map2_y_, cv::INTER_LINEAR);
  } catch (const cv::Exception &) {
    return false;
  }
  draw_epipolar_lines(&rl);
  draw_epipolar_lines(&rr);
  *left_out = mat_bgr_to_qimage_local(rl);
  *right_out = mat_bgr_to_qimage_local(rr);
  return !left_out->isNull() && !right_out->isNull();
}

int SessionController::stereo_loaded_pair_count() const {
  if (!uses_stereo_dual_session()) {
    return 0;
  }
  return std::min(stereo_left_paths_.size(), stereo_right_paths_.size());
}

QString SessionController::stereo_pair_brightness_hint() const {
  cv::Mat left;
  cv::Mat right;
  const int idx = stereo_pair_index_ >= 0 ? stereo_pair_index_ : 0;
  if (!load_stereo_pair_bgr(idx, &left, &right)) {
    return QStringLiteral("—");
  }
  cv::Mat gl;
  cv::Mat gr;
  cv::cvtColor(left, gl, cv::COLOR_BGR2GRAY);
  cv::cvtColor(right, gr, cv::COLOR_BGR2GRAY);
  const double ml = cv::mean(gl)[0];
  const double mr = cv::mean(gr)[0];
  const double denom = std::max(1.0, std::max(ml, mr));
  const double rel = std::abs(ml - mr) / denom;
  if (rel > 0.18) {
    return QStringLiteral("⚠ 左右亮度差 %1%（建议检查曝光）")
        .arg(rel * 100.0, 0, 'f', 1);
  }
  return QStringLiteral("✓ 亮度一致 L=%1 R=%2")
      .arg(ml, 0, 'f', 1)
      .arg(mr, 0, 'f', 1);
}

}  // namespace gui
}  // namespace hs_calib
