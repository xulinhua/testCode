#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

#include "hs_calib_suite/core/base/target_model_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 直角三面靶的单面模型（正方形面板上的内角点网格）
struct TrihedralFaceModel {
  int face_id = 0;          ///< 面 ID：0=XY，1=XZ，2=YZ（靶标坐标系）
  int squares_x = 8;        ///< 第一轴方向内角点数（正方形面：与 squares_y 相同）
  int squares_y = 8;        ///< 第二轴方向内角点数
  double square_length_m = 0.025;  ///< 方格边长（米）
  double border_m = 0.025;  ///< 图案相对板顶点的白边宽度（米）
  /// 该面在公共靶标系下的物点（Nx3，单位米）
  Eigen::MatrixXd object_points;
};

/// \brief 直角三面靶几何：三正交正方形面交于公共顶点，图案四周留白边
///
/// 本类只描述**三面角点物点**；棋盘 / ChArUco / ArUco 图案由对应 Detector 解释。
/// 配置里的 `target`（trihedral_chess / trihedral_charuco …）选择检测器。
///
/// 靶标坐标系（与 Isaac 本地建模、立起旋转前一致）：
/// 白边宽度 \f$b\f$，方格边长 \f$s\f$，内角点 \f$(i,j)\f$：
///   - 面 0 XY：\f$(b+(i+1)s,\ b+(j+1)s,\ 0)\f$
///   - 面 1 XZ：\f$(b+(i+1)s,\ 0,\ b+(j+1)s)\f$
///   - 面 2 YZ：\f$(0,\ b+(i+1)s,\ b+(j+1)s)\f$
/// 全局特征 ID：面 0 为 \f$[0,n_0)\f$，面 1 从 1000 起，面 2 从 2000 起。
class TrihedralTarget : public TargetModelBase {
public:
  /// \param border_m 白边；&lt;0 时默认等于 square_length_m
  /// \note 构造时会将 squares_x/y 规范为 max(sx,sy) 的正方形网格
  TrihedralTarget(
      int squares_x, int squares_y, double square_length_m, double angle_deg = 90.0,
      double border_m = -1.0);

  /// \brief 返回几何类型 ID "trihedral"（图案由配置 target=trihedral_chess/charuco 区分）
  std::string target_id() const override;
  /// \brief 按全局特征 ID 查询物点坐标
  Eigen::MatrixXd object_points(const std::vector<int> &ids) const override;

  /// \brief 每面内角点列数
  int squares_x() const { return squares_x_; }
  /// \brief 每面内角点行数
  int squares_y() const { return squares_y_; }
  /// \brief 方格边长（米）
  double square_length_m() const { return square_length_m_; }
  /// \brief 图案白边宽度（米）
  double border_m() const { return border_m_; }
  /// \brief 三面夹角（度，预留）
  double angle_deg() const { return angle_deg_; }

  /// \brief 三个面的物点模型
  const std::vector<TrihedralFaceModel> &faces() const { return faces_; }
  /// \brief 三面全部物点拼接（Nx3）
  Eigen::MatrixXd all_object_points() const;
  /// \brief 由全局点 ID 解析面 ID；非法返回 -1
  int face_id_of_point_id(int id) const;
  /// \brief 面内局部索引 → 全局特征 ID
  static int point_id(int face_id, int local_index);

private:
  void build_faces();

  int squares_x_ = 8;
  int squares_y_ = 8;
  double square_length_m_ = 0.025;
  double border_m_ = 0.025;
  double angle_deg_ = 90.0;
  std::vector<TrihedralFaceModel> faces_;
};

}  // namespace core
}  // namespace hs_calib
