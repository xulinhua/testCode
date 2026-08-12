#pragma once

#include <vector>

#include "hs_calib_suite/core/base/target_model_base.hpp"
#include "hs_calib_suite/core/types/types.hpp"

namespace hs_calib {
namespace core {

/// \brief 靶标检测器抽象基类
/// 节点 / 离线管线只依赖本基类；具体检测器在独立实现中提供。
class DetectorBase {
public:
  virtual ~DetectorBase() = default;

  /// \brief 在图像帧上检测靶标特征
  /// \param frame 输入图像视图
  /// \param target 靶标几何模型
  /// \return 对应点集合；失败返回空
  virtual std::vector<Correspondence> detect(
      const ImageFrame &frame, const TargetModelBase &target) const = 0;
};

}  // namespace core
}  // namespace hs_calib
