#include "hs_calib_suite/core/registry/registry.hpp"

#include <stdexcept>
#include <vector>

namespace hs_calib {
namespace core {

/// \brief 获取标定器注册表单例
CalibratorRegistry &CalibratorRegistry::instance() {
  static CalibratorRegistry registry;
  return registry;
}

/// \brief 登记标定器工厂函数
void CalibratorRegistry::register_calibrator(
    const std::string &id, CalibratorFactory factory) {
  factories_[id] = std::move(factory);
}

/// \brief 按 ID 创建标定器实例
std::unique_ptr<CalibratorBase> CalibratorRegistry::create(
    const std::string &id) const {
  const auto it = factories_.find(id);
  if (it == factories_.end()) {
    throw std::runtime_error("Unknown calibrator_id: " + id);
  }
  return it->second();
}

/// \brief 返回已登记的全部标定器 ID
std::vector<std::string> CalibratorRegistry::list_ids() const {
  std::vector<std::string> ids;
  ids.reserve(factories_.size());
  for (const auto &kv : factories_) {
    ids.push_back(kv.first);
  }
  return ids;
}

}  // namespace core
}  // namespace hs_calib
