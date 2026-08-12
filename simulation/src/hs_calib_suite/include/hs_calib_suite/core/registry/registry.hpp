#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hs_calib_suite/core/base/calibrator_base.hpp"

namespace hs_calib {
namespace core {

/// \brief 标定器工厂：无参构造并返回基类指针
using CalibratorFactory = std::function<std::unique_ptr<CalibratorBase>()>;

/// \brief 标定器注册表（单例）
/// 静态库 / 插件通过 HS_CALIB_REGISTER 在加载期登记实现。
class CalibratorRegistry {
public:
  /// \brief 获取全局注册表
  static CalibratorRegistry &instance();

  /// \brief 登记标定器工厂
  void register_calibrator(const std::string &id, CalibratorFactory factory);

  /// \brief 按 ID 创建实例；未知 ID 抛 std::runtime_error
  std::unique_ptr<CalibratorBase> create(const std::string &id) const;

  /// \brief 已登记 ID 列表
  std::vector<std::string> list_ids() const;

private:
  CalibratorRegistry() = default;

  std::map<std::string, CalibratorFactory> factories_;  // id -> 工厂
};

/// \brief 静态注册辅助：构造时写入注册表
struct CalibratorRegistrar {
  CalibratorRegistrar(const std::string &id, CalibratorFactory factory) {
    CalibratorRegistry::instance().register_calibrator(id, std::move(factory));
  }
};

}  // namespace core
}  // namespace hs_calib

/// \brief 在翻译单元内静态注册标定器类型
#define HS_CALIB_REGISTER(ID, TYPE)                                            \
  static ::hs_calib::core::CalibratorRegistrar HS_CALIB_REG_##TYPE(            \
      ID, []() {                                                               \
        return std::unique_ptr<::hs_calib::core::CalibratorBase>(              \
            std::make_unique<TYPE>());                                         \
      })
