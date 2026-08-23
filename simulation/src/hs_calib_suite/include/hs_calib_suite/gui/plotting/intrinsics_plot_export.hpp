#pragma once

#include <QString>
#include <QStringList>

namespace hs_calib {
namespace gui {

class SessionController;

/// \brief 将 Tier4 三张统计图导出为 PNG（同步，用于「导出结果」）
/// \return 成功写入的文件名列表（仅 basename）
QStringList export_intrinsics_statistics_pngs(
    const SessionController &session,
    const QString &output_dir,
    const std::string &stats_backend,
    QString *summary_out = nullptr);

}  // namespace gui
}  // namespace hs_calib
