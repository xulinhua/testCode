#include "hs_calib_suite/gui/window/main_window.hpp"

#include <QApplication>
#include <QFont>
#include <QFontDatabase>
#include <QMetaObject>
#include <QStyleFactory>
#include <QTimer>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

namespace {

/// \brief 终端日志时间：YYYYMMDD-HHMMSS-mmm（本地时区）
void format_log_stamp(rcutils_time_point_value_t timestamp_ns, char *out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return;
  }
  const int64_t ns = static_cast<int64_t>(timestamp_ns);
  const time_t sec = static_cast<time_t>(ns / 1000000000LL);
  const int ms = static_cast<int>((ns / 1000000LL) % 1000LL);
  std::tm tm_buf{};
#if defined(_WIN32)
  localtime_s(&tm_buf, &sec);
#else
  localtime_r(&sec, &tm_buf);
#endif
  std::snprintf(
      out, out_len, "%04d%02d%02d-%02d%02d%02d-%03d", tm_buf.tm_year + 1900, tm_buf.tm_mon + 1,
      tm_buf.tm_mday, tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, ms);
}

/// \brief 自定义控制台输出：可读时间，替代 epoch 秒
void hs_calib_console_output_handler(
    const rcutils_log_location_t * /*location*/, int severity, const char *name,
    rcutils_time_point_value_t timestamp, const char *format, va_list *args) {
  static std::mutex mu;
  std::lock_guard<std::mutex> lock(mu);

  FILE *stream = stdout;
  if (severity >= RCUTILS_LOG_SEVERITY_WARN) {
    stream = stderr;
  }

  const char *severity_string = "UNKNOWN";
  if (severity >= 0 && severity <= RCUTILS_LOG_SEVERITY_FATAL &&
      g_rcutils_log_severity_names[severity] != nullptr) {
    severity_string = g_rcutils_log_severity_names[severity];
  }

  char stamp[32];
  format_log_stamp(timestamp, stamp, sizeof(stamp));

  char msg[2048];
  msg[0] = '\0';
  if (format != nullptr && args != nullptr) {
    std::vsnprintf(msg, sizeof(msg), format, *args);
  }

  std::fprintf(
      stream, "[%s] [%s] [%s]: %s\n", severity_string, stamp, name != nullptr ? name : "", msg);
  std::fflush(stream);
}

/// \brief 本机字体库是否含该族名
bool has_family(const QString &name) {
  return QFontDatabase().families().contains(name);
}

/// \brief 按候选列表挑选 UI 字体
QFont pick_ui_font(int point_size, int weight = QFont::Medium) {
  const char *candidates[] = {
      "Noto Sans CJK SC",
      "Noto Sans",
      "Source Han Sans SC",
      "DejaVu Sans",
      "IBM Plex Sans",
      "Inter",
      "PingFang SC",
      "Microsoft YaHei UI",
      "Segoe UI",
      "Sans Serif",
  };
  for (const char *name : candidates) {
    const QString qname = QString::fromUtf8(name);
    if (has_family(qname)) {
      QFont f(qname, point_size, weight);
      f.setStyleHint(QFont::SansSerif);
      f.setHintingPreference(QFont::PreferFullHinting);
      return f;
    }
  }
  QFont f;
  f.setPointSize(point_size);
  f.setWeight(weight);
  f.setStyleHint(QFont::SansSerif);
  return f;
}

std::atomic<QCoreApplication *> g_qt_app{nullptr};

/// \brief 仅投递退出，不在信号/ROS 线程直接碰窗口
void request_qt_quit() {
  QCoreApplication *app = g_qt_app.load(std::memory_order_acquire);
  if (app != nullptr) {
    QMetaObject::invokeMethod(app, "quit", Qt::QueuedConnection);
  }
}

}  // namespace

/// \brief 程序入口：初始化 ROS2 / Qt；Ctrl+C 后 Qt 主线程退出事件循环
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Humble 默认 {time} 为 epoch 秒；改为 YYYYMMDD-HHMMSS-mmm
  rcutils_logging_set_output_handler(hs_calib_console_output_handler);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication app(argc, argv);
  g_qt_app.store(&app, std::memory_order_release);
  app.setQuitOnLastWindowClosed(true);
  // Fusion respects QSS consistently (native GTK often leaves Combo/Spin white).
  if (auto *fusion = QStyleFactory::create(QStringLiteral("Fusion"))) {
    app.setStyle(fusion);
  }
  app.setApplicationName(QStringLiteral("HS Calib Suite"));
  app.setOrganizationName(QStringLiteral("HS"));
  app.setFont(pick_ui_font(12, QFont::Medium));

  // —— 注册 ROS 关闭钩子（仅投递 quit） ——
  // Humble：SIGINT 在 rclcpp 信号线程里 shutdown，不能在那里直接关 Qt。
  // pre/on_shutdown 只投递 quit；主线程再轮询 rclcpp::ok() 关窗口。
  rclcpp::on_shutdown([]() { request_qt_quit(); });
  const auto ros_ctx = rclcpp::contexts::get_global_default_context();
  if (ros_ctx != nullptr) {
    ros_ctx->add_pre_shutdown_callback([]() { request_qt_quit(); });
  }

  RCLCPP_INFO(
      rclcpp::get_logger("hs_calib_gui"),
      "HS Calib Suite starting (ROS2 Humble + Qt5)");

  int result = 0;
  {
    hs_calib::gui::MainWindow window;
    window.show();

    QTimer ros_ok_watch;
    ros_ok_watch.setInterval(50);
    QObject::connect(&ros_ok_watch, &QTimer::timeout, &window, [&]() {
      if (rclcpp::ok()) {
        return;
      }
      ros_ok_watch.stop();
      window.close();
      app.quit();
    });
    ros_ok_watch.start();

    result = app.exec();
  }

  g_qt_app.store(nullptr, std::memory_order_release);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
