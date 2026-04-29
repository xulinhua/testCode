#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "image_debug_gui/main_window.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  ImageDebug::MainWindow window;
  window.show();

  const int result = app.exec();
  rclcpp::shutdown();
  return result;
}
