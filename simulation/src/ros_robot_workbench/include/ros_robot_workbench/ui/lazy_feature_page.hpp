#ifndef ROS_ROBOT_WORKBENCH__UI__LAZY_FEATURE_PAGE_HPP_
#define ROS_ROBOT_WORKBENCH__UI__LAZY_FEATURE_PAGE_HPP_

#include <functional>

#include <QShowEvent>
#include <QWidget>

namespace ros_robot_workbench::ui
{

class LazyFeaturePage : public QWidget
{
public:
  using Factory = std::function<QWidget *(QWidget * parent)>;
  explicit LazyFeaturePage(Factory factory, QWidget * parent = nullptr);

  void ensureBuilt();
  QWidget * content() const { return content_; }

protected:
  void showEvent(QShowEvent * event) override;

private:
  Factory factory_;
  QWidget * content_{nullptr};
};

}  // namespace ros_robot_workbench::ui

#endif
