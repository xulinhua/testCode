#include "ros_robot_workbench/ui/lazy_feature_page.hpp"

#include <QVBoxLayout>

namespace ros_robot_workbench::ui
{

LazyFeaturePage::LazyFeaturePage(Factory factory, QWidget * parent)
: QWidget(parent), factory_(std::move(factory))
{
  auto * lay = new QVBoxLayout(this);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(0);
}

void LazyFeaturePage::ensureBuilt()
{
  if (content_ != nullptr || !factory_) {
    return;
  }
  content_ = factory_(this);
  if (!content_) {
    return;
  }
  auto * lay = qobject_cast<QVBoxLayout *>(layout());
  if (lay) {
    lay->addWidget(content_, 1);
  }
}

void LazyFeaturePage::showEvent(QShowEvent * event)
{
  QWidget::showEvent(event);
  ensureBuilt();
}

}  // namespace ros_robot_workbench::ui
