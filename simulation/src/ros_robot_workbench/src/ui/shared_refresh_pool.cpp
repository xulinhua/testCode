#include "ros_robot_workbench/ui/shared_refresh_pool.h"

#include <QThreadPool>
#include <QRunnable>

namespace ros_robot_workbench::ui
{
namespace
{

QThreadPool & SharedPool()
{
  static QThreadPool pool;
  static bool initialized = false;
  if (!initialized) {
    pool.setMaxThreadCount(2);
    initialized = true;
  }
  return pool;
}

QThreadPool & SharedImagePool()
{
  static QThreadPool pool;
  static bool initialized = false;
  if (!initialized) {
    pool.setMaxThreadCount(2);
    initialized = true;
  }
  return pool;
}

}  // namespace

void RunOnSharedRefreshPool(std::function<void()> task)
{
  if (!task) return;
  auto * runnable = QRunnable::create([task = std::move(task)]() mutable { task(); });
  runnable->setAutoDelete(true);
  SharedPool().start(runnable);
}

void RunOnSharedImageRefreshPool(std::function<void()> task)
{
  if (!task) return;
  auto * runnable = QRunnable::create([task = std::move(task)]() mutable { task(); });
  runnable->setAutoDelete(true);
  SharedImagePool().start(runnable);
}

}  // namespace ros_robot_workbench::ui
