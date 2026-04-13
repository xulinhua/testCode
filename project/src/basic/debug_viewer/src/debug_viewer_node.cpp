// debug_viewer_node.cpp
#include <rclcpp/rclcpp.hpp>
#include "DebugViewer.h"

using namespace std::chrono_literals;

class DebugViewerNode : public rclcpp::Node
{
public:
    DebugViewerNode() : Node("debug_viewer_node")
    {
        // 初始化 DebugViewer
        // m_debug_viewer = std::make_unique<DebugView::CHsDebugViewer>();
        
        RCLCPP_INFO(this->get_logger(), "Debug Viewer node has been started");
        
        // 创建定时器来定期发布调试信息
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&DebugViewerNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Debug Viewer is running");
        // 在这里可以调用 DebugViewer 的功能
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    // std::unique_ptr<DebugView::CHsDebugViewer> m_debug_viewer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugViewerNode>());
    rclcpp::shutdown();
    return 0;
}