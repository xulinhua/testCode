#include "visualization/visualization_mgr.hpp"
#include "log_system/log_macros.hpp"
#include <iostream>

// 将所有实现放在命名空间中
namespace visualization {

VisualizationMgr::VisualizationMgr()
    : window_created_(false)
    , window_width_(1280)
    , window_height_(720)
{

}

VisualizationMgr::~VisualizationMgr() 
{
    closeWindows();
}

bool VisualizationMgr::initialize(const std::string& window_name, int width, int height) 
{
    window_name_ = window_name;
    window_width_ = width;
    window_height_ = height;
    
    try {
        // 尝试创建窗口
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name_, window_width_, window_height_);
        window_created_ = true;
        
        LOG_INFO("✅ 可视化窗口已创建: %s", window_name_.c_str());
        return true;
    } catch (const cv::Exception& e) {
        LOG_ERROR("❌ OpenCV异常 - 创建可视化窗口失败: %s", e.what());
        window_created_ = false;
        
        // 提供更多信息帮助诊断
        LOG_INFO("OpenCV版本: %s", CV_VERSION);
        LOG_INFO("可能的原因:");
        LOG_INFO("1. GUI后端未正确安装(如GTK)");
        LOG_INFO("2. DISPLAY环境变量未设置(SSH连接时)");
        LOG_INFO("3. 权限不足");
        
        return false;
    } catch (const std::exception& e) {
        LOG_ERROR("❌ : %s", e.what());
        window_created_ = false;
        return false;
    }
}

void VisualizationMgr::showImage(const cv::Mat& image) 
{
    if (!window_created_ || image.empty()) {
        return;
    }
    
    try {
        cv::imshow(window_name_, image);
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV异常 - 显示图像时出错: %s", e.what());
    } catch (const std::exception& e) {
        LOG_ERROR("显示图像时出错: %s", e.what());
    }
}

void VisualizationMgr::closeWindows() 
{
    closeWindows(true);
}

void VisualizationMgr::closeWindows(bool destroy_all) 
{
    if (window_created_) {
        try {
            if (destroy_all) {
                cv::destroyAllWindows();
                LOG_INFO("✅ 所有可视化窗口已关闭");
            } else {
                cv::destroyWindow(window_name_);
                LOG_INFO("✅ 可视化窗口已关闭: %s", window_name_.c_str());
            }
            window_created_ = false;
        } catch (const cv::Exception& e) {
            LOG_ERROR("OpenCV异常 - 关闭可视化窗口时出错: %s", e.what());
        } catch (const std::exception& e) {
            LOG_ERROR("关闭可视化窗口时出错: %s", e.what());
        }
    }
}

bool VisualizationMgr::isWindowOpen() const 
{
    return window_created_;
}

} // namespace visualization