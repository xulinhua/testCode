#ifndef VISUALIZATION__VISUALIZATION_MGR_HPP_
#define VISUALIZATION__VISUALIZATION_MGR_HPP_

#include <opencv2/opencv.hpp>
#include <string>

// 命名空间定义
namespace visualization {

/**
 * @class VisualizationMgr
 * @brief 可视化管理器
 * 
 * 负责通用可视化显示功能
 */
class VisualizationMgr {
public:
    /**
     * @brief 构造函数
     */
    VisualizationMgr();
    
    /**
     * @brief 析构函数
     */
    ~VisualizationMgr();
    
    /**
     * @brief 初始化可视化管理器
     * @param window_name 窗口名称
     * @param width 窗口宽度
     * @param height 窗口高度
     * @return 是否初始化成功
     */
    bool initialize(const std::string& window_name = "Visualization", 
                   int width = 1280, int height = 720);
    
    /**
     * @brief 显示图像
     * @param image 要显示的图像
     */
    void showImage(const cv::Mat& image);
    
    /**
     * @brief 关闭可视化窗口
     */
    void closeWindows();
    
    /**
     * @brief 关闭可视化窗口
     * @param destroy_all 是否关闭所有窗口
     */
    void closeWindows(bool destroy_all);
    
    /**
     * @brief 检查窗口是否打开
     * @return 窗口是否打开
     */
    bool isWindowOpen() const;

private:
    std::string window_name_;     ///< 窗口名称
    bool window_created_;         ///< 窗口是否已创建
    int window_width_;            ///< 窗口宽度
    int window_height_;           ///< 窗口高度
};

} // namespace visualization

#endif // VISUALIZATION__VISUALIZATION_MGR_HPP_