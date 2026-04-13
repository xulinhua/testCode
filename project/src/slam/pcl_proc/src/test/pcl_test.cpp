#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <thread>
#include <chrono>

// 包含log_system中的头文件
#include <log_system/log_macros.hpp>

// 包含pcl_proc中的头文件
#include "../include/pcl_proc/pcl_proc_base.h"
#include "../include/pcl_proc/pcl_proc_filter.h"
#include "../include/pcl_proc/pcl_proc_segment.h"
#include "../include/pcl_proc/pcl_proc_feature.h"

// 处理类别枚举
enum class ProcessCategory {
    FILTER,       // 滤波
    SEGMENT,      // 分割
    FEATURE       // 特征提取
};

// 滤波方法枚举
enum class FilterMethod {
    VOXEL_GRID,             // 体素网格滤波
    STATISTICAL_OUTLIER,    // 统计异常值移除
    PASS_THROUGH            // 直通滤波
};

// 分割方法枚举
enum class SegmentMethod {
    PLANE,      // 平面分割
    CYLINDER    // 圆柱体分割
};

// 特征提取方法枚举
enum class FeatureMethod {
    NORMAL,     // 法向量计算
    FPFH        // FPFH特征
};

using namespace PclProc;

// 全局变量用于在回调函数和主函数之间传递信息
PclProc::PointCloudPtr original_cloud;
PclProc::PointCloudPtr processed_cloud;

// 处理点云数据
PointCloudPtr processPointCloud(PointCloudPtr input_cloud, ProcessCategory category, FilterMethod filter_method, 
                              SegmentMethod segment_method, FeatureMethod feature_method)
{
    PointCloudPtr output_cloud = input_cloud;
    
    switch (category)
    {
        case ProcessCategory::FILTER:
            switch (filter_method)
            {
                case FilterMethod::VOXEL_GRID:
                    output_cloud = PclProc::voxelGridFilter(input_cloud, 0.01f);
                    break;
                case FilterMethod::STATISTICAL_OUTLIER:
                    output_cloud = PclProc::statisticalOutlierRemoval(input_cloud, 50, 1.0);
                    break;
                case FilterMethod::PASS_THROUGH:
                    // 直通滤波
                    break;
            }
            break;
            
        case ProcessCategory::SEGMENT:
            switch (segment_method)
            {
                case SegmentMethod::PLANE:
                    {
                        PointCloudPtr plane_cloud(new PointCloudT);
                        PointCloudPtr remaining_cloud(new PointCloudT);
                        PclProc::planeSegmentation(input_cloud, plane_cloud, remaining_cloud);
                        output_cloud = plane_cloud;
                    }
                    break;
                case SegmentMethod::CYLINDER:
                    {
                        PointCloudPtr cylinder_cloud(new PointCloudT);
                        PointCloudPtr remaining_cloud(new PointCloudT);
                        PclProc::CylinderSegmentation(input_cloud, cylinder_cloud, remaining_cloud);
                        output_cloud = cylinder_cloud;
                    }
                    break;
            }
            break;
            
        case ProcessCategory::FEATURE:
            switch (feature_method)
            {
                case FeatureMethod::NORMAL:
                    // 法线特征处理
                    break;
                case FeatureMethod::FPFH:
                    // FPFH特征处理
                    break;
            }
            break;
    }
    
    return output_cloud;
}

// 创建测试点云数据
PointCloudPtr createTestPointCloud()
{
    PointCloudPtr cloud(new PointCloudT);
    
    // 创建一个简单的立方体形状点云
    for (float x = -0.5f; x <= 0.5f; x += 0.1f)
    {
        for (float y = -0.5f; y <= 0.5f; y += 0.1f)
        {
            for (float z = -0.5f; z <= 0.5f; z += 0.1f)
            {
                // 只添加立方体表面的点
                if (abs(x) == 0.5f || abs(y) == 0.5f || abs(z) == 0.5f)
                {
                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    cloud->push_back(point);
                }
            }
        }
    }
    
    // 添加一些随机噪声点
    srand(time(nullptr));
    for (int i = 0; i < 350; ++i)
    {
        pcl::PointXYZ point;
        point.x = (rand() % 100) / 100.0f - 0.5f;
        point.y = (rand() % 100) / 100.0f - 0.5f;
        point.z = (rand() % 100) / 100.0f - 0.5f;
        cloud->push_back(point);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}

int main(int argc, char * argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pcl_test_node");
    
    // 创建发布者
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("processed_point_cloud", 10);
    
    // 当前处理类别和方法
    ProcessCategory current_category = ProcessCategory::FILTER;
    FilterMethod current_filter_method = FilterMethod::VOXEL_GRID;
    SegmentMethod current_segment_method = SegmentMethod::PLANE;
    FeatureMethod current_feature_method = FeatureMethod::NORMAL;
    
    // 创建测试点云
    PointCloudPtr original_cloud = createTestPointCloud();
    PointCloudPtr processed_cloud = original_cloud;
            
    LOG_INFO("pcl_test",  "创建测试点云，点数: " + std::to_string(original_cloud->size()));
            
    // 创建定时器，定期发布点云
    rclcpp::WallRate loop_rate(10); // 10 Hz
            
    LOG_INFO("pcl_test",  "按 '1' 切换处理类别 (滤波/分割/特征)");
    LOG_INFO("pcl_test",  "按 '2' 切换当前类别中的方法");
    LOG_INFO("pcl_test",  "按 'q' 或 'Q' 退出");
    
    bool quit = false;
    
    // 主循环
    while (rclcpp::ok() && !quit)
    {
        // 检查是否有键盘输入
        // 使用非阻塞方式读取stdin
        int ch = getchar();
        if (ch != EOF)
        {
            char key = static_cast<char>(ch);
            
            LOG_INFO("pcl_test",  "按键: " + std::string(1, key));
            
            if (key == '1')
            {
                // 切换处理类别
                switch (current_category)
                {
                    case ProcessCategory::FILTER:
                        current_category = ProcessCategory::SEGMENT;
                        LOG_INFO("pcl_test",  "切换到分割类别");
                        break;
                    case ProcessCategory::SEGMENT:
                        current_category = ProcessCategory::FEATURE;
                        LOG_INFO("pcl_test",  "切换到特征类别");
                        break;
                    case ProcessCategory::FEATURE:
                        current_category = ProcessCategory::FILTER;
                        LOG_INFO("pcl_test",  "切换到滤波类别");
                        break;
                }
            }
            else if (key == '2')
            {
                // 切换当前类别中的方法
                switch (current_category)
                {
                    case ProcessCategory::FILTER:
                        switch (current_filter_method)
                        {
                            case FilterMethod::VOXEL_GRID:
                                current_filter_method = FilterMethod::STATISTICAL_OUTLIER;
                                LOG_INFO("pcl_test",  "切换到统计异常值滤波");
                                break;
                            case FilterMethod::STATISTICAL_OUTLIER:
                                current_filter_method = FilterMethod::PASS_THROUGH;
                                LOG_INFO("pcl_test",  "切换到直通滤波");
                                break;
                            case FilterMethod::PASS_THROUGH:
                                current_filter_method = FilterMethod::VOXEL_GRID;
                                LOG_INFO("pcl_test",  "切换到体素网格滤波");
                                break;
                        }
                        break;
                    case ProcessCategory::SEGMENT:
                        switch (current_segment_method)
                        {
                            case SegmentMethod::PLANE:
                                current_segment_method = SegmentMethod::CYLINDER;
                                LOG_INFO("pcl_test",  "切换到圆柱体分割");
                                break;
                            case SegmentMethod::CYLINDER:
                                current_segment_method = SegmentMethod::PLANE;
                                LOG_INFO("pcl_test",  "切换到平面分割");
                                break;
                        }
                        break;
                    case ProcessCategory::FEATURE:
                        switch (current_feature_method)
                        {
                            case FeatureMethod::NORMAL:
                                current_feature_method = FeatureMethod::FPFH;
                                LOG_INFO("pcl_test",  "切换到FPFH特征");
                                break;
                            case FeatureMethod::FPFH:
                                current_feature_method = FeatureMethod::NORMAL;
                                LOG_INFO("pcl_test",  "切换到法线特征");
                                break;
                        }
                        break;
                }
            }
            else if (key == 'q' || key == 'Q')
            {
                LOG_INFO("pcl_test",  "收到退出命令");
                quit = true;
            }
        }
        
        // 处理点云
        processed_cloud = processPointCloud(original_cloud, current_category, current_filter_method, 
                                         current_segment_method, current_feature_method);
        
        // 将PCL点云转换为ROS消息
        sensor_msgs::msg::PointCloud2::SharedPtr msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*processed_cloud, *msg);
        msg->header.stamp = node->now();
        msg->header.frame_id = "map";
        
        // 发布点云
        publisher->publish(*msg);
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    // 清理ROS 2
    rclcpp::shutdown();
    return 0;
}