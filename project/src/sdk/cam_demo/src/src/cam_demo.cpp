#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "cam_manage/cam_manage.hpp"
#include "cam_manage/cam_com_struct.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
std::string package_share_dir = ament_index_cpp::get_package_share_directory("cam_demo");
std::string config_path = package_share_dir + "/../../../sys_config/cam_config.yaml";

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
        color_image = nullptr;
        cloud = nullptr;
        // 创建发布者
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/points", 10);

        // 初始化相机管理器
        cam_manager_ = &CameraManager::get_instance();
        // 遍历相机
        get_all_camera();
        // 从配置加载相机
        load_camera_config();
        // 获取内参
        get_cam_intr();
        // 创建定时器获取并发布数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&CameraPublisher::publish_data, this));
    }

private:
    void get_all_camera()
    {
        // 遍历所有相机
        CamDevInfoList cam_list;
        cout << "正在扫描所有相机..." << endl;

        RtnType rtn = cam_manager_->get_all_devices(cam_list);
        if (rtn != RtnType::RTN_SUCCESS)
        {
            cout << "获取相机列表失败！" << endl;
            return;
        }

        cout << "找到 " << cam_list.size() << " 个相机：" << endl;
        cout << "=========================" << endl;

        for (size_t i = 0; i < cam_list.size(); i++)
        {
            const CamDevInfo &info = cam_list[i];

            cout << "相机 " << i + 1 << ":" << endl;
            cout << "  类型: " << (info.cam_type == CamType::CAM_TYPE_RS ? "RealSense" : info.cam_type == CamType::CAM_TYPE_OB ? "Orbbec"
                : info.cam_type == CamType::CAM_TYPE_CSI ? "CSI" : "Unknown")
                 << endl;
            cout << "  设备名称: " << info.device_name << endl;
            cout << "  序列号: " << info.serial_number << endl;
            cout << "  固件版本: " << info.firmware_version << endl;
            cout << "  产品ID: " << info.product_id << endl;
            cout << "-------------------------" << endl;
        }

        cout << "相机扫描完成！" << endl;
    }
    void load_camera_config()
    {
        try
        {
            configs.clear();
            std::cout << "config_path: " << config_path << std::endl;
            
            // 检查配置文件是否存在
            if (std::filesystem::exists(config_path))
            {
                YAML::Node config_yaml = YAML::LoadFile(config_path);
                if (config_yaml["default_camera"])
                {
                    std::string cam_type = config_yaml["default_camera"]["camera_type"].as<string>();
                    std::string cam_resolution = config_yaml["default_camera"]["resolution"].as<string>();
                    int cam_fps = config_yaml["default_camera"]["fps"].as<int>();

                    cout << "加载相机配置:" << endl;
                    cout << "  相机类型: " << cam_type << endl;
                    cout << "  分辨率: " << cam_resolution << endl;
                    cout << "  帧率: " << cam_fps << endl;

                    CamConfigInfo config;
                    config.cam_id = 0;
                    if (cam_type == "realsense")
                    {
                        config.cam_type = CamType::CAM_TYPE_RS;
                    }
                    else if (cam_type == "orbbec")
                    {
                        config.cam_type = CamType::CAM_TYPE_OB;
                    } 
                    else if (cam_type == "CSI")
                    {
                        config.cam_index = 0;
                    }
                    config.depth_para.fps = cam_fps;
                    config.color_para.fps = cam_fps;
                    // 解析分辨率字符串格式为"宽x高"
                    size_t x_pos = cam_resolution.find('x');
                    int width = stoi(cam_resolution.substr(0, x_pos));
                    int height = stoi(cam_resolution.substr(x_pos + 1));
                    if (x_pos != string::npos)
                    {
                        config.color_para.width = width;
                        config.color_para.height = height;
                        config.depth_para.width = width;
                        config.depth_para.height = height;
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Invalid resolution format, using default 640x480");
                        config.color_para.width = 640;
                        config.color_para.height = 480;
                        config.depth_para.width = 640;
                        config.depth_para.height = 480;
                    }

                    configs.push_back(config);
                }
            }
            else
            {
                configs.clear();
                cout << "加载相机配置失败,使用默认相机配置" << endl;
                CamConfigInfo config;
                config.cam_id = 0;
                config.cam_index = 0;
                config.cam_type = CamType::CAM_TYPE_RS;
                config.serial_number="";
                config.depth_para.fps = 0;
                config.color_para.fps = 0;
                config.color_para.width = 1280;
                config.color_para.height = 720;
                config.depth_para.width = 1280;
                config.depth_para.height = 720;

                configs.push_back(config);
            }
            RtnType rtn = cam_manager_->init_all_camera(configs);
            if (rtn != RtnType::RTN_SUCCESS)
            {
                cout << "初始化相机失败!" << endl;
            }
            else
            {
                cout << "初始化相机成功!" << endl;
            }
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "加载相机配置失败");
        }
    }
    void get_cam_intr()
    {
        // 获取相机内参
        CamIntrinsics intrinsics;
        for (int i = 0; i < 2; i++)
        {
            CamStreamType stream_type = i == 1 ? CamStreamType::STREAM_COLOR : CamStreamType::STREAM_DEPTH;
            RtnType rtn = cam_manager_->get_cam_intrinsics(0, stream_type, intrinsics); // 获取相机内参

            if (rtn == RtnType::RTN_SUCCESS)
            {
                cout << (i == 1 ? "彩色相机内参：" : "深度相机内参：") << endl;
                cout << "  width: " << intrinsics.width << endl;
                cout << "  height: " << intrinsics.height << endl;
                cout << "  fx: " << intrinsics.fx << endl;
                cout << "  fy: " << intrinsics.fy << endl;
                cout << "  cx: " << intrinsics.cx << endl;
                cout << "  cy: " << intrinsics.cy << endl;
                cout << "  畸变系数: [";
                for (size_t j = 0; j < 5; j++)
                {
                    cout << intrinsics.dist_coeffs[j] << ", ";
                }
                cout << "]" << endl;
            }
            else
            {
                cout << (i == 1 ? "彩色相机内参：" : "深度相机内参：") << "获取失败!" << endl;
            }

            // 保存至配置文件
        }
    }

    void publish_data()
    {
        // 获取图像和点云
        cv::Mat *color_image = nullptr;
        cv::Mat *depth_image = nullptr;
        PointCloudXYZPtr cloud = nullptr;
       // cam_manager_->get_one_frame(0, color_image,depth_image,cloud);
        cam_manager_->get_one_frame_color(0, color_image);
        // 转换并发布消息
        if (color_image && !color_image->empty())
        {
            auto img_msg = cv_bridge::CvImage(
                               std_msgs::msg::Header(), "bgr8", *color_image)
                               .toImageMsg();
            image_publisher_->publish(*img_msg);

            std::cout<<"color_image: "<<color_image->cols<<" "<<color_image->rows<<std::endl;
            imshow("color_image", *color_image);
            cv::waitKey(10);
        }

        if (depth_image && !depth_image->empty())
        {
            std::cout<<"depth_image: "<<depth_image->cols<<" "<<depth_image->rows<<std::endl;
            imshow("depth_image", *depth_image);
            cv::waitKey(10);
        }

        if (cloud && !cloud->empty())
        {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header.frame_id = "camera_frame";
            pointcloud_publisher_->publish(cloud_msg);

            #ifdef ENABLE_VISUALIZATION
            try {
                if (!viewer_.contains("cloud"))
                {
                    // 如果不存在，添加新点云
                    viewer_.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
                    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                    viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
                }
                else
                {
                    // 如果已存在，更新点云
                    viewer_.updatePointCloud(cloud, "cloud");
                }
                viewer_.spinOnce(20);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "PCL可视化异常: %s", e.what());
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "PCL可视化发生未知异常");
            }
            #endif
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    CameraManager *cam_manager_;
    CamConfigInfo1D configs;
    cv::Mat *color_image;
    PointCloudXYZPtr cloud;
    pcl::visualization::PCLVisualizer viewer_;
};

int main(int argc, char **argv)
{
    cout << "=== Camera Demo Start ===" << endl;

    try
    {
        rclcpp::init(argc, argv);
        auto camera_publisher = std::make_shared<CameraPublisher>();
        rclcpp::spin(camera_publisher);
        rclcpp::shutdown();
    }
    catch (...)
    {
        cout << "未知异常发生！" << endl;
        return 1;
    }

    cout << "=== Camera Demo End ===" << endl;
    return 0;
}