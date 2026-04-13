#include "ppocr.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <freetype2/ft2build.h>
#include <freetype/freetype.h>
#include "bas_operate_ros/ros_comm_info.h"
#include "bas_operate_ros/param_utils.hpp"
#include "hand_eye_calib/calib_struct.hpp"
#include "hand_eye_calib/calib_utils.hpp"
#include "calib_info_src/calib_info_server.h"
#include "sys_info_src/sys_info_server.h"
#include "log_system/log_macros.hpp"

class OCR_Node : public rclcpp::Node
{
public:
    OCR_Node(const rclcpp::NodeOptions &options, int cam_id);

private:
    void Color_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void Depth_Callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void publish(const std::vector<PPOCRResult>& results, const std_msgs::msg::Header& header);
    void initTopicNames();  // 使用parseCommInfo初始化话题名
    void initCalibParamHandler();  // 初始化标定参数处理器
    bool getSysDat();  // 从参数服务器获取系统配置（包括相机对应的机械臂列表）
    bool getCamConfigInfo(SysConfig::CamConfigInfo& cam_info);  // 获取相机配置信息
    void calibDatChangedCallback(const handeyecalib::ArmCalibInfo& calib_data);  // 标定数据回调
    bool getCameraIntrinsicsFromServer();  // 从参数服务器获取相机内参
    void drawResultsWith3D(cv::Mat& img, const std::vector<PPOCRResult>& results,
                          const cv::Mat& depth_img, const cv::Mat& intrinsics);  // 绘制结果和3D位置
    void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rrect, const cv::Scalar& color);  // 绘制旋转矩形框
    cv::Point3d calculate3DPosition(const cv::Point2f& center, const cv::Mat& depth_img,
                                    const cv::Mat& intrinsics);  // 计算3D坐标
    void drawChineseText(cv::Mat& img, const std::string& text, const cv::Point& text_center,
                         const cv::Scalar& color);  // 绘制中文文本
    void drawEnglishText(cv::Mat& img, const std::string& text, const cv::Point& text_pos,
                         const cv::Scalar& color);  // 绘制英文文本
    void draw3DPositionText(cv::Mat& img, const cv::Point3d& pos, const cv::Point& center,
                            const cv::Scalar& color);  // 绘制3D坐标文本
    void drawResultText(cv::Mat& img, const std::string& text, const cv::RotatedRect& rrect,
                        const cv::Scalar& color);  // 绘制识别文本
    void printRecognitionResult(const PPOCRResult& result, const cv::Point3d& pos3d,
                                int idx);  // 打印识别结果详情

private:
    PPOCR ocr_;
    cv::Mat color_frame_;
    cv::Mat depth_frame_;
    std::string detection_engine_name_;
    std::string recognition_engine_name_;
    std::string character_dict_name_;
    int camera_id_;                        // 相机ID，支持多相机配置
    int arm_id_;                          // 机械臂ID，用于获取对应的标定矩阵
    std::vector<int> arm_id_list_;         // 机械臂ID列表，支持多机械臂检测
    std::string camera_type_;              // 相机类型: realsense, orbbec
    std::string color_image_topic_;        // rgb图像话题名称
    std::string depth_image_topic_;        // 深度图像话题名称
    std::string camera_info_topic_;        // 相机信息话题名称
    
    // 相机内参
    cv::Mat camera_matrix_;                // 相机内参矩阵
    cv::Mat dist_coeffs_;                 // 畸变系数
    bool camera_info_received_;           // 是否已接收相机内参
    
    // 深度图像相关
    bool depth_info_received_;            // 是否已接收深度图像
    std::mutex depth_mutex_;
    std::mutex camera_info_mutex_;
    
    // 标定参数
    bool usecalib_;                       // 是否使用标定模式
    std::unique_ptr<handeyecalib::CalibRes> calib_result_;  // 标定结果
    rclcpp::SyncParametersClient::SharedPtr sys_config_client_;  // 参数服务器客户端

    // FreeType支持中文绘制
    FT_Library ft_library_;
    FT_Face ft_face_;
    bool ft_initialized_;
    bool font_loaded_;
    
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr ocr_res_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ocr_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ocr_text_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // 日志相关
    std::string log_project_path_;  // 根据camera_id生成的日志项目路径
};
