#ifndef CAMERA_MANAGE
#define CAMERA_MANAGE
#include "cam_com_struct.hpp"
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <functional>
// Include log system
#include "log_system/log_macros.hpp"

namespace cv
{
    class Mat;
};
namespace pcl
{
    template <typename PointT>
    class PointCloud;

    struct PointXYZ;
}

// 声明具体的智能指针类型
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = std::shared_ptr<PointCloudXYZ>;
using PointCloudXYZConstPtr = std::shared_ptr<const PointCloudXYZ>;

class CamBase;
class CameraManager
{

protected:
    CameraManager();

public:
    ~CameraManager();

    static CameraManager &get_instance();

    RtnType get_all_devices(CamDevInfoList &cam_list);
    RtnType init_all_camera(const CamConfigInfo1D &cam_configs);

    RtnType get_stream_enable(short cam_id, CamStreamType modu, bool &enable); // 获取各模块使能
    RtnType set_stream_enable(short cam_id, CamStreamType modu, bool enable);  // 设置各模块使能

    RtnType get_cam_intrinsics(short cam_id, CamStreamType modu, CamIntrinsics &intrinsics); // 获取相机内参
    RtnType save_cam_intrinsics(int cam_id, std::string file_path); // 保存单个相机内参到文件
    RtnType save_all_cam_intrinsics(std::string file_path); // 保存所有相机内参到文件

    // 获取相机变量
    CamType get_cam_type(short cam_id);                    // 获取相机类型
    CamInterfaceType get_cam_interface_type(short cam_id); // 获取相机接口类型
    std::string get_serial_number(short cam_id);                       // 获取相机序列号

    RtnType get_cam_roi_fps_list(short cam_id,std::map<std::string,CamRoiFpsList> &cam_roi_fps_list);
    // 曝光
    RtnType get_exposure_range(short cam_id, float &min_exposure, float &max_exposure, CamStreamType modu); // 获取最大曝光
    RtnType get_exposure_val(short cam_id, float &exposure, CamStreamType modu);                            // 获取曝光时间（快门时间）
    RtnType set_exposure_val(short cam_id, float exposure, CamStreamType modu);                             // 设置曝光时间（快门时间）

    // 帧率
    RtnType get_acqu_fps_enable(short cam_id, bool &enable, CamStreamType modu);         // 获取帧率使能
    RtnType set_acqu_fps_enable(short cam_id, bool enable, CamStreamType modu);          // 设置帧率使能
    RtnType get_fps_range(short cam_id, int &min_fps, int &max_fps, CamStreamType modu); // 获取帧率范围
    RtnType get_fps(short cam_id, int &fps, CamStreamType modu);                         // 获取帧率值
    RtnType set_fps(short cam_id, int fps, CamStreamType modu);                          // 设置帧率值
    // RtnType get_result_fps(short cam_id,int &fps, CamStreamType modu);                  // 获取实际帧率值

    // 增益
    RtnType get_gain_range(short cam_id, float &min_gain, float &max_gain, CamStreamType modu); // 获取最大增益
    RtnType get_gain_val(short cam_id, float &gain, CamStreamType modu);                        // 获取增益
    RtnType set_gain_val(short cam_id, float gain, CamStreamType modu);                         // 设置增益

    // 亮度
    RtnType get_brightness_range(short cam_id, float &min_bright, float &max_bright); // 获取亮度范围
    RtnType get_brightness(short cam_id, float &bright);                              // 获取图像亮度
    RtnType set_brightness(short cam_id, float bright);                               // 设置图像亮度

    // Gamma
    RtnType get_gamma_range(short cam_id, float &min_gamma, float &max_gamma); // 获取最大Gamma值
    RtnType get_gamma(short cam_id, float &gamma);                             // 获取Gamma值
    RtnType set_gamma(short cam_id, float gamma);                              // 设置Gamma值

    RtnType get_one_frame(short cam_id, CamFramelist &frames); // 获取一帧图像
    RtnType get_one_frame(short cam_id, cv::Mat *&img_color, cv::Mat *&img_depth, PointCloudXYZPtr &cloud);
    RtnType get_one_frame_color(short cam_id, cv::Mat *&img);
    RtnType get_one_frame_depth(short cam_id, cv::Mat *&img);
    RtnType get_one_frame_cloud(short cam_id, PointCloudXYZPtr &cloud);

private:
    std::unordered_map<short, std::shared_ptr<CamBase>> cam_all_device;
    std::unordered_map<short, std::shared_ptr<CamComPara>> cam_all_paras;
    CamDevInfoList cam_list; // 相机信息列表

public:
    CamBase *get_cam_obj(short cam_id);

private:
    static std::unique_ptr<CameraManager> instance_;
    static std::mutex instance_mutex_;
    std::mutex mutex_;
};

#endif