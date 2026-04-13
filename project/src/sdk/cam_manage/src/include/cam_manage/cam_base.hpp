#ifndef CAMERA_BASE
#define CAMERA_BASE

#include <vector>
#include <map>
#include <string>
#include <memory>
#include "cam_com_struct.hpp"

// 前置声明
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

class CamBase
{
public:
	CamBase() {};
	virtual ~CamBase() {};
	virtual RtnType init(const CamConfigInfo &config) = 0;						 // 初始化相机
	// 功能封装
	virtual RtnType open_cam() = 0;											 // 打开相机
	virtual RtnType close_cam() = 0;										 // 关闭相机
	virtual RtnType init_defult_para() = 0;									 // 设置相机默认参数（层的默认值）[不常修改]
	virtual RtnType init_special_para() = 0;								 // 设置不同相机的特殊参数（根据功能要求修改）
	virtual RtnType get_stream_enable(CamStreamType modu, bool &enable) = 0; // 获取各模块使能
	virtual RtnType set_stream_enable(CamStreamType modu, bool enable) = 0;	 // 设置各模块使能
	virtual RtnType set_cam_config(const CamConfigInfo &config) = 0;		 // 相机配置	 
	virtual RtnType is_cam_start(bool &is_start) = 0;									   // 相机是否开启
	virtual bool is_cam_start() = 0;													   // 相机是否开启
	virtual RtnType get_cam_intrinsics(CamStreamType modu, CamIntrinsics &intrinsics) = 0; // 相机内参获取

	// 像素格式[获取相机变量]
	virtual RtnType init_pixel_type() = 0; // 初始化

	// 获取相机变量
	virtual CamType get_cam_type() = 0;					   // 获取相机类型
	virtual CamInterfaceType get_cam_interface_type() = 0; // 获取相机接口类型
	virtual std::string get_serial_number() = 0;          // 获取相机序列号

	// 常用参数设置接口
	virtual RtnType get_cam_roi_fps_list(std::map<std::string,CamRoiFpsList> &cam_roi_fps_list) = 0;
	virtual RtnType get_cam_para_enable(CamParaType para_type, CamStreamType modu, bool &enable) = 0; // 获取参数使能
	virtual RtnType set_cam_para_enable(CamParaType para_type, CamStreamType modu, bool enable) = 0;  // 设置参数使能

	virtual RtnType get_cam_para_range(CamParaType para_type, float &min_val, float &max_val, CamStreamType modu) = 0; // 获取相机参数范围
	virtual RtnType set_cam_para(CamParaType para_type, float value, CamStreamType modu) = 0;
	virtual RtnType get_cam_para(CamParaType para_type, float &value, CamStreamType modu) = 0;

	virtual RtnType get_cam_para_range(CamParaType para_type, int &min_val, int &max_val, CamStreamType modu) = 0; // 获取相机参数范围
	virtual RtnType set_cam_para(CamParaType para_type, int value, CamStreamType modu) = 0;
	virtual RtnType get_cam_para(CamParaType para_type, int &value, CamStreamType modu) = 0;

	// 曝光
	virtual RtnType get_exposure_range(float &min_exposure, float &max_exposure, CamStreamType modu); // 获取最大曝光
	virtual RtnType get_exposure_val(float &exposure, CamStreamType modu);							  // 获取曝光时间（快门时间）
	virtual RtnType set_exposure_val(float exposure, CamStreamType modu);							  // 设置曝光时间（快门时间）

	// 帧率
	virtual RtnType get_acqu_fps_enable(bool &enable, CamStreamType modu);		   // 获取帧率使能
	virtual RtnType set_acqu_fps_enable(bool enable, CamStreamType modu);		   // 设置帧率使能
	virtual RtnType get_fps_range(int &min_fps, int &max_fps, CamStreamType modu); // 获取帧率范围
	virtual RtnType get_fps(int &fps, CamStreamType modu);						   // 获取帧率值
	virtual RtnType set_fps(int fps, CamStreamType modu);						   // 设置帧率值
	// virtual RtnType get_result_fps(int &fps, CamStreamType modu) = 0;				// 获取实际帧率值

	// 增益
	virtual RtnType get_gain_range(float &min_gain, float &max_gain, CamStreamType modu); // 获取最大增益
	virtual RtnType get_gain_val(float &gain, CamStreamType modu);						  // 获取增益
	virtual RtnType set_gain_val(float gain, CamStreamType modu);						  // 设置增益

	// 亮度
	virtual RtnType get_brightness_range(float &min_bright, float &max_bright); // 获取亮度范围
	virtual RtnType get_brightness(float &bright);								// 获取图像亮度
	virtual RtnType set_brightness(float bright);								// 设置图像亮度

	// Gamma
	virtual RtnType get_gamma_range(float &min_gamma, float &max_gamma); // 获取最大Gamma值
	virtual RtnType get_gamma(float &gamma);							 // 获取Gamma值
	virtual RtnType set_gamma(float gamma);								 // 设置Gamma值

	// #########################################彩色相机独有接口###################################################
	// 相机白平衡调节（范围、设置&获取）————彩色相机
	virtual RtnType set_white_balance_mode(WhiteBalenceMode balanceWhiteMode) = 0;	// 设置相机白平衡模式
	virtual RtnType get_white_balance_mode(WhiteBalenceMode &balanceWhiteMode) = 0; // 获取相机(自动)白平衡模式
	//
	virtual RtnType get_balance_ratio_range(int &balan_min, int &balan_max) = 0; // 获取相机图像白平衡红色比例范围
	virtual RtnType set_balance_ratio(int balan_ratio) = 0;						 // 设置相机图像白平衡红色比例
	virtual RtnType get_balance_ratio(int &balan_ratio) = 0;					 // 获取相机图像白平衡红色比例

	// 获取一帧图像
	virtual RtnType get_one_frame(CamFramelist &frames) = 0;
	virtual RtnType get_one_frame(cv::Mat *&img_color, cv::Mat *&img_depth, PointCloudXYZPtr &cloud)=0;//获取一帧彩色图、深度图、点云
	virtual RtnType get_one_frame_color(cv::Mat *&img) = 0;
	virtual RtnType get_one_frame_depth(cv::Mat *&img) = 0;
	virtual RtnType get_one_frame_cloud(PointCloudXYZPtr & cloud) = 0;
public:
	// 相机固有的变量

	// 相机状态变量

	// 相机参数变量

protected:
	CamDevInfo cam_info_;		 // 相机信息
	CamDevInfoList cam_dev_list_; // 遍历的所有相机列表
	//static std::vector<CamDevInfo> cam_info_list_; // 静态设备信息列表
	CamComPara cam_para;		 // 相机参数
	std::map<std::string,CamRoiFpsList> cam_roi_fps_list_; //可选择的ROI和FPS列表
	CamFrameData depth_frame_data; // 深度流
	CamFrameData color_frame_data; // 彩图流

	bool is_show_roi_fps_log_{true};
};

#endif