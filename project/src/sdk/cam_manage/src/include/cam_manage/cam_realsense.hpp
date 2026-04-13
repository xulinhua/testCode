#ifndef CAMERA_REALSENSE
#define CAMERA_REALSENSE
#include "cam_com_struct.hpp"
#include "cam_base.hpp"
// Include log system
#include "log_system/log_macros.hpp"
#include <librealsense2/rs.hpp>

// Include system headers for process management
#include <dirent.h>
#include <signal.h>
#include <unistd.h>
#include <sys/stat.h>

// namespace rs2
// {
// 	class device;
// 	class pipeline;
// 	class config;
// 	class device_list;
// };
class CamRealsense : public CamBase
{
public:
	CamRealsense();
	virtual ~CamRealsense();
	static RtnType get_all_devices(std::vector<CamDevInfo> &cam_info_list); // 获取所有相机
	static std::vector<CamDevInfo> cam_info_list_; // 静态设备信息列表
	static rs2::device_list device_list_; // 静态设备列表
	virtual RtnType init(const CamConfigInfo &config);						 // 初始化相机
	// 功能封装
	virtual RtnType open_cam();											 // 打开相机
	virtual RtnType close_cam();										 // 关闭相机
	virtual RtnType init_defult_para();									 // 设置相机默认参数（层的默认值）[不常修改]
	virtual RtnType init_special_para();								 // 设置不同相机的特殊参数（根据功能要求修改）
	virtual RtnType get_stream_enable(CamStreamType modu, bool &enable); // 获取各模块使能
	virtual RtnType set_stream_enable(CamStreamType modu, bool enable);	 // 设置各模块使能
	virtual RtnType set_cam_config(const CamConfigInfo &config);

	virtual RtnType is_cam_start(bool &is_start);
	virtual bool is_cam_start();
	virtual RtnType get_cam_intrinsics(CamStreamType modu, CamIntrinsics &intrinsics); // 相机内参获取
	
	// 像素格式[获取相机变量]
	virtual RtnType init_pixel_type(); // 初始化

	// 获取相机变量
	virtual CamType get_cam_type();					   // 获取相机类型
	virtual CamInterfaceType get_cam_interface_type(); // 获取相机接口类型
	virtual std::string get_serial_number();          // 获取相机序列号

	// 常用参数设置接口
	virtual RtnType get_cam_roi_fps_list(std::map<std::string,CamRoiFpsList> &cam_roi_fps_list);
	
	virtual RtnType get_closest_roi_fps(CamStreamType stream_type, int target_width, int target_height, int target_fps, int& closest_width, int& closest_height, int& closest_fps);
	virtual RtnType get_cam_para_enable(CamParaType para_type, CamStreamType modu, bool &enable); // 获取参数使能
	virtual RtnType set_cam_para_enable(CamParaType para_type, CamStreamType modu, bool enable);  // 设置参数使能

	virtual RtnType get_cam_para_range(CamParaType para_type, float &min_val, float &max_val, CamStreamType modu); // 获取相机参数范围
	virtual RtnType set_cam_para(CamParaType para_type, float value, CamStreamType modu);
	virtual RtnType get_cam_para(CamParaType para_type, float &value, CamStreamType modu);

	virtual RtnType get_cam_para_range(CamParaType para_type, int &min_val, int &max_val, CamStreamType modu); // 获取相机参数范围
	virtual RtnType set_cam_para(CamParaType para_type, int value, CamStreamType modu);
	virtual RtnType get_cam_para(CamParaType para_type, int &value, CamStreamType modu);

	// 采集图像格式
	virtual RtnType get_pixel_format(PixelFormat &pixel_format);
	virtual RtnType set_pixel_format(PixelFormat pixel_format);

	virtual RtnType set_white_balance_mode(WhiteBalenceMode balanceWhiteMode);	// 设置相机白平衡模式
	virtual RtnType get_white_balance_mode(WhiteBalenceMode &balanceWhiteMode); // 获取相机(自动)白平衡模式
	//
	virtual RtnType get_balance_ratio_range(int &balan_min, int &balan_max); // 获取相机图像白平衡红色比例范围
	virtual RtnType set_balance_ratio(int balan_ratio);						 // 设置相机图像白平衡红色比例
	virtual RtnType get_balance_ratio(int &balan_ratio);					 // 获取相机图像白平衡红色比例

	virtual RtnType get_one_frame(CamFramelist &frames);
	virtual RtnType get_one_frame(cv::Mat *&img_color, cv::Mat *&img_depth, PointCloudXYZPtr &cloud);
	virtual RtnType get_one_frame_color(cv::Mat *&img);
	virtual RtnType get_one_frame_depth(cv::Mat *&img);
	virtual RtnType get_one_frame_cloud(PointCloudXYZPtr & cloud);

private:
	// 设备占用处理函数
	void killOccupyingProcesses(const std::string& serial_number);
	bool isProcessUsingDevice(int pid, const std::string& serial_number);
	rs2::device device_;		   // 设备
	rs2::pipeline pipe_;		   // 管道
	rs2::config config_;		   // 配置
	//rs2::align align_to_color;
};

#endif