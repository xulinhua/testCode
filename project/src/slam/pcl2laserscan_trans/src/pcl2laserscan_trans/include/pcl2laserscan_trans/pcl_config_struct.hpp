#ifndef PCL_CONFIG_STRUCT
#define PCL_CONFIG_STRUCT

#include <string>
#include <vector>

namespace PclConfig
{
    // 系统参数结构体
    typedef struct _SystemConfig
    {
        std::string log_dir; // 日志目录
    } SystemConfig;

    // 启动参数结构体
    typedef struct _LaunchConfig
    {
        std::string default_mode;         // 默认启动模式
        // 话题名参数
        std::string camera_input_topic;   // 相机输入点云话题
        std::string camera_output_topic;  // 相机输出激光扫描话题
        std::string lidar_input_topic;    // 雷达输入点云话题
        std::string lidar_output_topic;   // 雷达输出激光扫描话题
        
        _LaunchConfig()
        {
            default_mode = "both";
            camera_input_topic = "/camera/depth/color/points";
            camera_output_topic = "/camera/laser_scan";
            lidar_input_topic = "/sensor/lidar_3d/top/point_cloud";
            lidar_output_topic = "/lidar/laser_scan";
        }
    } LaunchConfig;

    // 相机参数结构体
    typedef struct _CameraConfig
    {
        // 通用参数
        std::string target_frame;          // 目标坐标系
        float min_height;               // 最小高度
        float max_height;               // 最大高度
        bool bSaveLogInfo2Files;        // 是否保存日志到文件
        bool bOutputToTerminal;         // 是否输出到终端
        float range_min;                // 最小距离
        float range_max;                // 最大距离
        float angle_min;                // 最小角度
        float angle_max;                // 最大角度
        float angle_increment;          // 角度增量
        float voxel_leaf_size;          // 体素大小
        int sor_mean_k;                 // 统计滤波均值K
        float sor_stddev_mul_thresh;    // 统计滤波标准差阈值
        
        _CameraConfig()
        {
            target_frame = "camera_link";
            min_height = 0.1;
            max_height = 1.5;
            bSaveLogInfo2Files = false;
            bOutputToTerminal = false;
            range_min = 0.2;
            range_max = 8.0;
            angle_min = -3.14;
            angle_max = 3.14;
            angle_increment = 0.0087;
            voxel_leaf_size = 0.02;
            sor_mean_k = 20;
            sor_stddev_mul_thresh = 0.5;
        }
    } CameraConfig;

    // 激光雷达参数结构体
    typedef struct _LidarConfig
    {
        // 通用参数
        std::string target_frame;          // 目标坐标系
        float min_height;                  // 最小高度
        float max_height;                  // 最大高度
        float range_min;                   // 最小距离
        float tilt_compensation_angle;     // 倾斜补偿角度
        std::string tilt_axis;             // 倾斜轴
        bool debug;                        // 调试模式
        int filter_mean_k;                 // 滤波均值K
        float filter_stddev;               // 滤波标准差
        float voxel_leaf_size;             // 体素大小
        
        _LidarConfig()
        {
            target_frame = "base_link";
            min_height = 0.4;
            max_height = 1.5;
            range_min = 0.25;
            tilt_compensation_angle = 0.07;
            tilt_axis = "y";
            debug = true;
            filter_mean_k = 30;
            filter_stddev = 0.8;
            voxel_leaf_size = 0.1;
        }
    } LidarConfig;

    // TF变换参数结构体
    typedef struct _TFConfig
    {
        // 相机TF
        float camera_x;          // X坐标（米）
        float camera_y;          // Y坐标（米）
        float camera_z;          // Z坐标（米）
        float camera_roll;       // 滚转角（弧度）
        float camera_pitch;      // 俯仰角（弧度）
        float camera_yaw;        // 偏航角（弧度）
        std::string camera_parent_frame;  // 父坐标系
        std::string camera_child_frame;   // 子坐标系
        
        // 雷达TF
        float lidar_x;          // X坐标（米）
        float lidar_y;          // Y坐标（米）
        float lidar_z;          // Z坐标（米）
        float lidar_roll;       // 滚转角（弧度）
        float lidar_pitch;      // 俯仰角（弧度）
        float lidar_yaw;        // 偏航角（弧度）
        std::string lidar_parent_frame;  // 父坐标系
        std::string lidar_child_frame;   // 子坐标系
        
        _TFConfig()
        {
            // 相机TF默认值
            camera_x = 0.1;
            camera_y = 0.0;
            camera_z = 1.1;
            camera_roll = 0.0;
            camera_pitch = 0.23;
            camera_yaw = 0.0;
            camera_parent_frame = "base_link";
            camera_child_frame = "camera_link";
            
            // 雷达TF默认值
            lidar_x = 0.0;
            lidar_y = 0.0;
            lidar_z = 0.0;
            lidar_roll = 0.0;
            lidar_pitch = 0.0;
            lidar_yaw = 0.0;
            lidar_parent_frame = "jetbot";
            lidar_child_frame = "lidar_Xform";
        }
    } TFConfig;

    // 完整配置结构体
    typedef struct _PclConfig
    {
        SystemConfig system;                           // 系统参数
        LaunchConfig launch;                           // 启动参数
        CameraConfig camera;                           // 相机参数
        LidarConfig lidar;                             // 激光雷达参数
        TFConfig tf;                                   // TF变换参数
    } PclConfig;

}  // namespace PclConfig

#endif