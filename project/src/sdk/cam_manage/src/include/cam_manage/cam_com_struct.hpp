#ifndef CAM_COM_STRUCT
#define CAM_COM_STRUCT

#include <vector>
#include <string>

///////////////////////////////枚举定义///////////////////////////////////

// 返回值类型
typedef enum class _RtnType
{
    RTN_SUCCESS = 0,           // 成功
    RTN_FAILIURE,              // 通用失败（调用相机接口失败）
    RTN_FAILIURE_NONE,         // 没有找到相机
    RTN_FAILIURE_NOFOUND,      // 没有找到对应相机
    RTN_FAILIURE_HANDLE,       // 生成句柄失败
    RTN_FAILIURE_REGISTER,     // 注册回调失败
    RTN_SET_CAMID_INVALID,     // 设置相机ID无效
    RTN_SET_CAMNAME_INVALID,   // 设置相机名称无效
    RTN_FAILIURE_OPEN,         // 打开相机失败
    RTN_FAILIURE_PIXELTYPE,    // 初始化格式失败
    RTN_FAILIURE_INIT,         // 初始化失败
    RTN_FAILIURE_CLOSE,        // 关闭相机失败
    RTN_FAILIURE_RECONNECT,    // 重连相机失败
    RTN_FAILIURE_LIVE,         // 视频流失败
    RTN_FAILIURE_TEIGGERMODE,  // 触发模式失败
    RTN_FAILIURE_TEIGGER,      // 触发失败
    RTN_FAILIURE_EXPOSUREMODE, // 曝光模式失败
    RTN_FAILIURE_MULTISTREAM,  // 多流模式失败
    RTN_FAILIURE_NOFILES,      // 找不到文件
    RTN_FAILIURE_GETPARA,      // 获取参数失败
    RTN_FAILIURE_SETPARA,      // 设置参数失败
    RTN_FAILIURE_VERSION,      // 初始化格式失败
} RtnType;

// 相机类型
typedef enum class _CamType
{
    CAM_TYPE_NONE = -1, // 未知相机
    CAM_TYPE_RS,        // realsense相机
    CAM_TYPE_OB,        // orbbec相机
    CAM_TYPE_CSI,        // CSI摄像头
} CamType;

// 相机接口类型
typedef enum class _CamInterfaceType
{
    INF_TYPE_NONE = -1, // 未知相机
    INF_TYPE_USB,       // USB相机
    INF_TYPE_GIGE,      // 网口相机
    INF_TYPE_CSI,      // CSI相机
} CamInterfaceType;

// 流类型
typedef enum class _CamStreamType
{
    STREAM_DEPTH = 0, // 深度图
    STREAM_COLOR,     // 彩色图
    STREAM_IR,        // 红外
} CamStreamType;

// 自动白平衡模式
typedef enum class _WhiteBalenceMode
{
    AUTO = 0,
    MANUAL,
} WhiteBalenceMode;

typedef enum class _CamParaType
{
    // float、int
    PARA_EXPOSURE = 0,
    PARA_GAIN,
    PARA_GAMMA,
    PARA_BRIGHTNESS,
    PARA_CONTRAST,
    PARA_SATURATION,
    PARA_SHARPNESS,
    PARA_HUE,
    PARA_FPS,

    PARA_BALANCE_RATIO_RED,
    PARA_BALANCE_RATIO_GREEN,
    PARA_BALANCE_RATIO_BLUE,

    // bool
    PARA_AUTO_EXPOSURE,
    PARA_AUTO_WHITE_BALANCE,

    MAX,

} CamParaType;
// 像素格式
typedef enum class _PixelFormat
{
    PIX_FMT_MONO8 = 0,
    PIX_FMT_YUYV,
    PIX_FMT_BGR8,
    PIX_FMT_RGBA8,
    PIX_FMT_BGRA8,
    PIX_FMT_Y8,
    PIX_FMT_RGB8,
    PIX_FMT_RAW16,

} PixelFormat;

///////////////////////////////数据定义///////////////////////////////////

// 相机信息
typedef struct _CamDevInfo
{
    CamType cam_type;             // 相机类型
    std::string device_id;        // 设备id
    std::string product_id;       // 产品id
    std::string device_name;      // 设备名称
    std::string serial_number;    // 序列号
    std::string physical_port;    // 接口类型
    std::string user_name;        // 用户名称
    std::string facturer_name;    // 制造商名字
    std::string firmware_version; // 固件版本
} CamDevInfo;
typedef std::vector<CamDevInfo> CamDevInfoList;

//Roi与FPS对应数据
typedef struct _CamRoiFps
{
    int width;
    int height;
    int fps;
}CamRoiFps;
typedef std::vector<CamRoiFps> CamRoiFpsList;

// 相机内参
typedef struct _CamIntrinsics
{
    _CamIntrinsics()
    {
        width = 0;
        height = 0;
        fx = 0;
        fy = 0;
        cx = 0;
        cy = 0;
        for (int i = 0; i < 5; i++)
        {
            dist_coeffs[i] = 0;
        }
    }
    int width;
    int height;
    float fx;
    float fy;
    float cx;
    float cy;
    float dist_coeffs[5];
} CamIntrinsics;

// 相机外参
typedef struct _CamExtrinsics
{
    _CamExtrinsics()
    {
        for (int i = 0; i < 9; i++)
        {
            rotation[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            translation[i] = 0;
        }
    }
    float rotation[9];
    float translation[3];
} CamExtrinsics;


// 深度流参数
typedef struct _DepthStreamPara
{
    _DepthStreamPara()
    {
        auto_exposure = true;
        width = 640;
        height = 480;
        exposure = -1;
        gamma = -1;
        brightness = -1;
        contrast = -1;
        saturation = -1;
        hue = -1;
        fps = -1;
    }
    bool auto_exposure;

    int width;
    int height;

    float exposure;   // 曝光时间
    float gain;       // 增益
    float gamma;      // Gamma
    float brightness; // 亮度
    float contrast;   // 对比度
    float saturation; // 饱和度
    float sharpness;  // 锐度
    float hue;
    int fps; // 帧率
} DepthStreamPara;

// 图像流参数
typedef struct _ColorStreamPara
{
    _ColorStreamPara()
    {
        auto_exposure = true;
        auto_white_balance = true;
        width = 640;
        height = 480;
        exposure = -1;
        gamma = -1;
        brightness = -1;
        contrast = -1;
        saturation = -1;
        hue = -1;
        fps = -1;
    }
    bool auto_exposure;
    bool auto_white_balance;

    int width;
    int height;

    float exposure;   // 曝光时间
    float gain;       // 增益
    float gamma;      // Gamma
    float brightness; // 亮度
    float contrast;   // 对比度
    float saturation; // 饱和度
    float sharpness;  // 锐度
    float hue;
    int fps; // 帧率
} ColorStreamPara;
// 相机参数
typedef struct _CamComPara
{
    _CamComPara()
    {
        enable_depth_stream = false;
        enable_color_stream = true;
        enable_ir_stream = false;
        enable_cloud_stream = false;
    }
    bool enable_depth_stream;
    bool enable_color_stream;
    bool enable_ir_stream;
    bool enable_cloud_stream;
    DepthStreamPara depth_stream_para; // 深度流参数
    ColorStreamPara color_stream_para; // 图像流参数
    CamIntrinsics depth_intr;          // 深度流内参
    CamIntrinsics color_intr;          // 图像流内参

} CamComPara;

typedef struct _CamFrameData
{
    _CamFrameData()
    {
        frame_type = CamStreamType::STREAM_COLOR;
        pixel_format = PixelFormat::PIX_FMT_BGR8;
        width = 0;
        height = 0;
        frame_index = 0;
        stride = 0;
        time_stamp = 0;
        data_buffer = nullptr;
        data_size = 0;
    }
    ~_CamFrameData()
    {
        if (data_buffer)
        {
            delete[] data_buffer;
            data_buffer = nullptr;
        }
    }
    CamStreamType frame_type; // 帧类型
    PixelFormat pixel_format; // 像素格式
    int width;                // 图像宽度
    int height;               // 图像高度
    int frame_index;          // 帧索引
    int stride;               // 图像步长
    uint64_t time_stamp;      // 时间戳
    void *data_buffer;        // 图像数据缓冲区
    int data_size;            // 数据大小
} CamFrameData;
typedef std::vector<CamFrameData *> CamFramelist;

// 相机配置参数 外部传入，初始化相机用
typedef struct _CamConfigInfo
{
    _CamConfigInfo()
    {
        cam_id = 0;
        cam_usr_name = "";
        serial_number = "";
        cam_type = CamType::CAM_TYPE_NONE;
        cam_model = "";
        cam_index = 0;

        enable_depth_stream = false;
        enable_color_stream = true;
        enable_ir_stream = false;
        enable_cloud_stream = false;
        
        color_para = ColorStreamPara();
        depth_para = DepthStreamPara();
    }
    int cam_id;                //
    std::string cam_usr_name;  // 自定义相机名
    std::string serial_number; // 相机序列号
    CamType cam_type;          // 相机型号
    std::string cam_model;     // 相机型号 例：D435
    int cam_index;             // 无序列号时使用相机初始化对应扫描到的相机序号

    bool enable_depth_stream;
    bool enable_color_stream;
    bool enable_ir_stream;
    bool enable_cloud_stream;
    ColorStreamPara color_para;
    DepthStreamPara depth_para;

} CamConfigInfo;
typedef std::vector<CamConfigInfo> CamConfigInfo1D;

#endif