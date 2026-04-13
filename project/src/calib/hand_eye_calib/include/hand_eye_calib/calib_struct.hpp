#ifndef HAND_EYE_CALIB__CALIB_STRUCT_HPP_
#define HAND_EYE_CALIB__CALIB_STRUCT_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>

// 命名空间定义
namespace handeyecalib 
{

/**
 * @brief 将字符串转换为OpenCV Mat矩阵
 * @param str 输入的字符串
 * @return 转换后的cv::Mat矩阵
 */
cv::Mat stringToMat(const std::string& str);

/**
 * @brief 将字符串转换为Eigen Matrix4d矩阵
 * @param str 输入的字符串
 * @return 转换后的Eigen::Matrix4d矩阵
 */
Eigen::Matrix4d stringToEigenMatrix4d(const std::string& str);

/**
 * @brief 将OpenCV Mat矩阵转换为字符串
 * @param mat 输入的cv::Mat矩阵
 * @return 转换后的字符串
 */
std::string matToString(const cv::Mat& mat);

/**
 * @brief 将Eigen Matrix4d矩阵转换为字符串
 * @param eigen_mat 输入的Eigen::Matrix4d矩阵
 * @return 转换后的字符串
 */
std::string eigenMatrixToString(const Eigen::Matrix4d& eigen_mat);

/**
 * @brief 格式化输出 4x4 变换矩阵到流中
 * @param ss 输出流
 * @param title 矩阵标题
 * @param transform 4x4 变换矩阵
 * @param precision 小数点位数(默认为6)
 */
void formatMatrix4x4(std::stringstream& ss, const std::string& title, const cv::Mat& transform, int precision = 6);

/**
 * @brief 格式化输出 4x4 Eigen变换矩阵到流中
 * @param ss 输出流
 * @param title 矩阵标题
 * @param transform 4x4 Eigen变换矩阵
 * @param precision 小数点位数(默认为6)
 */
void formatEigenMatrix4x4(std::stringstream& ss, const std::string& title, const Eigen::Matrix4d& transform, int precision = 6);

bool getCalibMatrixInfo(const cv::Mat& cam_to_base_transform, const cv::Mat& base_to_cam_transform, std::string& res);
void printLog_CalibMatrix(const std::string& project_path, int log_level, const cv::Mat& cam_to_base_transform, const cv::Mat& base_to_cam_transform, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line);

bool getEigenCalibMatrixInfo(const Eigen::Matrix4d& eigen_cam_to_base, const Eigen::Matrix4d& eigen_base_to_cam, std::string& res);
void printLog_EigenCalibMatrix(const std::string& project_path, int log_level, const Eigen::Matrix4d& eigen_cam_to_base, const Eigen::Matrix4d& eigen_base_to_cam, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line);

bool getTcpOffset(const std::vector<double>& offset_compensation, std::string& res);
void printLog_TcpOffset(const std::string& project_path, int log_level, const std::vector<double>& offset_compensation, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line);
    
bool getHeadMotorAnglesInfo(const std::vector<double>& head_motor_angles, std::string& res);
void printLog_HeadMotorAngles(const std::string& project_path, int log_level, const std::vector<double>& head_motor_angles, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line);

/**
 * @struct TransformResult
 * @brief 坐标变换结果结构
 */
struct TransformResult {
    std::vector<double> transformed_pose;  ///< 变换后的位姿 [x, y, z, rx, ry, rz]
    cv::Mat transformation_matrix;         ///< 变换矩阵 (4x4)
};


// 手眼标定结果数据类
class CalibRes
{
public:
    // 标定类型枚举
    enum class CalibType : uint8_t {
        EYE_TO_HAND = 0,  ///< 眼在手外（相机固定）
        EYE_IN_HAND,      ///< 眼在手上（相机安装在机械臂末端）
        UNKNOWN           ///< 未知类型
    };

    // 标定结果参数名称枚举
    enum class ParamName : uint8_t {
        CALIB_TYPE = 0,            ///< 标定类型
        CAM_TO_BASE_TRANSFORM,     ///< 相机到基座变换矩阵
        BASE_TO_CAM_TRANSFORM,     ///< 基座到相机变换矩阵
        EIGEN_CAM_TO_BASE,         ///< 相机到基座变换矩阵(Eigen格式)
        EIGEN_BASE_TO_CAM,         ///< 基座到相机变换矩阵(Eigen格式)
        OFFSET_COMPENSATION,       ///< 偏移补偿值
        HEAD_MOTOR_ANGLES,         ///< 头部电机角度
        PARA_MAX
    };

    // 获取标定类型字符串
    static const char* getCalibTypeString(CalibType type);
    static CalibType getCalibTypeEnum(const std::string& typeStr);

    static const char* getParamNameString(ParamName paramName);  // 获取参数名称字符串
    static ParamName getParamNameEnum(const std::string& paramNameStr); // 根据参数名称字符串获取ParamName枚举值

    CalibType calib_type;             ///< 标定类型（眼在手上/眼在手外）
    cv::Mat cam_to_base_transform;    ///< 相机到基座的变换矩阵 (4x4)
    cv::Mat base_to_cam_transform;    ///< 基座到相机的变换矩阵 (4x4)
    Eigen::Matrix4d eigen_cam_to_base;  ///< 相机到基座的变换矩阵 (Eigen格式)
    Eigen::Matrix4d eigen_base_to_cam;  ///< 基座到相机的变换矩阵 (Eigen格式)
    std::vector<double> offset_compensation; ///< 偏移补偿值 [x, y, z, rx, ry, rz]，用于夹爪末端位姿转换
    std::vector<double> head_motor_angles; ///< 机器人头部电机角度 [pitch, yaw]，单位为度
    CalibRes();// 构造函数
    CalibRes(const CalibRes& para);
    CalibRes& operator = (const CalibRes& para);
    virtual ~CalibRes();

    // 初始化函数
    void Init();
    void Rst();

    // 拷贝函数
    void CopyFrom(const CalibRes& para);  // 从para拷贝数据
    void CopyTo(CalibRes& para) const;    // 拷贝数据到para

    // 生成标定矩阵信息的字符串表示
    bool getCalibMatrixInfo(std::string& res) const;

    // 生成Eigen标定矩阵信息的字符串表示
    bool getEigenCalibMatrixInfo(std::string& res) const;

    // 生成偏移补偿信息的字符串表示
    bool getTcpOffset(std::string& res) const;

    // 生成头部电机角度信息的字符串表示
    bool getHeadMotorAnglesInfo(std::string& res) const;

private:
    // 私有辅助函数
    void cleanup();  // 清理资源
};

/**
 * @class QualityMetrics
 * @brief 标定质量评估指标类
 */
/**
 * @class QualityMetrics
 * @brief 标定质量评估指标类，用于存储和管理手眼标定的质量评估数据
 */
class QualityMetrics
{
public:
    /**
     * @brief 质量评估指标参数名称枚举
     * 定义了QualityMetrics类中各个成员变量的参数名称
     */
    enum class ParamName : uint8_t {
        REPROJECTION_ERROR = 0,  ///< 重投影误差
        TRANSLATION_ERROR,       ///< 平移误差
        ROTATION_ERROR,          ///< 旋转误差
        CONDITION_NUMBER,        ///< 条件数（矩阵条件数）
        DATA_POINT_COUNT,        ///< 数据点数量
        PARA_MAX         
    };
    
    /**
     * @brief 获取参数名称字符串
     * @param paramName 参数名称枚举值
     * @return 对应参数的字符串表示
     */
    static const char* getParamNameString(ParamName paramName);
    static ParamName getParamNameEnum(const std::string& paramNameStr); // 根据参数名称字符串获取ParamName枚举值
    
    double reprojection_error;     ///< 重投影误差
    double translation_error;      ///< 平移误差
    double rotation_error;         ///< 旋转误差
    double condition_number;       ///< 条件数（矩阵条件数）
    uint16_t data_point_count;     ///< 数据点数量
    
    /**
     * @brief 默认构造函数
     */
    QualityMetrics();
    
    /**
     * @brief 拷贝构造函数
     * @param para 要拷贝的QualityMetrics对象
     */
    QualityMetrics(const QualityMetrics& para);
    
    /**
     * @brief 赋值运算符重载
     * @param para 要拷贝的QualityMetrics对象
     * @return 当前对象的引用
     */
    QualityMetrics& operator=(const QualityMetrics& para);
    
    /**
     * @brief 析构函数
     */
    virtual ~QualityMetrics();
    
    /**
     * @brief 初始化函数，将所有成员变量设置为默认值
     */
    void Init();
    
    /**
     * @brief 重置函数，将所有成员变量重置为初始状态
     */
    void Rst();
    
    /**
     * @brief 从另一个QualityMetrics对象拷贝数据
     * @param para 源QualityMetrics对象
     */
    void CopyFrom(const QualityMetrics& para);
    
    /**
     * @brief 拷贝数据到另一个QualityMetrics对象
     * @param para 目标QualityMetrics对象
     */
    void CopyTo(QualityMetrics& para) const;
    
private:
    /**
     * @brief 清理资源的私有辅助函数
     */
    void cleanup();
};

/**
 * @class CalibInfo
 * @brief 标定信息类，包含标定结果、质量评估指标、时间戳和标定方法
 */
class CalibInfo
{
public:
    /**
     * @brief 标定信息参数名称枚举
     * 定义了CalibInfo类中各个成员变量的参数名称
     */
    enum class ParamName : uint8_t {
        CALIB_METHOD = 0,   ///< 标定方法
        //CALIB_RES,          ///< 标定结果
        //QUALITY_METRICS,    ///< 质量评估指标
        TIMESTAMP,          ///< 时间戳
        PARA_MAX
    };
    
    /**
     * @brief 获取参数名称字符串
     * @param paramName 参数名称枚举值
     * @return 对应参数的字符串表示
     */
    static const char* getParamNameString(ParamName paramName);
    static ParamName getParamNameEnum(const std::string& paramNameStr); // 根据参数名称字符串获取ParamName枚举值
    
    std::string calib_method;         ///< 标定方法
    CalibRes calib_res;               ///< 标定结果
    QualityMetrics quality_metrics;   ///< 质量评估指标
    std::string timestamp;            ///< 标定时间戳
    
    /**
     * @brief 默认构造函数
     */
    CalibInfo();
    
    /**
     * @brief 带参数的构造函数
     * @param res 标定结果
     * @param metrics 质量评估指标
     * @param time 时间戳
     * @param method 标定方法
     */
    CalibInfo(const CalibRes& res, const QualityMetrics& metrics, 
              const std::string& time, const std::string& method);
    
    /**
     * @brief 拷贝构造函数
     * @param para 要拷贝的CalibInfo对象
     */
    CalibInfo(const CalibInfo& para);
    
    /**
     * @brief 赋值运算符重载
     * @param para 要拷贝的CalibInfo对象
     * @return 当前对象的引用
     */
    CalibInfo& operator=(const CalibInfo& para);
    
    /**
     * @brief 析构函数
     */
    virtual ~CalibInfo();
    
    /**
     * @brief 初始化函数，将所有成员变量设置为默认值
     */
    void Init();
    
    /**
     * @brief 重置函数，将所有成员变量重置为初始状态
     */
    void Rst();
    
    /**
     * @brief 从另一个CalibInfo对象拷贝数据
     * @param para 源CalibInfo对象
     */
    void CopyFrom(const CalibInfo& para);
    
    /**
     * @brief 拷贝数据到另一个CalibInfo对象
     * @param para 目标CalibInfo对象
     */
    void CopyTo(CalibInfo& para) const;
    
private:
    /**
     * @brief 清理资源的私有辅助函数
     */
    void cleanup();
};

/**
 * @class ArmCalibInfo
 * @brief 机械臂标定信息类，包含任意多个分辨率模式下的标定结果
 */
class ArmCalibInfo
{
public:
    /**
     * @brief 机械臂标定信息参数名称枚举
     * 定义了ArmCalibInfo类中各个成员变量的参数名称
     */
    enum class ParamName : uint8_t {
        ARM_ID = 0,     ///< 机械臂ID
        PARA_MAX
    };
    
    /**
     * @brief 获取参数名称字符串
     * @param paramName 参数名称枚举值
     * @return 对应参数的字符串表示
     */
    static const char* getParamNameString(ParamName paramName);
    static ParamName getParamNameEnum(const std::string& paramNameStr); // 根据参数名称字符串获取ParamName枚举值
    
    uint8_t arm_id;         ///< 机械臂ID
    CalibInfo calib_info;   ///< 当前机械臂的标定信息
    
    /**
     * @brief 默认构造函数
     */
    ArmCalibInfo();
    
    /**
     * @brief 拷贝构造函数
     * @param para 要拷贝的ArmCalibInfo对象
     */
    ArmCalibInfo(const ArmCalibInfo& para);
    
    /**
     * @brief 赋值运算符重载
     * @param para 要拷贝的ArmCalibInfo对象
     * @return 当前对象的引用
     */
    ArmCalibInfo& operator=(const ArmCalibInfo& para);
    
    /**
     * @brief 析构函数
     */
    virtual ~ArmCalibInfo();

    /**
     * @brief 初始化函数，将所有成员变量设置为默认值
     */
    void Init();
    
    /**
     * @brief 重置函数，将所有成员变量重置为初始状态
     */
    void Rst();

    /**
     * @brief 从另一个ArmCalibInfo对象拷贝数据
     * @param para 源ArmCalibInfo对象
     */
    void CopyFrom(const ArmCalibInfo& para);
    
    /**
     * @brief 拷贝数据到另一个ArmCalibInfo对象
     * @param para 目标ArmCalibInfo对象
     */
    void CopyTo(ArmCalibInfo& para) const;

    /**
     * @brief 设置标定结果
     * @param info 标定信息
     */
    void setCalibInfo(const CalibInfo& info);

    /**
     * @brief 获取指定分辨率的标定结果
     * @return 标定信息
     */
    CalibInfo getCalibInfo() const;

private:
    /**
     * @brief 清理资源的私有辅助函数
     */
    void cleanup();
};

typedef std::map<uint8_t, ArmCalibInfo> ArmCalibInfoList; //某个相机下的所有机械臂的标定信息<机械臂ID, 标定结果信息>
/**
 * @class CamCalibInfo
 * @brief 相机标定信息类，包含任意多个机械臂标定信息
 */
class CamCalibInfo
{
public:
    /**
     * @brief 相机标定信息参数名称枚举
     * 定义了CamCalibInfo类中各个成员变量的参数名称
     */
    enum class ParamName : uint8_t {
        CAM_ID = 0,   ///< 相机ID
        ARM_IDS,      ///< 机械臂ID列表
        PARA_MAX
    };
    
    /**
     * @brief 获取参数名称字符串
     * @param paramName 参数名称枚举值
     * @return 对应参数的字符串表示
     */
    static const char* getParamNameString(ParamName paramName);
    static ParamName getParamNameEnum(const std::string& paramNameStr); // 根据参数名称字符串获取ParamName枚举值
    
    uint8_t cam_id;                ///< 相机ID
    ArmCalibInfoList arm_calib1D;  ///< 机械臂ID到标定信息的映射 <机械臂ID, 标定结果信息>

    /**
     * @brief 默认构造函数
     */
    CamCalibInfo();
    
    /**
     * @brief 拷贝构造函数
     * @param para 要拷贝的CamCalibInfo对象
     */
    CamCalibInfo(const CamCalibInfo& para);
    
    /**
     * @brief 赋值运算符重载
     * @param para 要拷贝的CamCalibInfo对象
     * @return 当前对象的引用
     */
    CamCalibInfo& operator=(const CamCalibInfo& para);
    
    /**
     * @brief 析构函数
     */
    virtual ~CamCalibInfo();

    /**
     * @brief 初始化函数，将所有成员变量设置为默认值
     */
    void Init();
    
    /**
     * @brief 重置函数，将所有成员变量重置为初始状态
     */
    void Rst();

    /**
     * @brief 从另一个CamCalibInfo对象拷贝数据
     * @param para 源CamCalibInfo对象
     */
    void CopyFrom(const CamCalibInfo& para);
    
    /**
     * @brief 拷贝数据到另一个CamCalibInfo对象
     * @param para 目标CamCalibInfo对象
     */
    void CopyTo(CamCalibInfo& para) const;

    /**
     * @brief 设置机械臂所有标定信息
     * @param arm_id 机械臂ID
     * @param info 标定信息
     */
    void setArmCalibInfo(uint8_t arm_id, const ArmCalibInfo& info);

    /**
     * @brief 检查是否存在指定机械臂的标定信息
     * @param arm_id 机械臂ID
     * @return 是否存在标定信息
     */
    bool hasArmCalibInfo(uint8_t arm_id) const;

    /**
     * @brief 获取指定机械臂的所有标定信息
     * @param arm_id 机械臂ID
     * @return 机械臂标定信息
     */
    ArmCalibInfo getArmCalibInfo(uint8_t arm_id) const;

    /**
     * @brief 设置指定机械臂的标定信息
     * @param arm_id 机械臂ID
     * @param info 标定信息
     */
    void setCalibInfo(uint8_t arm_id, const CalibInfo& info);

    /**
     * @brief 设置指定机械臂和彩色分辨率的标定结果
     * @param arm_id 机械臂ID
     * @param res 标定结果
     */
    void setCalibRes(uint8_t arm_id, const CalibRes& res);

    /**
     * @brief 检查是否存在指定机械臂和彩色分辨率的标定结果
     * @param arm_id 机械臂ID
     * @return 是否存在标定结果
     */
    bool hasCalibRes(uint8_t arm_id) const;

    /**
     * @brief 获取指定机械臂和彩色分辨率的标定结果
     * @param arm_id 机械臂ID
     * @return 标定结果
     */
    CalibRes getCalibRes(uint8_t arm_id) const;

private:
    /**
     * @brief 清理资源的私有辅助函数
     */
    void cleanup();
};

typedef std::vector<CamCalibInfo> CamCalibInfoList;

std::vector<uint8_t> getArmIds(const ArmCalibInfoList& arm_calib_list);

std::vector<uint8_t> getCamIds(const CamCalibInfoList& cam_calib_list);
} // namespace handeyecalib

#endif // HAND_EYE_CALIB__CALIB_STRUCT_HPP_