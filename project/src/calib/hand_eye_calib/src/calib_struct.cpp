#include "hand_eye_calib/calib_struct.hpp"
#include "log_system/log_macros.hpp"
#include "internal_constants.h"
#define varName(x) #x // 定义获取变量名字符串的宏

namespace handeyecalib {

/**
 * @brief 将字符串转换为OpenCV Mat矩阵
 * @param str 输入的字符串
 * @return 转换后的cv::Mat矩阵
 */
cv::Mat stringToMat(const std::string& str) 
{
    if (str.empty()) {
        return cv::Mat();
    }
    
    // 处理可能存在的方括号
    std::string clean_str = str;
    if (clean_str.length() >= 2 && clean_str.front() == '[' && clean_str.back() == ']') {
        clean_str = clean_str.substr(1, clean_str.length() - 2);
    }
    
    // 解析字符串格式，假设格式为 "row0_col0,row0_col1,row0_col2,row0_col3;row1_col0,row1_col1,..."
    cv::Mat res;
    std::vector<std::string> rows;
    std::string current_row;
    // 分割行
    std::istringstream row_stream(clean_str);
    while (std::getline(row_stream, current_row, ';')) {
        rows.push_back(current_row);
    }
    if (rows.empty()) {
        return cv::Mat();
    } 
    // 解析每行的元素
    std::vector<std::vector<double>> matrix_data;
    for (const auto& row : rows) 
    {
        std::vector<std::string> elements;
        std::string current_element;
        std::istringstream element_stream(row);
        // 分割列
        while (std::getline(element_stream, current_element, ',')) {
            elements.push_back(current_element);
        }
        std::vector<double> row_data;
        for (const auto& element : elements) 
        {
            try {
                row_data.push_back(std::stod(element));
            } catch (const std::exception& e) {
                // 如果转换失败，返回空矩阵
                return cv::Mat();
            }
        }
        matrix_data.push_back(row_data);
    }  
    if (matrix_data.empty() || matrix_data[0].empty()) {
        return cv::Mat();
    }
    // 创建矩阵
    int rows_num = matrix_data.size();
    int cols_num = matrix_data[0].size();
    res = cv::Mat::zeros(rows_num, cols_num, CV_64F); 
    for (int i = 0; i < rows_num; i++) 
    {
        for (int j = 0; j < cols_num; j++) 
        {
            if (i < matrix_data.size() && j < matrix_data[i].size()) {
                res.at<double>(i, j) = matrix_data[i][j];
            }
        }
    }
    return res;
}

/**
 * @brief 将字符串转换为Eigen Matrix4d矩阵
 * @param str 输入的字符串
 * @return 转换后的Eigen::Matrix4d矩阵
 */
Eigen::Matrix4d stringToEigenMatrix4d(const std::string& str) 
{
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity(); 
    if (str.empty()) {
        return res;
    }
    // 解析字符串格式，假设格式为 "row0_col0,row0_col1,row0_col2,row0_col3;row1_col0,row1_col1,..."
    std::vector<std::string> rows;
    std::string current_row;
    // 分割行
    std::istringstream row_stream(str);
    while (std::getline(row_stream, current_row, ';')) {
        rows.push_back(current_row);
    }
    if (rows.empty()) {
        return res;
    }
    // 解析每行的元素
    for (int i = 0; i < rows.size() && i < 4; i++) 
    {
        std::vector<std::string> elements;
        std::string current_element;
        std::istringstream element_stream(rows[i]);
        // 分割列
        while (std::getline(element_stream, current_element, ',')) {
            elements.push_back(current_element);
        }
        for (int j = 0; j < elements.size() && j < 4; j++) 
        {
            try {
                res(i, j) = std::stod(elements[j]);
            } catch (const std::exception& e) {
                // 如果转换失败，返回单位矩阵
                return Eigen::Matrix4d::Identity();
            }
        }
    }
    return res;
}

/**
 * @brief 将OpenCV Mat矩阵转换为字符串
 * @param mat 输入的cv::Mat矩阵
 * @return 转换后的字符串
 */
 std::string matToString(const cv::Mat& mat) 
{
    if (mat.empty()) return "[]";
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < mat.rows; ++i) 
    {
        for (int j = 0; j < mat.cols; ++j) 
        {
            oss << mat.at<double>(i, j);
            if (j < mat.cols - 1) oss << ",";
        }
        if (i < mat.rows - 1) oss << ";";
    }
    oss << "]";
    return oss.str();
}

/**
 * @brief 将Eigen Matrix4d矩阵转换为字符串
 * @param eigen_mat 输入的Eigen::Matrix4d矩阵
 * @return 转换后的字符串
 */
std::string eigenMatrixToString(const Eigen::Matrix4d& matrix) 
{
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < 4; ++i) 
    {
        for (int j = 0; j < 4; ++j) 
        {
            oss << matrix(i, j);
            if (j < 3) oss << ",";
        }
        if (i < 3) oss << ";";
    }
    oss << "]";
    return oss.str();
}

/**
 * @brief 格式化输出 4x4 变换矩阵到流中
 * @param ss 输出流
 * @param title 矩阵标题
 * @param transform 4x4 变换矩阵
 * @param precision 小数点位数(默认为6)
 */
void formatMatrix4x4(std::stringstream& ss, const std::string& title, const cv::Mat& transform, int precision/* = 6*/)
{
    ss << "    " << title << ":\n";
    for (int i = 0; i < 4; i++)
    {
        ss << std::fixed << std::setprecision(precision) << std::right;
        ss << "    [";
        for (int j = 0; j < 4; j++)
        {
            double value = transform.at<double>(i, j);
            if (j > 0) ss << " ";  // 列之间一个空格

            if (value >= 0.0)
            {
                ss << " " << std::setw(12) << value;
            }
            else
            {
                ss << std::setw(13) << value;
            }
        }
        ss << " ]\n";
    }
}

/**
 * @brief 格式化输出 4x4 Eigen变换矩阵到流中
 * @param ss 输出流
 * @param title 矩阵标题
 * @param transform 4x4 Eigen变换矩阵
 * @param precision 小数点位数(默认为6)
 */
void formatEigenMatrix4x4(std::stringstream& ss, const std::string& title, const Eigen::Matrix4d& transform, int precision/* = 6*/)
{
    ss << "    " << title << ":\n";
    for (int i = 0; i < 4; i++)
    {
        ss << std::fixed << std::setprecision(precision) << std::right;
        ss << "    [";
        for (int j = 0; j < 4; j++)
        {
            double value = transform(i, j);
            if (j > 0) ss << " ";  // 列之间一个空格

            if (value >= 0.0)
            {
                ss << " " << std::setw(12) << value;
            }
            else
            {
                ss << std::setw(13) << value;
            }
        }
        ss << " ]\n";
    }
}

bool getCalibMatrixInfo(const cv::Mat& cam_to_base_transform, const cv::Mat& base_to_cam_transform, std::string& res)
{
    std::stringstream ss;// 打印标定矩阵信息
    try 
    {
        if (cam_to_base_transform.rows != 4 || cam_to_base_transform.cols != 4 ||
            base_to_cam_transform.rows != 4 || base_to_cam_transform.cols != 4) // 验证矩阵维度
        {
            ss << "错误: 变换矩阵维度不正确，期望4x4矩阵";
            // 输出实际矩阵维度信息
            ss << ", 当前实际维度为: cam_to_base_transform(" << cam_to_base_transform.rows << "x" << cam_to_base_transform.cols 
               << "), base_to_cam_transform(" << base_to_cam_transform.rows << "x" << base_to_cam_transform.cols << ")";
            res = ss.str();
            return false; // 返回失败状态
        }
        formatMatrix4x4(ss, "相机到基座变换矩阵[" CAM_TO_BASE_NAME "]", cam_to_base_transform);
        ss << "\n";
        formatMatrix4x4(ss, "基座到相机变换矩阵[" BASE_TO_CAM_NAME "]", base_to_cam_transform);
        res = ss.str();
        return true; // 返回成功状态
    } catch (const std::exception& e) {
        ss.str(""); // 清空流
        ss << "错误: 获取标定矩阵信息时发生异常 - " << e.what();
    } catch (...) {
        ss.str(""); // 清空流
        ss << "错误: 获取标定矩阵信息时发生未知异常";
    }
    res = ss.str();
    return false; // 发生异常，返回失败状态
}

void printLog_CalibMatrix(const std::string& project_path, int log_level, const cv::Mat& cam_to_base_transform, const cv::Mat& base_to_cam_transform, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line) 
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    std::string strLogMsg;
    bool bRet = getCalibMatrixInfo(cam_to_base_transform, base_to_cam_transform, strLogMsg); // 生成标定矩阵信息的字符串表示
    if (bRet) 
    {
        LOG_INFO(project_path, false, logsys::Color::BLUE, (prefix_msg + strLogMsg).c_str());
    }
    else {
        LOG_ERROR(project_path, false, logsys::Color::BLUE, (prefix_msg + strLogMsg).c_str());
    }
}

bool getEigenCalibMatrixInfo(const Eigen::Matrix4d& eigen_cam_to_base, const Eigen::Matrix4d& eigen_base_to_cam, std::string& res) 
{
    std::stringstream ss;// 打印Eigen标定矩阵信息
    try 
    {
        formatEigenMatrix4x4(ss, "Eigen格式-相机到基座变换矩阵[" CAM_TO_BASE_NAME "]", eigen_cam_to_base);
        ss << "\n";
        formatEigenMatrix4x4(ss, "Eigen格式-基座到相机变换矩阵[" BASE_TO_CAM_NAME "]", eigen_base_to_cam);
        res = ss.str();
        return true; // 返回成功状态
    } catch (const std::exception& e) {
        ss.str(""); // 清空流
        ss << "错误: 获取Eigen标定矩阵信息时发生异常 - " << e.what();
    } catch (...) {
        ss.str(""); // 清空流
        ss << "错误: 获取Eigen标定矩阵信息时发生未知异常";
    }
    res = ss.str();
    return false; // 发生异常，返回失败状态
}

void printLog_EigenCalibMatrix(const std::string& project_path, int log_level, const Eigen::Matrix4d& eigen_cam_to_base, const Eigen::Matrix4d& eigen_base_to_cam, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    std::string strLogMsg;
    bool bRet = getEigenCalibMatrixInfo(eigen_cam_to_base, eigen_base_to_cam, strLogMsg); // 生成标定矩阵信息的字符串表示
    if (bRet) 
    {
        LOG_INFO(project_path, (prefix_msg + strLogMsg).c_str(), false);
    }
    else {
        LOG_ERROR(project_path, (prefix_msg + strLogMsg).c_str(), false);
    }
}
    
bool getTcpOffset(const std::vector<double>& offset_compensation, std::string& res)
{
    bool bRet = false;
    std::stringstream ss;// 打印偏移补偿信息
    try 
    {
        size_t size = offset_compensation.size();
        if (size == 0) // 如果没有元素，则输出提示信息
        {
            ss << "    错误: 偏移补偿值[" << CalibRes::getParamNameString(CalibRes::ParamName::OFFSET_COMPENSATION) << "]数组为空: []\n";
            res = ss.str();
            return false; // 返回失败状态
        }
        int intervalSize = 6;
        if (size >= 6) // 根据元素数量决定输出格式
        {
            ss << "    偏移补偿值[" << CalibRes::getParamNameString(CalibRes::ParamName::OFFSET_COMPENSATION) << "]:\n";
            // 按照现有格式输出，分为两行显示6个元素
            ss << "    [ offset_x: " << std::fixed << std::setprecision(3) << std::right << std::setw(intervalSize) << offset_compensation[0] << ",  offset_y: " << std::setw(intervalSize) << offset_compensation[1] << ",  offset_z: " << std::setw(intervalSize) << offset_compensation[2] << "]\n";
            ss << "    [offset_rx: " << std::fixed << std::setprecision(3) << std::right << std::setw(intervalSize) << offset_compensation[3] << ", offset_ry: " << std::setw(intervalSize) << offset_compensation[4] << ", offset_rz: " << std::setw(intervalSize) << offset_compensation[5] << "]\n";
            
            // 如果还有更多元素，继续输出
            for (size_t i = 6; i < size; i += 3) 
            {
                ss << "    [";
                for (size_t j = 0; j < 3 && (i+j) < size; j++) 
                {
                    if (j > 0) ss << ", ";
                    ss << " elem_" << (i+j) << ": " << std::setw(intervalSize) << offset_compensation[i+j];
                }
                ss << "]\n";
            }
            bRet = true;
        } 
        else 
        {
            ss << "    错误: 偏移补偿值[" << CalibRes::getParamNameString(CalibRes::ParamName::OFFSET_COMPENSATION) << "]数组大小不足6个元素:\n";
            // 如果少于6个元素，按每行最多3个元素输出
            for (size_t i = 0; i < size; i += 3) 
            {
                ss << "    [";
                for (size_t j = 0; j < 3 && (i+j) < size; j++) 
                {
                    if (j > 0) ss << ", ";
                    size_t idx = i + j;
                    switch(idx) 
                    {
                        case 0: ss << " offset_x: " << std::fixed << std::setprecision(3) << std::right << std::setw(intervalSize) << offset_compensation[idx]; break;
                        case 1: ss << " offset_y: " << std::setw(intervalSize) << offset_compensation[idx]; break;
                        case 2: ss << " offset_z: " << std::setw(intervalSize) << offset_compensation[idx]; break;
                        case 3: ss << " offset_rx: " << std::fixed << std::setprecision(3) << std::right << std::setw(intervalSize) << offset_compensation[idx]; break;
                        case 4: ss << " offset_ry: " << std::setw(intervalSize) << offset_compensation[idx]; break;
                        case 5: ss << " offset_rz: " << std::setw(intervalSize) << offset_compensation[idx]; break;
                        default: ss << " elem_" << idx << ": " << std::setw(intervalSize) << offset_compensation[idx]; break;
                    }
                }
                ss << "]\n";
            }
            bRet = false; // 返回失败状态
        } 
    } catch (const std::exception& e) {
        bRet = false;// 发生异常，返回失败状态
        ss.str(""); // 清空流
        ss << "错误: 获取偏移补偿信息时发生异常 - " << e.what();
    } catch (...) {
        bRet = false;// 发生异常，返回失败状态
        ss.str(""); // 清空流
        ss << "错误: 获取偏移补偿信息时发生未知异常";
    }
    res = ss.str();
    return bRet; 
}

void printLog_TcpOffset(const std::string& project_path, int log_level, const std::vector<double>& offset_compensation, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    std::string strLogMsg;
    bool bRet = getTcpOffset(offset_compensation, strLogMsg); // 生成偏移补偿信息的字符串表示
    if (bRet) 
    {
        LOG_INFO(project_path, false, logsys::Color::BLUE, (prefix_msg + strLogMsg).c_str());
    }
    else 
    {
        LOG_ERROR(project_path, false, logsys::Color::BLUE, (prefix_msg + strLogMsg).c_str());
    }
}

bool getHeadMotorAnglesInfo(const std::vector<double>& head_motor_angles, std::string& res)
{
    bool bRet = false;
    std::stringstream ss;// 打印头部电机角度信息
    try 
    {
        size_t size = head_motor_angles.size();
        if (size == 0) // 如果没有元素，则输出提示信息
        {
            ss << "    错误: 头部电机角度[" << CalibRes::getParamNameString(CalibRes::ParamName::HEAD_MOTOR_ANGLES) << "]数组为空: []\n";
            res = ss.str();
            return false; // 返回失败状态
        }
        int intervalSize = 6;
        // 根据元素数量决定输出格式
        if (size >= 2) 
        {
            ss << "    头部电机角度[" << CalibRes::getParamNameString(CalibRes::ParamName::HEAD_MOTOR_ANGLES) << "]:\n";
            // 按照现有格式输出，显示标准的两个元素
            ss << "    [pitch(俯仰角): " << std::fixed << std::setprecision(3) << std::right << std::setw(intervalSize) << head_motor_angles[0] << ", yaw(转向角): " << std::setw(intervalSize) << head_motor_angles[1] << "]\n";
            
            // 如果还有更多元素，继续输出
            for (size_t i = 2; i < size; i += 3) 
            {
                ss << "    [";
                for (size_t j = 0; j < 3 && (i+j) < size; j++) 
                {
                    if (j > 0) ss << ", ";
                    ss << " elem_" << (i+j) << ": " << std::setw(intervalSize) << head_motor_angles[i+j];
                }
                ss << "]\n";
            }
            bRet = true;
        } 
        else 
        {
            ss << "    错误: 头部电机角度[" << CalibRes::getParamNameString(CalibRes::ParamName::HEAD_MOTOR_ANGLES) << "]数组大小不足2个元素:\n";
            // 如果少于2个元素，按单个元素输出
            for (size_t i = 0; i < size; i += 3) 
            {
                ss << "    [";
                for (size_t j = 0; j < 3 && (i+j) < size; j++) 
                {
                    if (j > 0) ss << ", ";
                    size_t idx = i + j;
                    switch(idx) 
                    {
                        case 0: ss << " pitch(俯仰角): " << std::fixed << std::setprecision(3) << std::right << std::setw(intervalSize) << head_motor_angles[idx]; break;
                        case 1: ss << " yaw(转向角): " << std::setw(intervalSize) << head_motor_angles[idx]; break;
                        default: ss << " elem_" << idx << ": " << std::setw(intervalSize) << head_motor_angles[idx]; break;
                    }
                }
                ss << "]\n";
            }
            bRet = false; // 返回失败状态
        }
    } catch (const std::exception& e) {
        bRet = false;// 发生异常，返回失败状态
        ss.str(""); // 清空流
        ss << "错误: 获取头部电机角度信息时发生异常 - " << e.what();
    } catch (...) {
        bRet = false;// 发生异常，返回失败状态
        ss.str(""); // 清空流
        ss << "错误: 获取头部电机角度信息时发生未知异常";
    }
    res = ss.str();
    return bRet; 
}

void printLog_HeadMotorAngles(const std::string& project_path, int log_level, const std::vector<double>& head_motor_angles, 
    const std::string& prefix_msg, int color, const char* file_name_path, const char* func, int line)
{
    if (!LOG_ON(project_path, static_cast<logsys::Level>(log_level)))
        return;
    int max_log_level = (int)logsys::Level::INFO;
    if (log_level > max_log_level)
        log_level = max_log_level;
    std::string strLogMsg;
    bool bRet = getHeadMotorAnglesInfo(head_motor_angles, strLogMsg); // 生成头部电机角度信息的字符串表示
    if (bRet) 
    {
        LOG_INFO(project_path, false, logsys::Color::BLUE, (prefix_msg + strLogMsg).c_str());
    }
    else {
        LOG_ERROR(project_path, false, logsys::Color::BLUE, (prefix_msg + strLogMsg).c_str());
    }
}

// CalibRes 类实现
CalibRes::CalibRes()
{
    Init();
}

CalibRes::CalibRes(const CalibRes& para)
{
    CopyFrom(para);
}

CalibRes& CalibRes::operator=(const CalibRes& para)
{
    if (this != &para) {
        CopyFrom(para);
    }
    return *this;
}

CalibRes::~CalibRes()
{
    cleanup();
}

// 获取标定类型字符串
const char* CalibRes::getCalibTypeString(CalibType type)
{
    switch (type)
    {
        case CalibType::EYE_TO_HAND: return "eye_to_hand";
        case CalibType::EYE_IN_HAND: return "eye_in_hand";
        case CalibType::UNKNOWN: return "unknown";
        default: return "unknown";
    }
}

// 根据字符串获取标定类型枚举
CalibRes::CalibType CalibRes::getCalibTypeEnum(const std::string& typeStr)
{
    if (typeStr == "eye_to_hand") return CalibType::EYE_TO_HAND;
    if (typeStr == "eye_in_hand") return CalibType::EYE_IN_HAND;
    return CalibType::UNKNOWN;
}

// 获取参数名称字符串
const char* CalibRes::getParamNameString(ParamName paramName)
{
    switch (paramName)
    {
        case ParamName::CALIB_TYPE: return varName(calib_type);
        case ParamName::CAM_TO_BASE_TRANSFORM: return varName(cam_to_base_transform);
        case ParamName::BASE_TO_CAM_TRANSFORM: return varName(base_to_cam_transform);
        case ParamName::EIGEN_CAM_TO_BASE: return varName(eigen_cam_to_base);
        case ParamName::EIGEN_BASE_TO_CAM: return varName(eigen_base_to_cam);
        case ParamName::OFFSET_COMPENSATION: return varName(offset_compensation);
        case ParamName::HEAD_MOTOR_ANGLES: return varName(head_motor_angles);
        default: return "";
    }
}

// 根据参数名称字符串获取ParamName枚举值
CalibRes::ParamName CalibRes::getParamNameEnum(const std::string& paramNameStr)
{
    if (paramNameStr == varName(calib_type)) return ParamName::CALIB_TYPE;
    if (paramNameStr == varName(cam_to_base_transform)) return ParamName::CAM_TO_BASE_TRANSFORM;
    if (paramNameStr == varName(base_to_cam_transform)) return ParamName::BASE_TO_CAM_TRANSFORM;
    if (paramNameStr == varName(eigen_cam_to_base)) return ParamName::EIGEN_CAM_TO_BASE;
    if (paramNameStr == varName(eigen_base_to_cam)) return ParamName::EIGEN_BASE_TO_CAM;
    if (paramNameStr == varName(offset_compensation)) return ParamName::OFFSET_COMPENSATION;
    if (paramNameStr == varName(head_motor_angles)) return ParamName::HEAD_MOTOR_ANGLES;
    return ParamName::PARA_MAX; // 表示无效的参数名称
}

void CalibRes::Init()
{
    calib_type = CalibType::UNKNOWN;
    cam_to_base_transform = cv::Mat::eye(4, 4, CV_64F);
    base_to_cam_transform = cv::Mat::eye(4, 4, CV_64F);
    eigen_cam_to_base = Eigen::Matrix4d::Identity();
    eigen_base_to_cam = Eigen::Matrix4d::Identity();
    offset_compensation = std::vector<double>(6, 0.0);  // 默认偏移补偿值为0
    head_motor_angles = std::vector<double>(2, 0.0); ///< 机器人头部电机角度 [pitch, yaw]，单位为度
}

void CalibRes::Rst()
{
    Init();
}

void CalibRes::CopyFrom(const CalibRes& para)
{
    if (this != &para)
        para.CopyTo(*this);
}

void CalibRes::CopyTo(CalibRes& para) const
{
    if (this != &para)
    {
        para.calib_type = calib_type;
        cam_to_base_transform.copyTo(para.cam_to_base_transform);
        base_to_cam_transform.copyTo(para.base_to_cam_transform);
        para.eigen_cam_to_base = eigen_cam_to_base;
        para.eigen_base_to_cam = eigen_base_to_cam;
        para.offset_compensation = offset_compensation;  // 拷贝偏移补偿值
        para.head_motor_angles = head_motor_angles; ///< 机器人头部电机角度 [pitch, yaw]，单位为度
    }
}

void CalibRes::cleanup()
{
    // 清理资源的实现（如果需要的话）
}

bool CalibRes::getCalibMatrixInfo(std::string& res) const   
{
    return handeyecalib::getCalibMatrixInfo(cam_to_base_transform, base_to_cam_transform, res);
}

bool CalibRes::getEigenCalibMatrixInfo(std::string& res) const
{
    return handeyecalib::getEigenCalibMatrixInfo(eigen_cam_to_base, eigen_base_to_cam, res);
}

bool CalibRes::getTcpOffset(std::string& res) const
{
    return handeyecalib::getTcpOffset(offset_compensation, res);
}

bool CalibRes::getHeadMotorAnglesInfo(std::string& res) const
{
    return handeyecalib::getHeadMotorAnglesInfo(head_motor_angles, res);
}

// QualityMetrics 类实现

/**
 * @brief QualityMetrics类的默认构造函数
 * 调用Init()函数初始化所有成员变量
 */
QualityMetrics::QualityMetrics()
{
    Init();
}

/**
 * @brief QualityMetrics类的拷贝构造函数
 * @param para 要拷贝的QualityMetrics对象
 */
QualityMetrics::QualityMetrics(const QualityMetrics& para)
{
    CopyFrom(para);
}

/**
 * @brief QualityMetrics类的赋值运算符重载
 * @param para 要拷贝的QualityMetrics对象
 * @return 当前对象的引用
 */
QualityMetrics& QualityMetrics::operator=(const QualityMetrics& para)
{
    if (this != &para) {
        CopyFrom(para);
    }
    return *this;
}

/**
 * @brief QualityMetrics类的析构函数
 * 调用cleanup()函数清理资源
 */
QualityMetrics::~QualityMetrics()
{
    cleanup();
}

/**
 * @brief 获取参数名称字符串
 * 根据参数名称枚举值返回对应的字符串表示
 * @param paramName 参数名称枚举值
 * @return 对应参数的字符串表示
 */
const char* QualityMetrics::getParamNameString(ParamName paramName)
{
    switch (paramName)
    {
        case ParamName::REPROJECTION_ERROR: return varName(reprojection_error);
        case ParamName::TRANSLATION_ERROR: return varName(translation_error);
        case ParamName::ROTATION_ERROR: return varName(rotation_error);
        case ParamName::CONDITION_NUMBER: return varName(condition_number);
        case ParamName::DATA_POINT_COUNT: return varName(data_point_count);
        default: return "";
    }
}

/**
 * @brief 根据参数名称字符串获取ParamName枚举值
 * @param paramNameStr 参数名称字符串
 * @return 对应的参数名称枚举值
 */
QualityMetrics::ParamName QualityMetrics::getParamNameEnum(const std::string& paramNameStr)
{
    if (paramNameStr == varName(reprojection_error)) return ParamName::REPROJECTION_ERROR;
    if (paramNameStr == varName(translation_error)) return ParamName::TRANSLATION_ERROR;
    if (paramNameStr == varName(rotation_error)) return ParamName::ROTATION_ERROR;
    if (paramNameStr == varName(condition_number)) return ParamName::CONDITION_NUMBER;
    if (paramNameStr == varName(data_point_count)) return ParamName::DATA_POINT_COUNT;
    return ParamName::PARA_MAX; // 表示无效的参数名称
}

/**
 * @brief 初始化函数，将所有成员变量设置为默认值
 * 重投影误差、平移误差、旋转误差和条件数初始化为0.0
 * 数据点数量初始化为0
 */
void QualityMetrics::Init()
{
    reprojection_error = 0.0;
    translation_error = 0.0;
    rotation_error = 0.0;
    condition_number = 0.0;
    data_point_count = 0;
}

/**
 * @brief 重置函数，将所有成员变量重置为初始状态
 * 调用Init()函数实现重置功能
 */
void QualityMetrics::Rst()
{
    Init();
}

/**
 * @brief 从另一个QualityMetrics对象拷贝数据
 * @param para 源QualityMetrics对象
 */
void QualityMetrics::CopyFrom(const QualityMetrics& para)
{
    if (this != &para)
        para.CopyTo(*this);
}

/**
 * @brief 拷贝数据到另一个QualityMetrics对象
 * @param para 目标QualityMetrics对象
 */
void QualityMetrics::CopyTo(QualityMetrics& para) const
{
    if (this != &para)
    {
        para.reprojection_error = reprojection_error;
        para.translation_error = translation_error;
        para.rotation_error = rotation_error;
        para.condition_number = condition_number;
        para.data_point_count = data_point_count;
    }
}

/**
 * @brief 清理资源的私有辅助函数
 * 当前实现为空，为将来扩展预留
 */
void QualityMetrics::cleanup()
{
    // 清理资源的实现（如果需要的话）
}

// CalibInfo 类实现

/**
 * @brief CalibInfo类的默认构造函数
 * 调用Init()函数初始化所有成员变量
 */
CalibInfo::CalibInfo()
{
    Init();
}

/**
 * @brief CalibInfo类的带参构造函数
 * @param res 标定结果
 * @param metrics 质量评估指标
 * @param time 时间戳
 * @param method 标定方法
 */
CalibInfo::CalibInfo(const CalibRes& res, const QualityMetrics& metrics, 
          const std::string& time, const std::string& method)
    : calib_res(res), quality_metrics(metrics), timestamp(time), calib_method(method)
{
}

/**
 * @brief CalibInfo类的拷贝构造函数
 * @param para 要拷贝的CalibInfo对象
 */
CalibInfo::CalibInfo(const CalibInfo& para)
{
    CopyFrom(para);
}

/**
 * @brief CalibInfo类的赋值运算符重载
 * @param para 要拷贝的CalibInfo对象
 * @return 当前对象的引用
 */
CalibInfo& CalibInfo::operator=(const CalibInfo& para)
{
    if (this != &para) {
        CopyFrom(para);
    }
    return *this;
}

/**
 * @brief CalibInfo类的析构函数
 * 调用cleanup()函数清理资源
 */
CalibInfo::~CalibInfo()
{
    cleanup();
}

/**
 * @brief 获取参数名称字符串
 * 根据参数名称枚举值返回对应的字符串表示
 * @param paramName 参数名称枚举值
 * @return 对应参数的字符串表示
 */
const char* CalibInfo::getParamNameString(ParamName paramName)
{
    switch (paramName)
    {
        case ParamName::CALIB_METHOD: return varName(calib_method);
        #if 0
            case ParamName::CALIB_RES: return varName(calib_res);
            case ParamName::QUALITY_METRICS: return varName(quality_metrics);
        #endif
        case ParamName::TIMESTAMP: return varName(timestamp);
        default: return "";
    }
}

/**
 * @brief 根据参数名称字符串获取ParamName枚举值
 * @param paramNameStr 参数名称字符串
 * @return 对应的参数名称枚举值
 */
CalibInfo::ParamName CalibInfo::getParamNameEnum(const std::string& paramNameStr)
{
    if (paramNameStr == varName(calib_method)) return ParamName::CALIB_METHOD;
    #if 0
        if (paramNameStr == varName(calib_res)) return ParamName::CALIB_RES;
        if (paramNameStr == varName(quality_metrics)) return ParamName::QUALITY_METRICS;
    #endif
    if (paramNameStr == varName(timestamp)) return ParamName::TIMESTAMP;
    return ParamName::PARA_MAX; // 表示无效的参数名称
}

/**
 * @brief 初始化函数，将所有成员变量设置为默认值
 * 标定方法和时间戳初始化为空字符串
 */
void CalibInfo::Init()
{
    calib_method = "";
    timestamp = "";
}

/**
 * @brief 重置函数，将所有成员变量重置为初始状态
 * 调用Init()函数实现重置功能
 */
void CalibInfo::Rst()
{
    Init();
}

/**
 * @brief 从另一个CalibInfo对象拷贝数据
 * @param para 源CalibInfo对象
 */
void CalibInfo::CopyFrom(const CalibInfo& para)
{
    if (this != &para)
        para.CopyTo(*this);
}

/**
 * @brief 拷贝数据到另一个CalibInfo对象
 * @param para 目标CalibInfo对象
 */
void CalibInfo::CopyTo(CalibInfo& para) const
{
    if (this != &para)
    {
        para.calib_method = calib_method;
        para.timestamp = timestamp;
        calib_res.CopyTo(para.calib_res);
        quality_metrics.CopyTo(para.quality_metrics);
    }
}

/**
 * @brief 清理资源的私有辅助函数
 * 当前实现为空，为将来扩展预留
 */
void CalibInfo::cleanup()
{
    // 清理资源的实现（如果需要的话）
}

// ArmCalibInfo 类实现

/**
 * @brief ArmCalibInfo类的默认构造函数
 * 调用Init()函数初始化所有成员变量
 */
ArmCalibInfo::ArmCalibInfo()
{
    Init();
}

/**
 * @brief ArmCalibInfo类的拷贝构造函数
 * @param para 要拷贝的ArmCalibInfo对象
 */
ArmCalibInfo::ArmCalibInfo(const ArmCalibInfo& para)
{
    CopyFrom(para);
}

/**
 * @brief ArmCalibInfo类的赋值运算符重载
 * @param para 要拷贝的ArmCalibInfo对象
 * @return 当前对象的引用
 */
ArmCalibInfo& ArmCalibInfo::operator=(const ArmCalibInfo& para)
{
    if (this != &para) {
        CopyFrom(para);
    }
    return *this;
}

/**
 * @brief ArmCalibInfo类的析构函数
 * 调用cleanup()函数清理资源
 */
ArmCalibInfo::~ArmCalibInfo()
{
    cleanup();
}

/**
 * @brief 获取参数名称字符串
 * 根据参数名称枚举值返回对应的字符串表示
 * @param paramName 参数名称枚举值
 * @return 对应参数的字符串表示
 */
const char* ArmCalibInfo::getParamNameString(ParamName paramName)
{
    switch (paramName)
    {
        case ParamName::ARM_ID: return varName(arm_id);
        default: return "";
    }
}

/**
 * @brief 根据参数名称字符串获取ParamName枚举值
 * @param paramNameStr 参数名称字符串
 * @return 对应的参数名称枚举值
 */
ArmCalibInfo::ParamName ArmCalibInfo::getParamNameEnum(const std::string& paramNameStr)
{
    if (paramNameStr == varName(arm_id)) return ParamName::ARM_ID;
    return ParamName::PARA_MAX; // 表示无效的参数名称
}

/**
 * @brief 初始化函数，将所有成员变量设置为默认值
 * 机械臂ID初始化为0，标定信息调用其Init()函数初始化
 */
void ArmCalibInfo::Init()
{
    arm_id = 0;
    calib_info.Init();
}

/**
 * @brief 重置函数，将所有成员变量重置为初始状态
 * 调用Init()函数实现重置功能
 */
void ArmCalibInfo::Rst()
{
    Init();
}

/**
 * @brief 从另一个ArmCalibInfo对象拷贝数据
 * @param para 源ArmCalibInfo对象
 */
void ArmCalibInfo::CopyFrom(const ArmCalibInfo& para)
{
    if (this != &para)
        para.CopyTo(*this);
}

/**
 * @brief 拷贝数据到另一个ArmCalibInfo对象
 * @param para 目标ArmCalibInfo对象
 */
void ArmCalibInfo::CopyTo(ArmCalibInfo& para) const
{
    if (this != &para)
    {
        para.arm_id = arm_id;
        para.calib_info = calib_info;
    }
}

/**
 * @brief 设置标定结果
 * @param info 标定信息
 */
void ArmCalibInfo::setCalibInfo(const CalibInfo& info)
{
     calib_info = info;
}

/**
 * @brief 获取指定分辨率的标定结果
 * @return 标定信息
 */
CalibInfo ArmCalibInfo::getCalibInfo() const
{
    return calib_info;
}

/**
 * @brief 清理资源的私有辅助函数
 * 当前实现为空，为将来扩展预留
 */
void ArmCalibInfo::cleanup()
{
    // 清理资源的实现（如果需要的话）
}

// CamCalibInfo 类实现

/**
 * @brief CamCalibInfo类的默认构造函数
 * 调用Init()函数初始化所有成员变量
 */
CamCalibInfo::CamCalibInfo()
{
    Init();
}

/**
 * @brief CamCalibInfo类的拷贝构造函数
 * @param para 要拷贝的CamCalibInfo对象
 */
CamCalibInfo::CamCalibInfo(const CamCalibInfo& para)
{
    CopyFrom(para);
}

/**
 * @brief CamCalibInfo类的赋值运算符重载
 * @param para 要拷贝的CamCalibInfo对象
 * @return 当前对象的引用
 */
CamCalibInfo& CamCalibInfo::operator=(const CamCalibInfo& para)
{
    if (this != &para) {
        CopyFrom(para);
    }
    return *this;
}

/**
 * @brief CamCalibInfo类的析构函数
 * 调用cleanup()函数清理资源
 */
CamCalibInfo::~CamCalibInfo()
{
    cleanup();
}

/**
 * @brief 获取参数名称字符串
 * 根据参数名称枚举值返回对应的字符串表示
 * @param paramName 参数名称枚举值
 * @return 对应参数的字符串表示
 */
const char* CamCalibInfo::getParamNameString(ParamName paramName)
{
    switch (paramName)
    {
        case ParamName::CAM_ID: return varName(cam_id);
        case ParamName::ARM_IDS: return varName(arm_ids);
        default: return "";
    }
}

/**
 * @brief 根据参数名称字符串获取ParamName枚举值
 * @param paramNameStr 参数名称字符串
 * @return 对应的参数名称枚举值
 */
CamCalibInfo::ParamName CamCalibInfo::getParamNameEnum(const std::string& paramNameStr)
{
    if (paramNameStr == varName(cam_id)) return ParamName::CAM_ID;
    if (paramNameStr == varName(arm_ids)) return ParamName::ARM_IDS;
    return ParamName::PARA_MAX; // 表示无效的参数名称
}

/**
 * @brief 初始化函数，将所有成员变量设置为默认值
 * 相机ID初始化为0，机械臂标定信息列表清空
 */
void CamCalibInfo::Init()
{
    cam_id = 0;
    arm_calib1D.clear();
}

/**
 * @brief 重置函数，将所有成员变量重置为初始状态
 * 调用Init()函数实现重置功能
 */
void CamCalibInfo::Rst()
{
    Init();
}

/**
 * @brief 从另一个CamCalibInfo对象拷贝数据
 * @param para 源CamCalibInfo对象
 */
void CamCalibInfo::CopyFrom(const CamCalibInfo& para)
{
    if (this != &para)
        para.CopyTo(*this);
}

/**
 * @brief 拷贝数据到另一个CamCalibInfo对象
 * @param para 目标CamCalibInfo对象
 */
void CamCalibInfo::CopyTo(CamCalibInfo& para) const
{
    if (this != &para)
    {
        para.cam_id = cam_id;
        para.arm_calib1D.clear();
        for (const auto& pair : arm_calib1D) 
        {
            ArmCalibInfo armCalibInfo;
            pair.second.CopyTo(armCalibInfo);
            para.arm_calib1D.insert(std::make_pair(pair.first, armCalibInfo));
        }
    }
}

/**
 * @brief 设置机械臂所有标定信息
 * @param arm_id 机械臂ID
 * @param info 标定信息
 */
void CamCalibInfo::setArmCalibInfo(uint8_t arm_id, const ArmCalibInfo& info)
{
    // 检查是否已存在相同机械臂ID的标定信息
    auto it = arm_calib1D.find(arm_id);
    if (it != arm_calib1D.end()) 
    {
        // 如果存在，更新而不是添加
        info.CopyTo(it->second);
    } 
    else 
    {
        // 如果不存在，添加新的标定信息
        ArmCalibInfo new_info;
        info.CopyTo(new_info);
        arm_calib1D.insert(std::make_pair(arm_id, new_info));
    }
}

/**
 * @brief 检查是否存在指定机械臂的标定信息
 * @param arm_id 机械臂ID
 * @return 是否存在标定信息
 */
bool CamCalibInfo::hasArmCalibInfo(uint8_t arm_id) const
{
    return arm_calib1D.find(arm_id) != arm_calib1D.end();
}

/**
 * @brief 获取指定机械臂的所有标定信息
 * @param arm_id 机械臂ID
 * @return 机械臂标定信息
 */
ArmCalibInfo CamCalibInfo::getArmCalibInfo(uint8_t arm_id) const
{
    auto it = arm_calib1D.find(arm_id);
    if (it != arm_calib1D.end()) 
    {
        return it->second;
    }
    // 如果未找到，返回默认的标定信息
    return ArmCalibInfo();
}

/**
 * @brief 设置指定机械臂的标定信息
 * @param arm_id 机械臂ID
 * @param info 标定信息
 */
void CamCalibInfo::setCalibInfo(uint8_t arm_id, const CalibInfo& info)
{
    // 查找或创建机械臂条目
    auto arm_it = arm_calib1D.find(arm_id);
    if (arm_it == arm_calib1D.end()) 
    {
        // 如果机械臂条目不存在，创建一个新的
        ArmCalibInfo new_arm_info;
        new_arm_info.arm_id = arm_id;
        new_arm_info.setCalibInfo(info);
        arm_calib1D[arm_id] = new_arm_info;
    } 
    else 
    {
        // 如果机械臂条目存在，直接设置标定信息
        arm_it->second.setCalibInfo(info);
    }
}

/**
 * @brief 设置指定机械臂和彩色分辨率的标定结果
 * @param arm_id 机械臂ID
 * @param res 标定结果
 */
void CamCalibInfo::setCalibRes(uint8_t arm_id, const CalibRes& res)
{
    // 查找或创建机械臂条目
    auto arm_it = arm_calib1D.find(arm_id);
    if (arm_it == arm_calib1D.end()) 
    {
        // 如果机械臂条目不存在，创建一个新的
        ArmCalibInfo new_arm_info;
        // 创建一个CalibInfo对象来包装CalibRes
        CalibInfo calib_info;
        calib_info.calib_res = res;
        new_arm_info.setCalibInfo(calib_info);
        arm_calib1D[arm_id] = new_arm_info;
    } 
    else 
    {
        // 如果不存在，添加新的标定结果
        CalibInfo calib_info;
        calib_info.calib_res = res;
        arm_it->second.setCalibInfo(calib_info);
    }
}

/**
 * @brief 检查是否存在指定机械臂和彩色分辨率的标定结果
 * @param arm_id 机械臂ID
 * @return 是否存在标定结果
 */
bool CamCalibInfo::hasCalibRes(uint8_t arm_id) const
{
    auto arm_it = arm_calib1D.find(arm_id);
    if (arm_it != arm_calib1D.end()) {
        return true;
    }else {
        return false;
    }
}

/**
 * @brief 获取指定机械臂和彩色分辨率的标定结果
 * @param arm_id 机械臂ID
 * @return 标定结果
 */
CalibRes CamCalibInfo::getCalibRes(uint8_t arm_id) const
{
    auto arm_it = arm_calib1D.find(arm_id);
    if (arm_it != arm_calib1D.end()) {
        return arm_it->second.calib_info.calib_res;
    }
    else{
        return CalibRes(); // 如果未找到，返回默认的标定结果
    }
}

/**
 * @brief 清理资源的私有辅助函数
 * 当前实现为空，为将来扩展预留
 */
void CamCalibInfo::cleanup()
{
    // 清理资源的实现（如果需要的话）
}

std::vector<uint8_t> getArmIds(const ArmCalibInfoList& arm_calib_list) 
{
    std::vector<uint8_t> arm_ids;
    for (const auto& arm_info : arm_calib_list) 
    {
        arm_ids.push_back(arm_info.second.arm_id);
    }
    return arm_ids;
}

std::vector<uint8_t> getCamIds(const CamCalibInfoList& cam_calib_list)
{
    std::vector<uint8_t> cam_ids;
    for (const auto& cam_info : cam_calib_list) 
    {
        cam_ids.push_back(cam_info.cam_id);
    }
    return cam_ids;
}

} // namespace handeyecalib