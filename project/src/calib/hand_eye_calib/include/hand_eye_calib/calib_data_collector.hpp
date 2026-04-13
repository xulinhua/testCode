#ifndef HAND_EYE_CALIB__CALIB_DATA_COLLECTOR_HPP_
#define HAND_EYE_CALIB__CALIB_DATA_COLLECTOR_HPP_

#include <vector>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <utility>

// 命名空间定义
namespace handeyecalib {

/**
 * @struct CalibrationPoint
 * @brief 标定点数据结构
 */
struct CalibrationPoint {
    int index;                                    ///< 数据点索引
    double timestamp;                             ///< 时间戳
    std::vector<double> robot_pose;               ///< 机械臂位姿 [x, y, z, rx, ry, rz]
    std::vector<double> marker_position;          ///< 标记位置 [x, y, z]
};

/**
 * @class CalibDataCollector
 * @brief 手眼标定数据管理类
 * 
 * 该类仅负责管理标定点数据，不负责获取机械手坐标数据或Aruco码识别数据。
 * 数据都是从其他功能模块传入的，确保传入的机械手坐标数据和Aruco识别数据是同一个标定点位置的。
 */
class CalibDataCollector {
public:
    /**
     * @brief 构造函数
     */
    CalibDataCollector();
    
    /**
     * @brief 析构函数
     */
    ~CalibDataCollector();
    
    /**
     * @brief 添加标定点数据
     * @param robot_pose 机械臂位姿 [x, y, z, rx, ry, rz]
     * @param marker_position 标记位置 [x, y, z]
     * @return 是否添加成功
     */
    bool addCalibrationPoint(const std::vector<double>& robot_pose, 
                            const std::vector<double>& marker_position);
    
    /**
     * @brief 设置指定索引的标定点数据
     * @param index 标定点索引
     * @param robot_pose 机械臂位姿 [x, y, z, rx, ry, rz]
     * @param marker_position 标记位置 [x, y, z]
     * @return 是否设置成功
     */
    bool setCalibrationPoint(int index,
                            const std::vector<double>& robot_pose,
                            const std::vector<double>& marker_position);
    
    /**
     * @brief 添加完整的标定点数据
     * @param point 标定点数据
     */
    void addCalibrationPoint(const CalibrationPoint& point);
    
    /**
     * @brief 设置指定索引的完整标定点数据
     * @param index 标定点索引
     * @param point 标定点数据
     * @return 是否设置成功
     */
    bool setCalibrationPoint(int index, const CalibrationPoint& point);
    
    /**
     * @brief 加载本地标定数据
     * @param data_dir 数据目录路径
     * @return 是否加载成功
     */
    bool loadCalibrationData(const std::string& data_dir);
    
    /**
     * @brief 保存标定数据
     * @param save_dir 保存目录路径
     * @return 是否保存成功
     */
    bool saveCalibrationData(const std::string& save_dir);
    
    /**
     * @brief 获取所有标定点数据
     * @return 标定点数据列表
     */
    const std::vector<CalibrationPoint>& getCalibrationPoints() const;
    
    /**
     * @brief 获取标定计算所需的数据格式
     * @return pair<robot_poses, marker_positions> 机器人位姿列表和标记位置列表
     */
    std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> 
    getCalibrationData() const;
    
    /**
     * @brief 获取指定索引的标定点数据
     * @param index 标定点索引
     * @return 标定点数据指针，如果不存在则返回nullptr
     */
    const CalibrationPoint* getCalibrationPoint(int index) const;
    
    /**
     * @brief 清除所有标定点数据
     */
    void clearCalibrationPoints();
    
    /**
     * @brief 检查是否有足够的标定点数据
     * @param min_points 最少需要的点数
     * @return 是否满足条件
     */
    bool hasEnoughPoints(size_t min_points = 4) const;
    
    /**
     * @brief 获取标定点数量
     * @return 标定点数量
     */
    size_t getPointCount() const;

private:
    // 数据存储
    std::vector<CalibrationPoint> calibration_points_;        ///< 标定点数据
    
    // 线程安全
    mutable std::mutex data_mutex_;                           ///< 数据访问互斥锁
};

} // namespace handeyecalib

#endif // HAND_EYE_CALIB__CALIB_DATA_COLLECTOR_HPP_