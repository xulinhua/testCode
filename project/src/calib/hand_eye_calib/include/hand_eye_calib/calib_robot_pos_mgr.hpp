#ifndef HAND_EYE_CALIB__CALIB_ROBOT_POS_MGR_HPP_
#define HAND_EYE_CALIB__CALIB_ROBOT_POS_MGR_HPP_

#include <vector>
#include <utility>
#include <cstddef>
#include <string>  // 添加缺失的string头文件包含

// 命名空间定义
namespace handeyecalib {

// 全局变量，控制是否加载预保存的点数据
// true: 加载预保存的点数据 false: 使用现有的方式生成移动用的标定点
extern bool g_load_saved_points;

/**
 * @struct RobotPoseData
 * @brief 机械臂姿态数据结构
 */
struct RobotPoseData {
    double x, y, z;      ///< 位置坐标 (mm)
    double rx, ry, rz;   ///< 姿态角 (degrees)
};

/**
 * @class CalibRobotPosMgr
 * @brief 机械臂坐标数据管理器
 * 
 * 该类用于生成和管理手眼标定时所需的机械臂坐标数据点
 * 参照nova_data_collector.py中第58行到第75行的实现
 */
class CalibRobotPosMgr {
public:
    /**
     * @brief 构造函数
     */
    CalibRobotPosMgr();
    
    /**
     * @brief 析构函数
     */
    ~CalibRobotPosMgr();
    
    /**
     * @brief 生成数据采集点列表
     * @return 机械臂姿态数据点列表
     */
    std::vector<RobotPoseData> generateDataPoints() const;
    
    /**
     * @brief 初始化缓存数据点列表
     * 外部应在调用getPointCount或getDataPoint之前调用此方法
     */
    void initializeCachedDataPoints();
    
    /**
     * @brief 获取指定索引的数据点
     * @param index 数据点索引
     * @return 机械臂姿态数据，如果索引无效则返回默认值
     * @note 调用前必须先调用initializeCachedDataPoints初始化缓存
     */
    RobotPoseData getDataPoint(std::size_t index) const;
    
    /**
     * @brief 获取数据点数量
     * @return 数据点数量
     * @note 调用前必须先调用initializeCachedDataPoints初始化缓存
     */
    std::size_t getPointCount() const;
    
    /**
     * @brief 设置标准位置
     * @param x X坐标 (mm)
     * @param y Y坐标 (mm)
     * @param z Z坐标 (mm)
     * @param rx Rx角度 (degrees)
     * @param ry Ry角度 (degrees)
     * @param rz Rz角度 (degrees)
     */
    void setStandardPose(double x, double y, double z, double rx, double ry, double rz);
    
    /**
     * @brief 获取标准位置
     * @return 标准位置姿态数据
     */
    RobotPoseData getStandardPose() const;
    
    /**
     * @brief 设置缓存的数据点列表
     * @param data_points 数据点列表
     */
    void setCachedDataPoints(const std::vector<RobotPoseData>& data_points);

private:
    // XY平面偏移点配置
    std::vector<std::pair<double, double>> xy_offsets_;
    
    // Z轴偏移值配置
    std::vector<double> z_vals_;
    std::vector<std::vector<double>> ang_vals_;  // 角度值配置
    
    // 标准位置
    RobotPoseData standard_pose_;
    
    // 缓存的数据点列表
    std::vector<RobotPoseData> cached_data_points_;
    
    /**
     * @brief 加载预保存的数据点
     * @return 机械臂姿态数据点列表
     */
    std::vector<RobotPoseData> loadSavedDataPoints() const;
};
} // namespace handeyecalib

#endif // HAND_EYE_CALIB__CALIB_ROBOT_POS_MGR_HPP_