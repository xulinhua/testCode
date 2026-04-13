#ifndef NOVA_ROBOT_CTRL_CALIB_ROBOT_POS_MGR_H
#define NOVA_ROBOT_CTRL_CALIB_ROBOT_POS_MGR_H

#include <vector>
#include <utility>
#include <string>
#include <memory>

namespace nova_robot_ctrl {

/**
 * @brief 机器人位姿数据结构体
 * 包含机械手的完整位姿信息
 */
struct RobotPoseData {
    double x = 0.0;   ///< X坐标 (mm)
    double y = 0.0;   ///< Y坐标 (mm)
    double z = 0.0;   ///< Z坐标 (mm)
    double rx = 0.0;  ///< Rx旋转角 (degrees)
    double ry = 0.0;  ///< Ry旋转角 (degrees)
    double rz = 0.0;  ///< Rz旋转角 (degrees)
};

/**
 * @brief Nova机械手标定点管理类
 * 用于加载或生成机械手标定点坐标，支持多个机械手
 */
class CalibRobotPosMgr {
public:
    /**
     * @brief 构造函数
     * @param robot_id 机械手ID，用于区分不同机械手的配置
     */
    explicit CalibRobotPosMgr(int robot_id = 0);

    /**
     * @brief 析构函数
     */
    ~CalibRobotPosMgr();

    /**
     * @brief 生成数据采集点列表
     * @return 机械臂姿态数据点列表
     * @note 如果设置了加载预保存的点数据，则加载并返回这些点；
     *       否则使用现有的方式生成移动用的标定点
     */
    std::vector<RobotPoseData> generateDataPoints() const;

    /**
     * @brief 初始化缓存数据点列表
     * @note 外部应在调用getPointCount或getDataPoint之前调用此方法
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
     * @brief 设置机械手ID
     * @param robot_id 机械手ID
     */
    void setRobotId(int robot_id);

    /**
     * @brief 获取机械手ID
     * @return 机械手ID
     */
    int getRobotId() const;

private:
    // XY平面偏移点配置
    std::vector<std::pair<double, double>> xy_offsets_;

    // Z轴偏移值配置
    std::vector<double> z_vals_;

    // 标准位置
    RobotPoseData standard_pose_;

    // 缓存的数据点列表
    mutable std::vector<RobotPoseData> cached_data_points_;

    // 机械手ID
    int robot_id_;

    /**
     * @brief 加载预保存的数据点
     * @return 机械臂姿态数据点列表
     */
    std::vector<RobotPoseData> loadSavedDataPoints() const;

    /**
     * @brief 根据机械手ID获取对应的标定数据目录
     * @return 标定数据目录路径
     */
    std::string getCalibDataDirectory() const;
};

} // namespace nova_robot_ctrl

#endif // NOVA_ROBOT_CTRL_CALIB_ROBOT_POS_MGR_H