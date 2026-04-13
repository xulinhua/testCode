#ifndef HAND_EYE_CALIB__CALIB_CONFIG_HPP_
#define HAND_EYE_CALIB__CALIB_CONFIG_HPP_

#include <string>
#include <yaml-cpp/yaml.h>

namespace handeyecalib {

/**
 * @class CalibConfig
 * @brief 手眼标定配置管理类
 */
class CalibConfig {
public:
    /**
     * @brief 构造函数
     */
    CalibConfig();
    
    /**
     * @brief 析构函数
     */
    ~CalibConfig();
    
    /**
     * @brief 加载配置文件
     * @param config_file 配置文件路径
     * @return 是否加载成功
     */
    bool loadConfig(const std::string& config_file);
    
    /**
     * @brief 保存配置文件
     * @param config_file 配置文件路径
     * @return 是否保存成功
     */
    bool saveConfig(const std::string& config_file);
    
    /**
     * @brief 获取最小标定点数量
     * @return 最小标定点数量
     */
    int getMinCalibrationPoints() const;
    
    /**
     * @brief 设置最小标定点数量
     * @param min_points 最小标定点数量
     */
    void setMinCalibrationPoints(int min_points);
    
    /**
     * @brief 获取源数据目录
     * @return 源数据目录
     */
    std::string getSourceDataDir() const;
    
    /**
     * @brief 设置源数据目录
     * @param dir 源数据目录
     */
    void setSourceDataDir(const std::string& dir);
    
    /**
     * @brief 获取输出数据目录
     * @return 输出数据目录
     */
    std::string getOutputDataDir() const;
    
    /**
     * @brief 设置输出数据目录
     * @param dir 输出数据目录
     */
    void setOutputDataDir(const std::string& dir);

private:
    // 配置参数
    int min_calibration_points_;           ///< 最小标定点数量
    std::string source_data_dir_;          ///< 源数据目录
    std::string output_data_dir_;          ///< 输出数据目录
    
    // YAML节点
    YAML::Node config_node_;               ///< YAML配置节点
};

} // namespace handeyecalib

#endif // HAND_EYE_CALIB__CALIB_CONFIG_HPP_