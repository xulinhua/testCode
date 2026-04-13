#include "cam_mgr_ros/cam_base.hpp"
#include "cam_config_mgr/cam_com_struct.hpp"
#include <dirent.h>
#include <fstream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <climits>

namespace cam_mgr_ros
{

CamBase::CamBase() : initialized_(false), camera_type_(CamMgr::CamType::CAM_TYPE_NONE)
{
}

CamBase::~CamBase()
{
}

pid_t CamBase::find_descendant_exec_pid(pid_t root, const std::vector<std::string>& exec_names, const std::string& ros_node_name, int timeout_ms)
{
    if (root <= 0)
        return 0;

    const int interval_ms = 100;
    int waited = 0;
    while (waited <= timeout_ms)
    {
        // 建立 pid->ppid 映射
        std::map<pid_t, pid_t> ppid_map;
        DIR* proc = opendir("/proc");
        if (!proc)
            return 0;
        struct dirent* ent;
        while ((ent = readdir(proc)) != nullptr)
        {
            if (ent->d_type != DT_DIR)
                continue;
            std::string dname(ent->d_name);
            if (!std::all_of(dname.begin(), dname.end(), ::isdigit))
                continue;
            pid_t pid = static_cast<pid_t>(std::stoi(dname));
            std::string stat_path = std::string("/proc/") + dname + "/stat";
            std::ifstream statf(stat_path);
            if (!statf.is_open())
                continue;
            std::string line;
            std::getline(statf, line);
            statf.close();
            auto rparen = line.rfind(')');
            if (rparen == std::string::npos)
                continue;
            std::istringstream iss(line.substr(rparen + 2));
            std::string state;
            pid_t ppid = 0;
            if (!(iss >> state))
                continue;
            if (!(iss >> ppid))
                continue;
            ppid_map[pid] = ppid;
        }
        closedir(proc);

        // BFS 查找所有后代
        std::vector<pid_t> q;
        q.push_back(root);
        size_t qi = 0;
        std::set<pid_t> visited;
        visited.insert(root);
        while (qi < q.size())
        {
            pid_t cur = q[qi++];
            for (const auto &kv : ppid_map)
            {
                pid_t pid = kv.first;
                pid_t ppid = kv.second;
                if (ppid == cur && visited.insert(pid).second)
                {
                    std::string comm_path = "/proc/" + std::to_string(pid) + "/comm";
                    std::ifstream commf(comm_path);
                    std::string comm;
                    if (commf.is_open())
                    {
                        std::getline(commf, comm);
                        commf.close();
                    }
                    std::string cmdline_path = "/proc/" + std::to_string(pid) + "/cmdline";
                    std::ifstream cmdf(cmdline_path);
                    std::string cmdline;
                    if (cmdf.is_open())
                    {
                        // cmdline 各参数间以 '\0' 分隔，必须读完整内容并替换为空格
                        // 否则只能读到第一个 '\0' 前的执行文件名，后续参数全部丢失
                        std::string raw((std::istreambuf_iterator<char>(cmdf)),
                                        std::istreambuf_iterator<char>());
                        cmdf.close();
                        std::replace(raw.begin(), raw.end(), '\0', ' ');
                        cmdline = raw;
                    }

                    for (const auto &ename : exec_names)
                    {
                        if ((!comm.empty() && comm.find(ename) != std::string::npos) || (!cmdline.empty() && cmdline.find(ename) != std::string::npos))
                        {
                            if (ros_node_name.empty() || cmdline.find(ros_node_name) != std::string::npos)
                            {
                                return pid;
                            }
                        }
                    }

                    q.push_back(pid);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        waited += interval_ms;
    }

    return 0;
}

bool CamBase::initialize()
{
    // 基类默认实现
    LOG_INFO("初始化相机基类");
    initialized_ = true;
    return true;
}

void CamBase::destroy()
{
    // 基类默认实现
    LOG_INFO("销毁相机基类资源");
    initialized_ = false;
}

void CamBase::set_camera_type(CamMgr::CamType type)
{
    camera_type_ = type;
}

CamMgr::CamType CamBase::get_camera_type() const
{
    return camera_type_;
}

bool CamBase::get_camera_status(int cam_id)
{
    // 基类默认实现
    LOG_INFO("获取相机ID %d 的状态", cam_id);
    return initialized_;
}

bool CamBase::set_camera_param(int cam_id, const std::string& param_name, const std::string& param_value)
{
    // 基类默认实现
    LOG_INFO("设置相机ID %d 的参数 %s = %s", cam_id, param_name.c_str(), param_value.c_str());
    return true;
}

std::string CamBase::get_camera_info_topic(int cam_id, CamMgr::CamStreamType stream_type, const CamMgr::CamInfo& cam_info)
{
    (void)cam_id; (void)stream_type; (void)cam_info;
    // pure virtual in base; should not be called
    return std::string();
}

CamMgr::CamType CamBase::get_cam_type_by_str(const std::string& device_name, const std::string& facturer_name)
{
    if (device_name.find("RealSense") != std::string::npos ||
        facturer_name.find("RealSense") != std::string::npos)
    {
        return CamMgr::CamType::CAM_TYPE_RS;
    }
    else if (device_name.find("Orbbec") != std::string::npos ||
             facturer_name.find("Orbbec") != std::string::npos)
    {
        return CamMgr::CamType::CAM_TYPE_OB;
    }
    else if (device_name.find("CSI") != std::string::npos || 
             facturer_name.find("CSI") != std::string::npos ||
             device_name.find("vi-output") != std::string::npos ||
             facturer_name.find("vi-output") != std::string::npos)
    {
        return CamMgr::CamType::CAM_TYPE_CSI;
    }
    return CamMgr::CamType::CAM_TYPE_NONE;
}

CamMgr::CamModelType CamBase::get_device_type_by_str(const std::string& device_name)
{
    if (device_name.find("D405") != std::string::npos)
    {
        return CamMgr::CamModelType::CAM_MODEL_RS_D405;
    }
    else if (device_name.find("D435") != std::string::npos)
    {
        return CamMgr::CamModelType::CAM_MODEL_RS_D435;
    }
    else if (device_name.find("D455") != std::string::npos)
    {
        return CamMgr::CamModelType::CAM_MODEL_RS_D455;
    }
    else if (device_name.find("L515") != std::string::npos)
    {
        return CamMgr::CamModelType::CAM_MODEL_RS_L515;
    }
    else if (device_name.find("Gemini 335") != std::string::npos)
    {
        return CamMgr::CamModelType::CAM_MODEL_OB_GEMINI_335;
    }
    else if (device_name.find("Gemini 2") != std::string::npos)
    {
        return CamMgr::CamModelType::CAM_MODEL_OB_GEMINI_2;
    }
    else if (device_name.find("vi") != std::string::npos)
    {
        return CamMgr::CamModelType::CAM_MODEL_CSI_MIPI;
    }
    else
    {
        return CamMgr::CamModelType::CAM_MODEL_NONE;
    }
}

void CamBase::kill_camera_processes()
{
    LOG_INFO("检查并清理可能占用相机的进程");
    //  杀死RealSense相机相关进程
    int kill_result1 = system("pkill -f 'realsense2_camera' 2>/dev/null || true; "
                              "pkill -f 'rs_launch.py' 2>/dev/null || true");
    int exit_status1 = WEXITSTATUS(kill_result1);
    if (exit_status1 == 0 || exit_status1 == 1)
    { // 0:找到并kill, 1:没找到进程
        LOG_INFO("成功清理RealSense相机进程");
    }

    // 杀死Orbbec/Gemini相机相关进程
    int kill_result2 = system("pkill -f 'orbbec_camera' 2>/dev/null || true; "
                              "pkill -f 'orbbec_camera.launch.py' 2>/dev/null || true");
    int exit_status2 = WEXITSTATUS(kill_result2);
    if (exit_status2 == 0 || exit_status2 == 1)
    {
        LOG_INFO("成功清理Orbbec/Gemini相机进程");
    }

    // 杀死Realsense相机相关进程 (原有方法作为备份)
    int kill_result3 = system("killall -q -9 realsense2_camera_node || true");
    if (kill_result3 == 0)
    {
        LOG_INFO("成功清理Realsense相机进程");
    }

    // 杀死Orbbec相机相关进程 (原有方法作为备份)
    int kill_result4 = system("killall -q -9 orbbec_camera_node || true");
    if (kill_result4 == 0)
    {
        LOG_INFO("成功清理Orbbec相机进程");
    }

    // 杀死通用的相机进程
    int kill_result5 = system("pkill -f camera_node || true");
    if (kill_result5 == 0)
    {
        LOG_INFO("成功清理通用相机进程");
    }

    // 等待一段时间确保进程完全退出
    if (kill_result3 == 0 || kill_result4 == 0 || kill_result5 == 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        LOG_INFO("相机进程清理完成");
    }
    else
    {
        LOG_INFO("未发现占用相机的进程");
    }
}

bool CamBase::find_closest_roi_fps(const std::vector<CamMgr::CamRoiFps>& roi_fps_list, int target_width, int target_height, int target_fps,
                                     int& closest_width, int& closest_height, int& closest_fps)
{
    try
    {
        closest_width = target_width;
        closest_height = target_height;
        closest_fps = 0;
        
        if (roi_fps_list.empty())
        {
            LOG_WARN("ROI和FPS列表为空");
            return false;
        }

        // 查找匹配的分辨率和最接近的FPS
        int best_match_index = -1;
        int min_resolution_diff = INT_MAX;
        int closest_fps_diff = INT_MAX;

        // 第一步：查找最匹配的分辨率（长宽）
        for (size_t i = 0; i < roi_fps_list.size(); i++)
        {
            const CamMgr::CamRoiFps &roi_fps = roi_fps_list[i];

            // 计算与目标配置的分辨率差异
            int width_diff = abs(roi_fps.width - target_width);
            int height_diff = abs(roi_fps.height - target_height);
            int resolution_diff = width_diff + height_diff;

            if (resolution_diff < min_resolution_diff)
            {
                // 找到更接近的分辨率，更新最佳匹配
                min_resolution_diff = resolution_diff;
                best_match_index = static_cast<int>(i);
                // 在相同分辨率下，查找最接近的FPS
                closest_fps_diff = abs(roi_fps.fps - target_fps);
            }
            else if (resolution_diff == min_resolution_diff)
            {
                // 分辨率相同，比较FPS差异
                int fps_diff = abs(roi_fps.fps - target_fps);
                if (fps_diff < closest_fps_diff)
                {
                    best_match_index = static_cast<int>(i);
                    closest_fps_diff = fps_diff;
                }
            }
        }

        // 检查是否找到了匹配项
        if (best_match_index == -1)
        {
            LOG_WARN("未找到ROI和FPS匹配");
            return false;
        }

        // 使用找到的最佳匹配配置
        const CamMgr::CamRoiFps &best_match = roi_fps_list[best_match_index];
        closest_width = best_match.width;
        closest_height = best_match.height;
        closest_fps = best_match.fps;
        LOG_INFO("找到最接近的ROI和FPS匹配: %dx%d@%d (目标: %dx%d@%d)",
                 closest_width, closest_height, closest_fps, target_width, target_height, target_fps);
        return true;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("查找最接近的ROI和FPS匹配时发生错误: %s", e.what());
        return false;
    }
}

}  // namespace cam_mgr_ros