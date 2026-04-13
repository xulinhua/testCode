#include "nova_robot_ctrl/nova_robot_ctrl.h"
#include <fmt/format.h>
#include <iostream>
#include <stdexcept>

namespace nova_robot_ctrl
{

    template <typename T>
    T NovaRobotCtrl::get_data_from_bytes(const std::array<char, 1440> &array, int start, int end)
    {
        if (end - start + 1 == sizeof(T))
        {
            T result;
            std::memcpy(&result, &array[start], sizeof(T));
            return result;
        }
        throw std::runtime_error("Byte count does not match type size");
    }

    template <typename T>
    std::vector<T> NovaRobotCtrl::get_datas_from_bytes(const std::array<char, 1440> &array, int start, int end, int datasize)
    {
        if (end - start + 1 == sizeof(T) * datasize)
        {
            std::vector<T> results(datasize);
            for (int i = 0; i < datasize; i++)
            {
                std::memcpy(&(results[i]), &array[start + sizeof(T) * i], sizeof(T));
            }
            return results;
        }
        throw std::runtime_error("Byte count does not match type size");
    }

    NovaRobotCtrl::NovaRobotCtrl(int user_coordinate_id, int tool_coordinate_id)
        : is_running_(false),
          user_coordinate_id_(user_coordinate_id),
          tool_coordinate_id_(tool_coordinate_id),
          robot_id_(0)
    {
        io_context_ = std::make_shared<boost::asio::io_context>();
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - NovaRobotCtrl初始化完成，用户坐标系ID: " << user_coordinate_id_ << ", 工具坐标系ID: " << tool_coordinate_id_ << std::endl;
        
        // 初始化默认的移动范围参数
        set_parameters_for_tool_coordinate(tool_coordinate_id_);
        
        // 初始化标定点管理器
        // calib_robot_pos_mgr_ = std::make_unique<CalibRobotPosMgr>(robot_id_);
    }

    NovaRobotCtrl::~NovaRobotCtrl()
    {
        close();
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - NovaRobotCtrl销毁完成" << std::endl;
    }

    bool NovaRobotCtrl::open(const std::string &ip)
    {
        if (is_running_)
        {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人连接已在运行中" << std::endl;
            return true; // 已经在运行
        }

        try
        {
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在连接机器人，IP地址: " << ip << std::endl;

            // 创建 work_guard 和启动 io_context 线程
            work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(
                boost::asio::make_work_guard(*io_context_));
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 创建 work_guard 并启动 io_context 线程" << std::endl;
            worker_thread_ = std::thread([this]()
                                         { io_context_->run(); });

            // 创建并连接两个 TCP socket
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 创建并连接两个 TCP socket" << std::endl;
            boost::asio::ip::tcp::resolver resolver(*io_context_);

            // 连接端口 29999
            socket_29999_ = std::make_unique<boost::asio::ip::tcp::socket>(*io_context_);
            auto endpoints_29999 = resolver.resolve(ip, "29999");
            boost::asio::connect(*socket_29999_, endpoints_29999);
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已连接到端口 29999" << std::endl;

            socket_30003_ = std::make_unique<boost::asio::ip::tcp::socket>(*io_context_);
            auto endpoints_30003 = resolver.resolve(ip, "30003");
            boost::asio::connect(*socket_30003_, endpoints_30003);
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已连接到端口 30003" << std::endl;

            // 连接端口 30004
            socket_30004_ = std::make_unique<boost::asio::ip::tcp::socket>(*io_context_);
            auto endpoints_30004 = resolver.resolve(ip, "30004");
            boost::asio::connect(*socket_30004_, endpoints_30004);
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已连接到端口 30004" << std::endl;

            // 开始异步接收数据
            start_receive();

            is_running_ = true;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人连接建立成功" << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 连接机器人失败: " << e.what() << std::endl;
            // 连接失败，清理资源
            socket_29999_.reset();
            socket_30003_.reset();
            socket_30004_.reset();
            work_guard_.reset();
            io_context_->stop();
            if (worker_thread_.joinable())
            {
                worker_thread_.join();
            }
            io_context_->restart();
            return false;
        }
    }

    void NovaRobotCtrl::close()
    {
        if (!is_running_)
        {
            return; // 已经关闭
        }

        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在关闭机器人连接" << std::endl;

        // 关闭两个 socket 连接
        if (socket_29999_ && socket_29999_->is_open())
        {
            boost::system::error_code ec;
            socket_29999_->close(ec);
            if (ec)
            {
                std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 关闭 socket 29999 时出错: " << ec.message() << std::endl;
            }
        }
        socket_29999_.reset();

        if (socket_30003_ && socket_30003_->is_open())
        {
            boost::system::error_code ec;
            socket_30003_->close(ec);
            if (ec)
            {
                std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 关闭 socket 30003 时出错: " << ec.message() << std::endl;
            }
        }
        socket_30003_.reset();

        if (socket_30004_ && socket_30004_->is_open())
        {
            boost::system::error_code ec;
            socket_30004_->close(ec);
            if (ec)
            {
                std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 关闭 socket 30004 时出错: " << ec.message() << std::endl;
            }
        }
        socket_30004_.reset();

        // 释放空闲任务，让 io_context->run() 可以退出
        work_guard_.reset();

        // 停止 io_context
        io_context_->stop();

        // 等待工作线程结束
        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }

        // 重置 io_context 以便下次使用
        io_context_->restart();

        is_running_ = false;
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人连接已关闭" << std::endl;
    }

    void NovaRobotCtrl::start_receive()
    {
        if (!socket_30004_ || !socket_30004_->is_open())
        {
            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - Socket 30004 未打开，无法开始接收数据" << std::endl;
            return;
        }

        // 异步接收1440字节数据
        boost::asio::async_read(*socket_30004_,
                                boost::asio::buffer(receive_buffer_, 1440),
                                [this](const boost::system::error_code &error, std::size_t bytes_transferred)
                                {
                                    on_receive(error, bytes_transferred);
                                });
    }

    void NovaRobotCtrl::on_receive(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        if (!error)
        {
            // 数据接收成功，处理接收到的数据
            try
            {
                pushed_info_.time_stamp = get_data_from_bytes<uint64_t>(receive_buffer_, 32, 39);
                pushed_info_.q_actual = get_datas_from_bytes<double>(receive_buffer_, 192, 239, 6);
                pushed_info_.q_d_actual = get_datas_from_bytes<double>(receive_buffer_, 240, 287, 6);
                pushed_info_.enable_status = get_data_from_bytes<uint8_t>(receive_buffer_, 1026, 1026);
                pushed_info_.error_status = get_data_from_bytes<uint8_t>(receive_buffer_, 1027, 1027);

                // 记录接收到的数据
                // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 接收到机器人数据 - 时间戳: " << pushed_info_.time_stamp << ", 关节角度: [" << pushed_info_.q_actual[0] << ", " << pushed_info_.q_actual[1] << ", " << pushed_info_.q_actual[2] << ", " << pushed_info_.q_actual[3] << ", " << pushed_info_.q_actual[4] << ", " << pushed_info_.q_actual[5] << "]" << std::endl;

                start_receive(); // 继续接收
            }
            catch (const std::exception &e)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 处理接收到的数据时出错: " << e.what() << std::endl;
            }
        }
        else
        {
            // 接收出错
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 从机器人接收数据时出错: " << error.message() << std::endl;
        }
    }

    bool NovaRobotCtrl::servo_j(const std::vector<double> &joint, float t /* = 0.1 */, float aheadtime /* = 50 */, float gain /*  = 50 */)
    {
        if (!is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
            return false;
        }

        if (joint.size() != 6)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无效的关节数量。期望6个，实际得到" << joint.size() << "个" << std::endl;
            return false;
        }

        try
        {
            // 将关节角度从vector<double>转换为逗号分隔的字符串
            std::string joint_str;
            for (size_t i = 0; i < joint.size(); ++i)
            {
                if (i > 0)
                    joint_str += ",";
                joint_str += std::to_string(joint[i]);
            }

            // 使用fmt库格式化ServoJ命令字符串
            std::string command = fmt::format("ServoJ({{{}}},t={},aheadtime={},gain={})",
                                              joint_str, t, aheadtime, gain);

            // 发送命令到端口30003
            boost::asio::write(*socket_30003_, boost::asio::buffer(command));

            std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已发送 ServoJ 命令: " << command << std::endl;

            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 发送 ServoJ 命令时出错: " << e.what() << std::endl;
            return false;
        }
    }

    bool NovaRobotCtrl::servo_p(const std::vector<double> &joint, float t /* = 0.1 */, float aheadtime /* = 50 */, float gain /*  = 50 */)
    {
        if (!is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
            return false;
        }

        if (joint.size() != 6)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无效的关节数量。期望6个，实际得到" << joint.size() << "个" << std::endl;
            return false;
        }
        // 检查目标位置是否在允许范围内
        if (!is_position_valid(joint[0], joint[1], joint[2])) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 目标位置超出允许范围，拒绝移动。目标位置：x=" << joint[0] << "，y=" << joint[1] << "，z=" << joint[2] << std::endl;
            return false;
        }
        try
        {
            // 将关节角度从vector<double>转换为逗号分隔的字符串
            std::string joint_str;
            for (size_t i = 0; i < joint.size(); ++i)
            {
                if (i > 0)
                    joint_str += ",";
                joint_str += std::to_string(joint[i]);
            }

            // 使用fmt库格式化ServoJ命令字符串
            std::string command = fmt::format("ServoP({{{}}})",
                                              joint_str);

            // 发送命令到端口30003
            boost::asio::write(*socket_30003_, boost::asio::buffer(command));

            std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已发送 ServoP 命令: " << command << std::endl;

            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 发送 ServoP 命令时出错: " << e.what() << std::endl;
            return false;
        }
    }

    bool NovaRobotCtrl::get_angle(std::vector<double> &joint)
    {
        if (!is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
            return false;
        }

        try
        {
            boost::asio::write(*socket_29999_, boost::asio::buffer("GetAngle()"));
            // 同步读取响应数据
            boost::asio::streambuf response_buffer;
            boost::system::error_code ec;
            std::size_t bytes_read = boost::asio::read_until(*socket_29999_, response_buffer, ";", ec);

            if (ec)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 读取响应时出错: " << ec.message() << std::endl;
                return false;
            }

            const auto bufs = response_buffer.data();
            std::string response = std::string(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));

            // 解析响应字符串，格式：ErrorID,{J1,J2,J3,J4,J5,J6},GetAngle();
            std::size_t start = response.find('{');
            std::size_t end = response.find('}');

            if (start == std::string::npos || end == std::string::npos || start >= end)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 响应格式无效: " << response << std::endl;
                return false;
            }

            // 提取大括号内的内容
            std::string joint_values = response.substr(start + 1, end - start - 1);

            // 按逗号分割字符串
            joint.clear();
            std::stringstream ss(joint_values);
            std::string token;

            while (std::getline(ss, token, ','))
            {
                try
                {
                    joint.push_back(std::stod(token));
                }
                catch (const std::exception &)
                {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 解析关节角度值时出错: " << token << std::endl;
                    return false;
                }
            }

            // 确保有6个关节值
            if (joint.size() != 6)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无效的关节数量。期望6个，实际得到" << joint.size() << "个" << std::endl;
                return false;
            }

            std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 当前关节角度: [" << joint[0] << ", " << joint[1] << ", " << joint[2] << ", " << joint[3] << ", " << joint[4] << ", " << joint[5] << "]" << std::endl;

            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 获取关节角度时出错: " << e.what() << std::endl;
            return false;
        }
    }

    bool NovaRobotCtrl::enable_robot()
    {
        if (!is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
            return false;
        }

        try
        {
            boost::asio::write(*socket_29999_, boost::asio::buffer("EnableRobot()"));
            boost::asio::streambuf response_buffer;
            boost::system::error_code ec;
            std::size_t bytes_read = boost::asio::read_until(*socket_29999_, response_buffer, ";", ec);

            if (ec)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 读取使能响应时出错: " << ec.message() << std::endl;
                return false;
            }

            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人使能成功" << std::endl;
            
            // 设置默认的用户坐标系和工具坐标系
            if (!set_user_coordinate(user_coordinate_id_))
            {
                std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 设置用户坐标系 " << user_coordinate_id_ << " 失败" << std::endl;
            }
            
            if (!set_tool_coordinate(tool_coordinate_id_))
            {
                std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 设置工具坐标系 " << tool_coordinate_id_ << " 失败" << std::endl;
            }
            
            return true;
        }
        catch (const std::exception &e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 使能机器人时出错: " << e.what() << std::endl;
            return false;
        }
    }

    NovaRobotCtrl::pushed_info NovaRobotCtrl::get_pushed_info()
    {
        return pushed_info_;
    }

    bool NovaRobotCtrl::set_user_coordinate(int user_id)
    {
        if (!is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
            return false;
        }

        try
        {
            // 格式化User命令字符串
            std::string command = fmt::format("User({})", user_id);
            
            // 发送命令到端口29999
            boost::asio::write(*socket_29999_, boost::asio::buffer(command));
            
            // 读取响应
            boost::asio::streambuf response_buffer;
            boost::system::error_code ec;
            std::size_t bytes_read = boost::asio::read_until(*socket_29999_, response_buffer, ";", ec);
            
            if (ec)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 读取用户坐标系响应时出错: " << ec.message() << std::endl;
                return false;
            }
            
            const auto bufs = response_buffer.data();
            std::string response = std::string(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));
            
            // 检查响应中是否包含错误信息
            if (response.find("error") != std::string::npos || response.find("Error") != std::string::npos)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 设置用户坐标系失败，响应: " << response << std::endl;
                return false;
            }
            
            // 更新用户坐标系ID
            user_coordinate_id_ = user_id;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 用户坐标系已设置为: " << user_id << std::endl;
            
            return true;
        }
        catch (const std::exception& e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 设置用户坐标系时出错: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool NovaRobotCtrl::set_tool_coordinate(int tool_id)
    {
        if (!is_connected())
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
            return false;
        }

        try
        {
            // 格式化Tool命令字符串
            std::string command = fmt::format("Tool({})", tool_id);
            
            // 发送命令到端口29999
            boost::asio::write(*socket_29999_, boost::asio::buffer(command));
            
            // 读取响应
            boost::asio::streambuf response_buffer;
            boost::system::error_code ec;
            std::size_t bytes_read = boost::asio::read_until(*socket_29999_, response_buffer, ";", ec);
            
            if (ec)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 读取工具坐标系响应时出错: " << ec.message() << std::endl;
                return false;
            }
            
            const auto bufs = response_buffer.data();
            std::string response = std::string(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));
            
            // 检查响应中是否包含错误信息
            if (response.find("error") != std::string::npos || response.find("Error") != std::string::npos)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 设置工具坐标系失败，响应: " << response << std::endl;
                return false;
            }
            
            // 更新工具坐标系ID
            tool_coordinate_id_ = tool_id;
            std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 工具坐标系已设置为: " << tool_id << std::endl;
            
            // 更新参数以匹配新的工具坐标系
            set_parameters_for_tool_coordinate(tool_id);
            
            return true;
        }
        catch (const std::exception& e)
        {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 设置工具坐标系时出错: " << e.what() << std::endl;
            return false;
        }
    }
    
    void NovaRobotCtrl::set_parameters_for_tool_coordinate(int tool_id)
    {
        // 根据工具坐标系ID设置相应的标准位置和移动范围
        // 参照eye2hand_test/nova_robot.py中的实现
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 当前设定的工具坐标系为: " << tool_id << std::endl;
        switch (tool_id) 
        {
            case 0:
                move_range_params_.x_max = 275.00;
                move_range_params_.x_min = -400.00;
                move_range_params_.y_max = -165.00;
                move_range_params_.y_min = -500.00;
                move_range_params_.z_max = 525.00;
                move_range_params_.z_min = 240.00;
            
                // 设置标准位置
                standard_pose_params_.x = -117.1344;
                standard_pose_params_.y = -344.7371;
                standard_pose_params_.z = 415.8233;
                standard_pose_params_.rx = -179.9815;
                standard_pose_params_.ry = 0.0051;
                standard_pose_params_.rz = 90.0711;
                break;
            case 1:
                move_range_params_.x_max = 270.00;
                move_range_params_.x_min = -390.00;
                move_range_params_.y_max = -195.00;
                move_range_params_.y_min = -510.00;
                move_range_params_.z_max = 350.00;
                move_range_params_.z_min = 44.00;

                // 设置标准位置
                standard_pose_params_.x = -163.0518;
                standard_pose_params_.y = -325.5861;
                standard_pose_params_.z = 262.9900;
                standard_pose_params_.rx = -179.9815;
                standard_pose_params_.ry = 0.0051;
                standard_pose_params_.rz = 90.0711;
                break;
            case 2:
                move_range_params_.x_max = 270.00;
                move_range_params_.x_min = -390.00;
                move_range_params_.y_max = -195.00;
                move_range_params_.y_min = -510.00;
                move_range_params_.z_max = 350.00;
                move_range_params_.z_min = 44.00;

                // 设置标准位置
                standard_pose_params_.x = -163.0518;
                standard_pose_params_.y = -325.5861;
                standard_pose_params_.z = 262.9900;
                standard_pose_params_.rx = -179.9815;
                standard_pose_params_.ry = 0.0051;
                standard_pose_params_.rz = 90.0711;
                break;
        }
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 移动范围参数已更新，工具坐标系ID " << tool_id << ": x(" << move_range_params_.x_min << "~" << move_range_params_.x_max << "), y(" << move_range_params_.y_min << "~" << move_range_params_.y_max << "), z(" << move_range_params_.z_min << "~" << move_range_params_.z_max << ")" << std::endl;
        
        // 将更新后的标准位置同步到标定点管理器
        // if (calib_robot_pos_mgr_) {
        //     calib_robot_pos_mgr_->setStandardPose(
        //         standard_pose_params_.x,
        //         standard_pose_params_.y,
        //         standard_pose_params_.z,
        //         standard_pose_params_.rx,
        //         standard_pose_params_.ry,
        //         standard_pose_params_.rz
        //     );
        //     std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 标准位置已同步到标定点管理器: x=" << standard_pose_params_.x << ", y=" << standard_pose_params_.y << ", z=" << standard_pose_params_.z << ", rx=" << standard_pose_params_.rx << ", ry=" << standard_pose_params_.ry << ", rz=" << standard_pose_params_.rz << std::endl;
        // }
    }
    
    bool NovaRobotCtrl::get_standard_pose(Pose& pose)
    {
        // 获取标准位置
        pose.x = standard_pose_params_.x;
        pose.y = standard_pose_params_.y;
        pose.z = standard_pose_params_.z;
        pose.rx = standard_pose_params_.rx;
        pose.ry = standard_pose_params_.ry;
        pose.rz = standard_pose_params_.rz;
        
        std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 标准位置: 位置(" << pose.x << ", " << pose.y << ", " << pose.z << "), 姿态(" << pose.rx << ", " << pose.ry << ", " << pose.rz << ")" << std::endl;
        
        return true;
    }
    
    bool NovaRobotCtrl::move_to_standard_pose()
    {
        // 移动到标准位置，使用默认速度50%
        Pose standard_pose;
        if (!get_standard_pose(standard_pose)) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 获取标准位置失败" << std::endl;
            return false;
        }
        
        // 打印要移动的位置坐标值
        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在移动到标准位置: x=" << standard_pose.x << ", y=" << standard_pose.y << ", z=" << standard_pose.z << ", rx=" << standard_pose.rx << ", ry=" << standard_pose.ry << ", rz=" << standard_pose.rz << std::endl;
        
        // 调用move_l函数，使用50%的速度
        return move_l(standard_pose.x, standard_pose.y, standard_pose.z, 
                      standard_pose.rx, standard_pose.ry, standard_pose.rz, 50);
    }
    
    void NovaRobotCtrl::set_robot_id(int robot_id)
    {
        robot_id_ = robot_id;
        // 更新标定点管理器的机械手ID
        // if (calib_robot_pos_mgr_) {
        //     calib_robot_pos_mgr_->setRobotId(robot_id_);
        // }
    }
    
    // CalibRobotPosMgr* NovaRobotCtrl::get_calib_robot_pos_mgr()
    // {
    //     return calib_robot_pos_mgr_.get();
    // }
    
    bool NovaRobotCtrl::is_position_valid(double x, double y, double z)
    {
        // 检查位置是否在允许范围内
        if (x > move_range_params_.x_max || x < move_range_params_.x_min) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - X坐标超出范围: " << x << " (允许范围: " << move_range_params_.x_min << " ~ " << move_range_params_.x_max << ")" << std::endl;
            return false;
        }
        if (y > move_range_params_.y_max || y < move_range_params_.y_min) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - Y坐标超出范围: " << y << " (允许范围: " << move_range_params_.y_min << " ~ " << move_range_params_.y_max << ")" << std::endl;
            return false;
        }
        if (z > move_range_params_.z_max || z < move_range_params_.z_min) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - Z坐标超出范围: " << z << " (允许范围: " << move_range_params_.z_min << " ~ " << move_range_params_.z_max << ")" << std::endl;
            return false;
        }
        return true;
    }
    
    bool NovaRobotCtrl::is_connected() const
    {
        return is_running_ &&
               socket_29999_ && socket_29999_->is_open() &&
               socket_30003_ && socket_30003_->is_open() &&
               socket_30004_ && socket_30004_->is_open();
    }

    bool NovaRobotCtrl::get_current_pose(Pose &pose)
    {
        // 添加重试机制，最多尝试10次获取位姿，避免在机械臂移动后立即读取时返回空值
        const int max_retries = 10;
        const int retry_delay_ms = 50;  // 重试间隔50毫秒
        
        for (int attempt = 0; attempt < max_retries; ++attempt) 
        {
            if (!is_connected())
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
                return false;
            }

            try
            {
                // 发送获取位姿命令，支持指定用户坐标系和工具坐标系
                std::string command;
                if (user_coordinate_id_ != 0 || tool_coordinate_id_ != 0)
                {
                    // 如果设置了用户坐标系或工具坐标系，则使用指定的坐标系
                    command = fmt::format("GetPose(User={}, Tool={})", user_coordinate_id_, tool_coordinate_id_);
                }
                else
                {
                    // 否则使用默认方式获取位姿
                    command = "GetPose()";
                }
                
                // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 正在发送位姿命令: " << command << std::endl;
                
                // 发送命令
                boost::asio::write(*socket_29999_, boost::asio::buffer(command));

                // 同步读取响应数据
                boost::asio::streambuf response_buffer;
                boost::system::error_code ec;
                std::size_t bytes_read = boost::asio::read_until(*socket_29999_, response_buffer, ";", ec);

                if (ec)
                {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 读取位姿响应时出错: " << ec.message() << std::endl;
                    // 如果不是最后一次尝试，等待后重试
                    if (attempt < max_retries - 1) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第 " << attempt + 1 << " 次尝试失败，" << retry_delay_ms << " 毫秒后重试..." << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                        continue;
                    }
                    return false;
                }

                const auto bufs = response_buffer.data();
                std::string response = std::string(boost::asio::buffers_begin(bufs), boost::asio::buffers_end(bufs));
                
                // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 接收到位姿响应: " << response << std::endl;

                // 解析响应字符串，格式：ErrorID,{X,Y,Z,Rx,Ry,Rz},GetPose();
                std::size_t start = response.find('{');
                std::size_t end = response.find('}');

                if (start == std::string::npos || end == std::string::npos || start >= end)
                {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 位姿响应格式无效: " << response << std::endl;
                    // 如果不是最后一次尝试，等待后重试
                    if (attempt < max_retries - 1) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第 " << attempt + 1 << " 次尝试失败，" << retry_delay_ms << " 毫秒后重试..." << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                        continue;
                    }
                    return false;
                }

                // 提取大括号内的内容
                std::string pose_values = response.substr(start + 1, end - start - 1);
                
                // 检查坐标字符串是否为空
                if (pose_values.empty() || pose_values.find_first_not_of(" \t\n\r\f\v") == std::string::npos)
                {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 响应中的坐标值为空: " << response << std::endl;
                    // 如果不是最后一次尝试，等待后重试
                    if (attempt < max_retries - 1) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第 " << attempt + 1 << " 次尝试失败 (坐标为空)，" << retry_delay_ms << " 毫秒后重试..." << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                        continue;
                    }
                    return false;
                }

                // 按逗号分割字符串
                std::vector<double> values;
                std::stringstream ss(pose_values);
                std::string token;

                while (std::getline(ss, token, ','))
                {
                    try
                    {
                        values.push_back(std::stod(token));
                    }
                    catch (const std::exception &)
                    {
                        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 解析坐标值时出错: " << token << std::endl;
                        // 如果不是最后一次尝试，等待后重试
                        if (attempt < max_retries - 1) {
                            std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第 " << attempt + 1 << " 次尝试失败 (解析错误)，" << retry_delay_ms << " 毫秒后重试..." << std::endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                            continue;
                        }
                        return false;
                    }
                }

                // 确保有6个值(X,Y,Z,Rx,Ry,Rz)
                if (values.size() != 6)
                {
                    std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 坐标值数量无效。期望6个，实际得到" << values.size() << "个" << std::endl;
                    // 如果不是最后一次尝试，等待后重试
                    if (attempt < max_retries - 1) {
                        std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第 " << attempt + 1 << " 次尝试失败 (数量错误)，" << retry_delay_ms << " 毫秒后重试..." << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                        continue;
                    }
                    return false;
                }

                // 设置位姿信息
                pose.x = values[0];
                pose.y = values[1];
                pose.z = values[2];
                pose.rx = values[3];
                pose.ry = values[4];
                pose.rz = values[5];

                // std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 当前位姿: 位置(" << pose.x << ", " << pose.y << ", " << pose.z << "), 姿态(" << pose.rx << ", " << pose.ry << ", " << pose.rz << ")" << std::endl;

                return true;
            }
            catch (const std::exception &e)
            {
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 获取当前位姿时出错: " << e.what() << std::endl;
                // 如果不是最后一次尝试，等待后重试
                if (attempt < max_retries - 1) {
                    std::cout << "[WARN] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 第 " << attempt + 1 << " 次尝试失败 (异常)，" << retry_delay_ms << " 毫秒后重试..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                    continue;
                }
                return false;
            }
        }
        
        // 所有重试都失败了
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 经过 " << max_retries << " 次尝试后仍未能获取当前位姿" << std::endl;
        return false;
    }
bool NovaRobotCtrl::move_l(double x, double y, double z, double rx /* = 180.0 */, double ry /* = 0.0 */, double rz /* = 90.0 */, int speed /* = 100 */)
{
    if (!is_connected()) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
        return false;
    }

    // 检查目标位置是否在允许范围内
    if (!is_position_valid(x, y, z)) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 目标位置超出允许范围，拒绝移动。目标位置：x=" << x << "，y=" << y << "，z=" << z << "，rx=" << rx << "，ry=" << ry << "，rz=" << rz << std::endl;
        return false;
    }

    try {
        // 使用fmt库格式化MovL命令字符串，支持指定用户坐标系和工具坐标系
        std::string command;
        if (user_coordinate_id_ != 0 || tool_coordinate_id_ != 0)
        {
            // 如果设置了用户坐标系或工具坐标系，则使用指定的坐标系
            command = fmt::format("MovL({{{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}}},User={},Tool={},Speed={})",
                               x, y, z, rx, ry, rz, user_coordinate_id_, tool_coordinate_id_, speed);
        }
        else
        {
            // 否则使用默认方式
            command = fmt::format("MovL({{{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}}},Speed={})",
                               x, y, z, rx, ry, rz, speed);
        }

        // 发送命令到端口29999
        boost::asio::write(*socket_30003_, boost::asio::buffer(command));

        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已发送 MovL 命令: " << command.c_str() << std::endl;

        return true;
    } catch (const std::exception& e) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 发送 MovL 命令时出错: " << e.what() << std::endl;
        return false;
    }
}

bool NovaRobotCtrl::jog_move(char axis, double distance, int speed /* = 100 */)
{
    if (!is_connected()) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
        return false;
    }

    try {
        // 获取当前位姿
        Pose current_pose;
        if (!get_current_pose(current_pose)) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 获取当前位姿失败" << std::endl;
            return false;
        }

        // 根据轴向计算新位置
        double new_x = current_pose.x;
        double new_y = current_pose.y;
        double new_z = current_pose.z;

        switch (axis) {
            case 'x':
            case 'X':
                new_x += distance;
                break;
            case 'y':
            case 'Y':
                new_y += distance;
                break;
            case 'z':
            case 'Z':
                new_z += distance;
                break;
            default:
                std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 无效的轴向: " << axis << std::endl;
                return false;
        }

        // 检查目标位置是否在允许范围内
        if (!is_position_valid(new_x, new_y, new_z)) {
            std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 目标位置超出允许范围，拒绝移动。目标位置：x=" << new_x << "，y=" << new_y << "，z=" << new_z << std::endl;
            return false;
        }

        // 保持姿态角不变
        double rx = current_pose.rx;
        double ry = current_pose.ry;
        double rz = current_pose.rz;

        // 使用fmt库格式化MovL命令字符串，支持指定用户坐标系和工具坐标系
        std::string command;
        if (user_coordinate_id_ != 0 || tool_coordinate_id_ != 0)
        {
            // 如果设置了用户坐标系或工具坐标系，则使用指定的坐标系
            command = fmt::format("MovL({{{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}}},User={},Tool={},Speed={})",
                               new_x, new_y, new_z, rx, ry, rz, user_coordinate_id_, tool_coordinate_id_, speed);
        }
        else
        {
            // 否则使用默认方式
            command = fmt::format("MovL({{{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}}},Speed={})",
                               new_x, new_y, new_z, rx, ry, rz, speed);
        }

        // 发送命令到端口29999
        boost::asio::write(*socket_30003_, boost::asio::buffer(command));

        std::cout << "[INFO] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 已发送 MovL 命令: " << command.c_str() << std::endl;

        return true;
    } catch (const std::exception& e) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 发送 MovL 命令时出错: " << e.what() << std::endl;
        return false;
    }
}

bool NovaRobotCtrl::sync()
{
    if (!is_connected()) {
        std::cout << "[ERROR] " << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << " - 机器人未连接" << std::endl;
        return false;
    }
    boost::asio::write(*socket_30003_, boost::asio::buffer("Sync()"));

    return true;
}


} // namespace nova_robot_ctrl