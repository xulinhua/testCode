#!/bin/bash
#
# @file start_bas_control.sh
# @brief bas_control_node 启动脚本（带进程检查）
# 
# 功能说明：
# 1. 检查 bas_control_node 是否已在运行
# 2. 如果未运行，则启动进程
# 3. 如果已运行，则提示并退出
#
# 使用方法：
# ./start_bas_control.sh              # 前台运行
# ./start_bas_control.sh --daemon     # 后台运行
# ./start_bas_control.sh --check      # 仅检查状态
#
# 作者：bas_control 开发团队
# 日期：2024
#

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_debug() {
    echo -e "${BLUE}[DEBUG]${NC} $1"
}

# ROS2 环境设置
setup_ros_environment() {
    # 检查 ROS2 安装路径
    if [ -z "$ROS_DISTRO" ]; then
        # 尝试自动检测 ROS2 版本
        if [ -d "/opt/ros/humble" ]; then
            ROS_DISTRO="humble"
        elif [ -d "/opt/ros/foxy" ]; then
            ROS_DISTRO="foxy"
        else
            log_error "未找到 ROS2 安装，请先安装 ROS2"
            exit 1
        fi
    fi
    
    # Source ROS2 环境
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
        log_debug "已加载 ROS2 $ROS_DISTRO 环境"
    else
        log_error "ROS2 环境文件不存在: /opt/ros/$ROS_DISTRO/setup.bash"
        exit 1
    fi
    
    # Source 工作空间（自动检测）
    local workspace_paths=(
        "$HOME/ros2_ws"
        "$HOME/testCode/dev"
        "$(dirname $(dirname $(dirname $(readlink -f $0))))/.."  # 脚本所在目录的上级
    )
    
    for ws_path in "${workspace_paths[@]}"; do
        if [ -f "$ws_path/install/setup.bash" ]; then
            source "$ws_path/install/setup.bash"
            log_debug "已加载工作空间: $ws_path"
            break
        fi
    done
}

# 检查 bas_control_node 是否在运行
check_process() {
    local pid=$(pgrep -f "bas_control_node")
    
    if [ -n "$pid" ]; then
        return 0  # 进程存在
    else
        return 1  # 进程不存在
    fi
}

# 获取进程详细信息
get_process_info() {
    local pid=$(pgrep -f "bas_control_node")
    
    if [ -n "$pid" ]; then
        log_info "bas_control_node 正在运行"
        log_info "  PID: $pid"
        
        # 获取进程启动时间
        local start_time=$(ps -p $pid -o lstart= 2>/dev/null)
        if [ -n "$start_time" ]; then
            log_info "  启动时间: $start_time"
        fi
        
        # 获取进程运行时长
        local elapsed=$(ps -p $pid -o etime= 2>/dev/null)
        if [ -n "$elapsed" ]; then
            log_info "  运行时长: $elapsed"
        fi
        
        # 获取进程内存使用
        local mem=$(ps -p $pid -o rss= 2>/dev/null)
        if [ -n "$mem" ]; then
            local mem_mb=$((mem / 1024))
            log_info "  内存使用: ${mem_mb} MB"
        fi
        
        # 获取进程 CPU 使用
        local cpu=$(ps -p $pid -o %cpu= 2>/dev/null)
        if [ -n "$cpu" ]; then
            log_info "  CPU 使用: ${cpu}%"
        fi
        
        return 0
    else
        log_warn "bas_control_node 未运行"
        return 1
    fi
}

# 启动 bas_control_node
start_process() {
    log_info "正在启动 bas_control_node..."
    
    # 设置 ROS2 环境
    setup_ros_environment
    
    # 检查是否已运行
    if check_process; then
        log_warn "bas_control_node 已在运行中，无需重复启动"
        get_process_info
        return 0
    fi
    
    # 启动参数
    local log_level="info"  # 默认日志级别
    local background=false
    
    # 解析参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --daemon)
                background=true
                shift
                ;;
            --log-level)
                log_level="$2"
                shift 2
                ;;
            *)
                shift
                ;;
        esac
    done
    
    # 启动进程
    if [ "$background" = true ]; then
        log_info "以后台模式启动..."
        ros2 run bas_control bas_control_node --ros-args --log-level $log_level > /var/log/bas/bas_control.log 2>&1 &
        local pid=$!
        sleep 1
        
        if check_process; then
            log_info "✅ bas_control_node 启动成功 (PID: $pid)"
            log_info "   日志文件: /var/log/bas/bas_control.log"
            return 0
        else
            log_error "❌ bas_control_node 启动失败"
            return 1
        fi
    else
        log_info "以前台模式启动..."
        ros2 run bas_control bas_control_node --ros-args --log-level $log_level
    fi
}

# 停止进程
stop_process() {
    log_info "正在停止 bas_control_node..."
    
    if ! check_process; then
        log_warn "bas_control_node 未运行，无需停止"
        return 0
    fi
    
    # 调用 ROS 服务停止系统（优雅停止）
    log_info "调用 /bas/stop_system 服务..."
    setup_ros_environment
    
    ros2 service call /bas/stop_system std_srvs/srv/Trigger 2>/dev/null
    sleep 2
    
    # 检查是否还有进程残留
    if check_process; then
        log_warn "进程仍在运行，发送 SIGTERM 信号..."
        pkill -TERM -f "bas_control_node"
        sleep 2
    fi
    
    # 如果还在运行，强制终止
    if check_process; then
        log_warn "进程未响应，发送 SIGKILL 信号..."
        pkill -KILL -f "bas_control_node"
        sleep 1
    fi
    
    if ! check_process; then
        log_info "✅ bas_control_node 已停止"
        return 0
    else
        log_error "❌ 无法停止 bas_control_node"
        return 1
    fi
}

# 重启进程
restart_process() {
    log_info "正在重启 bas_control_node..."
    
    stop_process
    sleep 1
    start_process --daemon
}

# 查看状态
show_status() {
    log_info "========================================="
    log_info "bas_control_node 状态检查"
    log_info "========================================="
    
    setup_ros_environment
    
    if check_process; then
        get_process_info
        
        # 调用 ROS 服务获取系统状态
        log_info ""
        log_info "调用 /bas/get_status 服务获取系统状态..."
        ros2 service call /bas/get_status std_srvs/srv/Trigger 2>/dev/null
    else
        log_warn "bas_control_node 未运行"
        return 1
    fi
}

# 显示帮助信息
show_help() {
    echo "bas_control_node 启动脚本"
    echo ""
    echo "使用方法:"
    echo "  $0 [选项]"
    echo ""
    echo "选项:"
    echo "  start [--daemon]           启动进程（可选后台模式）"
    echo "  stop                       停止进程（优雅停止）"
    echo "  restart                    重启进程"
    echo "  status                     查看进程状态"
    echo "  check                      检查进程是否运行"
    echo "  --log-level <level>        设置日志级别 (debug/info/warn/error)"
    echo "  --help                     显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 start                   # 前台启动"
    echo "  $0 start --daemon          # 后台启动"
    echo "  $0 stop                    # 停止进程"
    echo "  $0 restart                 # 重启进程"
    echo "  $0 status                  # 查看状态"
    echo "  $0 start --log-level debug # 启动并设置日志级别为 debug"
}

# 主函数
main() {
    case "$1" in
        start)
            shift
            start_process "$@"
            ;;
        stop)
            stop_process
            ;;
        restart)
            restart_process
            ;;
        status)
            show_status
            ;;
        check)
            if check_process; then
                log_info "✅ bas_control_node 正在运行"
                get_process_info
                exit 0
            else
                log_warn "❌ bas_control_node 未运行"
                exit 1
            fi
            ;;
        --help|-h)
            show_help
            ;;
        "")
            # 默认：检查并启动
            if check_process; then
                log_info "bas_control_node 已在运行中"
                get_process_info
            else
                log_info "bas_control_node 未运行，正在启动..."
                start_process --daemon
            fi
            ;;
        *)
            log_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@"
