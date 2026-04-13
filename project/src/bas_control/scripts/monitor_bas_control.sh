#!/bin/bash
#
# @file monitor_bas_control.sh
# @brief bas_control_node 自动监控脚本（守护进程）
# 
# 功能说明：
# 定期检查 bas_control_node 是否在运行，如果未运行则自动启动
# 适用于生产环境，确保系统持续可用
#
# 使用方法：
# ./monitor_bas_control.sh          # 前台运行
# ./monitor_bas_control.sh --daemon # 后台运行
# ./monitor_bas_control.sh --stop   # 停止监控
#
# 配置项：
# MONITOR_INTERVAL   - 监控间隔（秒），默认10秒
# MAX_RESTART_COUNT  - 最大重启次数（每小时），默认5次
# LOG_FILE           - 日志文件路径
#
# 作者：bas_control 开发团队
# 日期：2024
#

# 配置项
MONITOR_INTERVAL=${MONITOR_INTERVAL:-10}       # 监控间隔（秒）
MAX_RESTART_COUNT=${MAX_RESTART_COUNT:-5}       # 每小时最大重启次数
LOG_FILE=${LOG_FILE:-"/var/log/bas/monitor.log"}  # 日志文件
PID_FILE="/var/run/bas_control_monitor.pid"     # PID文件

# 重启计数器（用于防止无限重启）
restart_count=0
last_restart_time=0

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 日志函数
log() {
    local level=$1
    shift
    local message="$@"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    # 输出到控制台
    case $level in
        INFO)
            echo -e "${GREEN}[$timestamp] [INFO]${NC} $message"
            ;;
        WARN)
            echo -e "${YELLOW}[$timestamp] [WARN]${NC} $message"
            ;;
        ERROR)
            echo -e "${RED}[$timestamp] [ERROR]${NC} $message"
            ;;
        *)
            echo "[$timestamp] [$level] $message"
            ;;
    esac
    
    # 输出到日志文件
    if [ -d "$(dirname $LOG_FILE)" ]; then
        echo "[$timestamp] [$level] $message" >> "$LOG_FILE"
    fi
}

# 检查 bas_control_node 是否在运行
check_process() {
    pgrep -f "bas_control_node" > /dev/null 2>&1
    return $?
}

# 启动 bas_control_node
start_process() {
    log INFO "正在启动 bas_control_node..."
    
    # Source 环境
    source /opt/ros/humble/setup.bash 2>/dev/null
    source ~/ros2_ws/install/setup.bash 2>/dev/null
    
    # 后台启动
    ros2 run bas_control bas_control_node > /var/log/bas/bas_control.log 2>&1 &
    local pid=$!
    
    sleep 2
    
    if check_process; then
        log INFO "✅ bas_control_node 启动成功 (PID: $pid)"
        return 0
    else
        log ERROR "❌ bas_control_node 启动失败"
        return 1
    fi
}

# 停止监控脚本
stop_monitor() {
    if [ -f "$PID_FILE" ]; then
        local pid=$(cat "$PID_FILE")
        if kill -0 $pid 2>/dev/null; then
            log INFO "正在停止监控脚本 (PID: $pid)..."
            kill $pid
            rm -f "$PID_FILE"
            log INFO "✅ 监控脚本已停止"
        else
            log WARN "监控脚本未运行，清理 PID 文件"
            rm -f "$PID_FILE"
        fi
    else
        log WARN "未找到 PID 文件，监控脚本可能未运行"
    fi
}

# 检查重启频率（防止无限重启）
check_restart_frequency() {
    local current_time=$(date +%s)
    local time_diff=$((current_time - last_restart_time))
    
    # 如果距离上次重启超过1小时，重置计数器
    if [ $time_diff -gt 3600 ]; then
        restart_count=0
    fi
    
    # 检查是否超过最大重启次数
    if [ $restart_count -ge $MAX_RESTART_COUNT ]; then
        log WARN "重启次数超过限制 ($MAX_RESTART_COUNT 次/小时)，暂停自动重启"
        log WARN "请检查系统是否存在严重问题"
        return 1
    fi
    
    return 0
}

# 监控主循环
monitor_loop() {
    log INFO "========================================="
    log INFO "bas_control_node 监控脚本启动"
    log INFO "========================================="
    log INFO "监控间隔: ${MONITOR_INTERVAL} 秒"
    log INFO "最大重启次数: ${MAX_RESTART_COUNT} 次/小时"
    log INFO "日志文件: $LOG_FILE"
    log INFO "========================================="
    
    # 写入 PID 文件
    echo $$ > "$PID_FILE"
    
    while true; do
        # 检查进程是否运行
        if check_process; then
            # 进程正常运行，无操作
            :  # 空操作
        else
            # 进程未运行，尝试启动
            log WARN "⚠️  bas_control_node 未运行"
            
            # 检查重启频率
            if check_restart_frequency; then
                start_process
                
                if [ $? -eq 0 ]; then
                    restart_count=$((restart_count + 1))
                    last_restart_time=$(date +%s)
                    log INFO "重启计数: $restart_count / $MAX_RESTART_COUNT (本小时内)"
                else
                    log ERROR "重启失败，将在 ${MONITOR_INTERVAL} 秒后重试"
                fi
            fi
        fi
        
        # 等待下次检查
        sleep $MONITOR_INTERVAL
    done
}

# 显示帮助信息
show_help() {
    echo "bas_control_node 自动监控脚本"
    echo ""
    echo "使用方法:"
    echo "  $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --daemon    后台运行（守护进程模式）"
    echo "  --stop      停止监控脚本"
    echo "  --status    查看监控脚本状态"
    echo "  --help      显示帮助信息"
    echo ""
    echo "环境变量配置:"
    echo "  MONITOR_INTERVAL   监控间隔（秒），默认10"
    echo "  MAX_RESTART_COUNT  每小时最大重启次数，默认5"
    echo "  LOG_FILE           日志文件路径"
    echo ""
    echo "示例:"
    echo "  $0                          # 前台运行监控"
    echo "  $0 --daemon                 # 后台运行监控"
    echo "  $0 --stop                   # 停止监控"
    echo "  MONITOR_INTERVAL=5 $0       # 5秒检查一次"
}

# 查看状态
show_status() {
    if [ -f "$PID_FILE" ]; then
        local pid=$(cat "$PID_FILE")
        if kill -0 $pid 2>/dev/null; then
            echo "✅ 监控脚本正在运行 (PID: $pid)"
            echo "   PID 文件: $PID_FILE"
            echo "   日志文件: $LOG_FILE"
            
            # 显示最近的日志
            echo ""
            echo "最近的日志:"
            tail -n 10 "$LOG_FILE" 2>/dev/null || echo "   (无日志)"
        else
            echo "❌ 监控脚本未运行（PID 文件存在但进程不存在）"
            rm -f "$PID_FILE"
        fi
    else
        echo "❌ 监控脚本未运行"
    fi
}

# 主函数
main() {
    case "$1" in
        --daemon)
            # 后台运行
            nohup "$0" > /dev/null 2>&1 &
            log INFO "监控脚本已启动为后台进程"
            ;;
        --stop)
            stop_monitor
            ;;
        --status)
            show_status
            ;;
        --help|-h)
            show_help
            ;;
        "")
            # 前台运行（默认）
            monitor_loop
            ;;
        *)
            echo "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

# 捕获退出信号
trap 'log INFO "收到退出信号，停止监控..."; rm -f "$PID_FILE"; exit 0' SIGINT SIGTERM

# 执行主函数
main "$@"
