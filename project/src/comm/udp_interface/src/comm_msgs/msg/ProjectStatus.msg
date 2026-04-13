# 项目状态响应
uint8 status  # 状态码
string project_name  # 项目名称
string message  # 附加信息

# 状态枚举
uint8 STATUS_STOPPED=0    # 已停止
uint8 STATUS_RUNNING=1    # 运行中
uint8 STATUS_STARTING=2   # 启动中
uint8 STATUS_STOPPING=3   # 停止中
uint8 STATUS_ERROR=4      # 出错
