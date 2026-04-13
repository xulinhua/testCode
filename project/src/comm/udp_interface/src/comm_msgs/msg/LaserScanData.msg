# 2D Laser Scan数据消息
float32 angle_min                 # 起始扫描角度(弧度)
float32 angle_max                 # 结束扫描角度(弧度)
float32 angle_increment           # 角度增量(弧度)
float32 time_increment            # 每束扫描时间(秒)
float32 scan_time                 # 扫描周期(秒)
float32 range_min                 # 最小有效距离(米)
float32 range_max                 # 最大有效距离(米)
float32[] ranges                  # 距离数据数组(米)
float32[] intensities             # 强度数据数组
string frame_id                   # 坐标系标识
builtin_interfaces/Time stamp     # 时间戳

