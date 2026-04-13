# 点云数据消息
uint32 height                             # 图像高度
uint32 width                              # 图像宽度
uint32 point_step                         # 每个点的字节步长
uint8[] data                              # 点云数据字节流
float32[] camera_intrinsic                # 相机内参矩阵(9元素)
string frame_id                           # 坐标系标识
builtin_interfaces/Time stamp             # 时间戳

