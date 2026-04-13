# AI识别坐标数据
string object_class                      # 识别物体类别
float32[3] position                      # 3D坐标位置(x,y,z)
float32[4] orientation                   # 四元数方向
float32 confidence                       # 识别置信度
string frame_id                          # 坐标系标识
builtin_interfaces/Time stamp            # 时间戳

