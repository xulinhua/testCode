# 料盒的检测信息（包含2D边界框和3D位姿）
vision_msgs/Detection2D cassette_det

# 料盒的孔洞列表（每个孔洞包含2D边界框和3D位姿）
vision_msgs/Detection2D[] holes_dets

# 料盒的唯一标识符（可选）
string cassette_id