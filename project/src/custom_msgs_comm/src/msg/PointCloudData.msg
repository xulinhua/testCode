# Point cloud data message
std_msgs/Header header                    # Header with timestamp and frame id
uint32 height                             # Height of the point cloud
uint32 width                              # Width of the point cloud
sensor_msgs/PointField[] fields           # Description of individual point fields
bool is_bigendian                         # Is this data bigendian?
uint32 point_step                         # Length of a point in bytes
uint32 row_step                           # Length of a row in bytes
uint8[] data                              # Actual point data
bool is_dense                             # True if there are no invalid points