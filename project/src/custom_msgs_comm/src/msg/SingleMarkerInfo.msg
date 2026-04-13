# Single MarkerInfo data message
string object_class                       # Object class
int32 marker_id                           # Marker ID
geometry_msgs/Point center_2d             # 2D center point in pixel coordinates
geometry_msgs/Point position              # 3D position coordinates [x, y, z]
geometry_msgs/Vector3 rotation            # Euler angles in degrees [roll, pitch, yaw]
float32[] rotation_matrix                 # 3x3 rotation matrix (9 elements in row-major order)
float64[] rvec                            # Rotation vector (3 elements)
float64[] tvec                            # Translation vector (3 elements)
geometry_msgs/Point[] corners             # Corner points in pixel coordinates
geometry_msgs/Point[] world_corners       # Corner points in world coordinates
float32 distance                          # Distance to camera
float32 confidence                        # Confidence level
