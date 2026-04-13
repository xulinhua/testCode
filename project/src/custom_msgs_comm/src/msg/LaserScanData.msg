# Laser scan data message
std_msgs/Header header                    # Header with timestamp and frame id
float32 angle_min                         # Start angle of the scan [rad]
float32 angle_max                         # End angle of the scan [rad]
float32 angle_increment                   # Angular distance between measurements [rad]
float32 time_increment                    # Time between measurements [seconds]
float32 scan_time                         # Time between scans [seconds]
float32 range_min                         # Minimum range value [m]
float32 range_max                         # Maximum range value [m]
float32[] ranges                          # Range data [m]
float32[] intensities                     # Intensity data