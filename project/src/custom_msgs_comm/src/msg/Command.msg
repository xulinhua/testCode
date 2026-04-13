# DDS command message format
# Command type enumeration
uint8 START=1
uint8 STOP=2
uint8 STATUS=3
uint8 PAUSE=4
uint8 RESUME=5

# Project type enumeration
uint8 HAND_EYE_CALIB=1
uint8 CAMERA=2
uint8 PCL2LASERSCAN=3
uint8 YOLO_DET=4

uint8 command_type        # Command type
uint8 project_type        # Project type
string project_name       # Project name
string data               # Additional data (JSON format)