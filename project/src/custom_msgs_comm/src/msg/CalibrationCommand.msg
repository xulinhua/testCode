# Calibration command message
uint8 command_type                        # Command type
float32[] robot_pose                      # Robot pose [x, y, z, roll, pitch, yaw]

# Command type definitions
uint8 CALIB_START=0                       # Start calibration
uint8 CALIB_NEXT_POINT=1                  # Next calibration point
uint8 CALIB_FINISHED=2                    # Finish calibration