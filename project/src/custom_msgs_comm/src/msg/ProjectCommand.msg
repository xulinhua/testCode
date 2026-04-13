# Project control command message
uint8 command_type                        # Command type
string project_name                       # Project name

# Command type definitions
uint8 PROJECT_START=0                     # Start project
uint8 PROJECT_STOP=1                      # Stop project
uint8 PROJECT_STATUS=2                    # Query status