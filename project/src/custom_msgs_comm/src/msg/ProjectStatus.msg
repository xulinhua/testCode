# Project status message
uint8 status                              # Status code
string project_name                      # Project name
string message                           # Status message

# Status code definitions
uint8 STATUS_STOPPED=0                   # Stopped
uint8 STATUS_RUNNING=1                  # Running
uint8 STATUS_STARTING=2                 # Starting
uint8 STATUS_STOPPING=3                  # Stopping
uint8 STATUS_ERROR=4                     # Error