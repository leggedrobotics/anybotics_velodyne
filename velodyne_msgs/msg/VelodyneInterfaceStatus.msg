# Velodyne LIDAR interface status.
# Status levels
uint8 OK = 0
uint8 WARN = 1
uint8 ERROR = 2

## Message
Header           header                             # standard ROS message header
uint8            interface_status                   # Interface status level
std_msgs/String  interface_status_message           # Additional feedback on the interface status
