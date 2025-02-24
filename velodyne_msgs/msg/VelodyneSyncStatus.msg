# Velodyne LIDAR time sync status.
## Constants (status codes).
## Following Velodyne's documentation - 0 = No PPS detected, 1 = Syncing to PPS, 2 = PPS locked, 3 = Error
uint8 PPS_UNKNOWN = 0
uint8 PPS_SYNCING = 1
uint8 PPS_LOCKED = 2
uint8 PPS_ERROR = 3

## Message
Header           header                             # standard ROS message header
std_msgs/Time    lidar_timestamp                    # Parsed lidar internal timestamp
int32            current_delay_usecs                # Calculated delay between the current time and the lidar polled time
uint8            pps_lock                           # PPS lock status. This field is expected to take a value from the PPS_XYZ constants previously declared.
time             last_successful_pulse_timestamp    # Timestamp when the last pulse was emitted successfully
bool             last_pulse_emitted                 # True if the last pulse was successfully emitted. Pulse will not be sent if the delay is higher than the threshold
uint32           last_pulse_delay_ms                # Delay of the last pulse emitted. Units = ms
uint32           max_delay_threshold                # Current maximum allowed delay for emitting the pulse. Units = ms
uint32           stat_window_size                   # Number of samples currently taken into account for the windowed stats
float32          average_window_delay_ms            # Mean delay of the pulse emission during the last N cycles (N = window size). Units = ms
float32          std_dev_window_delay_ms            # Standard deviation of the delay of the pulse emission during the last N cycles (N = window size). Units = ms
int32            delayed_pulse_counter              # Number of delayed (over the max_delay_threshold) pulses since startup.