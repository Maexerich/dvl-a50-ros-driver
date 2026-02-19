# Water Linked DVL A50 ROS Driver

## Overview
This node (`a50_pub`) interfaces with the Water Linked DVL A50 via TCP. It performs dead-reckoning and velocity parsing, publishing both standard ROS messages and custom DVL-specific messages.

## Usage
`rosrun dvl_a50_ros_driver publisher.py _ip:=192.168.1.95`

## Interface

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `dvl/data` | `dvl_a50_ros_driver/DVL` | Full velocity report including raw beam data, covariance, and status. |
| `dvl/estimate` | `dvl_a50_ros_driver/DVLEstimate` | **Legacy**. Internal dead-reckoning position/orientation (custom msg). |
| `/dvl/odom` | `nav_msgs/Odometry` | Standard odometry message derived from DVL dead-reckoning. Frame: `dvl_odom` -> `dvl_link`. |
| `dvl/json_data` | `std_msgs/String` | Raw JSON output from the sensor (enabled via `~do_log_raw_data`). |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `dvl/send_command` | `std_msgs/String` | Accepts JSON commands (e.g., reset dead reckoning, calibrate gyro). |

### Command Logging
Commands sent to `dvl/send_command` are logged. The driver listens for JSON responses on the TCP connection and publishes them to `dvl/json_data` if `~do_log_raw_data` is true. The `dvl_mission_control.py` script uses this to verify command execution (e.g., `reset_dead_reckoning`).

### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `~ip` | "10.42.0.186" | DVL IP address. |
| `~port` | 16171 | TCP port. |
| `~do_log_raw_data`| `False` | Enable publishing to `dvl/json_data`. |
| `~speed_of_sound` | 1480 | Sound speed in m/s. |
| `~mounting_rotation_offset` | 0 | Rotation offset in degrees. |
| `~acoustic_enabled` | `True` | Enable acoustic transmission. |
| `~dark_mode_enabled` | `False` | Disable status LEDs. |
| `~range_mode` | "auto" | Range setting ("auto" or fixed). |

## Implementation Details
*   **Timestamp Synchronization**: The node calculates a time offset upon receiving the first packet to align the unsynchronized sensor timestamp with `rospy.Time.now()`. All subsequent messages use this constant offset + sensor delta.
*   **Coordinate Frames**: 
    *   Velocity data is in `dvl_link` frame.
    *   Odometry is published in `dvl_odom` fixed frame.
