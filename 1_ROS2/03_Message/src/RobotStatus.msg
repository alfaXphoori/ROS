# RobotStatus.msg
# Comprehensive status message for autonomous mobile robot

# Header
std_msgs/Header header          # Timestamp and frame_id

# Robot Identification
string robot_id                 # Unique robot identifier (e.g., "AMR-001")
string robot_name               # Human-readable name (e.g., "Warehouse Bot Alpha")

# Position and Pose (in meters and radians)
float64 position_x              # X coordinate in map frame
float64 position_y              # Y coordinate in map frame
float64 orientation_z           # Yaw angle (rotation around Z axis)

# Velocity (m/s and rad/s)
float32 linear_velocity         # Forward/backward speed
float32 angular_velocity        # Rotation speed

# Battery Status
float32 battery_voltage         # Battery voltage (V)
uint8 battery_percentage        # Battery level 0-100%
bool is_charging                # Charging status
uint16 battery_time_remaining   # Estimated minutes remaining

# Safety Status
bool emergency_stop             # Emergency stop engaged
bool obstacle_detected          # Obstacle in path
float32 obstacle_distance       # Distance to nearest obstacle (meters)

# Mission Status
string current_mission          # Current task (e.g., "PICKING", "DELIVERING", "IDLE")
uint8 mission_progress          # Mission completion 0-100%
string mission_status           # Status: "IN_PROGRESS", "COMPLETED", "FAILED", "PAUSED"

# System Health
float32 cpu_usage               # CPU usage percentage
float32 temperature             # System temperature (Celsius)
bool motors_enabled             # Motor system status
string health_status            # Overall: "HEALTHY", "WARNING", "ERROR", "CRITICAL"
