# RobotStatusLaunch.msg
# Comprehensive robot status message for multi-robot fleet management
# Real-world use: Fleet monitoring and coordination systems

# Robot identification
string robot_id              # Unique robot identifier (e.g., "AMR-TRANSPORT-001")
string robot_type            # Robot type: transport, picker, sorter
string name_space            # ROS namespace this robot operates in (e.g., "/robot1")
int32 fleet_number           # Fleet number (1-N)

# Location and zone
string zone_id               # Current operating zone (e.g., "WAREHOUSE-A-DOCK")
string current_location      # Specific location (e.g., "AISLE-B-12-SHELF-3")

# Operational status
string operational_status    # OPERATIONAL, IDLE, CHARGING, MAINTENANCE, CRITICAL_BATTERY
string assigned_task         # Current task assignment
string assigned_operator     # Operator/supervisor ID (or "AUTO")

# Robot capabilities and limits
float64 max_payload_kg       # Maximum payload capacity (kg)
float64 current_payload_kg   # Current payload weight (kg)

# Battery and power
float64 battery_level        # Battery percentage (0-100)
float64 battery_voltage      # Battery voltage (V)
bool charging                # True if currently charging

# Performance metrics
float64 operation_hours      # Total operational hours
int32 tasks_completed        # Total tasks completed today
int32 errors_count           # Error count since last reset

# Safety and certification
bool safety_certified        # Safety certification status
bool emergency_stop          # Emergency stop activated
int32 priority_level         # Task priority (1=low, 10=critical)

# Timestamp
builtin_interfaces/Time timestamp   # Message timestamp
