# FleetCommand.msg
# Fleet-wide command message for coordinating multiple robots
# Real-world use: Central fleet management, emergency stops, coordinated operations

# Command identification
string command_id            # Unique command identifier
string issuer                # Who issued the command (operator ID or system)
builtin_interfaces/Time timestamp

# Command type and scope
string command_type          # EMERGENCY_STOP, RESUME, PAUSE, RETURN_TO_BASE, 
                            # ZONE_EVACUATION, MAINTENANCE_MODE, SPEED_LIMIT
string target_scope          # ALL, ZONE, ROBOT_TYPE, SPECIFIC (individual robot)
string target_identifier     # Zone ID, robot type, or specific robot ID

# Command parameters
int32 priority               # Command priority (1=low, 10=critical)
bool immediate               # Execute immediately without confirmation
float64 timeout_seconds      # Command timeout (0 = no timeout)

# Zone-specific commands
string zone_id               # Target zone (if scope = ZONE)
string[] affected_zones      # List of affected zones

# Robot type filter
string robot_type_filter     # Filter by robot type: transport, picker, sorter, ALL

# Speed and movement limits
float64 speed_limit_ms       # Speed limit in m/s (0 = full stop)
float64 max_acceleration     # Maximum acceleration (m/s²)

# Safety and coordination
bool safety_critical         # Safety-critical command
bool requires_confirmation   # Requires robot acknowledgment
int32 expected_responses     # Expected number of robot responses

# Additional information
string reason                # Reason for command (for logging)
string message               # Human-readable message
string[] parameters          # Additional command-specific parameters
