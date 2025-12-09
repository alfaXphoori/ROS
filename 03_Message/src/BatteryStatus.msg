# BatteryStatus.msg
# Message for monitoring robot battery status

string battery_id           # Unique battery identifier (e.g., "BAT-001")
float32 voltage             # Battery voltage in volts (e.g., 12.0-14.8V)
float32 current             # Current draw in amps (e.g., 0.0-10.0A)
uint8 charge_percentage     # Charge level 0-100%
string health_status        # Battery health: "Good", "Fair", "Poor", "Critical"
uint16 time_remaining       # Estimated time remaining in minutes
bool is_charging            # True if battery is currently charging
int64 timestamp             # Unix timestamp of reading
