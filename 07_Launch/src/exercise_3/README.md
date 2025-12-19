# Exercise 3: Event Handlers & Production Monitoring

Real-world robot system with intelligent failure recovery, production-grade monitoring, and safety-critical shutdown policies.

## Overview

This exercise demonstrates production-ready robot systems used in 24/7 warehouse operations. You'll learn:
- Event-driven node monitoring with `OnProcessStart` and `OnProcessExit`
- Intelligent failure recovery with escalation policies
- Persistent failure tracking for maintenance analytics
- Critical vs non-critical node handling
- Production deployment with systemd integration

## Directory Structure

```
exercise_3/
├── nodes/
│   ├── battery_monitor_node.py          # HIGH criticality - auto-restart
│   ├── navigation_controller_node.py    # CRITICAL - shutdown on failure
│   ├── task_processor_node.py          # NON-CRITICAL - log only
│   └── README.md
├── launch/
│   └── monitored_system_launch.py       # Event handler launch file
├── config/
│   ├── failure_counter.py               # Persistent failure tracking
│   ├── robot_monitoring.yaml            # Configuration file
│   └── robot_monitor.service            # Systemd service file
└── README.md
```

## Node Criticality Levels

### 🔴 CRITICAL: Navigation Controller
- **Purpose:** Safety-critical path planning and collision avoidance
- **Failure Policy:** Immediate system shutdown
- **Reason:** Failure could cause collisions or injuries
- **Real-world:** Like brakes in a car - must work or stop everything

### 🟡 HIGH: Battery Monitor
- **Purpose:** Battery voltage, current, temperature monitoring
- **Failure Policy:** Auto-restart up to 3 times, then escalate
- **Reason:** Critical data but can recover from transient failures
- **Real-world:** Like fuel gauge - important but not immediately dangerous

### 🟢 NON-CRITICAL: Task Processor
- **Purpose:** Warehouse task queue (picking, packing, transport)
- **Failure Policy:** Log only, system continues
- **Reason:** Business operations can pause without safety risk
- **Real-world:** Like order processing - important for business but not safety

## Quick Start

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot ce_robot_launch --symlink-install
source install/setup.bash
```

### 2. Launch with Default Settings

```bash
ros2 launch ce_robot_launch monitored_system_launch.py
```

### 3. Test with Simulated Failures

```bash
# Simulate all failures for testing
ros2 launch ce_robot_launch monitored_system_launch.py simulate_failures:=true

# Battery monitor will crash after ~20 seconds → auto-restart
# Navigation will crash after ~15 seconds → system shutdown
# Task processor will crash after ~25 seconds → log only
```

### 4. Monitor Failure Logs

```bash
# View real-time failure counts
cat /tmp/robot_failures/failure_counts.json

# View failure log
tail -f /tmp/robot_failures/failure_log.txt
```

## Event Handler Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    BATTERY MONITOR                          │
├─────────────────────────────────────────────────────────────┤
│  OnProcessStart → ✅ "Battery Monitor started"              │
│  OnProcessExit  → ⚠️ "Monitor crashed!"                     │
│                   ├─ Attempt 1: Restart in 3s              │
│                   ├─ Attempt 2: Restart in 3s              │
│                   ├─ Attempt 3: Restart in 3s              │
│                   └─ Attempt 4: 🚨 ALERT MAINTENANCE        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                 NAVIGATION CONTROLLER                       │
├─────────────────────────────────────────────────────────────┤
│  OnProcessStart → ✅ "Navigation Controller started"        │
│                   ✅ "Safety systems online"                │
│  OnProcessExit  → ❌ "CRITICAL FAILURE!"                    │
│                   🚨 "SAFETY ALERT"                         │
│                   🛑 "EMERGENCY STOP"                       │
│                   └─ EmitEvent(Shutdown)                    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                    TASK PROCESSOR                           │
├─────────────────────────────────────────────────────────────┤
│  OnProcessStart → ✅ "Task Processor started"               │
│  OnProcessExit  → ⚠️ "Task Processor exited"                │
│                   📋 "Order processing paused"              │
│                   ✅ Robot continues safety operations      │
└─────────────────────────────────────────────────────────────┘
```

## Failure Counter System

The `failure_counter.py` provides persistent failure tracking:

```python
from failure_counter import FailureCounter

counter = FailureCounter()

# Increment failure count
count = counter.increment_failure_count('battery_monitor')

# Log failure with details
counter.log_failure('battery_monitor', 'Voltage sensor timeout')

# Check current count
if counter.get_failure_count('battery_monitor') >= 3:
    # Escalate to maintenance

# Reset after recovery
counter.reset_failure_count('battery_monitor')

# View all failures
counter.print_summary()
```

**Data Storage:**
- `/tmp/robot_failures/failure_counts.json` - Per-node failure counts
- `/tmp/robot_failures/failure_log.txt` - Detailed maintenance log

## Production Deployment

### Install as Systemd Service

```bash
# Copy service file
sudo cp config/robot_monitor.service /etc/systemd/system/

# Edit service file with your paths
sudo nano /etc/systemd/system/robot_monitor.service

# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable robot_monitor.service

# Start service
sudo systemctl start robot_monitor.service

# Check status
sudo systemctl status robot_monitor.service

# View logs
sudo journalctl -u robot_monitor.service -f
```

### Environment Variables

Set these in the service file or environment:

```bash
export ROS_DOMAIN_ID=42              # ROS network domain
export ROBOT_ID=AMR-PROD-001         # Robot identifier
export WAREHOUSE_ZONE=MAIN-FLOOR     # Operating zone
```

## Configuration

Edit `config/robot_monitoring.yaml` to customize:

```yaml
robot_monitoring:
  failure_tracking:
    max_restart_attempts: 3          # Attempts before escalation
    restart_delay_seconds: 3.0       # Wait between restarts
  
  critical_nodes:                     # Shutdown on failure
    - navigation_controller
    - safety_system
  
  high_priority_nodes:                # Auto-restart
    - battery_monitor
    - motor_controller
  
  alerts:
    email:
      recipients:
        - "maintenance@warehouse.com"
    sms:
      phone_numbers:
        - "+1-800-ROBOT-911"
```

## Testing Scenarios

### Test 1: Battery Monitor Auto-Restart
```bash
ros2 launch ce_robot_launch monitored_system_launch.py \
  simulate_failures:=true

# Expected: Battery monitor crashes → auto-restarts 3 times → escalates
```

### Test 2: Critical Navigation Failure
```bash
ros2 launch ce_robot_launch monitored_system_launch.py \
  simulate_failures:=true

# Expected: Navigation crashes → immediate system shutdown
```

### Test 3: Non-Critical Task Failure
```bash
ros2 launch ce_robot_launch monitored_system_launch.py \
  simulate_failures:=true

# Expected: Task processor crashes → logged but system continues
```

### Test 4: Disable Auto-Restart
```bash
ros2 launch ce_robot_launch monitored_system_launch.py \
  enable_auto_restart:=false

# Expected: No auto-restart, all failures logged only
```

## Key Concepts

1. **OpaqueFunction** - Execute custom Python functions in event handlers
2. **RegisterEventHandler** - Register callbacks for node lifecycle events
3. **OnProcessExit** - Triggered when a node exits (crash or normal)
4. **OnProcessStart** - Triggered when a node starts successfully
5. **EmitEvent(Shutdown)** - Gracefully shut down the entire launch system
6. **TimerAction** - Delay execution (e.g., restart after 3 seconds)
7. **Persistent Tracking** - JSON-based failure storage for analytics

## Real-World Applications

This exercise simulates actual production systems:

- **Amazon Robotics:** Kiva robots with battery monitoring and auto-recovery
- **Ocado Smart Platform:** Warehouse robots with safety-critical navigation
- **AutoStore:** Grid robots with intelligent failure escalation
- **Boston Dynamics Stretch:** Task queue management with non-critical failure handling

## Maintenance & Analytics

View failure analytics:
```bash
# Summary of all failures
python3 config/failure_counter.py

# JSON data for analytics dashboard
cat /tmp/robot_failures/failure_counts.json

# Failure timeline
grep "battery_monitor" /tmp/robot_failures/failure_log.txt
```

## Integration with Lab_Exercises.md

This reference implementation matches Exercise 3 in `Lab_Exercises.md`:
- Same node types (battery, navigation, task)
- Same criticality levels
- Same event handler patterns
- Production-ready code ready for deployment

## Troubleshooting

**Q: Nodes restart too many times?**
A: Adjust `max_restart_attempts` in launch arguments

**Q: Want to test without crashes?**
A: Set `simulate_failures:=false` (default)

**Q: Failure counter not working?**
A: Check `/tmp/robot_failures/` permissions

**Q: Need custom storage location?**
A: Edit `failure_counter.py` storage_dir parameter

## Next Steps

- Integrate with ROS 2 diagnostics
- Add Prometheus metrics export
- Connect to fleet management dashboard
- Implement predictive maintenance ML models
- Deploy across multi-robot fleet
