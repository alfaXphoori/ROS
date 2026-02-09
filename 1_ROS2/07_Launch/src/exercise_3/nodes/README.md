# Exercise 3: Production Monitoring Nodes

Real-world robot nodes designed for production monitoring and failure recovery testing.

## Nodes

### 1. Battery Monitor Node (`battery_monitor_node.py`)
**Purpose:** Critical battery monitoring system  
**Criticality:** HIGH - Auto-restart on failure  
**Features:**
- Real-time voltage, current, temperature monitoring
- State of charge (SoC) tracking
- Battery health and cycle count
- Low/critical battery alerts
- Realistic discharge simulation

**Topics:**
- Publishes: `/battery_status` (String - JSON format)

**Parameters:**
- `robot_id`: Robot identifier (default: 'AMR-BATTERY-001')
- `battery_capacity_ah`: Battery capacity in Amp-hours (default: 100.0)
- `battery_voltage_nominal`: Nominal voltage (default: 48.0V)
- `low_battery_threshold`: Low battery percentage (default: 20.0%)
- `critical_battery_threshold`: Critical level (default: 10.0%)
- `monitor_rate_hz`: Publishing rate (default: 2.0 Hz)
- `simulate_failure`: Enable crash simulation (default: false)

**Real-world application:** Prevents unexpected robot shutdowns, optimizes charging schedules, extends battery life

---

### 2. Navigation Controller Node (`navigation_controller_node.py`)
**Purpose:** Safety-critical navigation and collision avoidance  
**Criticality:** CRITICAL - System shutdown on failure  
**Features:**
- Real-time position tracking
- Path planning and obstacle avoidance
- LIDAR integration simulation
- Emergency stop capability
- Safety radius monitoring

**Topics:**
- Publishes: `/navigation_status` (String - JSON format)

**Services:**
- `/navigation_command` (SetBool) - Start/stop navigation

**Parameters:**
- `robot_id`: Robot identifier (default: 'AMR-NAV-001')
- `max_speed_ms`: Maximum speed in m/s (default: 2.5)
- `safety_radius_m`: Safety bubble radius (default: 0.75m)
- `lidar_range_m`: LIDAR detection range (default: 20.0m)
- `status_rate_hz`: Publishing rate (default: 1.0 Hz)
- `simulate_failure`: Enable crash simulation (default: false)

**Real-world application:** Core safety system - failure could cause collisions or injuries, requires immediate shutdown

---

### 3. Task Processor Node (`task_processor_node.py`)
**Purpose:** Warehouse task queue management  
**Criticality:** NON-CRITICAL - Log only on failure  
**Features:**
- Task queue processing (pick, pack, transport)
- Performance metrics tracking
- Success/failure rate monitoring
- Dynamic task assignment
- Priority handling

**Topics:**
- Publishes: `/task_status` (String - JSON format)

**Parameters:**
- `robot_id`: Robot identifier (default: 'AMR-TASK-001')
- `robot_type`: Robot type (default: 'picker')
- `zone_id`: Operating zone (default: 'WAREHOUSE-PICKING-AREA')
- `max_tasks_per_hour`: Maximum task throughput (default: 50)
- `process_rate_hz`: Processing rate (default: 0.5 Hz)
- `simulate_failure`: Enable crash simulation (default: false)

**Real-world application:** Order fulfillment and inventory management - failure doesn't impact safety, can be restarted

---

## Usage Examples

### Run Battery Monitor
```bash
ros2 run ce_robot 07_battery_monitor --ros-args \
  -p robot_id:=AMR-BATTERY-001 \
  -p battery_capacity_ah:=100.0 \
  -p monitor_rate_hz:=2.0 \
  -p simulate_failure:=false
```

### Run Navigation Controller
```bash
ros2 run ce_robot 07_navigation_controller --ros-args \
  -p robot_id:=AMR-NAV-001 \
  -p max_speed_ms:=2.5 \
  -p safety_radius_m:=0.75 \
  -p simulate_failure:=false
```

### Run Task Processor
```bash
ros2 run ce_robot 07_task_processor --ros-args \
  -p robot_id:=AMR-TASK-001 \
  -p robot_type:=picker \
  -p max_tasks_per_hour:=50 \
  -p simulate_failure:=false
```

### Test with Simulated Failures
```bash
# Battery monitor - will crash after 20 iterations (auto-restart)
ros2 run ce_robot 07_battery_monitor --ros-args -p simulate_failure:=true

# Navigation - will crash after 15 iterations (critical shutdown)
ros2 run ce_robot 07_navigation_controller --ros-args -p simulate_failure:=true

# Task processor - will crash after 25 iterations (log only)
ros2 run ce_robot 07_task_processor --ros-args -p simulate_failure:=true
```

---

## Integration with Event Handlers

These nodes are designed to work with Exercise 3's event handler launch file:

- **Battery Monitor**: Auto-restarts up to 3 times, then escalates to maintenance alert
- **Navigation Controller**: Critical failure triggers immediate system shutdown
- **Task Processor**: Failure is logged but system continues operating

---

## Production Monitoring Features

All nodes publish JSON-formatted status messages that include:
- Robot identification (ID, type, zone)
- Timestamps
- System metrics (voltage, position, task count)
- Health indicators
- Alert/warning flags

This data integrates with the `failure_counter.py` production monitoring system for:
- Persistent failure tracking
- Escalation policies
- Maintenance alerts
- Fleet-wide analytics

---

## Node Executable Names

Add to `setup.py`:
```python
entry_points={
    'console_scripts': [
        '07_battery_monitor = ce_robot.battery_monitor_node:main',
        '07_navigation_controller = ce_robot.navigation_controller_node:main',
        '07_task_processor = ce_robot.task_processor_node:main',
    ],
},
```
