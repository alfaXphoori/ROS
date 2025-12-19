# Exercise 4: Quality Control Inspection System

Production-grade ROS2 event handler implementation for manufacturing quality control with camera-based defect detection, multi-sensor fusion, and automated reporting.

## Overview

This exercise demonstrates intelligent failure recovery for a **24/7 manufacturing quality inspection system** with differentiated criticality levels:

- **CRITICAL** (Vision Inspector): Camera failure stops production - defects must not pass
- **HIGH** (Sensor Fusion): Auto-restart up to 3 times - improves accuracy
- **NON-CRITICAL** (Report Generator): Logs failures but continues inspection

## Real-World Scenario

**Manufacturing Quality Control Station:**
- Vision system inspects products for defects at 1 Hz
- Multi-sensor fusion validates dimensions, weight, temperature
- Automated reports track quality metrics and compliance
- Critical camera failure halts production line
- Sensor degradation triggers calibration alerts
- Database issues pause reporting but not inspection

## Directory Structure

```
exercise_4/
├── nodes/
│   ├── vision_inspector_node.py      # CRITICAL: Camera-based defect detection
│   ├── sensor_fusion_node.py         # HIGH: Multi-sensor data fusion
│   ├── report_generator_node.py      # NON-CRITICAL: Quality reporting
│   └── README.md
├── launch/
│   └── qc_inspection_launch.py       # Event handler launch file
├── config/
│   ├── failure_counter.py            # Persistent failure tracking
│   ├── qc_inspection.yaml            # System configuration
│   └── qc_inspection.service         # Systemd service file
└── README.md (this file)
```

## Nodes

### 1. Vision Inspector (`07_vision_inspector`)
**Criticality:** CRITICAL

Camera-based defect detection system that identifies scratches, dents, color mismatches, dimensional errors, and missing components.

**Why Critical:** Missing defects could lead to product recalls, customer complaints, and brand damage.

**Failure Behavior:** Immediate production line shutdown

**Key Metrics:**
- Items inspected: 3600/hour
- Defect detection rate: ~10%
- Inspection accuracy: 98.5%
- Camera health monitoring
- Lighting and focus quality

### 2. Sensor Fusion (`07_sensor_fusion`)
**Criticality:** HIGH

Multi-sensor data fusion combining thermal camera, ultrasonic, weight sensor, and laser scanner for comprehensive quality validation.

**Why High Priority:** Enhances inspection accuracy but vision system can operate independently.

**Failure Behavior:** Auto-restart up to 3 times, then escalate to maintenance

**Sensors:**
- Thermal camera (temperature, hotspots)
- Ultrasonic (surface quality, distance)
- Weight sensor (precision measurement)
- Laser scanner (3D dimensional accuracy)

### 3. Report Generator (`07_report_generator`)
**Criticality:** NON-CRITICAL

Automated report generation for hourly summaries, shift reports, defect analysis, trend tracking, and compliance audits.

**Why Non-Critical:** Inspection can continue without real-time reporting. Data buffers to temporary storage.

**Failure Behavior:** Log only, continue operation

**Report Types:**
- Hourly production summaries
- Shift performance reports
- Defect analysis and breakdown
- Quality trend predictions
- Compliance audit trails

## Event Handler Features

### OpaqueFunction Implementation

```python
def handle_vision_exit(event, context):
    """CRITICAL failure - immediate shutdown"""
    # Vision failure is a quality risk
    # Defective products must not pass
    return [
        LogInfo(msg='❌ CRITICAL: Vision Inspector failed!'),
        LogInfo(msg='🛑 EMERGENCY STOP: Halting production line'),
        EmitEvent(event=Shutdown(reason='Critical quality system failure'))
    ]

def handle_fusion_exit(event, context):
    """HIGH priority - intelligent restart with escalation"""
    count = failure_counter.increment_failure_count('sensor_fusion')
    
    if count >= 3:
        # Escalate after 3 failures
        return [
            LogInfo(msg='📧 Email sent to: sensors@factory.com'),
            EmitEvent(event=Shutdown(reason='Persistent sensor failure'))
        ]
    else:
        # Auto-restart with 3-second delay
        return [TimerAction(period=3.0, actions=[restart_node])]

def handle_report_exit(event, context):
    """NON-CRITICAL - log only"""
    return [
        LogInfo(msg='📋 Reports suspended - inspection continues'),
        LogInfo(msg='💾 Data buffering to temporary storage')
    ]
```

### Persistent Failure Tracking

```python
# failure_counter.py - QCFailureCounter class
{
  "vision_inspector": {
    "count": 0,
    "first_failure": "2025-12-19T10:30:00",
    "last_failure": null,
    "robot_id": "QC-SYSTEM-001"
  },
  "sensor_fusion": {
    "count": 2,
    "first_failure": "2025-12-19T11:15:00",
    "last_failure": "2025-12-19T11:45:00",
    "robot_id": "QC-SYSTEM-001"
  }
}
```

## Configuration

### qc_inspection.yaml

```yaml
critical_nodes:
  - vision_inspector  # Camera failure stops production

high_priority_nodes:
  - sensor_fusion  # Auto-restart with limits

non_critical_nodes:
  - report_generator  # Log only

quality_thresholds:
  vision_camera:
    min_inspection_accuracy: 96.0
    defect_threshold: 0.85
  sensor_fusion:
    min_sensor_health: 90.0
    min_data_quality: 95.0
```

## Quick Start

### Build and Install

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

### Test Individual Nodes

**Vision Inspector:**
```bash
ros2 run ce_robot 07_vision_inspector --ros-args \
  -p inspection_rate_hz:=2.0 \
  -p defect_threshold:=0.90

# Monitor output
ros2 topic echo /vision_status --once
```

**Sensor Fusion:**
```bash
ros2 run ce_robot 07_sensor_fusion --ros-args \
  -p fusion_rate_hz:=2.0 \
  -p sensor_count:=4

# Monitor output
ros2 topic echo /fusion_status --once
```

**Report Generator:**
```bash
ros2 run ce_robot 07_report_generator --ros-args \
  -p report_rate_hz:=0.5 \
  -p report_format:=JSON

# Monitor output
ros2 topic echo /report_status --once
```

### Launch Complete System

```bash
ros2 launch ce_robot_launch qc_inspection_launch.py
```

**Expected output:**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔍 Quality Control Inspection System Starting...
📊 Failure tracking: /path/to/config/failure_counter.py
🚨 Critical nodes: vision_inspector
🔄 Auto-restart enabled: sensor_fusion
📋 Non-critical: report_generator
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ Vision Inspector started successfully
🎥 Camera system online - quality inspection active
✅ Sensor Fusion started successfully
🔬 Multi-sensor analysis active
✅ Report Generator started successfully
📊 Quality reporting active
```

## Testing Scenarios

### Scenario 1: Vision Failure (CRITICAL)

```bash
# Terminal 1: Launch system
ros2 launch ce_robot_launch qc_inspection_launch.py simulate_failures:=true

# Wait 15 cycles for vision to crash
```

**Expected:**
```
❌ CRITICAL: Vision inspection system crashed - QUALITY RISK!
❌ CRITICAL: Vision Inspector failed!
🚨 QUALITY ALERT: Defect detection offline
🛑 EMERGENCY STOP: Halting production line
📞 Emergency contact: QC Manager +1-800-QUALITY
⚠️ Defective products may have passed inspection!
[Immediate system shutdown]
```

### Scenario 2: Sensor Fusion Auto-Restart (HIGH)

```bash
# Terminal 1: Launch
ros2 launch ce_robot_launch qc_inspection_launch.py simulate_failures:=true

# Wait 20 cycles for sensor fusion to crash
```

**Expected (3 restart attempts):**
```
⚠️ Sensor Fusion exited! Attempt 1/3. Restarting in 3 seconds...
📊 Failure logged with sensor tracking ID
[3 second delay, then restart]
⚠️ Sensor Fusion exited! Attempt 2/3. Restarting in 3 seconds...
[continues until 3 failures]
❌ CRITICAL: Sensor fusion failed 3 times! Manual intervention required.
📧 Email sent to: sensors@factory.com
📱 SMS alert sent to calibration technician
[System shutdown after escalation]
```

### Scenario 3: Report Generator Failure (NON-CRITICAL)

```bash
# Terminal 1: Launch
ros2 launch ce_robot_launch qc_inspection_launch.py simulate_failures:=true

# Wait 25 cycles for report generator to crash
```

**Expected (no restart, continues):**
```
⚠️ Report Generator exited (non-critical)
📋 Reports suspended - inspection continues
💾 Data buffering to temporary storage
[Vision and sensor fusion continue running normally]
```

### Scenario 4: Check Failure Tracking

```bash
# View failure counts
cat /tmp/qc_failures/failure_counts.json

# View failure log
tail -f /tmp/qc_failures/failure_log.txt
```

## Production Deployment

### Systemd Service Installation

```bash
# Copy service file
sudo cp config/qc_inspection.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable qc_inspection.service

# Start service
sudo systemctl start qc_inspection.service

# Check status
sudo systemctl status qc_inspection.service
```

### Environment Variables

```bash
export ROS_DOMAIN_ID=42
export ROBOT_ID=QC-SYSTEM-001
export PRODUCTION_LINE=MAIN-LINE-A
export QC_STATION=STATION-1
```

## Key Learning Outcomes

1. **Differentiated Criticality**: Vision is CRITICAL (safety), sensors are HIGH (quality), reports are NON-CRITICAL (convenience)

2. **Intelligent Failure Recovery**: 
   - Critical nodes shutdown immediately
   - High-priority nodes auto-restart with limits
   - Non-critical nodes log and continue

3. **OpaqueFunction Usage**: Dynamic restart logic based on failure count and node type

4. **Persistent Tracking**: JSON-based failure counter survives restarts for maintenance analytics

5. **Production Ready**: 24/7 operation with systemd, environment config, and monitoring

## Comparison with Exercise 3

| Aspect | Exercise 3 (Warehouse) | Exercise 4 (QC Inspection) |
|--------|------------------------|----------------------------|
| Domain | Warehouse robotics | Manufacturing QC |
| Critical Node | Navigation (safety) | Vision (quality) |
| High Priority | Battery monitor | Sensor fusion |
| Non-Critical | Task processor | Report generator |
| Failure Storage | /tmp/robot_failures/ | /tmp/qc_failures/ |
| Key Risk | Collision | Defects pass inspection |

## Troubleshooting

**Vision camera degradation:**
```bash
# Check camera health
ros2 topic echo /vision_status | grep camera_health

# Recalibrate if health < 85%
```

**Sensor disagreement:**
```bash
# Check sensor alignment
ros2 topic echo /fusion_status | grep sensor_agreement

# Recalibrate if agreement < 85%
```

**Database issues:**
```bash
# Check storage
ros2 topic echo /report_status | grep storage_used_mb

# Archive old reports if > 800 MB
```

## Additional Resources

- [ROS2 Launch Event Handlers](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
- [Manufacturing Quality Standards](https://www.iso.org/iso-9001-quality-management.html)
- [Vision System Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
