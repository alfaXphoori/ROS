# Exercise 4: Quality Control Inspection Nodes

This directory contains production-grade ROS2 nodes for manufacturing quality control inspection systems.

## Nodes Overview

### 1. Vision Inspector Node (`vision_inspector_node.py`)
**Executable:** `07_vision_inspector`  
**Criticality:** CRITICAL  
**Purpose:** Camera-based defect detection for quality control

**Features:**
- Real-time vision-based inspection
- Defect detection with confidence scoring
- Multiple defect type classification (SCRATCH, DENT, COLOR_MISMATCH, etc.)
- Camera health monitoring
- Lighting and focus quality tracking
- Inspection accuracy metrics

**Topics:**
- `/vision_status` (std_msgs/String) - JSON-formatted inspection status

**Parameters:**
- `robot_id`: Unique identifier for the vision system
- `robot_type`: Type of inspection system (default: 'inspection')
- `zone_id`: Factory zone identifier
- `camera_resolution`: Camera resolution (default: '1920x1080')
- `inspection_rate_hz`: Inspection frequency (default: 1.0 Hz)
- `defect_threshold`: Confidence threshold for defect detection (default: 0.85)
- `simulate_failure`: Enable failure simulation for testing (default: false)

**Example Run:**
```bash
ros2 run ce_robot 07_vision_inspector --ros-args \
  -p robot_id:=QC-VISION-001 \
  -p zone_id:=FACTORY-QC-STATION-1 \
  -p inspection_rate_hz:=2.0 \
  -p defect_threshold:=0.90
```

---

### 2. Sensor Fusion Node (`sensor_fusion_node.py`)
**Executable:** `07_sensor_fusion`  
**Criticality:** HIGH  
**Purpose:** Multi-sensor data fusion for comprehensive quality assessment

**Features:**
- Thermal camera integration
- Ultrasonic surface quality detection
- Precision weight measurement
- Laser scanner dimensional analysis
- Sensor health monitoring
- Data quality scoring
- Anomaly detection across sensors

**Topics:**
- `/fusion_status` (std_msgs/String) - JSON-formatted fusion status

**Parameters:**
- `robot_id`: Unique identifier for the fusion system
- `robot_type`: Type of system (default: 'inspection')
- `zone_id`: Factory zone identifier
- `fusion_rate_hz`: Fusion cycle frequency (default: 2.0 Hz)
- `sensor_count`: Number of active sensors (default: 4)
- `simulate_failure`: Enable failure simulation for testing (default: false)

**Sensors:**
1. **Thermal Camera** - Temperature monitoring and hotspot detection
2. **Ultrasonic** - Surface quality and distance measurement
3. **Weight Sensor** - Precision weight validation
4. **Laser Scanner** - 3D dimensional accuracy

**Example Run:**
```bash
ros2 run ce_robot 07_sensor_fusion --ros-args \
  -p robot_id:=QC-FUSION-001 \
  -p zone_id:=FACTORY-QC-STATION-1 \
  -p fusion_rate_hz:=2.0 \
  -p sensor_count:=4
```

---

### 3. Report Generator Node (`report_generator_node.py`)
**Executable:** `07_report_generator`  
**Criticality:** NON-CRITICAL  
**Purpose:** Automated quality control report generation and data logging

**Features:**
- Multiple report types (hourly, shift, defect analysis, trends, compliance)
- Defect breakdown by type
- Trend analysis and predictions
- Database health monitoring
- Storage management
- Queue tracking

**Topics:**
- `/report_status` (std_msgs/String) - JSON-formatted report status

**Parameters:**
- `robot_id`: Unique identifier for the reporting system
- `robot_type`: Type of system (default: 'inspection')
- `zone_id`: Factory zone identifier
- `report_rate_hz`: Report generation frequency (default: 0.5 Hz)
- `report_format`: Output format (default: 'JSON')
- `simulate_failure`: Enable failure simulation for testing (default: false)

**Report Types:**
- `HOURLY_SUMMARY` - Hourly production statistics
- `SHIFT_REPORT` - Complete shift analysis
- `DEFECT_ANALYSIS` - Detailed defect breakdown
- `TREND_REPORT` - Quality trend analysis
- `COMPLIANCE_AUDIT` - Regulatory compliance data

**Example Run:**
```bash
ros2 run ce_robot 07_report_generator --ros-args \
  -p robot_id:=QC-REPORT-001 \
  -p zone_id:=FACTORY-QC-STATION-1 \
  -p report_rate_hz:=0.5 \
  -p report_format:=JSON
```

---

## Criticality Levels

| Node | Level | Failure Behavior |
|------|-------|------------------|
| Vision Inspector | **CRITICAL** | Immediate shutdown - quality risk |
| Sensor Fusion | **HIGH** | Auto-restart 3x, then escalate |
| Report Generator | **NON-CRITICAL** | Log only, continue operation |

## Installation

Add to your `setup.py`:

```python
entry_points={
    'console_scripts': [
        '07_vision_inspector = ce_robot.vision_inspector_node:main',
        '07_sensor_fusion = ce_robot.sensor_fusion_node:main',
        '07_report_generator = ce_robot.report_generator_node:main',
    ],
},
```

## Testing

**Test vision inspector:**
```bash
ros2 run ce_robot 07_vision_inspector

# In another terminal
ros2 topic echo /vision_status --once
```

**Test sensor fusion:**
```bash
ros2 run ce_robot 07_sensor_fusion

# In another terminal
ros2 topic echo /fusion_status --once
```

**Test report generator:**
```bash
ros2 run ce_robot 07_report_generator

# In another terminal
ros2 topic echo /report_status --once
```

## Production Deployment

These nodes are designed for 24/7 manufacturing environments:
- Real-time defect detection
- Multi-sensor validation
- Persistent failure tracking
- Automated reporting and analytics
- Integration with systemd for auto-start

See [../config/qc_inspection.service](../config/qc_inspection.service) for systemd configuration.
