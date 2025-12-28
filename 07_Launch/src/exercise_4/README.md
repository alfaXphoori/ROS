# Exercise 4: YAML Configuration & Composition

## Overview

This exercise demonstrates how to:
- Load parameters from external YAML files
- Create modular, reusable launch configurations
- Compose multiple launch files together
- Manage different robot configurations without code changes

## Directory Structure

```
exercise_4/
├── yaml_config_launch.py       # Main launch file
└── launch/
    └── config/
        ├── robot_small.yaml     # Small robot configuration
        ├── robot_large.yaml     # Large robot configuration
        ├── robot_simulation.yaml # Simulation mode
        └── robot_hardware.yaml  # Hardware mode
```

## Configuration Files

### robot_small.yaml
Compact picker robot for narrow aisles:
- Max payload: 25kg
- Fast updates: 3.0 Hz
- Narrow width: 0.45m
- Quick charging: 2.5 hours

### robot_large.yaml
Heavy-duty industrial transport robot:
- Max payload: 1500kg
- Stable updates: 1.0 Hz
- Wide body: 1.20m
- Extended charging: 6.0 hours

### robot_simulation.yaml
Fast simulation mode for testing:
- Unrealistic speeds allowed
- Fast refresh rate: 10.0 Hz
- Gazebo integration
- Quick testing cycles

### robot_hardware.yaml
Production-ready hardware deployment:
- Safe speeds for human interaction
- Real sensor topics
- Physical e-stop integration
- Realistic performance limits

## Usage

### Basic Usage

**Launch with small robot config:**
```bash
cd /Volumes/ExDisk/Google\ Drive\ Ksu/KSU/Git/ROS/07_Launch/src/exercise_4
python3 yaml_config_launch.py robot_config:=small
```

**Launch with large robot config:**
```bash
python3 yaml_config_launch.py robot_config:=large
```

**Launch with simulation config:**
```bash
python3 yaml_config_launch.py robot_config:=simulation
```

**Launch with hardware config:**
```bash
python3 yaml_config_launch.py robot_config:=hardware
```

### Advanced Usage

**Include composition (if simple_launch.py exists):**
```bash
python3 yaml_config_launch.py robot_config:=small include_simple_launch:=true
```

## Testing

After launching, verify the parameters were loaded correctly:

**Check robot_tag_publisher parameters:**
```bash
ros2 param list /robot_tag_publisher
ros2 param get /robot_tag_publisher robot_id
ros2 param get /robot_tag_publisher max_payload_kg
ros2 param get /robot_tag_publisher tag_publish_rate
```

**Expected outputs:**

For small config:
- robot_id: "AMR-COMPACT-PICKER-S01"
- max_payload_kg: 25.0
- tag_publish_rate: 3.0

For large config:
- robot_id: "AMR-HEAVY-TRANSPORT-L01"
- max_payload_kg: 1500.0
- tag_publish_rate: 1.0

For simulation config:
- robot_id: "SIM-TEST-001"
- max_payload_kg: 100.0
- tag_publish_rate: 10.0

For hardware config:
- robot_id: "AMR-PROD-001"
- max_payload_kg: 200.0
- tag_publish_rate: 2.0

## Key Concepts

1. **YAML Parameters** - Store configurations in external files
2. **PathJoinSubstitution** - Build file paths dynamically
3. **Launch Composition** - Include other launch files
4. **Configuration Switching** - Change behavior without code changes
5. **Modular Design** - Separate configuration from logic

## Real-World Benefits

- **One codebase, multiple deployments** - Same code, different configs
- **Easy testing** - Switch between sim and hardware modes
- **Customer customization** - Each customer gets their own config file
- **No recompilation** - Change parameters without rebuilding
- **Version control** - Track configuration changes in git

## Troubleshooting

**Issue: "No module named 'ce_robot'"**
- Solution: This is a standalone demo. The nodes are placeholders in the launch file.

**Issue: Config file not found**
- Solution: Ensure you're running from the exercise_4 directory or update the config_dir path.

**Issue: simple_launch.py not found**
- Solution: This is expected. The launch file handles this gracefully with a try/except block.

## Challenge

Create your own config file for a specialized robot:
- Medical delivery robot (sterile environment)
- Outdoor delivery robot (weather resistant)
- High-speed racing robot (testing mode)

Copy one of the existing YAML files and modify the parameters to match your robot's specifications!
