# 📏 011 - Wheel Encoder Mission

> **Level 1.1 - Proprioceptive Sensing | Autonomous Distance Control via Encoders**

---

## 📌 Overview

The **Wheel Encoder Mission** demonstrates **closed-loop distance control** using proprioceptive sensors. The robot moves forward exactly 5.4 meters (from y=-2.7 to y=2.7) by continuously reading wheel encoders and stopping when the target distance is reached—no time-based estimation, pure sensor feedback!

### ✨ Key Features

- 📐 PositionSensor (wheel encoders) integration
- 🔄 Closed-loop distance control with real-time feedback
- 📊 Pure sensor-based movement (no time estimation)
- ✅ Automatic mission execution and completion detection
- 🎯 Precise distance tracking and error reporting

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `011_wheel_encoder_mission.py` | Mission controller script |
| `011_wheel_encoder.wbt` | Webots world file |
| `011_wheel_encoder_mission.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/011_wheel_encoder.wbt
```

### Step 2️⃣: Run Mission

```bash
ros2 run ce_webots 011_wheel_encoder_mission
```

### Expected Output

```
============================================================
🎯 LEVEL 1.1: WHEEL ENCODER MISSION
============================================================
Mission: Move forward exactly 5.4m and stop
Wheel radius: 0.08m
Speed: 0.2m/s
⚠️  No time.sleep - using encoder feedback only!
============================================================
Starting mission in 3 seconds...

🚀 MISSION START - Moving forward...
📏 Distance: 0.100m | Remaining: 5.300m
📏 Distance: 0.200m | Remaining: 5.200m
...
📏 Distance: 5.300m | Remaining: 0.100m
📏 Distance: 5.400m | Remaining: 0.000m

============================================================
✅ MISSION COMPLETE!
============================================================
Target distance: 5.4000m
Actual distance: 5.4023m
Error: 0.0023m (0.04%)
============================================================
```

🎯 Watch as the robot reaches its destination with incredible precision!

---

## 🔧 How It Works

### 1. Initialize Encoders

The robot retrieves encoder devices and enables them for data collection:

```python
# Get encoder devices
self.left_encoder = self.robot.getDevice('left_wheel_sensor')
self.right_encoder = self.robot.getDevice('right_wheel_sensor')

# Enable encoders (required!)
self.left_encoder.enable(self.timestep)
self.right_encoder.enable(self.timestep)

# Tracking variables
self.prev_left_encoder = 0.0
self.prev_right_encoder = 0.0
self.distance_traveled = 0.0
```

**Key Point:** Encoders must be explicitly enabled!

### 2. Read Encoder Values

Encoders provide rotational position in radians. We calculate the distance traveled:

```python
def read_encoders_and_update_distance(self):
    # Read current positions (in radians)
    left_pos = self.left_encoder.getValue()
    right_pos = self.right_encoder.getValue()
    
    # Calculate rotation delta
    left_delta = left_pos - self.prev_left_encoder
    right_delta = right_pos - self.prev_right_encoder
    
    # Convert radians to distance: distance = radius × radians
    left_distance = left_delta * self.WHEEL_RADIUS
    right_distance = right_delta * self.WHEEL_RADIUS
    
    # Average for differential drive
    distance_this_step = (left_distance + right_distance) / 2.0
    
    # Accumulate
    self.distance_traveled += distance_this_step
    
    # Remember for next iteration
    self.prev_left_encoder = left_pos
    self.prev_right_encoder = right_pos
    
    return self.distance_traveled
```

### 3. Mission Control Loop

```python
def run_mission(self):
    target_distance = 5.4  # meters
    speed = 0.2            # m/s
    
    # Start moving
    self.set_motor_speed(speed)
    
    while True:
        # Update distance from encoders
        distance = self.read_encoders_and_update_distance()
        
        # Check if target reached
        if distance >= target_distance:
            self.set_motor_speed(0)  # Stop
            break
        
        # Display progress
        remaining = target_distance - distance
        print(f"📏 Distance: {distance:.3f}m | Remaining: {remaining:.3f}m")
```

---

## � Sensor Knowledge: PositionSensor (Wheel Encoder)

### 🎯 How It Works

**PositionSensor** in Webots measures the angular position of rotating objects. It outputs values in **radians** that represent cumulative rotation:

```
Angular Position (radians) ──→ Distance (meters)
    rotation = new_angle - old_angle
    distance = rotation × wheel_radius
```

### 📊 Specifications

| Property | Value | Note |
|---------|-------|------|
| **Type** | Rotational Position Sensor | Measures rotation |
| **Units** | Radians (rad) | Must convert to meters |
| **Range** | Unlimited | Accumulates over time |
| **Accuracy** | ±0.001 rad (~0.06°) | Very good |
| **Resolution** | 10 decimal places | Excellent for odometry |
| **Slip Impact** | ❌ Affected | If wheels slip on ground |

### 💡 Usage Tips

**✅ Do:**
- Use for closed-loop distance control
- Combine 2 encoders to detect skidding
- Enable before use and allow stabilization

**❌ Avoid:**
- Using single encoder on differential drive
- Forgetting to enable() sensor
- Not converting radians → meters

### ⚠️ Limitations

```
1. No Awareness of Wheel Slippage
   ┌─ Road Condition ─┐
   │ Grass: 15% slip │ → Encoder reads wrong!
   │ Mud: 30% slip   │ → Must verify with IMU
   │ Smooth: 0% slip │ ✓ Accurate
   └─────────────────┘

2. Accumulation Errors
   - Errors accumulate over time
   - For long distances, must calibrate

3. Direction Ambiguity
   - Encoder doesn't know forward/backward
   - Must store old value to find delta
```

### 🔧 Calibration & Best Practices

```python
# 1. Measure actual wheel diameter
measured_diameter = 0.16  # meters
WHEEL_RADIUS = measured_diameter / 2

# 2. Allow encoder to stabilize (stabilization)
for _ in range(5):
    robot.step(timestep)

# 3. Record initial values
initial_left = left_encoder.getValue()
initial_right = right_encoder.getValue()

# 4. Check for slippage by comparing L/R
if abs(left_distance - right_distance) > 0.05:
    print("⚠️ Slippage detected!")
```

---

## �📊 Physics Reference

### Wheel-to-Distance Conversion

```
Encoder Reading: Angular Position (radians)
                      ↓
            delta_angle = new_position - old_position
                      ↓
            distance = delta_angle × wheel_radius
                      ↓
            Total Distance = sum of all distances
```

### Example Calculation

```
Wheel Radius: 0.08 m
Left Encoder:   old = 10 rad → new = 10.628 rad (Δ = 0.628 rad)
Right Encoder:  old = 10 rad → new = 10.628 rad (Δ = 0.628 rad)

Distance per wheel: 0.628 rad × 0.08 m/rad = 0.050 m (5 cm)
Average distance: (0.050 + 0.050) / 2 = 0.050 m
```

---

## 🎓 Learning Outcomes

After completing this mission, you will understand:

- ✅ Encoder sensor basics and integration
- ✅ Angular to linear distance conversion
- ✅ Closed-loop control principles
- ✅ Differential drive odometry
- ✅ Precision autonomous movement
- ✅ Error measurement and analysis

---

## 🔍 Advantages Over Time-Based Control

| Aspect | Time-Based | Encoder-Based |
|--------|-----------|---------------|
| **Accuracy** | ❌ Low (affected by speed fluctuations) | ✅ High (direct measurement) |
| **Slippage Tolerance** | ❌ Fails with wheel slippage | ✅ Compensates for slippage |
| **Terrain Dependence** | ❌ Varies on different surfaces | ✅ Reliable on all surfaces |
| **Verification** | ❌ Cannot verify actual movement | ✅ Actual distance known |
| **Control Loop** | ⚠️ Open-loop (blind hope) | ✅ Closed-loop (real feedback) |

---

## 📝 Customization

### Modify Mission Parameters

```python
class WheelEncoderMission:
    WHEEL_RADIUS = 0.08      # meters - adjust for your robot
    AXLE_LENGTH = 0.24       # meters between wheels
    TARGET_DISTANCE = 5.4    # meters - change mission distance
    SPEED = 0.2              # m/s - slower for precision
```

### Add Waypoints

```python
waypoints = [
    5.4,   # Move 5.4m forward
    -2.0,  # Move 2.0m backward
    3.0,   # Move 3.0m forward
]

for target in waypoints:
    execute_movement(target)
```

---

## 📚 Related Resources

- 📖 [Webots PositionSensor](https://cyberbotics.com/doc/reference/positionsensor)
- 🔗 [Differential Drive Kinematics](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
- 📐 [Odometry in Mobile Robotics](https://en.wikipedia.org/wiki/Odometry)
- 🤖 [ROS 2 Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot doesn't move** | Check encoder enabling; verify motor setup |
| **Distance not accurate** | Calibrate wheel radius; check for slippage |
| **Encoder readings zero** | Ensure `enable()` is called before use |
| **Mission overshoots** | Reduce speed for better precision |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
