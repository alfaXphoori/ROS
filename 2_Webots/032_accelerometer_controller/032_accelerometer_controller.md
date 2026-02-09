# Level 3.2: Accelerometer Controller 🚀

## Overview
This controller demonstrates the use of **Accelerometer sensors** to detect and monitor linear acceleration in robots. The accelerometer measures changes in velocity along the X, Y, and Z axes, enabling detection of acceleration, deceleration, impacts, and orientation changes.

## Learning Objectives
- Understand accelerometer sensor principles and data
- Read and interpret linear acceleration in 3 axes
- Implement motion pattern analysis
- Detect impacts and collisions
- Calculate acceleration magnitude
- Apply data smoothing for noise reduction
- Publish acceleration data to ROS2

## Sensor Information

### Accelerometer
- **Type**: Webots `Accelerometer` device
- **Measures**: Linear acceleration in X, Y, Z axes (m/s²)
- **Range**: Typically ±50 m/s² or more
- **Includes**: Gravitational acceleration (9.81 m/s² on Z-axis when level)
- **Applications**: 
  - Motion detection
  - Impact/collision detection
  - Tilt sensing
  - Vibration monitoring
  - Free-fall detection

### Acceleration Axes
```
X-axis: Forward/Backward acceleration (robot's movement direction)
Y-axis: Left/Right acceleration (sideways motion)
Z-axis: Up/Down acceleration (includes gravity: ~9.81 m/s²)
```

### Gravity Compensation
When the robot is level, the accelerometer reads:
- **X ≈ 0** m/s² (no forward motion)
- **Y ≈ 0** m/s² (no sideways motion)
- **Z ≈ 9.81** m/s² (Earth's gravity)

To get true motion acceleration on Z-axis: `motion_z = measured_z - 9.81`

## Configuration Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MAX_SPEED` | 0.8 rad/s | Maximum wheel velocity |
| `ACCEL_TIME` | 2.0 s | Duration of acceleration phase |
| `DECEL_TIME` | 2.0 s | Duration of deceleration phase |
| `IMPACT_THRESHOLD` | 20.0 m/s² | Threshold for impact detection |
| `PUBLISH_RATE` | 10 Hz | ROS2 data publishing frequency |
| `HISTORY_SIZE` | 5 samples | Buffer size for smoothing |

## Features

### 1. Real-Time Dashboard
The controller displays a comprehensive acceleration dashboard:

```
==============================================================
   🚀  ACCELEROMETER DASHBOARD (Real-time)
==============================================================

📊 LINEAR ACCELERATION (m/s²)
   X-axis (Forward):     2.45  [>>>>>>              ]
   Y-axis (Left):        0.12  [                    ]
   Z-axis (Up):          9.82  [>>>>>>>>>>>         ]
   Magnitude:           10.12  ✓

🌍 GRAVITY COMPENSATION
   Z (no gravity):       0.01 m/s²

🎮 STATUS: 🚀 Accelerating Forward...
==============================================================
```

### 2. Visual Acceleration Bars
- **Right arrows `>`**: Positive acceleration
- **Left arrows `<`**: Negative acceleration
- Bar length proportional to acceleration magnitude
- Easy visual interpretation of motion

### 3. Data Smoothing
Uses running average filter with 5-sample history:
```python
accel_smooth = sum(last_5_readings) / 5
```
Benefits:
- Reduces sensor noise
- Smoother acceleration profiles
- More stable readings

### 4. Impact Detection
Monitors total acceleration magnitude:
```python
magnitude = sqrt(accel_x² + accel_y² + accel_z²)
if magnitude > 20.0 m/s²:
    💥 IMPACT DETECTED!
```

### 5. Automatic Motion Demo
The controller runs through 5 phases:

1. **ACCELERATING** (2s): Gradual speed increase, positive X-axis acceleration
2. **CRUISING** (2s): Constant speed, near-zero acceleration
3. **DECELERATING** (2s): Gradual speed decrease, negative X-axis acceleration
4. **PAUSE** (2s): Stopped, zero motion acceleration
5. **REVERSING** (4s): Backward motion with reverse acceleration

## ROS2 Integration

### Published Topics

| Topic | Message Type | Rate | Content |
|-------|--------------|------|---------|
| `/imu/data` | `sensor_msgs/Imu` | 10 Hz | Acceleration in IMU format |
| `/accelerometer/data` | `geometry_msgs/Vector3` | 10 Hz | Raw acceleration (x, y, z) |

### Message Structure

**sensor_msgs/Imu**:
```python
header:
  stamp: current_time
  frame_id: "accelerometer_link"
linear_acceleration:
  x: forward_accel
  y: sideways_accel
  z: vertical_accel
```

**geometry_msgs/Vector3**:
```python
x: forward_acceleration
y: sideways_acceleration
z: vertical_acceleration
```

## Understanding Acceleration Patterns

### Forward Acceleration
```
Speed:        0 → → → → → MAX
Acceleration: [+++++++++] → [0] (positive then zero)
Reading:      X > 0 m/s²
```

### Deceleration
```
Speed:        MAX → → → → → 0
Acceleration: [---------] (negative)
Reading:      X < 0 m/s²
```

### Constant Speed
```
Speed:        ========== (constant)
Acceleration: [0] (zero)
Reading:      X ≈ 0 m/s²
```

### Reverse
```
Speed:        0 ← ← ← ← ← MAX
Acceleration: [----------] then [++++] (backward accel, forward decel)
Reading:      X < 0, then X > 0
```

## Code Structure

### Main Functions

1. **`read_accelerometer_data()`**
   - Gets raw acceleration values
   - Applies smoothing filter
   - Calculates magnitude
   - Detects impacts

2. **`publish_accelerometer_data()`**
   - Converts to ROS2 messages
   - Publishes to topics

3. **`_print_dashboard()`**
   - Clears screen for updates
   - Displays acceleration values
   - Shows visual bars
   - Indicates status

4. **`run()`**
   - Main control loop
   - Implements state machine
   - Controls motion phases
   - Reads and publishes data

## Running the Controller

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ce_webots --symlink-install
source install/setup.bash
```

### 2. Launch Webots Simulation
```bash
webots ~/ros2_ws/src/ce_webots/worlds/032_accelerometer.wbt
```

### 3. Run the Controller
In a new terminal:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ce_webots 032_accelerometer_controller
```

### 4. Monitor ROS2 Topics (Optional)
```bash
# View acceleration data
ros2 topic echo /accelerometer/data

# View IMU data
ros2 topic echo /imu/data

# Check topic list
ros2 topic list
```

## Expected Observations

### During Acceleration Phase
- **X-axis**: Increases (positive acceleration)
- **Dashboard**: Right arrows `>>>` on X-axis
- **Magnitude**: Increases temporarily
- **Speed**: Robot moves faster

### During Cruising Phase
- **X-axis**: Near zero (constant velocity)
- **Dashboard**: Minimal bars
- **Magnitude**: Close to gravity (9.81 m/s²)
- **Speed**: Constant

### During Deceleration Phase
- **X-axis**: Decreases (negative acceleration)
- **Dashboard**: Left arrows `<<<` on X-axis
- **Magnitude**: Increases temporarily
- **Speed**: Robot slows down

### During Reverse Phase
- **X-axis**: Negative (backward acceleration)
- **Dashboard**: Left arrows `<<<` on X-axis
- **Direction**: Robot moves backward

## Exercises

### 🎯 Exercise 1: Acceleration Analysis
**Task**: Observe and record acceleration values during each phase
1. Note peak X-axis acceleration during ACCELERATING
2. Verify X ≈ 0 during CRUISING
3. Compare acceleration vs deceleration magnitudes

### 🎯 Exercise 2: Modify Motion Parameters
**Task**: Change motion characteristics
```python
self.MAX_SPEED = 1.2        # Faster speed → higher acceleration
self.ACCEL_TIME = 1.0       # Shorter time → sharper acceleration
self.IMPACT_THRESHOLD = 15.0  # Lower threshold → more sensitive
```
Observe how changes affect acceleration readings.

### 🎯 Exercise 3: Impact Testing
**Task**: Create manual impact scenarios
1. Add obstacles to world file
2. Drive robot into obstacles
3. Observe impact detection alerts
4. Record maximum magnitude during impact

### 🎯 Exercise 4: Gravity Analysis
**Task**: Understand gravity compensation
1. Record Z-axis reading when stationary (should be ~9.81)
2. Calculate `motion_z = measured_z - 9.81`
3. Verify motion_z ≈ 0 when no vertical movement

### 🎯 Exercise 5: Custom Motion Pattern
**Task**: Implement new motion sequence
```python
# Add zigzag motion
elif self.mode == "ZIGZAG":
    # Alternate left/right turns while moving
    # Observe Y-axis acceleration during turns
```

## Advanced Applications

### 1. Free-Fall Detection
```python
if magnitude < 2.0:  # Much less than gravity
    print("🪂 FREE FALL DETECTED!")
```

### 2. Vibration Monitoring
```python
accel_variance = calculate_variance(accel_history)
if accel_variance > threshold:
    print("⚠️ VIBRATION DETECTED!")
```

### 3. Tilt Angle Estimation
```python
# Estimate tilt from gravity vector
tilt_x = math.atan2(accel_y, accel_z)
tilt_y = math.atan2(accel_x, accel_z)
```

### 4. Dead Reckoning
```python
# Estimate position from double-integrated acceleration
velocity += acceleration * dt
position += velocity * dt
```

### 5. Energy Monitoring
```python
# Estimate kinetic energy changes
energy_change = mass * acceleration * distance
```

## Common Issues & Solutions

### Issue 1: Noisy Readings
**Symptoms**: Erratic acceleration values, unstable bars
**Solutions**:
- Increase `HISTORY_SIZE` for more smoothing
- Reduce `PUBLISH_RATE` for less frequent updates
- Add low-pass filter for signal processing

### Issue 2: Constant Offset
**Symptoms**: Non-zero reading when stationary
**Solutions**:
- Calibrate sensor (subtract bias)
- Use gravity compensation for Z-axis
- Account for sensor drift over time

### Issue 3: No Impact Detection
**Symptoms**: Collisions not detected
**Solutions**:
- Lower `IMPACT_THRESHOLD`
- Check magnitude calculation
- Verify sensor is enabled

### Issue 4: Dashboard Too Fast
**Symptoms**: Screen flickers, hard to read
**Solutions**:
- Add `time.sleep()` in display loop
- Reduce update frequency
- Use terminal with better refresh rate

## Physics Background

### Newton's Second Law
```
F = m × a
Force = mass × acceleration
```

### Acceleration Definition
```
a = Δv / Δt
acceleration = change_in_velocity / time_interval
```

### Gravity
- Earth's gravitational acceleration: **g = 9.81 m/s²**
- Always present on Z-axis when level
- Must be compensated for true motion acceleration

### Magnitude Calculation
```
|a| = √(ax² + ay² + az²)
Total acceleration magnitude
```

## Key Takeaways

✅ Accelerometers measure **linear acceleration** in 3 axes  
✅ Readings include **gravitational acceleration** (9.81 m/s² on Z)  
✅ **Data smoothing** reduces noise and improves stability  
✅ **Magnitude** indicates total acceleration intensity  
✅ Useful for **impact detection**, **motion analysis**, and **tilt sensing**  
✅ ROS2 integration enables **data logging** and **multi-robot coordination**  
✅ Combine with other sensors (IMU, gyro) for **complete motion understanding**

## Next Steps

After mastering accelerometer control:
- **Level 3.3**: Combine IMU + Gyro + Accelerometer for full 6-DOF sensing
- **Level 4**: GPS and positioning sensors
- **Level 5**: Sensor fusion and Kalman filtering
- **Advanced**: SLAM (Simultaneous Localization and Mapping)

---

**Happy Sensing! 🚀📊**
