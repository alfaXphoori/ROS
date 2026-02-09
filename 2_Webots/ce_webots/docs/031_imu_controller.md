# 031 IMU Controller

**Level:** 3.1 - Inertial Sensors  
**Type:** Orientation-based control  
**Purpose:** Precise angle control using Inertial Measurement Unit (IMU)

## Overview

The IMU controller demonstrates **inertial sensing** for accurate orientation tracking and precise angle control. Unlike wheel encoders that suffer from slippage, IMU provides absolute orientation readings independent of wheel movement.

**Key Features:**
- InertialUnit sensor for orientation (Roll, Pitch, Yaw)
- Gyroscope for angular velocity measurement
- Accelerometer for linear acceleration
- Quaternion to Euler angle conversion
- Precise turn-to-angle control with tolerance
- Beautiful real-time compass display
- ROS2 sensor_msgs/Imu publishing
- Compensation for wheel slippage

**Files:**
- Controller: `ce_webots/031_imu_controller.py`
- World: `worlds/031_imu.wbt`
- Entry point: `ros2 run ce_webots 031_imu_controller`

**Mission:** Turn to specific angles (90°, 0°, -90°, 0°) using gyro feedback

---

## Quick Start

### Launch

```bash
# Terminal 1 - Start Webots
webots ~/ros2_ws/src/ce_webots/worlds/031_imu.wbt

# Terminal 2 - Run IMU controller
ros2 run ce_webots 031_imu_controller
```

### Expected Behavior

```
================================================================================
🧭 IMU CONTROLLER - Configuration
================================================================================
  Max Speed:           0.6 rad/s
  Turn Speed:          0.4 rad/s
  Angle Tolerance:     2.0°
  Publish Rate:        10 Hz
================================================================================

🎯 Demo Sequence:
  1. Read initial orientation
  2. Turn to 90° (North)
  3. Turn to 0° (East)
  4. Turn to -90° (South)
  5. Return to 0° (East)
================================================================================

┌─ 🧭 IMU READINGS ──────────────────────────────────────────────────────────
│  Orientation:
│    Roll:     0.15°  ━
│    Pitch:   -0.08°  ━
│    Yaw:      2.34°  ➡️ E
│  
│  Angular Velocity:  0.000 rad/s  ⏸️  STILL
└──────────────────────────────────────────────────────────────────────────────

🎯 Turning to 90°...

┌─ 🧭 IMU READINGS ──────────────────────────────────────────────────────────
│  Orientation:
│    Roll:     0.12°  ━
│    Pitch:   -0.05°  ━
│    Yaw:     45.67°  ↗️ NE
│  
│  Angular Velocity:  1.234 rad/s  🔄 CCW
│  
│  🎯 Target:   90.00°  |  Error:  44.33°
└──────────────────────────────────────────────────────────────────────────────

┌─ 🧭 IMU READINGS ──────────────────────────────────────────────────────────
│  Orientation:
│    Roll:     0.08°  ━
│    Pitch:   -0.03°  ━
│    Yaw:     89.85°  ⬆️ N
│  
│  Angular Velocity:  0.023 rad/s  ⏸️  STILL
│  
│  🎯 Target:   90.00°  |  Error:   0.15°
└──────────────────────────────────────────────────────────────────────────────
  ✅ Reached 89.9° (target: 90°)

🎯 Turning to 0°...
  ✅ Reached 0.3° (target: 0°)

🎯 Turning to -90°...
  ✅ Reached -89.8° (target: -90°)

🎯 Turning to 0°...
  ✅ Reached 0.1° (target: 0°)

  ✅ Demo Complete!
```

**Visual Features:**
- 🧭 Compass direction indicator (N, NE, E, SE, S, SW, W, NW)
- 🔄 Rotation status (CCW = counter-clockwise, CW = clockwise, STILL)
- 🎯 Target angle tracking with error display
- Real-time orientation in degrees

---

## Configuration Parameters

All behavior parameters are configurable:

```python
class IMUController(Node):
    def __init__(self):
        super().__init__('imu_controller')
        
        # === Configuration Parameters ===
        self.MAX_SPEED = 0.6               # Maximum wheel speed (rad/s)
        self.TURN_SPEED = 0.4              # Rotation speed for turns (rad/s)
        self.ANGLE_TOLERANCE = 2.0         # Angle precision tolerance (degrees)
        self.PUBLISH_RATE = 10             # ROS2 publish rate (Hz)
```

**Parameter Details:**

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `MAX_SPEED` | 0.6 rad/s | 0.1 - 2.0 | Maximum forward wheel velocity |
| `TURN_SPEED` | 0.4 rad/s | 0.1 - 1.0 | Rotation speed during turns |
| `ANGLE_TOLERANCE` | 2.0° | 0.5 - 10.0 | Acceptable angle error for completion |
| `PUBLISH_RATE` | 10 Hz | 1 - 50 | ROS2 topic publish frequency |

**Tuning Tips:**
- **Decrease `ANGLE_TOLERANCE`** for more precise turns (but may oscillate)
- **Reduce `TURN_SPEED`** for smoother, more accurate turns
- **Increase `PUBLISH_RATE`** for higher-frequency ROS2 data (more CPU usage)

---

## How It Works

### 1. IMU Sensor Setup

```python
# InertialUnit: Provides orientation (roll, pitch, yaw)
self.imu = self.robot.getDevice('imu')
self.imu.enable(self.timestep)

# Gyro: Provides angular velocity (rad/s)
self.gyro = self.robot.getDevice('gyro')
self.gyro.enable(self.timestep)

# Accelerometer: Provides linear acceleration (m/s²)
self.accelerometer = self.robot.getDevice('accelerometer')
self.accelerometer.enable(self.timestep)
```

**Sensor Placement:** All mounted at robot's center (0, 0, 0.05)

**World File Configuration:**
```vrml
InertialUnit {
  translation 0 0 0.05
  name "imu"
}

Gyro {
  translation 0 0 0.05
  name "gyro"
}

Accelerometer {
  translation 0 0 0.05
  name "accelerometer"
}
```

### 2. Reading IMU Data

```python
def read_imu_data(self):
    # Get orientation from InertialUnit (returns roll, pitch, yaw in radians)
    rpy = self.imu.getRollPitchYaw()
    roll_rad, pitch_rad, yaw_rad = rpy
    
    # Convert to degrees
    roll = math.degrees(roll_rad)
    pitch = math.degrees(pitch_rad)
    yaw = math.degrees(yaw_rad)
    
    # Get angular velocity from Gyro (rad/s)
    gyro_values = self.gyro.getValues()
    angular_velocity = {
        'x': gyro_values[0],  # Roll rate
        'y': gyro_values[1],  # Pitch rate
        'z': gyro_values[2]   # Yaw rate (most important for navigation)
    }
    
    # Get linear acceleration from Accelerometer (m/s²)
    accel_values = self.accelerometer.getValues()
    linear_acceleration = {
        'x': accel_values[0],
        'y': accel_values[1],
        'z': accel_values[2]
    }
    
    return {
        'orientation': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
        'angular_velocity': angular_velocity,
        'linear_acceleration': linear_acceleration
    }
```

**Return Values:**
- **Orientation:** Roll, Pitch, Yaw in degrees (-180° to 180°)
- **Angular Velocity:** Rotation rates in rad/s
- **Linear Acceleration:** m/s² in x, y, z axes

### 3. Quaternion to Euler Conversion

While Webots InertialUnit directly provides Euler angles, here's the conversion for reference:

```python
def quaternion_to_euler(self, w, x, y, z):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert to degrees
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
```

**Euler Angles:**
- **Roll (φ):** Rotation around X-axis (left/right tilt)
- **Pitch (θ):** Rotation around Y-axis (forward/backward tilt)
- **Yaw (ψ):** Rotation around Z-axis (compass heading)

### 4. Precise Angle Control

```python
def turn_to_angle(self, target_angle):
    """Turn robot to specific angle using gyro feedback"""
    # Calculate angle error
    error = self.normalize_angle(target_angle - self.current_yaw)
    
    # Check if we've reached target
    if abs(error) < self.ANGLE_TOLERANCE:
        return True
    
    # Determine turn direction
    if error > 0:
        # Turn left (counter-clockwise)
        left_speed = -self.TURN_SPEED
        right_speed = self.TURN_SPEED
    else:
        # Turn right (clockwise)
        left_speed = self.TURN_SPEED
        right_speed = -self.TURN_SPEED
    
    # Apply motor speeds
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)
    
    return False
```

**Control Flow:**
```
Current Yaw = 15°
Target Yaw = 90°
Error = normalize(90 - 15) = 75°

Error > 0 → Turn LEFT (CCW)
  left_motor = -0.4 rad/s
  right_motor = +0.4 rad/s

Continue until |error| < 2°
```

### 5. Angle Normalization

```python
def normalize_angle(self, angle):
    """Normalize angle to [-180, 180] degrees"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle
```

**Examples:**
- `normalize(270)` → `-90`
- `normalize(-270)` → `90`
- `normalize(185)` → `-175`

**Why Important:** Prevents robot from taking long path (e.g., turning 270° instead of -90°)

---

## ROS2 Integration

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | 10 Hz | Complete IMU data (orientation, angular velocity, linear acceleration) |
| `/imu/orientation_euler` | `geometry_msgs/Vector3` | 10 Hz | Euler angles (x=roll, y=pitch, z=yaw) in degrees |

### Publishing IMU Data

```python
def publish_imu_data(self, imu_data):
    # Create IMU message
    imu_msg = Imu()
    imu_msg.header.stamp = self.get_clock().now().to_msg()
    imu_msg.header.frame_id = 'imu_link'
    
    # Convert Euler back to quaternion for ROS standard
    roll = math.radians(imu_data['orientation']['roll'])
    pitch = math.radians(imu_data['orientation']['pitch'])
    yaw = math.radians(imu_data['orientation']['yaw'])
    
    # Euler to quaternion conversion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
    
    # Angular velocity (rad/s)
    imu_msg.angular_velocity.x = imu_data['angular_velocity']['x']
    imu_msg.angular_velocity.y = imu_data['angular_velocity']['y']
    imu_msg.angular_velocity.z = imu_data['angular_velocity']['z']
    
    # Linear acceleration (m/s²)
    imu_msg.linear_acceleration.x = imu_data['linear_acceleration']['x']
    imu_msg.linear_acceleration.y = imu_data['linear_acceleration']['y']
    imu_msg.linear_acceleration.z = imu_data['linear_acceleration']['z']
    
    self.imu_pub.publish(imu_msg)
```

### Monitoring ROS2 Topics

```bash
# Watch IMU data
ros2 topic echo /imu/data

# Output:
header:
  stamp:
    sec: 42
    nanosec: 123456789
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.7071068
  w: 0.7071068
angular_velocity:
  x: 0.0
  y: 0.0
  z: 1.234
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 9.81
---

# Watch Euler angles
ros2 topic echo /imu/orientation_euler

# Output:
x: 0.15    # Roll
y: -0.08   # Pitch
z: 45.67   # Yaw
---
```

---

## Beautiful Visual Output

### Compass Display

```python
def get_compass_icon(angle):
    """Get emoji compass direction"""
    angle = self.normalize_angle(angle)
    if -22.5 <= angle < 22.5:
        return "➡️ E"
    elif 22.5 <= angle < 67.5:
        return "↗️ NE"
    elif 67.5 <= angle < 112.5:
        return "⬆️ N"
    elif 112.5 <= angle < 157.5:
        return "↖️ NW"
    elif angle >= 157.5 or angle < -157.5:
        return "⬅️ W"
    elif -157.5 <= angle < -112.5:
        return "↙️ SW"
    elif -112.5 <= angle < -67.5:
        return "⬇️ S"
    else:
        return "↘️ SE"
```

**Direction Mapping:**
```
        ⬆️ N
       (90°)
         
  ↖️ NW     ↗️ NE
(135°)      (45°)

⬅️ W         ➡️ E
(180°)       (0°)

  ↙️ SW     ↘️ SE
(-135°)    (-45°)
         
        ⬇️ S
      (-90°)
```

### Rotation Indicators

- 🔄 **CCW** (Counter-Clockwise): Angular velocity Z > 0.1 rad/s
- 🔃 **CW** (Clockwise): Angular velocity Z < -0.1 rad/s
- ⏸️ **STILL**: |Angular velocity Z| < 0.1 rad/s

### Tilt Indicators

- 🎢 **Roll Warning**: |Roll| > 5° (robot leaning sideways)
- ⛰️ **Pitch Warning**: |Pitch| > 5° (robot nose up/down)
- ━ **Level**: Tilt < 5°

---

## Code Structure

```python
class IMUController(Node):
    # Configuration
    MAX_SPEED = 0.6
    TURN_SPEED = 0.4
    ANGLE_TOLERANCE = 2.0
    PUBLISH_RATE = 10
    
    def __init__(self):
        # Initialize Webots robot and IMU sensors
        # Setup ROS2 publishers
        # Initialize state variables
        pass
    
    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to Euler angles"""
        pass
    
    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180]"""
        pass
    
    def read_imu_data(self):
        """Read orientation, angular velocity, acceleration"""
        pass
    
    def publish_imu_data(self, imu_data):
        """Publish to ROS2 topics"""
        pass
    
    def turn_to_angle(self, target_angle):
        """Turn to specific angle with feedback control"""
        pass
    
    def _print_imu_status(self, imu_data):
        """Display beautiful IMU visualization"""
        pass
    
    def run(self):
        """Main control loop with demo sequence"""
        # Demo: Turn to [90°, 0°, -90°, 0°]
        pass
```

---

## Why IMU Over Odometry?

### Problem with Wheel Odometry

```python
# Wheel-based turning (unreliable)
def turn_90_degrees_odometry():
    # Calculate wheel rotations needed
    wheel_rotations = (pi / 2) * wheelbase / wheel_radius
    
    # Turn until encoders say 90°
    while encoder_diff < wheel_rotations:
        turn()
    
    # ❌ PROBLEM: Wheels can slip!
    # Actual angle: 75° or 105° (not 90°)
```

**Issues:**
- Wheel slippage on smooth floors
- Different friction on left/right wheels
- Accumulated error over time
- No absolute reference

### Solution with IMU

```python
# IMU-based turning (reliable)
def turn_90_degrees_imu():
    target_yaw = 90.0
    
    # Turn until IMU says 90°
    while abs(current_yaw - target_yaw) > tolerance:
        turn()
    
    # ✅ ACCURATE: IMU measures actual orientation
    # Actual angle: 90.0° ± 2° (configurable tolerance)
```

**Advantages:**
- Absolute orientation measurement
- No accumulated error
- Immune to wheel slippage
- Works on any surface

### Comparison Table

| Feature | Wheel Odometry | IMU |
|---------|----------------|-----|
| **Accuracy** | Degrades over time | Constant |
| **Slippage** | ❌ Causes errors | ✅ Unaffected |
| **Surface** | Sensitive to friction | Independent |
| **Drift** | Accumulates | Minimal (gyro drift) |
| **Cost** | Free (encoders) | Additional sensor |
| **Best Use** | Distance traveled | Orientation/heading |

**Best Practice:** Use both! IMU for heading, odometry for distance

---

## Troubleshooting

### Robot overshoots target angle

**Cause:** Turn speed too high or tolerance too small

**Solution:**
```python
self.TURN_SPEED = 0.2        # Reduce speed
self.ANGLE_TOLERANCE = 3.0   # Increase tolerance
```

### Robot oscillates around target

**Cause:** Tolerance too small, creating on-off behavior

**Solution:** Add damping or increase tolerance
```python
# Simple damping
error = target - current
if abs(error) < 10:  # Slow down near target
    speed = TURN_SPEED * (abs(error) / 10)
else:
    speed = TURN_SPEED
```

### IMU readings are noisy

**Cause:** Sensor noise or vibration

**Solution:** Apply low-pass filter
```python
# Exponential moving average
alpha = 0.8
filtered_yaw = alpha * current_yaw + (1 - alpha) * previous_yaw
```

### Yaw jumps from 180° to -180°

**Cause:** Angle wraparound not handled

**Already Solved:** `normalize_angle()` function handles this

**Example:**
```python
# Without normalization
error = 185 - (-175)  # = 360° (wrong!)

# With normalization
error = normalize(185 - (-175))  # = 10° (correct!)
```

### Robot turns wrong direction

**Cause:** Error sign convention issue

**Check:**
```python
# Positive error → turn left (CCW)
if error > 0:
    left_speed = -TURN_SPEED
    right_speed = +TURN_SPEED
```

---

## Exercises

### Beginner

**Exercise 1: Change Target Angles**

Modify the demo sequence:
```python
# In run() method
demo_angles = [45, 135, -135, -45, 0]  # Pentagon pattern
```

**Exercise 2: Adjust Precision**

Try different tolerance values:
```python
self.ANGLE_TOLERANCE = 0.5   # Very precise (may oscillate)
self.ANGLE_TOLERANCE = 5.0   # Less precise (faster)
self.ANGLE_TOLERANCE = 10.0  # Rough alignment
```

Question: What's the practical limit of precision?

**Exercise 3: Monitor ROS2 Topics**

```bash
# Terminal 3 - Watch IMU data
ros2 topic echo /imu/data

# Terminal 4 - Watch Euler angles
ros2 topic echo /imu/orientation_euler

# Terminal 5 - Plot yaw over time
ros2 topic hz /imu/data
```

### Intermediate

**Exercise 4: Add Proportional Control**

Replace on-off control with proportional:

```python
def turn_to_angle(self, target_angle):
    error = self.normalize_angle(target_angle - self.current_yaw)
    
    if abs(error) < self.ANGLE_TOLERANCE:
        return True
    
    # Proportional control
    Kp = 0.05  # Proportional gain
    turn_rate = error * Kp
    turn_rate = max(-self.TURN_SPEED, min(self.TURN_SPEED, turn_rate))
    
    left_speed = -turn_rate
    right_speed = turn_rate
    
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)
    
    return False
```

**Exercise 5: Add PID Control**

Full PID for minimal overshoot:

```python
# In __init__
self.integral_error = 0.0
self.previous_error = 0.0
self.Kp = 0.05
self.Ki = 0.001
self.Kd = 0.01

def turn_to_angle_pid(self, target_angle):
    error = self.normalize_angle(target_angle - self.current_yaw)
    
    # PID calculation
    self.integral_error += error * (self.timestep / 1000.0)
    derivative = (error - self.previous_error) / (self.timestep / 1000.0)
    
    output = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative
    
    # Clamp output
    turn_rate = max(-self.TURN_SPEED, min(self.TURN_SPEED, output))
    
    self.left_motor.setVelocity(-turn_rate)
    self.right_motor.setVelocity(turn_rate)
    
    self.previous_error = error
    
    return abs(error) < self.ANGLE_TOLERANCE
```

**Exercise 6: Square Path Navigation**

Combine turning with forward movement:

```python
def navigate_square(side_length=1.0):
    """Drive in a square using IMU for precise 90° turns"""
    for _ in range(4):
        # Drive forward
        drive_distance(side_length)
        
        # Turn 90° right
        current_target = self.current_yaw - 90
        turn_to_angle(current_target)
```

### Advanced

**Exercise 7: Heading Hold**

Maintain heading while driving forward:

```python
def drive_with_heading_hold(target_distance, target_heading):
    """Drive forward while maintaining specific heading"""
    start_position = get_position()
    
    while get_distance(start_position) < target_distance:
        # Read current heading
        imu_data = self.read_imu_data()
        current_heading = imu_data['orientation']['yaw']
        
        # Calculate correction
        error = self.normalize_angle(target_heading - current_heading)
        correction = error * 0.02  # Small proportional gain
        
        # Apply differential drive
        left_speed = MAX_SPEED + correction
        right_speed = MAX_SPEED - correction
        
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
```

**Exercise 8: Compass Following**

Navigate to absolute compass directions:

```python
def go_north(distance):
    """Travel north (90°) for specified distance"""
    drive_with_heading_hold(distance, 90.0)

def go_east(distance):
    """Travel east (0°) for specified distance"""
    drive_with_heading_hold(distance, 0.0)

def go_to_position(target_x, target_y):
    """Navigate to specific coordinates using compass"""
    # Calculate required heading
    dx = target_x - current_x
    dy = target_y - current_y
    target_heading = math.degrees(math.atan2(dy, dx))
    distance = math.sqrt(dx**2 + dy**2)
    
    # Turn to heading and drive
    turn_to_angle(target_heading)
    drive_with_heading_hold(distance, target_heading)
```

**Exercise 9: IMU Calibration**

Add offset calibration for improved accuracy:

```python
# In __init__
self.yaw_offset = 0.0
self.calibrated = False

def calibrate_imu(self, samples=100):
    """Calibrate IMU by averaging readings at rest"""
    print("🔧 Calibrating IMU... Stay still!")
    
    yaw_sum = 0.0
    for i in range(samples):
        imu_data = self.read_imu_data()
        yaw_sum += imu_data['orientation']['yaw']
        self.robot.step(self.timestep)
    
    self.yaw_offset = yaw_sum / samples
    self.calibrated = True
    print(f"✅ Calibration complete. Offset: {self.yaw_offset:.2f}°")

def get_calibrated_yaw(self):
    """Get yaw with calibration offset applied"""
    raw_yaw = self.read_imu_data()['orientation']['yaw']
    return self.normalize_angle(raw_yaw - self.yaw_offset)
```

---

## Related Controllers

**Previous Level:**
- [022_distance_sensor_controller.md](022_distance_sensor_controller.md) - Obstacle avoidance with distance sensors

**Same Level:**
- 032_imu_odometry_fusion.md - Combining IMU with wheel odometry (coming soon)

**Next Level:**
- Level 4.1 - GPS Navigation (coming soon)

---

## Reference

### IMU Sensor Specifications

| Sensor | Measurement | Range | Units | Update Rate |
|--------|-------------|-------|-------|-------------|
| InertialUnit | Orientation | ±180° | degrees | 32 ms |
| Gyro | Angular velocity | ±∞ | rad/s | 32 ms |
| Accelerometer | Linear accel | ±∞ | m/s² | 32 ms |

### Coordinate System

**Webots Convention (Right-Hand Rule):**
```
    Z (Up)
    │
    │
    │
    └─────── X (Forward)
   ╱
  ╱
 Y (Left)
```

**Euler Angles:**
- **Roll (X-axis):** Positive = tilt right
- **Pitch (Y-axis):** Positive = nose up
- **Yaw (Z-axis):** Positive = turn left (CCW)

### Angle Conventions

| Direction | Yaw Angle | Compass |
|-----------|-----------|---------|
| East (→)  | 0° | E |
| North (↑) | 90° | N |
| West (←)  | ±180° | W |
| South (↓) | -90° | S |

### Control Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| ANGLE_TOLERANCE | 2.0° | Acceptable error for turn completion |
| TURN_SPEED | 0.4 rad/s | Rotation speed during turns |
| MAX_SPEED | 0.6 rad/s | Maximum forward speed |
| PUBLISH_RATE | 10 Hz | ROS2 topic update frequency |

### ROS2 Message Format

**sensor_msgs/Imu:**
```
Header header
  stamp: time
  frame_id: "imu_link"
Quaternion orientation        # (w, x, y, z)
Vector3 angular_velocity      # rad/s
Vector3 linear_acceleration   # m/s²
float64[9] orientation_covariance
float64[9] angular_velocity_covariance
float64[9] linear_acceleration_covariance
```

---

**Status:** ✅ Production ready  
**Behavior:** Precise angle control with gyro feedback  
**See also:** Level 3.1 - Inertial Sensors
