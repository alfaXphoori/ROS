# 🧭 031 - IMU Controller

> **Level 3.1 - Inertial Sensors | Precise Angle Control Using IMU**

---

## 📌 Overview

IMU-Based Orientation Controller
=================================

Script demonstrating Inertial Measurement Unit (IMU) usage for:
- Reading accurate orientation (Roll, Pitch, Yaw)
- Precise angle turning using gyroscope
- Compensating for wheel slippage
- Publishing IMU data to ROS2

Features:
- Real-time Dashboard UI (Clear screen update)
- InertialUnit for orientation (quaternion → Euler)
- Gyro for angular velocity
- Keyboard control for precise turns (A = Left, D = Right)

Author: AI Assistant
Level: 3.1 - Inertial Sensors

### ✨ Key Features

- 📡 InertialUnit for precise orientation (Roll, Pitch, Yaw)
- 🔄 Gyroscope for angular velocity measurement
- ⚡ Accelerometer for linear acceleration
- 🔀 Quaternion to Euler angle conversion
- 🎯 Precise turn-to-angle control with tolerance
- 🧭 Beautiful real-time compass display
- 📊 ROS2 sensor_msgs/Imu publishing
- ✅ Compensation for wheel slippage

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `031_imu_controller.py` | IMU orientation controller |
| `031_imu.wbt` | Webots world file |


---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/031_imu.wbt
```

### Step 2️⃣: Run IMU Controller

```bash
ros2 run ce_webots 031_imu_controller
```

### Real-Time Compass Display

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
```

---

## 🔧 How It Works

### 1. Initialize IMU

```python
# Get IMU device
self.imu = self.robot.getDevice('inertial_unit')
self.imu.enable(self.timestep)

# Get gyroscope for angular velocity
self.gyro = self.robot.getDevice('gyro')
self.gyro.enable(self.timestep)

# Get accelerometer (bonus!)
self.accel = self.robot.getDevice('accelerometer')
self.accel.enable(self.timestep)
```

### 2. Read Orientation (Quaternion to Euler)

```python
def read_orientation(self):
    """Get roll, pitch, yaw from IMU quaternion"""
    # IMU returns quaternion: [x, y, z, w]
    quat = self.imu.getQuaternion()
    
    # Convert to Euler angles (radians)
    # Yaw is what we care about most for turning
    roll = math.atan2(
        2 * (quat[3] * quat[0] + quat[1] * quat[2]),
        1 - 2 * (quat[0]**2 + quat[1]**2)
    )
    
    pitch = math.asin(
        2 * (quat[3] * quat[1] - quat[2] * quat[0])
    )
    
    yaw = math.atan2(
        2 * (quat[3] * quat[2] + quat[0] * quat[1]),
        1 - 2 * (quat[1]**2 + quat[2]**2)
    )
    
    return {
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw
    }
```

### 3. Turn to Target Angle

```python
def turn_to_angle(self, target_angle):
    """Precise turn control using IMU feedback"""
    angle_tolerance = 0.035  # ~2 degrees in radians
    
    while True:
        # Read current yaw
        current_yaw = self.read_orientation()['yaw']
        
        # Calculate angle error
        error = target_angle - current_yaw
        
        # Normalize error to [-π, π]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        # Check if reached
        if abs(error) < angle_tolerance:
            self.stop()
            return True
        
        # Turn in appropriate direction
        if error > 0:
            self.turn_left()
        else:
            self.turn_right()
        
        # Update display
        self.display_status(current_yaw, target_angle, error)
```

### 4. Publish IMU Data (ROS2)

```python
def publish_imu_data(self):
    """Publish IMU readings to ROS2"""
    orientation = self.read_orientation()
    angular_vel = self.gyro.getValues()
    linear_accel = self.accel.getValues()
    
    imu_msg = Imu()
    imu_msg.header.stamp = self.get_clock().now().to_msg()
    
    # Orientation (from euler to quaternion)
    q = self.euler_to_quaternion(
        orientation['roll'],
        orientation['pitch'],
        orientation['yaw']
    )
    imu_msg.orientation.x = q[0]
    imu_msg.orientation.y = q[1]
    imu_msg.orientation.z = q[2]
    imu_msg.orientation.w = q[3]
    
    # Angular velocity
    imu_msg.angular_velocity.x = angular_vel[0]
    imu_msg.angular_velocity.y = angular_vel[1]
    imu_msg.angular_velocity.z = angular_vel[2]
    
    # Linear acceleration
    imu_msg.linear_acceleration.x = linear_accel[0]
    imu_msg.linear_acceleration.y = linear_accel[1]
    imu_msg.linear_acceleration.z = linear_accel[2]
    
    self.imu_publisher.publish(imu_msg)
```

---

## 📚 Sensor Knowledge: InertialUnit (IMU) & Gyroscope

### 🎯 How It Works

**InertialUnit** measures robot orientation absolutely (independent of wheel rotation):

```
Quaternion (4 values)  →  Euler Angles (Roll, Pitch, Yaw)
   ↓
IMU tells robot: "You're facing North" ✓
Even if wheels slip! ↑
```

### 📊 Specifications

| Property | Value | Notes |
|---------|-------|-------|
| **Type** | Inertial Measurement Unit | 9-axis (accel, gyro, mag) |
| **Orientation Output** | Quaternion (x,y,z,w) | Convert to Euler |
| **Roll Range** | ±π radians (±180°) | Rotation around X |
| **Pitch Range** | ±π/2 radians (±90°) | Rotation around Y |
| **Yaw Range** | ±π radians (±180°) | Rotation around Z (heading) |
| **Accuracy** | ±1-2° | Very good |
| **Drift Rate** | ~0.1-0.5°/min | Slow accumulation |
| **Response Time** | < 1ms | Instant |

### 💡 Usage Tips

**✅ Do:**
- Precise heading control (turn to angle)
- Compass-like navigation
- Detect tilting/tipping
- Wall-following with orientation

**❌ Avoid:**
- Use for long durations > 1 hour (drift)
- Forget quaternion → euler conversion
- Use raw quaternion (unintuitive)
- Don't check biases (offsets)

### ⚠️ Limitations

```
1. Gyro Drift
   ───────────────────────────────
   ⏱️  1 minute:  ±0.5-1° drift
   ⏱️  10 minutes: ±5-10° drift
   ⏱️  1 hour:     ±30-60° drift
   
   ✓ Fix: Fuse with compass/encoders

2. Quaternion Complexity
   - Needs conversion to Euler angles
   - Must normalize to [-π, π]
   
3. Environmental Factors
   - Magnetic interference affects magnetometer
   - Vibration affects accelerometer
```

### 🔧 Quaternion to Euler Conversion

```python
import math

def quaternion_to_euler(q):
    """Convert quaternion (x,y,z,w) to Euler (roll, pitch, yaw)"""
    x, y, z, w = q
    
    # Roll (X-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (Y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1, min(1, sinp))  # Clamp for asin
    pitch = math.asin(sinp)
    
    # Yaw (Z-axis rotation) ← Heading!
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

# Usage
quat = imu.getQuaternion()
roll, pitch, yaw = quaternion_to_euler(quat)
heading_degrees = math.degrees(yaw)  # Convert to degrees
```

### 🧭 Compass Reference

```
     North (90°)
         ↑
         |
West ← ─┼─ → East
180°/π  🤖  0°/0
         |
         ↓
    South (-90°/-π/2)
```

---

## 📐 Orientation Reference

```
        North (90°)
            ⬆️
            |
West ← ──── 🤖 ──── → East
180°/π    (0°)      (0°)
            |
            ↓
        South (-90°)
```

**Yaw Ranges:**
- **0° / 0 rad:** East (➡️)
- **90° / π/2 rad:** North (⬆️)
- **±180° / ±π rad:** West (⬅️)
- **-90° / -π/2 rad:** South (⬇️)

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ IMU sensor principles
- ✅ Quaternion to Euler conversion
- ✅ Angle normalization techniques
- ✅ Closed-loop angle control
- ✅ Gyroscope for rotation measurement
- ✅ Compass-based navigation

---

## 🔍 IMU vs. Encoders: Advantages

| Aspect | Encoders | IMU |
|--------|----------|-----|
| **Affected by Slippage** | ❌ Yes | ✅ No |
| **Absolute Orientation** | ❌ No | ✅ Yes |
| **Drift Over Time** | ❌ Accumulates | ⚠️ Slow drift |
| **Turning Accuracy** | ❌ Moderate | ✅ Excellent |
| **Cost** | ✅ Cheap | ⚠️ More expensive |

---

## 📝 Customization

### Adjust Angle Tolerance

```python
ANGLE_TOLERANCE = math.radians(2.0)  # ±2 degrees
```

### Add Navigation to Multiple Waypoints

```python
angles = [0, 90, 180, -90, 0]  # Square path
for angle in angles:
    self.turn_to_angle(angle)
    self.move_forward(distance=1.0)
```

### Implement Compass Display

```python
def get_direction(self, yaw_rad):
    """Return compass direction for yaw angle"""
    directions = ['E', 'NE', 'N', 'NW', 'W', 'SW', 'S', 'SE']
    index = int((yaw_rad + math.pi) / (2 * math.pi / 8)) % 8
    return directions[index]
```

---

## 📚 Related Resources

- 📖 [Webots InertialUnit](https://cyberbotics.com/doc/reference/inertialunit)
- 🔗 [Quaternion Basics](https://en.wikipedia.org/wiki/Quaternion)
- 📐 [Euler Angles](https://en.wikipedia.org/wiki/Euler_angles)
- 🤖 [ROS 2 IMU Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Yaw value incorrect** | Check IMU initialization and calibration |
| **Can't reach target angle** | Verify motor control; adjust tolerance |
| **Erratic readings** | Ensure steady movement; check IMU orientation |
| **Drift detected** | Compass/gyro calibration may help |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
