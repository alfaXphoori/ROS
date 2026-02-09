# 🚀 032 - Accelerometer Controller

> **Level 3.2 - Inertial Sensors | Motion Detection and Impact Sensing**

---

## 📌 Overview

The **Accelerometer Controller** demonstrates **linear acceleration measurement** in all three axes (X, Y, Z). Accelerometers detect motion patterns, impacts, and orientation changes—understand how robots sense collisions and extreme movements!

### ✨ Key Features

- 📊 3-axis acceleration measurement (X, Y, Z)
- 🎨 Real-time acceleration dashboard display
- 🌍 Gravity compensation (removes 9.81 m/s² Z component)
- 💥 Impact detection with configurable thresholds
- 📈 Data smoothing for noise reduction
- 📊 Visual acceleration bars (positive/negative)
- 📡 ROS2 sensor_msgs/Imu publishing
- 🎯 Motion pattern analysis

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `032_accelerometer_controller.py` | Acceleration controller script |
| `032_accelerometer.wbt` | Webots world file |
| `032_accelerometer_controller.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/032_accelerometer.wbt
```

### Step 2️⃣: Run Accelerometer Controller

```bash
ros2 run ce_webots 032_accelerometer_controller
```

### Real-Time Dashboard

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

---

## 🔧 How It Works

### 1. Initialize Accelerometer

```python
# Get accelerometer device
self.accel = self.robot.getDevice('accelerometer')
self.accel.enable(self.timestep)

# Configuration parameters
self.MAX_SPEED = 0.8              # rad/s
self.ACCEL_TIME = 2.0             # seconds
self.DECEL_TIME = 2.0             # seconds
self.IMPACT_THRESHOLD = 20.0      # m/s²
self.HISTORY_SIZE = 5             # samples for smoothing
```

### 2. Read Acceleration Values

```python
def read_acceleration(self):
    """Get 3-axis acceleration from accelerometer"""
    accel = self.accel.getValues()  # Returns [ax, ay, az]
    
    return {
        'x': accel[0],  # Forward/Backward
        'y': accel[1],  # Left/Right
        'z': accel[2],  # Up/Down
    }
```

### 3. Gravity Compensation

```python
def compensate_gravity(self, accel):
    """Remove gravitational component from Z-axis"""
    # When robot is level, accelerometer reads ~9.81 m/s² on Z-axis
    # This is gravity, not motion
    
    GRAVITY = 9.81  # m/s²
    
    return {
        'x': accel['x'],
        'y': accel['y'],
        'z': accel['z'] - GRAVITY,  # Remove gravity
    }
```

### 4. Data Smoothing

```python
def smooth_acceleration(self, accel):
    """Apply running average filter"""
    self.history.append(accel)
    
    if len(self.history) > self.HISTORY_SIZE:
        self.history.pop(0)
    
    # Average last N readings
    smooth = {
        'x': sum(h['x'] for h in self.history) / len(self.history),
        'y': sum(h['y'] for h in self.history) / len(self.history),
        'z': sum(h['z'] for h in self.history) / len(self.history),
    }
    
    return smooth
```

### 5. Impact Detection

```python
def detect_impact(self, accel):
    """Check for collisions/impacts"""
    # Calculate total acceleration magnitude
    magnitude = math.sqrt(
        accel['x']**2 + 
        accel['y']**2 + 
        accel['z']**2
    )
    
    if magnitude > self.IMPACT_THRESHOLD:
        return True
    else:
        return False
```

---

## 📊 Acceleration Reference

### 3-Axis Explanation

```
Z-axis (Vertical)
    ↑
    | ↓ Gravity = 9.81 m/s²
    |
    ╔════════════════════════════════ Robot
    ║
Y-axis (Left-Right) ←─────────────→ 
    ║
    └─→ X-axis (Forward-Backward)
```

### Typical Acceleration Values

| Scenario | X (Forward) | Y (Side) | Z (Vertical) |
|----------|---|---|---|
| **Idle (level)** | 0.0 | 0.0 | 9.81 |
| **Accelerating forward** | +2.5 | 0.0 | 9.81 |
| **Decelerating** | -2.5 | 0.0 | 9.81 |
| **Turning left** | 0.0 | -1.5 | 9.81 |
| **Going uphill** | 0.0 | 0.0 | 11.5 |
| **Collision** | ±20+ | 0.0 | 9.81+ |

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ Accelerometer sensor principles
- ✅ 3-axis motion measurement
- ✅ Gravity compensation techniques
- ✅ Impact detection methods
- ✅ Data filtering and smoothing
- ✅ Motion pattern recognition

---

## � Sensor Knowledge: Accelerometer

### 🎯 How It Works

**Accelerometer** measures force applied to sensor (including gravity):

```
3-Axis Acceleration Measurement:
  X-axis: Forward/Backward movement
  Y-axis: Left/Right sideways motion  
  Z-axis: Up/Down (includes gravity 9.81 m/s²)

sensor_reading[Z] = motion_z + gravity
                  = motion_z + 9.81 m/s²
```

### 📊 Specifications

| คุณสมบัติ | ค่า | หมายเหตุ |
|---------|-----|--------|
| **ประเภท** | 3-Axis Accelerometer | X, Y, Z axes |
| **ช่วงการวัด** | ±50 m/s² | Typical range |
| **ความแม่นยำ** | ±0.1 m/s² | ±1% of reading |
| **ความละเอียด** | 0.01 m/s² | Fine resolution |
| **Gravity Component** | 9.81 m/s² | Always present when level |
| **ตอบสนอง** | < 1ms | Very fast |
| **Output** | (accel_x, accel_y, accel_z) | 3 values per reading |

### 💡 Usage Tips

**✅ ทำได้:**
- Detect acceleration/deceleration
- Impact detection (collision)
- Motion pattern analysis
- Tilt/orientation measurement
- Free-fall detection

**❌ หลีกเลี่ยง:**
- ลืม gravity compensation (9.81 m/s²)
- ใช้สำหรับ absolute position
- ไม่ทำ data smoothing
- ไม่ enable() sensor

### ⚠️ Limitations

```
1. Gravity Interference
   ┌─ Level Robot ─┐
   │ X:   0.0 m/s² │  ✓ No motion
   │ Y:   0.0 m/s² │  ✓ No sideways
   │ Z:   9.81 m/s² │  ⚠️ Gravity!
   └────────────────┘
   
   Must subtract 9.81 from Z-axis!

2. Noise & Vibration
   - Small vibrations = big noise
   - Solution: Apply smoothing filter
   
3. Accumulation Error
   - Double-integral needed for position
   - Error accumulates quickly
   - Use with encoder/LIDAR
```

### 🔧 Gravity Compensation & Filtering

```python
import numpy as np

GRAVITY = 9.81  # m/s²
HISTORY_SIZE = 5

def read_and_filter_accel(accel_raw):
    """Read accelerometer with gravity compensation"""
    
    # Compensate gravity (remove from Z)
    accel_compensated = {
        'x': accel_raw[0],
        'y': accel_raw[1],
        'z': accel_raw[2] - GRAVITY,  # ← Remove gravity!
    }
    
    # Store in history for smoothing
    history.append(accel_compensated)
    if len(history) > HISTORY_SIZE:
        history.pop(0)
    
    # Apply moving average filter
    accel_smooth = {
        'x': np.mean([h['x'] for h in history]),
        'y': np.mean([h['y'] for h in history]),
        'z': np.mean([h['z'] for h in history]),
    }
    
    return accel_smooth

def detect_impact(accel, threshold=20.0):
    """Detect collisions/impacts"""
    magnitude = np.sqrt(
        accel['x']**2 + accel['y']**2 + accel['z']**2
    )
    return magnitude > threshold
```

### 📊 Acceleration Magnitude Reference

```
Magnitude = √(X² + Y² + Z²)

Example Scenarios:
  0-1 m/s²:    Idle/slow drift
  1-2 m/s²:    Gentle acceleration
  2-5 m/s²:    Normal acceleration
  5-10 m/s²:   Rapid movement
  10-20 m/s²:  Strong acceleration
  > 20 m/s²:   IMPACT! 💥
```

---

## 🌍 Practical Examples

### Phase 1: Acceleration

```
Speed: 0 → 0.5 m/s
Time: 2 seconds
Acceleration: ~0.25 m/s²

Dashboard shows:
🚀 X-axis increasing
📊 Magnitude increasing
```

### Phase 2: Constant Velocity

```
Speed: 0.5 m/s (steady)
Time: maintaining
Acceleration: ~0 m/s²

Dashboard shows:
➡️ X-axis near zero (except gravity on Z)
📊 Magnitude low and steady
```

### Phase 3: Deceleration

```
Speed: 0.5 → 0 m/s
Time: 2 seconds
Acceleration: ~-0.25 m/s²

Dashboard shows:
🔻 X-axis negative
📊 Magnitude decreasing
```

---

## 📝 Customization

### Modify Impact Threshold

```python
IMPACT_THRESHOLD = 15.0  # More sensitive (lower = more sensitive)
IMPACT_THRESHOLD = 30.0  # Less sensitive (higher = less sensitive)
```

### Change Motion Profile

```python
# Aggressive acceleration
MAX_SPEED = 1.5
ACCEL_TIME = 0.5  # Quick acceleration

# Smooth, gentle motion
MAX_SPEED = 0.3
ACCEL_TIME = 3.0  # Gentle acceleration
```

### Add Custom Motion Patterns

```python
def execute_motion_pattern(self):
    """Complex motion for testing"""
    # Accelerate forward
    self.accelerate(speed=0.8, duration=2.0)
    
    # Turn left (lateral acceleration)
    self.turn_left(angle=90, duration=1.5)
    
    # Stop suddenly (high deceleration = high Z)
    self.emergency_stop()
```

---

## 📚 Related Resources

- 📖 [Webots Accelerometer](https://cyberbotics.com/doc/reference/accelerometer)
- 🔗 [Physics: Linear Acceleration](https://en.wikipedia.org/wiki/Acceleration)
- 🌍 [Gravity Reference](https://en.wikipedia.org/wiki/Standard_gravity)
- 🤖 [ROS 2 IMU Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)
- 👀 [031 IMU Controller](../031_imu_controller/) (Complementary sensor)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **No acceleration detected** | Ensure motor is running; check enable() |
| **Gravity not removed** | Verify robot is level; check calibration |
| **Impact not detected** | Lower IMPACT_THRESHOLD; increase movement speed |
| **Noisy readings** | Increase HISTORY_SIZE for more smoothing |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
