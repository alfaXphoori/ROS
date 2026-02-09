# 🌊 022 - Distance Sensor Controller

> **Level 2.2 - Exteroceptive Range Sensing | Smooth Proactive Obstacle Avoidance**

---

## 📌 Overview

The **Distance Sensor Controller** demonstrates **proactive obstacle avoidance** using IR/Ultrasonic sensors. Unlike touch sensors that react AFTER collision, distance sensors detect obstacles BEFORE contact and smoothly adjust speed and direction—the robot navigates like a skilled driver!

### ✨ Key Features

- 📡 5 DistanceSensors (front, front-left, front-right, left, right)
- 🎨 Beautiful real-time visual feedback with color-coded sensor bars
- ⚙️ Configurable safety thresholds (speed, distance, turn ratio)
- 🔄 Adaptive turning based on available space
- 🧱 Wall-following behavior for intelligent navigation
- 🚫 Smooth collision-free movement (never touches obstacles!)
- 📊 Real-time visual status display

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `022_distance_sensor_controller.py` | Proactive navigator script |
| `022_distance_sensor.wbt` | Webots world file |
| `022_distance_sensor_controller.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/022_distance_sensor.wbt
```

### Step 2️⃣: Run Autonomous Navigator

```bash
ros2 run ce_webots 022_distance_sensor_controller
```

### Real-Time Dashboard

```
================================================================================
🤖 AUTONOMOUS WALKER - Configuration
================================================================================
  Max Speed:           0.8 rad/s
  Avoid Distance:      0.35 m
  Side Adjust Dist:    0.2 m
  Turn Speed Ratio:    0.8
================================================================================

┌─ 📡 SENSOR READINGS ────────────────────────────────────────────────────────
│  Front Left:  🟢 ░░░░░░░░░░░░░░░░░░░░ 0.95m
│  Front:       🟢 ░░░░░░░░░░░░░░░░░░   0.87m
│  Front Right: 🟢 ░░░░░░░░░░░░░░░░░░░░ 1.00m
│  Left:        🟢 ░░░░░░░░░░░░░░░░░░░░ 0.98m
│  Right:       🟢 ░░░░░░░░░░░░░░░░░░░  0.92m
└──────────────────────────────────────────────────────────────────────────────
  ➤ Action: 🟢 FORWARD

┌─ 📡 SENSOR READINGS ────────────────────────────────────────────────────────
│  Front Left:  🟡 ▓▓▓▓▓▓▓▓            0.42m
│  Front:       🟡 ▓▓▓▓▓▓▓▓▓           0.48m
│  Front Right: 🟢 ░░░░░░░░░░░░░░░░░░░░ 0.95m
│  Left:        🔴 ████                 0.18m
│  Right:       🟢 ░░░░░░░░░░░░░░░░░░░░ 1.00m
└──────────────────────────────────────────────────────────────────────────────
  ➤ Action: 🔴 ⬅️  TURN LEFT

┌─ 📡 SENSOR READINGS ────────────────────────────────────────────────────────
│  Front Left:  🟢 ░░░░░░░░░░░░░░░░░░░░ 0.92m
│  Front:       🟢 ░░░░░░░░░░░░░░░░░░░░ 0.95m
│  Front Right: 🟢 ░░░░░░░░░░░░░░░░░░░░ 0.88m
│  Left:        🟡 ▓▓▓▓▓▓              0.32m
│  Right:       🟢 ░░░░░░░░░░░░░░░░░░░░ 0.95m
└──────────────────────────────────────────────────────────────────────────────
  ➤ Action: 🟡 ADJUST RIGHT
```

**Visual Features:**
- 🟢 **Green bars:** Safe distance (> 0.5m)
- 🟡 **Yellow bars:** Warning zone (0.2m - 0.5m)
- 🔴 **Red bars:** Danger zone (< 0.2m)

---

## 🔧 How It Works

### 1. Initialize Distance Sensors

```python
# Get sensor devices
self.sensor_front_left = self.robot.getDevice('front_left_sensor')
self.sensor_front = self.robot.getDevice('front_sensor')
self.sensor_front_right = self.robot.getDevice('front_right_sensor')
self.sensor_left = self.robot.getDevice('left_sensor')
self.sensor_right = self.robot.getDevice('right_sensor')

# Enable all sensors (required!)
for sensor in [self.sensor_front_left, self.sensor_front,
               self.sensor_front_right, self.sensor_left,
               self.sensor_right]:
    sensor.enable(self.timestep)
```

### 2. Read Sensor Values

```python
def read_sensors(self):
    """Get current distance measurements"""
    return {
        'front_left': self.sensor_front_left.getValue(),
        'front': self.sensor_front.getValue(),
        'front_right': self.sensor_front_right.getValue(),
        'left': self.sensor_left.getValue(),
        'right': self.sensor_right.getValue(),
    }
```

### 3. Multi-Level Decision Logic

```python
def navigate(self, distances):
    front = distances['front']
    front_left = distances['front_left']
    front_right = distances['front_right']
    left = distances['left']
    right = distances['right']
    
    # LEVEL 1: Emergency - Obstacle very close
    if front < 0.2:
        return TURN_AWAY()  # Immediate aggressive turn
    
    # LEVEL 2: Warning - Moderate obstacle
    elif front < 0.35:
        return CHOOSE_TURN(front_left, front_right)
    
    # LEVEL 3: Mild - Keep distance from sides
    elif left < 0.2 or right < 0.2:
        return ADJUST_SIDE()
    
    # LEVEL 4: Clear - Safe navigation
    else:
        return MOVE_FORWARD()
```

### 4. Smooth Turning Logic

```python
def choose_turn_direction(self, front_left, front_right):
    """Turn toward more open space"""
    if front_left > front_right:
        return TURN_LEFT()
    else:
        return TURN_RIGHT()
```

---

## 📊 Sensor Information

### DistanceSensor Specification

| Property | Value |
|----------|-------|
| **Type** | IR or Ultrasonic sensor |
| **Return Value** | Distance in meters (float) |
| **Range** | Typically 0.05m to 2.0m |
| **Reading** | Continuous (every timestep) |
| **Positions** | Front, front-left, front-right, left, right |

### Sensor Layout

```
         Front
           📡
          / | \
         /  |  \
     Left   |  Right  Front-Left & Front-Right
       📡   |   📡    form a cone for better
           🤖         front obstacle detection
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ Distance sensor integration
- ✅ Range-based obstacle detection
- ✅ Multi-sensor decision logic
- ✅ Proactive collision avoidance
- ✅ Smooth navigation strategies
- ✅ Speed adaptation based on obstacles

---

## � Sensor Knowledge: DistanceSensor (IR/Ultrasonic)

### 🎯 How It Works

**DistanceSensor** emits light/sound, then measures the time it bounces back to calculate distance:

```
Sensor Emits → Light/Sound → Bounces → Returns
                └─→ Time = t milliseconds
                └─→ Distance = t × speed_of_light / 2
```

### 📊 Specifications

| Property | Value | Note |
|---------|-------|------|
| **Type** | IR or Ultrasonic | Depends on Webots setup |
| **Detection Range** | 0.05 - 2.0 m | ~5cm to 2 meters |
| **Accuracy** | ±0.05 m | ±5cm error |
| **Field of View (FOV)** | ~15-30 degrees | Focused beam |
| **Response Time** | ~10-20ms | Fairly fast |
| **Color Impact** | ⚠️ May affect | Dark colors absorb light |
| **Angle Impact** | ⚠️ Significant | Must be perpendicular |

### 💡 Usage Tips

**✅ Do:**
- Proactive obstacle avoidance
- Distance measurement (0.2-1.5m best)
- Multiple sensors for scanning
- Wall-following

**❌ Avoid:**
- Using under 0.1m (too close)
- Using over 2.0m (unreliable)
- Reflective/mirror objects (confuses sensor)
- Forgetting to enable() sensor

### ⚠️ Limitations

```
1. Dead Zone
   ╔════════════════════════╗
   │ < 0.05m: Can't measure │ ❌ Too close
   │ 0.05-0.2m: Unreliable  │ ⚠️  Margin area
   │ 0.2-2.0m: Optimal ✓    │ ✅ Best range
   │ > 2.0m: Unreliable     │ ⚠️  Too far
   ╚════════════════════════╝

2. Color Sensitivity
   - Black surfaces: Absorb light → longer range
   - White surfaces: Reflect more → shorter effective
   
3. Angle Dependency
   - Best at 0° (perpendicular)
   - Degrades at angles > 30°
```

### 🔧 Sensor Configuration & Tuning

```python
# Place sensors strategically
sensors = {
    'front': (0.15, 0, 0.08),     # Forward
    'front_left': (0.12, 0.06, 0.08),   # 45° left
    'front_right': (0.12, -0.06, 0.08),  # 45° right
    'left': (0.05, 0.10, 0.08),   # Side
    'right': (0.05, -0.10, 0.08), # Side
}

# Reading interpretation
distance = sensor.getValue()

if distance < 0.2:
    threat_level = "CRITICAL"      # Red zone
elif distance < 0.35:
    threat_level = "WARNING"       # Yellow zone  
elif distance < 0.6:
    threat_level = "CAUTION"       # Orange zone
else:
    threat_level = "SAFE"          # Green zone
```

---

## 🔍 Multi-Level Decision Making

| Aspect | Reactive (Touch) | Proactive (Distance) |
|--------|---|---|
| **Collision** | ❌ Happens first | ✅ Prevented |
| **Response Time** | ⚠️ After impact | ✅ Before contact |
| **Navigation** | Jerky, reset-based | Smooth, flowing |
| **Efficiency** | ❌ Wastes energy | ✅ Smooth travel |
| **Precision** | ❌ Rough | ✅ Fine-grained |

---

## 📝 Configuration Parameters

All behavior parameters are customizable:

```python
class AutonomousWalker:
    # === Configuration ===
    MAX_SPEED = 0.8           # Maximum wheel speed (rad/s)
    AVOID_DIST = 0.35          # Obstacle avoidance distance (m)
    SIDE_ADJUST_DIST = 0.2    # Side wall adjustment distance (m)
    TURN_SPEED_RATIO = 0.8     # Turn speed multiplier
    SLOW_SPEED_RATIO = 0.5     # Slow speed multiplier
```

### Tuning Guide

```
Too Aggressive (collides):
  → Increase AVOID_DIST (0.35 → 0.50)
  → Decrease MAX_SPEED (0.8 → 0.5)

Too Cautious (stops too much):
  → Decrease AVOID_DIST (0.35 → 0.25)
  → Increase MAX_SPEED (0.8 → 1.0)

Won't follow walls:
  → Increase SIDE_ADJUST_DIST (0.2 → 0.3)

Too jerky:
  → Decrease TURN_SPEED_RATIO (0.8 → 0.5)
```

---

## 📚 Related Resources

- 📖 [Webots DistanceSensor](https://cyberbotics.com/doc/reference/distancesensor)
- 🤖 [Obstacle Avoidance Algorithms](https://en.wikipedia.org/wiki/Obstacle_avoidance)
- 🔗 [ROS 2 Sensor Messages](https://docs.ros2.org/latest/api/sensor_msgs/)
- 👀 [021 Touch Sensor](../021_touch_sensor_controller/) (Reactive alternative)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot still collides** | Increase AVOID_DIST; decrease speed |
| **Robot won't move** | Check sensor enable; verify thresholds |
| **Erratic behavior** | Check for sensor errors; verify sensor placement |
| **Doesn't follow walls** | Adjust SIDE_ADJUST_DIST higher |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
