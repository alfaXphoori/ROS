# 🎮📏 012 - Keyboard with Distance

> **Level 1.1 - Proprioceptive Sensing | Manual Control with Real-Time Distance Feedback**

---

## 📌 Overview

The **Keyboard Distance Controller** combines manual teleoperation with live encoder feedback. Unlike autonomous missions, this lets you explore the environment while seeing real-time distance measurements for each wheel and the total traveled distance—perfect for learning how encoders measure movement!

### ✨ Key Features

- ⌨️ Keyboard teleoperation (like 002)
- 📊 Real-time distance display (total + per wheel)
- 🔄 Distance reset capability for sub-missions
- 🚫 Non-blocking input for smooth encoder updates
- 🛡️ Encoder stabilization (prevents NaN values)
- 📈 Visual progress tracking

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `012_keyboard_with_distance.py` | Teleop + feedback controller |
| `012_keyboard_with_distance.wbt` | Webots world file |
| `012_keyboard_with_distance.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/011_wheel_encoder.wbt
```

### Step 2️⃣: Run Manual Controller

```bash
ros2 run ce_webots 012_keyboard_with_distance
```

### Live Display

```
============================================================
🎮 KEYBOARD CONTROL WITH DISTANCE TRACKING
============================================================
Controls:
  W - Forward          A - Turn Left
  S - Backward         D - Turn Right
  X - Stop             R - Reset Distance
  ESC - Quit
============================================================
Speed: 0.50 m/s | Turn: 0.50 rad/s
============================================================

📏 Distance: 2.345m | L: 2.341m | R: 2.349m | ↑ Forward
```

**Watch:** Distance updates live as you drive!

---

## 🔧 How It Works

### 1. Initialize Encoders with Stabilization

```python
# Get encoder devices
self.left_encoder = self.robot.getDevice('left_wheel_sensor')
self.right_encoder = self.robot.getDevice('right_wheel_sensor')

# Enable encoders
self.left_encoder.enable(self.timestep)
self.right_encoder.enable(self.timestep)

# Wait for stabilization (prevents NaN)
for _ in range(5):
    self.robot.step(self.timestep)

# Initialize tracking
self.prev_left_encoder = self.left_encoder.getValue()
self.prev_right_encoder = self.right_encoder.getValue()
```

**Key:** 5-step stabilization ensures valid first readings!

### 2. Non-Blocking Keyboard Input

```python
import select

# Check if key pressed (don't wait)
if select.select([sys.stdin], [], [], 0)[0]:
    key = self.get_key()
    self.process_key(key)
```

**Why select.select()?**
- ✅ Doesn't block simulation
- ✅ Checks if input available
- ✅ Continues if no key pressed
- ✅ Smooth encoder updates every cycle

### 3. Update Distance Continuously

```python
def update_distance(self):
    # Read current positions
    left_pos = self.left_encoder.getValue()
    right_pos = self.right_encoder.getValue()
    
    # Calculate deltas
    left_delta = left_pos - self.prev_left_encoder
    right_delta = right_pos - self.prev_right_encoder
    
    # Convert to meters
    left_dist = left_delta * self.WHEEL_RADIUS
    right_dist = right_delta * self.WHEEL_RADIUS
    
    # Update totals
    self.left_distance += left_dist
    self.right_distance += right_dist
    self.total_distance += (left_dist + right_dist) / 2.0
    
    # Save for next iteration
    self.prev_left_encoder = left_pos
    self.prev_right_encoder = right_pos
```

### 4. Display Real-Time Feedback

```python
def display_status(self):
    direction = self.get_direction_emoji()
    print(f"📏 Distance: {self.total_distance:.3f}m | "
          f"L: {self.left_distance:.3f}m | "
          f"R: {self.right_distance:.3f}m | "
          f"{direction}")
```

---

## 🎮 Controls

```
┌─────────────────────────────────────────┐
│      ↑ W (Forward)                      │
│  ← A (Turn Left)  D (Turn Right) →      │
│      ↓ S (Backward)                     │
│      X (Stop)                           │
└─────────────────────────────────────────┘

Special Commands:
  R - Reset distance to 0.0m
  M - Toggle display mode (compact/detailed)
  ESC - Quit
```

---

## 📊 Real-Time Data

### Distance Tracking Information

| Metric | Description |
|--------|-------------|
| **Total Distance** | Average of left & right wheel distances |
| **Left Wheel** | Distance traveled by left wheel |
| **Right Wheel** | Distance traveled by right wheel |
| **Difference** | Indicates turning (slippage or intentional) |

### Interpreting Wheel Difference

```
Straight Movement:
  Total: 2.500m | L: 2.498m | R: 2.502m
  ✅ Difference ~0.004m (normal, within tolerance)

Left Turn:
  Total: 1.250m | L: 1.200m | R: 1.300m
  ⚠️  Difference 0.100m (slippage OR intentional turn)
```

---

## 🎓 Learning Outcomes

After exploring with this controller, you'll understand:

- ✅ Real-time encoder data collection
- ✅ Distance accumulation over time
- ✅ Non-blocking input handling
- ✅ Differential drive odometry
- ✅ Sensor feedback integration
- ✅ How wheel slippage affects measurements

---

## � Sensor Knowledge: PositionSensor (Wheel Encoder)

### 🎯 How It Works (PositionSensor)

**PositionSensor** วัดตำแหน่งเชิงมุมของล้อ ให้ข้อมูลแบบ real-time:

```
🔄 Angular Position Reading
    └─→ Old: 10.5 rad
        New: 10.628 rad
        Delta: 0.128 rad
        
🎯 Convert to Distance
    Distance = 0.128 rad × 0.08 m = 0.01024 m (≈ 1 cm)
```

### 📊 Specifications

| Property | Value |
|---------|-------|
| **Type** | Rotational Position Sensor |
| **Units** | Radians |
| **Accuracy** | ±0.001 rad |
| **Read Range** | Full rotation (0-2π+) |
| **Drift** | Minimal (accumulates over time) |

### 💡 Usage Tips

**✅ Do:**
- Real-time distance tracking
- Detect wheel slippage (compare L/R)
- Wall-following feedback

**❌ Avoid:**
- Using single encoder for dead reckoning
- Forgetting stabilization delay
- Not calibrating wheel radius

### ⚠️ Limitations

```
1. Slippage Effect
   ─────────────────────────────
   On Smooth Surface:   ✅ 0-1% error
   On Carpet:          ⚠️  2-5% error
   On Grass:           ❌ 10-15% error

2. Accumulation
   - Errors accumulate from calibration errors
   - Must verify with IMU for long distances

3. Resolution
   - Not very good for short distances < 1cm
```

### 🔧 Calibration Tips

```python
# Measure wheel diameter accurately
true_diameter = 0.16  # Use caliper/ruler
WHEEL_RADIUS = true_diameter / 2

# Test on flat smooth surface
# Move robot 1 meter exactly
# Compare encoder reading vs actual
calibration_factor = actual_distance / encoder_distance
```

---

## �🔍 Encoder vs. Time: Visual Comparison

**Time-Based (Bad) 🚫**
```
Speed: 0.5 m/s × 2 seconds = 1.0 m
But what if wheel slipped? ❌ Don't know!
```

**Encoder-Based (Good) ✅**
```
Left Encoder:  old=5 → new=11.428 rad (6.428 rad × 0.08m = 0.514m)
Right Encoder: old=5 → new=11.324 rad (6.324 rad × 0.08m = 0.506m)
Average: (0.514 + 0.506) / 2 = 0.510m ✅
Exact distance known, regardless of wheel condition!
```

---

## 📝 Customization

### Adjust Speed

```python
self.speed = 0.5          # Linear speed (m/s)
self.turn_speed = 0.5     # Angular speed (rad/s)
```

### Change Display Update Rate

```python
DISPLAY_INTERVAL = 5  # Update display every N steps
```

### Add Distance Threshold Alerts

```python
if self.total_distance > 10.0:
    print("⚠️  You've traveled 10 meters!")
```

---

## 📚 Related Resources

- 📖 [Webots PositionSensor](https://cyberbotics.com/doc/reference/positionsensor)
- 🔗 [Python select module](https://docs.python.org/3/library/select.html)
- 📐 [Odometry Principles](https://en.wikipedia.org/wiki/Odometry)
- 🤖 [ROS 2 Twist Message](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- 👀 [011 Wheel Encoder Mission](../011_wheel_encoder_mission/)
- 👀 [002 Keyboard Teleop](../002_keyboard_teleop/)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Distance not updating** | Check encoder enable; verify step() called |
| **Display frozen** | Ensure terminal focus; check select() timeout |
| **Erratic measurements** | Increase stabilization steps; check encoder enable |
| **NaN values** | Restart; verify encoder initialization |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
