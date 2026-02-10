# 🎮 002 - Keyboard Teleop

> **Tutorial 00 - Basic Control | Manual Robot Control via Keyboard**

---

## 📌 Overview

Tutorial 00: Keyboard Teleoperation Controller

Basic keyboard control for differential drive robot.
Learn manual control before adding sensors.

Controls:
    Q   W   E       Speed Control:
    A   S   D       +  Increase speed
        X           -  Decrease speed
                    ESC  Quit

Movement:
    W - Forward          Q - Forward-Left
    S - Backward         E - Forward-Right
    A - Spin Left        X - Stop
    D - Spin Right

### ✨ Key Features

- ⌨️ Real-time keyboard input (no Enter key needed)
- 🎚️ Variable speed control with +/- keys
- 🔄 Multiple movement modes (forward, backward, turn, curves)
- 🖥️ Clean terminal interface with intuitive layout
- 🔗 Integrates seamlessly with robot controller

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `002_keyboard_teleop.py` | Main teleop controller script |
| `002_keyboard_teleop.wbt` | Webots world file |


---

## 🚀 Quick Start

### Prerequisites

- 001_simple_robot_controller must be running in Webots
- ROS 2 Jazzy installed and sourced

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/001_basic_control.wbt
```

### Step 2️⃣: Run Robot Controller

```bash
ros2 run ce_webots 001_simple_robot_controller
```

### Step 3️⃣: Run Keyboard Teleop

```bash
ros2 run ce_webots 002_keyboard_teleop
```

---

## 🎮 Controls

```
============================================================
       ⌨️  KEYBOARD TELEOP CONTROLLER
============================================================

Movement:
    W / ↑ → Forward           Q → Forward-Left
    S / ↓ → Backward          E → Forward-Right
    A / ← → Turn Left         X → Stop
    D / → → Turn Right        ESC → Quit

Speed Control:
    + → Increase Speed
    - → Decrease Speed

============================================================
Current Speed: 0.50 m/s | Turn Rate: 0.50 rad/s
============================================================
```

**Pro Tips:**
- Hold keys for continuous movement
- Adjust speed before complex maneuvers
- Use diagonal movements (Q, E) for curves

---

## 🔧 How It Works

### 1. Initialize ROS2 Publisher

The controller creates a publisher for velocity commands:

```python
from geometry_msgs.msg import Twist

self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
```

### 2. Set Terminal to Raw Mode

Raw mode enables reading individual key presses without requiring Enter:

```python
import sys, tty, termios

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

try:
    tty.setraw(fd)  # Read keys immediately
    while True:
        key = sys.stdin.read(1)  # Single character input
        process_key(key)
        
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
```

**Why Raw Mode?**
- ✅ Responds immediately to key presses
- ✅ No need to press Enter
- ✅ Better for real-time control
- ✅ Professional teleoperation experience

### 3. Map Keys to Velocities

Each key maps to linear and angular velocity components:

```python
def process_key(self, key):
    if key == 'w' or key == 'W':
        # Move forward
        linear = self.speed
        angular = 0.0
    elif key == 'a' or key == 'A':
        # Turn left
        linear = 0.0
        angular = self.turn_speed
    # ... more keys ...
```

### 4. Publish Velocity Commands

Twist messages contain linear and angular components:

```python
twist = Twist()
twist.linear.x = linear_velocity    # m/s
twist.angular.z = angular_velocity  # rad/s

self.publisher.publish(twist)
```

---

## 📊 Topic Reference

| Topic | Message Type | Direction | Purpose |
|-------|--------------|-----------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | **OUT** | Velocity commands to robot |

---

## 🎓 Learning Outcomes

After using this controller, you will understand:

- ✅ Terminal raw mode for interactive input
- ✅ ROS 2 publisher pattern for topics
- ✅ Twist message structure and semantics
- ✅ Real-time keyboard handling
- ✅ Teleoperation best practices

---

## 📝 Customization

### Modify Speed Limits

Edit these parameters in the script:

```python
self.speed = 0.5          # Linear velocity (m/s)
self.turn_speed = 0.5     # Angular velocity (rad/s)
self.speed_increment = 0.1  # Speed adjustment per keypress
```

### Add New Key Bindings

Extend the key handler:

```python
def process_key(self, key):
    # ... existing code ...
    elif key == 'j':  # Your new key
        # Your custom behavior
        pass
```

---

## 📚 Related Resources

- 📖 [ROS 2 Geometry Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- 🔗 [Terminal I/O Control (termios)](https://docs.python.org/3/library/termios.html)
- 🤖 [Teleoperation Guide](https://docs.ros.org/en/ros2/Tutorials.html)
- 👀 [001 Simple Robot Controller](../001_simple_robot_controller/)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Keys not responding** | Ensure terminal is focused on the teleoperation window |
| **Robot moving erratically** | Reduce speed values; check motor calibration |
| **No motion** | Verify robot controller is running; check `/cmd_vel` topic |
| **Terminal settings broken** | Press Ctrl+C and the raw mode should restore automatically |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
