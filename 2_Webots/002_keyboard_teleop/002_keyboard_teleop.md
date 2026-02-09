# 002 Keyboard Teleop

**Level:** Tutorial 00 - Basic Control  
**Type:** User interface (runs in terminal)  
**Purpose:** Manual robot control via keyboard input

## Overview

The keyboard teleop controller provides **human-in-the-loop control** for testing and manually driving the robot. It reads keyboard input and publishes velocity commands to `/cmd_vel`.

**Key Features:**
- Real-time keyboard input (no Enter key needed)
- Variable speed control (+/- keys)
- Multiple movement modes (forward, turn, curves)
- Clean terminal interface

**Files:**
- Controller: `ce_webots/002_keyboard_teleop.py`
- Entry point: `ros2 run ce_webots 002_keyboard_teleop`

**Requires:** [001_simple_robot_controller](001_simple_robot_controller.md) running in Webots

---

## Quick Start

### Launch

```bash
# Terminal 1 - Start Webots
webots ~/ros2_ws/src/ce_webots/worlds/001_basic_control.wbt

# Terminal 2 - Run robot controller
ros2 run ce_webots 001_simple_robot_controller

# Terminal 3 - Run keyboard teleop
ros2 run ce_webots 002_keyboard_teleop
```

### Controls

```
============================================================
Keyboard Teleop Controller
============================================================
Controls:
    Q   W   E          Speed Control
    A   S   D          +  Increase
        X              -  Decrease

Legend:
W : Forward          Q : Forward-Left
S : Backward         E : Forward-Right  
A : Turn Left        X : Stop
D : Turn Right       ESC : Quit
============================================================
Current: v=0.50 m/s, ω=0.50 rad/s
```

---

## How It Works

### 1. Initialize ROS2 Publisher

```python
from geometry_msgs.msg import Twist

self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
```

### 2. Set Terminal to Raw Mode

```python
import sys, tty, termios

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

try:
    tty.setraw(fd)  # Read keys without Enter
    
    while True:
        key = sys.stdin.read(1)  # Single character
        process_key(key)
        
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
```

**Why raw mode?**
- Normal terminal requires Enter key
- Raw mode reads keys immediately
- Better for real-time control

### 3. Map Keys to Velocities

```python
def process_key(self, key):
    if key == 'w' or key == 'W':
        linear = self.linear_speed
        angular = 0.0
    elif key == 'a' or key == 'A':
        linear = 0.0
        angular = self.angular_speed
    elif key == 'q' or key == 'Q':
        linear = self.linear_speed
        angular = self.angular_speed
    elif key == 'x' or key == 'X':
        linear = 0.0
        angular = 0.0
    # ... more keys
```

### 4. Publish Twist Message

```python
def publish_velocity(self, linear, angular):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    self.publisher.publish(twist)
    
    self.update_display(linear, angular)
```

### 5. Speed Control

```python
def increase_speed(self):
    self.linear_speed += 0.1
    self.angular_speed += 0.1
    self.print_status()

def decrease_speed(self):
    self.linear_speed = max(0.1, self.linear_speed - 0.1)
    self.angular_speed = max(0.1, self.angular_speed - 0.1)
    self.print_status()
```

---

## Key Mapping

### Movement Keys

| Key | Linear (m/s) | Angular (rad/s) | Description |
|-----|--------------|-----------------|-------------|
| W | +speed | 0.0 | Forward |
| S | -speed | 0.0 | Backward |
| A | 0.0 | +speed | Turn left (spin) |
| D | 0.0 | -speed | Turn right (spin) |
| Q | +speed | +speed | Forward-left curve |
| E | +speed | -speed | Forward-right curve |
| X | 0.0 | 0.0 | Stop |

### Control Keys

| Key | Action |
|-----|--------|
| + | Increase speed |
| - | Decrease speed |
| ESC | Exit program |
| h | Show help |

---

## Code Structure

```python
class KeyboardTeleopController(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Create publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Default speeds
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Display instructions
        self.print_instructions()
    
    def get_key(self):
        """Read single keypress"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    break
                
                self.process_key(key)
                
        except KeyboardInterrupt:
            pass
        finally:
            # Stop robot
            self.publish_velocity(0.0, 0.0)

def main():
    rclpy.init()
    controller = KeyboardTeleopController()
    controller.run()
    controller.destroy_node()
    rclpy.shutdown()
```

---

## ROS2 Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | On keypress | Velocity commands |

**Message example:**
```python
# Forward at 0.5 m/s
twist = Twist()
twist.linear.x = 0.5
twist.angular.z = 0.0
```

---

## Usage Patterns

### Basic Driving

```
1. Press W - robot moves forward
2. Press X - robot stops
3. Press S - robot reverses
4. Press X - robot stops
```

### Turning

```
1. Press A - robot spins left
2. Press X - stop spinning
3. Press D - robot spins right
4. Press X - stop
```

### Smooth Curves

```
1. Press Q - forward-left curve
2. Hold for desired turn amount
3. Press W - straighten out
```

### Speed Adjustment

```
1. Press + - increase to 0.6 m/s
2. Drive around
3. Press - - decrease to 0.5 m/s
4. Try tighter maneuvers
```

---

## Troubleshooting

### Keys don't respond

**Problem:** Terminal not focused or in wrong mode

**Fix:**
1. Click on terminal running keyboard teleop
2. Restart controller (Ctrl+C, rerun)
3. Make sure terminal supports raw mode

### Robot doesn't move

**Problem:** Robot controller not running

**Fix:**
```bash
# Check if robot controller is active
ros2 node list
# Should show: /simple_robot_controller

# If missing, start it
ros2 run ce_webots 001_simple_robot_controller
```

### Can't stop robot

**Problem:** Last command still active

**Quick fix:**
```bash
# From any terminal
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Special characters appear instead of movement

**Problem:** Terminal encoding issue

**Fix:** Use ASCII keys (W/A/S/D), avoid special characters

---

## Exercises

### Beginner

**Exercise 1: Drive a Square**

Manual precision practice:
```
1. W for 2 seconds → forward 1m
2. X → stop
3. A for 1.5 seconds → turn 90° left
4. X → stop
5. Repeat 4 times
```

**Exercise 2: Figure-8 Pattern**

Smooth continuous control:
```
1. Q (forward-left) for 3 seconds
2. E (forward-right) for 3 seconds
3. Repeat to complete figure-8
```

### Intermediate

**Exercise 3: Add Velocity Display**

Show current commanded velocity:

```python
def publish_velocity(self, linear, angular):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    self.publisher.publish(twist)
    
    # Display current command
    print(f'\rCmd: v={linear:+.2f} m/s, ω={angular:+.2f} rad/s', 
          end='', flush=True)
```

**Exercise 4: Record and Replay**

Save key sequence:

```python
self.recording = []

def record_key(self, key, timestamp):
    self.recording.append((key, timestamp))

def replay_recording(self):
    for key, delay in self.recording:
        time.sleep(delay)
        self.process_key(key)
```

### Advanced

**Exercise 5: Add Joystick Support**

Use pygame for analog control:

```python
import pygame

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

while True:
    # Read analog axes
    linear = joystick.get_axis(1) * -0.5   # Forward/back
    angular = joystick.get_axis(0) * 0.5   # Left/right
    
    self.publish_velocity(linear, angular)
```

**Exercise 6: Safety Features**

Add dead-man switch:

```python
# Require holding spacebar to move
if key == ' ':
    self.enabled = True
else:
    self.enabled = False

if not self.enabled:
    # Override all commands to stop
    linear = 0.0
    angular = 0.0
```

---

## Alternative: Command Line Control

Don't want keyboard interface? Use direct ROS2 commands:

```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## Related Controllers

**Part of Tutorial 00:**
- [001_simple_robot_controller.md](001_simple_robot_controller.md) - Receives commands from this teleop

**Next Level:**
- [012_keyboard_with_distance.md](012_keyboard_with_distance.md) - Adds encoder feedback

---

## Reference

### Default Settings

```python
LINEAR_SPEED = 0.5    # m/s
ANGULAR_SPEED = 0.5   # rad/s
SPEED_INCREMENT = 0.1 # m/s per +/- press
```

### Terminal Control Codes

```python
'\x1b'    # ESC key
'\r'      # Enter
'\x7f'    # Backspace
' '       # Space
```

### ANSI Escape Codes

```python
'\033[2J'      # Clear screen
'\033[H'       # Move cursor to top
'\033[K'       # Clear line
'\r'           # Carriage return (overwrite line)
```

---

**Status:** ✅ Production ready  
**Dependencies:** Python termios, ROS2 Humble/Jazzy  
**See also:** Tutorial 00 - Basic Control
