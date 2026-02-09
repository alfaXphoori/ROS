# 001 Simple Robot Controller

**Level:** Tutorial 00 - Basic Control  
**Type:** Robot-side controller (runs in Webots)  
**Purpose:** Subscribes to /cmd_vel and controls robot motors

## Overview

The simple robot controller is the **bridge between ROS2 and Webots**. It runs inside the Webots simulation as an external controller and translates ROS2 velocity commands into motor movements.

**Key Features:**
- Subscribes to `/cmd_vel` topic
- Converts Twist messages to wheel speeds
- Uses differential drive kinematics
- Runs continuously in Webots

**Files:**
- Controller: `ce_webots/001_simple_robot_controller.py`
- World: `worlds/001_basic_control.wbt`
- Entry point: `ros2 run ce_webots 001_simple_robot_controller`

---

## Quick Start

### Launch

```bash
# Terminal 1 - Start Webots
webots ~/ros2_ws/src/ce_webots/worlds/00_basic_control.wbt

# Terminal 2 - Run this controller
ros2 run ce_webots 001_simple_robot_controller
```

**Expected output:**
```
[INFO] [simple_robot_controller]: Simple Robot Controller Started
[INFO] [simple_robot_controller]: Waiting for /cmd_vel commands...
```

### Test

```bash
# Terminal 3 - Send test command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

Robot should move forward at 0.5 m/s!

---

## How It Works

### 1. Initialize Webots Robot

```python
from controller import Robot

self.robot = Robot()
self.timestep = int(self.robot.getBasicTimeStep())  # 32ms typically
```

### 2. Get Motor Devices

```python
self.left_motor = self.robot.getDevice('left_motor')
self.right_motor = self.robot.getDevice('right_motor')

# Set to velocity control mode
self.left_motor.setPosition(float('inf'))  # Infinite position
self.right_motor.setPosition(float('inf'))
self.left_motor.setVelocity(0.0)
self.right_motor.setVelocity(0.0)
```

### 3. Subscribe to ROS2 Commands

```python
from geometry_msgs.msg import Twist

self.subscription = self.create_subscription(
    Twist,
    '/cmd_vel',
    self.cmd_vel_callback,
    10
)
```

### 4. Convert Twist to Wheel Speeds

```python
def cmd_vel_callback(self, msg):
    # Extract velocities
    linear = msg.linear.x   # m/s
    angular = msg.angular.z # rad/s
    
    # Differential drive kinematics
    # wheel_speed = (linear ± angular * wheel_distance/2) / wheel_radius
    
    left_speed = (linear - angular * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS
    right_speed = (linear + angular * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS
    
    # Set motor velocities
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)
```

### 5. Main Control Loop

```python
while self.robot.step(self.timestep) != -1:
    rclpy.spin_once(self, timeout_sec=0)
```

**Key:** `robot.step()` advances Webots simulation, `spin_once()` processes ROS2 messages.

---

## Differential Drive Kinematics

### Parameters

```python
WHEEL_RADIUS = 0.08      # meters
WHEEL_DISTANCE = 0.24    # meters (distance between wheels)
```

### Forward Motion (linear = 0.5, angular = 0)

```python
left_speed = (0.5 - 0 * 0.24/2) / 0.08 = 6.25 rad/s
right_speed = (0.5 + 0 * 0.24/2) / 0.08 = 6.25 rad/s
```

Both wheels same speed → straight line

### Turn Left (linear = 0, angular = 0.5)

```python
left_speed = (0 - 0.5 * 0.24/2) / 0.08 = -0.75 rad/s  # backward
right_speed = (0 + 0.5 * 0.24/2) / 0.08 = 0.75 rad/s  # forward
```

Opposite wheel speeds → spin in place

### Curve (linear = 0.5, angular = 0.3)

```python
left_speed = (0.5 - 0.3 * 0.24/2) / 0.08 = 5.8 rad/s
right_speed = (0.5 + 0.3 * 0.24/2) / 0.08 = 6.7 rad/s
```

Right wheel faster → curves left

---

## Code Structure

```python
class SimpleRobotController(Node):
    def __init__(self):
        # 1. Initialize ROS2 node
        super().__init__('simple_robot_controller')
        
        # 2. Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # 3. Get motors
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # 4. Configure motors
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # 5. Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', 
                                self.cmd_vel_callback, 10)
    
    def cmd_vel_callback(self, msg):
        # Convert Twist → wheel speeds
        ...
    
    def run(self):
        # Main loop
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0)

def main():
    rclpy.init()
    controller = SimpleRobotController()
    controller.run()
    rclpy.shutdown()
```

---

## ROS2 Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |

**Twist message structure:**
```python
linear:
  x: 0.5   # Forward/backward (m/s)
  y: 0.0   # Left/right (unused)
  z: 0.0   # Up/down (unused)
angular:
  x: 0.0   # Roll (unused)
  y: 0.0   # Pitch (unused)
  z: 0.5   # Yaw (rotation, rad/s)
```

---

## Troubleshooting

### Robot doesn't move

**Check:**
1. Is controller running? `ros2 node list` should show `/simple_robot_controller`
2. Are commands being published? `ros2 topic echo /cmd_vel`
3. Is Webots simulation running (not paused)?

**Debug:**
```python
# Add logging in callback
def cmd_vel_callback(self, msg):
    self.get_logger().info(
        f'Received: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
    )
```

### Motors not found

**Error:** `Device 'left_motor' not found`

**Fix:** Check world file has correct motor names:
```vrml
RotationalMotor {
  name "left_motor"   # Must match getDevice() call
}
```

### Controller exits immediately

**Problem:** `robot.step()` returns -1

**Cause:** Webots simulation closed or world file error

**Fix:** Make sure Webots is open and world loaded

---

## Exercises

### Beginner

**Exercise 1: Add Logging**

Add speed logging every second:

```python
import time

self.last_log_time = time.time()

def run(self):
    while self.robot.step(self.timestep) != -1:
        if time.time() - self.last_log_time > 1.0:
            self.get_logger().info(
                f'Motor speeds: L={self.left_speed:.2f}, R={self.right_speed:.2f}'
            )
            self.last_log_time = time.time()
        
        rclpy.spin_once(self, timeout_sec=0)
```

### Intermediate

**Exercise 2: Speed Limiting**

Add maximum speed limits:

```python
MAX_WHEEL_SPEED = 5.0  # rad/s

def set_motor_speeds(self, left, right):
    # Clamp to limits
    left = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, left))
    right = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, right))
    
    self.left_motor.setVelocity(left)
    self.right_motor.setVelocity(right)
```

**Exercise 3: Publish Odometry**

Calculate and publish position:

```python
from nav_msgs.msg import Odometry

self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

# In main loop: calculate position from wheel speeds
# Publish odometry
```

### Advanced

**Exercise 4: PID Speed Control**

Add closed-loop speed control using encoders:

```python
# Get encoder sensors
self.left_encoder = self.robot.getDevice('left_wheel_sensor')
self.left_encoder.enable(self.timestep)

# Measure actual speed
actual_speed = calculate_speed_from_encoder()

# PID control
error = target_speed - actual_speed
motor_command = pid_controller(error)
```

---

## Related Controllers

**Part of Tutorial 00:**
- [002_keyboard_teleop.md](002_keyboard_teleop.md) - Sends commands to this controller

**Next Level:**
- [011_wheel_encoder_mission.md](011_wheel_encoder_mission.md) - Uses encoders for precise movement

---

## Reference

### Motor Configuration

```python
# Velocity control mode
motor.setPosition(float('inf'))  # Disable position control
motor.setVelocity(speed)         # Set angular velocity (rad/s)
```

### Key Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Wheel radius | 0.08 | m |
| Wheel distance | 0.24 | m |
| Timestep | 32 | ms |
| Max speed | 6.0 | rad/s |

### Coordinate System

```
Webots coordinate system (robot-centric):
- X: Right
- Y: Forward  
- Z: Up

ROS2 Twist:
- linear.x: Forward velocity
- angular.z: Yaw rotation (counterclockwise positive)
```

---

**Status:** ✅ Production ready  
**Dependencies:** Webots R2025a, ROS2 Humble/Jazzy  
**See also:** Tutorial 00 - Basic Control
