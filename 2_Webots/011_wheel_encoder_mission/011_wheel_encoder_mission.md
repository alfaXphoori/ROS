# 011 Wheel Encoder Mission

**Level:** 1.1 - Proprioceptive Sensing  
**Type:** Autonomous controller  
**Purpose:** Move exactly 5.4 meters using encoder feedback

## Overview

The wheel encoder mission demonstrates **closed-loop control** using proprioceptive sensors. The robot moves forward exactly 5.4 meters (from y=-2.7 to y=2.7) by continuously reading wheel encoders and stopping when the target distance is reached.

**Key Features:**
- PositionSensor (wheel encoders) integration
- Closed-loop distance control
- No time-based estimation - pure sensor feedback
- Automatic mission execution

**Files:**
- Controller: `ce_webots/011_wheel_encoder_mission.py`
- World: `worlds/011_wheel_encoder.wbt`
- Entry point: `ros2 run ce_webots 011_wheel_encoder_mission`

**Mission:** Start at y=-2.7m, move forward to y=2.7m (5.4m total), stop automatically

---

## Quick Start

### Launch

```bash
# Terminal 1 - Start Webots
webots ~/ros2_ws/src/ce_webots/worlds/011_wheel_encoder.wbt

# Terminal 2 - Run mission
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

---

## How It Works

### 1. Initialize Encoders

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

### 2. Read Encoder Values

```python
def read_encoders_and_update_distance(self):
    # Read current positions (in radians)
    left_pos = self.left_encoder.getValue()
    right_pos = self.right_encoder.getValue()
    
    # Skip first reading (need delta)
    if self.first_reading:
        self.prev_left_encoder = left_pos
        self.prev_right_encoder = right_pos
        self.first_reading = False
        return
    
    # Calculate change in position
    delta_left = left_pos - self.prev_left_encoder
    delta_right = right_pos - self.prev_right_encoder
    
    # Update previous values
    self.prev_left_encoder = left_pos
    self.prev_right_encoder = right_pos
```

### 3. Convert Radians to Meters

```python
# Formula: distance = angle (radians) × radius
dist_left = delta_left * WHEEL_RADIUS   # 0.08m
dist_right = delta_right * WHEEL_RADIUS

# Average both wheels (differential drive)
delta_distance = (dist_left + dist_right) / 2.0

# Update total
self.distance_traveled += delta_distance
```

### 4. Check Mission Complete

```python
def check_mission_complete(self):
    if self.distance_traveled >= self.target_distance:
        # STOP MOTORS
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Calculate error
        error = abs(self.distance_traveled - self.target_distance)
        error_percent = (error / self.target_distance) * 100
        
        self.get_logger().info(f'✅ MISSION COMPLETE!')
        self.get_logger().info(f'Error: {error:.4f}m ({error_percent:.2f}%)')
```

---

## Encoder Mathematics

### PositionSensor Reading

- **Units:** Radians
- **Range:** Unlimited (accumulates continuously)
- **Direction:** Positive = forward, Negative = backward

### Distance Calculation

```
Distance = Δθ × r

Where:
  Δθ = Change in encoder angle (radians)
  r = Wheel radius (meters)
```

**Example:**
```python
# Encoder changes by 10 radians
delta_angle = 10.0  # radians

# Wheel radius is 0.08m
radius = 0.08

# Distance traveled
distance = 10.0 * 0.08 = 0.8 meters
```

### Why Average Both Wheels?

```python
# Perfect straight line
left_distance = 1.000m
right_distance = 1.000m
average = (1.000 + 1.000) / 2 = 1.000m  ✓

# Slight curve (right wheel slips)
left_distance = 1.000m
right_distance = 0.998m
average = (1.000 + 0.998) / 2 = 0.999m  ✓ (more accurate)

# Sharp turn
left_distance = 0.500m
right_distance = 1.500m
average = (0.500 + 1.500) / 2 = 1.000m  ✓ (forward motion only)
```

---

## Closed-Loop vs Open-Loop

### Open-Loop (Time-Based)

```python
# ❌ Inaccurate - depends on perfect speed, no slip
time.sleep(27.0)  # Assumes 0.2 m/s for 5.4m
# Error: ±10-20%
```

**Problems:**
- Speed varies with battery, surface, load
- Wheel slip not detected
- No feedback correction

### Closed-Loop (Encoder-Based)

```python
# ✅ Accurate - measures actual movement
while distance_traveled < target_distance:
    read_encoders()
    update_distance()
# Error: ±0.5%
```

**Advantages:**
- Measures actual distance traveled
- Compensates for speed variations
- Self-correcting
- High precision

---

## Code Structure

```python
class WheelEncoderMission(Node):
    def __init__(self):
        # Initialize robot and encoders
        self.robot = Robot()
        self.left_encoder = self.robot.getDevice('left_wheel_sensor')
        self.right_encoder = self.robot.getDevice('right_wheel_sensor')
        
        # Mission parameters
        self.target_distance = 5.4  # meters
        self.speed = 0.2            # m/s
        self.distance_traveled = 0.0
        
        # Start mission timer
        self.create_timer(3.0, self.start_mission)
    
    def start_mission(self):
        # Set motors to move forward
        wheel_speed = self.speed / WHEEL_RADIUS
        self.left_motor.setVelocity(wheel_speed)
        self.right_motor.setVelocity(wheel_speed)
    
    def run_step(self):
        if self.mission_active:
            # Update distance from encoders
            self.read_encoders_and_update_distance()
            
            # Check if done
            self.check_mission_complete()
            
            # Log progress
            if self.distance_traveled % 0.1 < 0.01:
                remaining = self.target_distance - self.distance_traveled
                self.get_logger().info(
                    f'📏 Distance: {self.distance_traveled:.3f}m | '
                    f'Remaining: {remaining:.3f}m'
                )

def main():
    rclpy.init()
    controller = WheelEncoderMission()
    
    # Main loop
    while rclpy.ok() and controller.robot.step(controller.timestep) != -1:
        controller.run_step()
        rclpy.spin_once(controller, timeout_sec=0)
```

---

## ROS2 Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/mission_complete` | `std_msgs/Bool` | Once | Mission completion status |

### Monitoring

```bash
# Watch mission completion
ros2 topic echo /mission_complete

# Output when mission completes:
data: true
---
```

---

## Parameters

### Mission Configuration

```python
TARGET_DISTANCE = 5.4  # meters (y=-2.7 to y=2.7)
SPEED = 0.2            # m/s (slower for better accuracy)
WHEEL_RADIUS = 0.08    # meters
WHEEL_DISTANCE = 0.24  # meters
```

### Accuracy Factors

**Typical error sources:**
1. Encoder resolution (Webots: very high)
2. Wheel slip (simulation: minimal)
3. Speed control precision
4. Timestep discretization (32ms)

**Expected accuracy:** ±0.01m (±0.2%)

---

## Troubleshooting

### Distance shows NaN

**Problem:** Encoders not initialized properly

**Fix:** Already handled in code with first_reading flag:
```python
if self.first_reading:
    self.prev_left_encoder = left_pos
    self.prev_right_encoder = right_pos
    self.first_reading = False
    return  # Skip first calculation
```

### Robot doesn't stop at target

**Problem:** Check condition logic

**Debug:**
```python
# Add logging before check
self.get_logger().info(
    f'Checking: {self.distance_traveled:.4f} >= {self.target_distance:.4f}'
)
```

### Distance accumulates even when stopped

**Problem:** Encoders continue to be read

**Fix:** Only update when mission active:
```python
if self.mission_active:
    self.read_encoders_and_update_distance()
```

### Mission never starts

**Problem:** Timer not triggering

**Check:** ROS2 spinning in main loop:
```python
while controller.robot.step(controller.timestep) != -1:
    rclpy.spin_once(controller, timeout_sec=0)  # Required!
```

---

## Exercises

### Beginner

**Exercise 1: Change Target Distance**

```python
self.target_distance = 3.0  # 3 meters instead of 5.4
```

Question: Does accuracy improve or worsen with shorter distances?

**Exercise 2: Increase Speed**

```python
self.speed = 0.4  # Double the speed
```

Question: Does higher speed affect final accuracy?

### Intermediate

**Exercise 3: Two-Point Mission**

Extend to multiple waypoints:

```python
self.waypoints = [2.0, 4.0, 5.4]  # Stop at each point
self.current_waypoint = 0

def check_waypoint_reached(self):
    if self.distance_traveled >= self.waypoints[self.current_waypoint]:
        self.stop_briefly()
        self.current_waypoint += 1
```

**Exercise 4: Reverse Mission**

Add backward movement:

```python
# Mission: forward 5.4m, then backward to start
if self.phase == "FORWARD":
    if self.distance_traveled >= 5.4:
        self.reverse()
        self.phase = "BACKWARD"
elif self.phase == "BACKWARD":
    if self.distance_traveled <= 0.0:
        self.stop()
```

### Advanced

**Exercise 5: Curved Path**

Follow an arc using differential speeds:

```python
# Left wheel slower = curve left
left_speed = 0.15 / WHEEL_RADIUS
right_speed = 0.25 / WHEEL_RADIUS

# Track arc length = average of both wheels
```

**Exercise 6: Odometry Publishing**

Publish continuous odometry:

```python
from nav_msgs.msg import Odometry

self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

def publish_odometry(self):
    odom = Odometry()
    odom.pose.pose.position.x = self.x
    odom.pose.pose.position.y = self.y
    odom.twist.twist.linear.x = self.velocity
    self.odom_pub.publish(odom)
```

---

## Comparison with Manual Control

See [012_keyboard_with_distance.md](012_keyboard_with_distance.md) for manual version with same encoder feedback.

| Feature | This (Autonomous) | Manual Version |
|---------|-------------------|----------------|
| **Control** | Automatic | Human-driven |
| **Distance** | Signed (forward/back) | Absolute (total) |
| **Purpose** | Mission execution | Testing/exploration |
| **Stopping** | Automatic at target | Manual (user decides) |

---

## Related Controllers

**Same Level:**
- [012_keyboard_with_distance.md](012_keyboard_with_distance.md) - Manual control with encoders

**Previous Level:**
- [001_simple_robot_controller.md](001_simple_robot_controller.md) - Basic control (no sensors)

**Next Level:**
- [021_touch_sensor_controller.md](021_touch_sensor_controller.md) - External sensing begins

---

## Reference

### Encoder Specifications

| Parameter | Value | Unit |
|-----------|-------|------|
| Type | PositionSensor | - |
| Measurement | Angular position | radians |
| Range | Unlimited | rad |
| Resolution | Very high (simulation) | - |
| Noise | None (simulation) | - |

### Formula Reference

```python
# Encoder to distance
distance = (angle_radians) × (wheel_radius_meters)

# Distance to encoder
angle = distance / wheel_radius

# Differential drive forward distance
distance = (left_distance + right_distance) / 2

# Rotation angle from encoders
rotation = (right_distance - left_distance) / wheel_distance
```

---

**Status:** ✅ Production ready  
**Accuracy:** ±0.01m typical  
**See also:** Level 1.1 - Wheel Encoders
