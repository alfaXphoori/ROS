# 021 Touch Sensor Controller

**Level:** 2.1 - Exteroceptive Contact Sensing  
**Type:** Reactive autonomous controller  
**Purpose:** Explore environment and react to collisions

## Overview

The touch sensor controller demonstrates **reactive behaviors** using bumpers. The robot explores autonomously, detects obstacles through physical contact, and executes escape maneuvers using a simple state machine.

**Key Features:**
- 3 TouchSensors (front, left, right bumpers)
- State machine: EXPLORING → BACKING_UP → TURNING → repeat
- Collision detection and counting
- Immediate reactive response
- ROS2 event publishing

**Files:**
- Controller: `ce_webots/021_touch_sensor_controller.py`
- World: `worlds/021_touch_sensor.wbt`
- Entry point: `ros2 run ce_webots 021_touch_sensor_controller`

**Mission:** Explore autonomously, bump into obstacles, escape and continue

---

## Quick Start

### Launch

```bash
# Terminal 1 - Start Webots
webots ~/ros2_ws/src/ce_webots/worlds/021_touch_sensor.wbt

# Terminal 2 - Run reactive controller
ros2 run ce_webots 021_touch_sensor_controller
```

### Expected Behavior

```
============================================================
🤖 LEVEL 2.1: TOUCH SENSOR CONTROLLER
============================================================
Mission: Explore and avoid obstacles using bumpers
Behaviors:
  - EXPLORING: Move forward, search for obstacles
  - COLLISION: Back up and turn away
  - REACTIVE: Immediate response to touch
============================================================
🚀 Starting exploration...

🔍 Exploring... (collisions: 0)
🔍 Exploring... (collisions: 0)
💥 COLLISION #1 - FRONT bumper!
🔄 Turning left...
✅ Maneuver complete - resuming exploration
🔍 Exploring... (collisions: 1)
💥 COLLISION #2 - RIGHT bumper!
🔄 Turning right...
✅ Maneuver complete - resuming exploration
...
```

**Watch:** Robot bumps obstacles, backs up, turns away, and continues!

---

## How It Works

### 1. Initialize Touch Sensors

```python
# Get bumper devices
self.bumper_front = self.robot.getDevice('bumper_front')
self.bumper_left = self.robot.getDevice('bumper_left')
self.bumper_right = self.robot.getDevice('bumper_right')

# Enable touch sensors (required!)
self.bumper_front.enable(self.timestep)
self.bumper_left.enable(self.timestep)
self.bumper_right.enable(self.timestep)
```

### 2. Read Bumper Values

```python
def read_bumpers(self):
    """Read all touch sensor values"""
    # TouchSensor returns > 0 when contact detected
    front = self.bumper_front.getValue()
    left = self.bumper_left.getValue()
    right = self.bumper_right.getValue()
    
    return {
        'front': front > 0,      # Boolean: True if pressed
        'left': left > 0,
        'right': right > 0,
        'any': front > 0 or left > 0 or right > 0
    }
```

**Key:** getValue() returns 0 (no contact) or 1 (contact detected)

### 3. Handle Collision

```python
def handle_collision(self, bumpers):
    """React to collision detection"""
    if not bumpers['any']:
        return False  # No collision
    
    self.collision_count += 1
    
    # Determine which bumper was hit
    if bumpers['front']:
        direction = "FRONT"
        turn_direction = 1 if self.collision_count % 2 == 0 else -1
    elif bumpers['left']:
        direction = "LEFT"
        turn_direction = -1  # Turn right (away from obstacle)
    elif bumpers['right']:
        direction = "RIGHT"
        turn_direction = 1   # Turn left (away from obstacle)
    
    self.get_logger().info(f'💥 COLLISION #{self.collision_count} - {direction} bumper!')
    
    # Transition to BACKING_UP state
    self.state = "BACKING_UP"
    self.state_timer = 0.0
    self.turn_direction = turn_direction
    
    # Publish collision event
    collision_msg = Bool()
    collision_msg.data = True
    self.collision_pub.publish(collision_msg)
    
    return True
```

### 4. State Machine

```python
def update_state_machine(self, dt):
    """Execute behavior based on current state"""
    
    # STATE 1: EXPLORING - Move forward, look for obstacles
    if self.state == "EXPLORING":
        self.set_velocity(FORWARD_SPEED, 0.0)  # 0.3 m/s forward
        
        bumpers = self.read_bumpers()
        if self.handle_collision(bumpers):
            return  # Collision detected, switched to BACKING_UP
    
    # STATE 2: BACKING_UP - Reverse away from obstacle
    elif self.state == "BACKING_UP":
        self.set_velocity(BACKUP_SPEED, 0.0)  # -0.2 m/s backward
        
        if self.state_timer >= BACKUP_TIME:  # 1.0 second
            self.state = "TURNING"
            self.state_timer = 0.0
            self.get_logger().info('🔄 Turning...')
    
    # STATE 3: TURNING - Rotate to new direction
    elif self.state == "TURNING":
        self.set_velocity(0.0, self.turn_direction * TURN_SPEED)  # ±0.5 rad/s
        
        if self.state_timer >= TURN_TIME:  # 1.5 seconds
            self.state = "EXPLORING"
            self.state_timer = 0.0
            self.get_logger().info('✅ Maneuver complete - resuming exploration')
    
    # Update timer
    self.state_timer += dt
```

---

## State Machine Diagram

```
   ┌─────────────┐
   │  EXPLORING  │ ◄─────────────┐
   │ (forward)   │                │
   └──────┬──────┘                │
          │                       │
          │ collision             │
          ↓                       │
   ┌─────────────┐                │
   │ BACKING_UP  │                │
   │ (reverse)   │                │
   └──────┬──────┘                │
          │                       │
          │ timer >= 1.0s         │
          ↓                       │
   ┌─────────────┐                │
   │   TURNING   │                │
   │ (rotate)    │                │
   └──────┬──────┘                │
          │                       │
          │ timer >= 1.5s         │
          └───────────────────────┘
```

---

## Reactive Behavior Pattern

### Stimulus → Response

```
Stimulus: Front bumper pressed
     ↓
Perception: Obstacle detected ahead
     ↓
Decision: Back up and turn away
     ↓
Action: Execute escape maneuver
     ↓
Result: Clear of obstacle, resume exploration
```

### No Planning, No Memory

**Reactive control characteristics:**
- ✓ Immediate response (no delay)
- ✓ Simple logic (if/then rules)
- ✓ Robust (works in unknown environments)
- ✗ No learning (repeats mistakes)
- ✗ Can get stuck (local minima)
- ✗ Inefficient paths (bouncing around)

---

## Code Structure

```python
class TouchSensorController(Node):
    # Behavior parameters
    FORWARD_SPEED = 0.3    # m/s
    TURN_SPEED = 0.5       # rad/s
    BACKUP_SPEED = -0.2    # m/s
    BACKUP_TIME = 1.0      # seconds
    TURN_TIME = 1.5        # seconds
    
    def __init__(self):
        # Initialize robot and sensors
        self.robot = Robot()
        self.bumper_front = self.robot.getDevice('bumper_front')
        self.bumper_left = self.robot.getDevice('bumper_left')
        self.bumper_right = self.robot.getDevice('bumper_right')
        
        # State machine
        self.state = "EXPLORING"
        self.state_timer = 0.0
        self.collision_count = 0
        
        # ROS2 publishers
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.collision_pub = self.create_publisher(Bool, '/collision_detected', 10)
    
    def run_step(self):
        """Execute one control step"""
        dt = self.timestep / 1000.0  # Convert to seconds
        
        # Update state machine
        self.update_state_machine(dt)
        
        # Publish current state
        self.publish_state()
        
        # Step simulation
        return self.robot.step(self.timestep) != -1

def main():
    rclpy.init()
    controller = TouchSensorController()
    
    while rclpy.ok() and controller.run_step():
        rclpy.spin_once(controller, timeout_sec=0)
```

---

## ROS2 Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/robot_state` | `std_msgs/String` | 10 Hz | Current behavior state |
| `/collision_detected` | `std_msgs/Bool` | On event | Collision notification |

### Monitoring

```bash
# Watch state transitions
ros2 topic echo /robot_state

# Output:
data: 'EXPLORING'
---
data: 'BACKING_UP'
---
data: 'TURNING'
---
data: 'EXPLORING'
---

# Watch collisions
ros2 topic echo /collision_detected

# Output when bumper pressed:
data: true
---
```

---

## Parameters

### Behavior Tuning

```python
# Forward exploration speed
FORWARD_SPEED = 0.3  # m/s (moderate speed)

# Backup maneuver
BACKUP_SPEED = -0.2  # m/s (slower reverse)
BACKUP_TIME = 1.0    # seconds (how far to back up)

# Turn maneuver
TURN_SPEED = 0.5     # rad/s (rotation speed)
TURN_TIME = 1.5      # seconds (≈ 45° turn)
```

**Adjust these to change behavior!**

---

## Troubleshooting

### Robot doesn't react to collisions

**Check:**
1. Are bumpers enabled? `bumper.enable(timestep)`
2. Does world file have TouchSensor devices?
3. Do bumpers have physics? `physics Physics {}`

**Debug:**
```python
# Log bumper values
bumpers = self.read_bumpers()
self.get_logger().info(
    f'Bumpers: F={bumpers["front"]}, L={bumpers["left"]}, R={bumpers["right"]}'
)
```

### Robot backs through obstacles

**Normal!** Rear collisions aren't detected (no rear bumper).

**Fix:** Add rear bumper in world file (see exercises)

### Robot gets stuck in corners

**Cause:** Escape maneuver insufficient

**Solutions:**
```python
# Increase backup time
BACKUP_TIME = 2.0  # Back up more

# Increase turn time
TURN_TIME = 2.5    # Turn farther away

# Add stuck detection (advanced)
if self.collision_count > 10 and time_elapsed < 30:
    # Too many collisions quickly = stuck
    execute_random_escape()
```

### State transitions too fast

**Problem:** Can't see states change

**Fix:**
```python
# Add delays
BACKUP_TIME = 2.0
TURN_TIME = 3.0

# Or slow down simulation in Webots
```

---

## Exercises

### Beginner

**Exercise 1: Adjust Backup Distance**

```python
BACKUP_TIME = 2.0  # Change from 1.0 to 2.0 seconds
```

Question: Does backing up farther help escape obstacles?

**Exercise 2: Change Turn Amount**

```python
TURN_TIME = 1.0  # Short turn (default 1.5)
TURN_TIME = 2.0  # Long turn
```

Question: What turn duration works best?

**Exercise 3: Monitor Collision Rate**

```bash
# Count collisions in 60 seconds
ros2 topic echo /collision_detected | grep -c "data: true"
```

### Intermediate

**Exercise 4: Add Rear Bumper**

World file modification:

```vrml
# In 02_touch_sensor.wbt, add:
Solid {
  translation -0.21 0 0  # Back of robot
  name "rear_bumper"
  children [
    Shape { ... }  # Orange bumper visual
    TouchSensor {
      name "bumper_rear"
      type "bumper"
    }
  ]
  boundingObject Box { size 0.02 0.25 0.12 }
  physics Physics { mass 0.1 }
}
```

Controller update:

```python
self.bumper_rear = self.robot.getDevice('bumper_rear')
self.bumper_rear.enable(self.timestep)

# In read_bumpers():
rear = self.bumper_rear.getValue()
return {
    ...,
    'rear': rear > 0
}
```

**Exercise 5: Smart Turn Direction**

Instead of alternating, turn away from obstacle:

```python
if bumpers['left']:
    turn_direction = -1  # Turn right
elif bumpers['right']:
    turn_direction = 1   # Turn left
elif bumpers['front']:
    # Turn toward more open side
    # (Need to add side sensors for this!)
    turn_direction = 1
```

### Advanced

**Exercise 6: Stuck Detection**

Detect repeated collisions:

```python
import time

self.recent_collisions = []  # List of timestamps

def handle_collision(self, bumpers):
    now = time.time()
    self.recent_collisions.append(now)
    
    # Remove old collisions (> 10 seconds ago)
    self.recent_collisions = [
        t for t in self.recent_collisions if now - t < 10.0
    ]
    
    # If 5 collisions in 10 seconds → STUCK
    if len(self.recent_collisions) >= 5:
        self.get_logger().warn('STUCK! Executing escape...')
        self.execute_long_escape()
```

**Exercise 7: Adaptive Behaviors**

Slow down after many collisions:

```python
if self.collision_count < 5:
    speed = 0.3  # Normal
elif self.collision_count < 10:
    speed = 0.2  # Cautious
else:
    speed = 0.1  # Very careful
```

---

## Comparison with Distance Sensors

| Feature | 021 (Touch) | 022 (Distance) |
|---------|-------------|----------------|
| **Detection** | Contact required | No contact |
| **Response** | After collision | Before collision |
| **Speed** | Constant until hit | Adaptive |
| **Path** | Bouncing, jerky | Smooth, flowing |
| **Use Case** | Last resort safety | Primary navigation |

**Next level:** [022_distance_sensor_controller.md](022_distance_sensor_controller.md) - Avoid obstacles BEFORE hitting them!

---

## Related Controllers

**Same Level:**
- [022_distance_sensor_controller.md](022_distance_sensor_controller.md) - Proactive avoidance

**Previous Level:**
- [012_keyboard_with_distance.md](012_keyboard_with_distance.md) - Manual with encoders

**Next Level:**
- Level 3.1 - IMU (coming soon)

---

## Reference

### TouchSensor Specifications

| Parameter | Value | Unit |
|-----------|-------|------|
| Type | TouchSensor (bumper) | - |
| Measurement | Contact (binary) | 0 or 1 |
| Response Time | Instant | 0 ms |
| False Positives | None (simulation) | - |

### State Duration

```python
# Typical escape maneuver timeline
t=0.0s:  Collision detected
t=0.0s:  → BACKING_UP state
t=1.0s:  → TURNING state  
t=2.5s:  → EXPLORING state
Total:   2.5 seconds per collision
```

### Differential Drive Math

```python
# Set velocity helper
def set_velocity(self, linear, angular):
    left = (linear - angular * WHEEL_DISTANCE/2) / WHEEL_RADIUS
    right = (linear + angular * WHEEL_DISTANCE/2) / WHEEL_RADIUS
    
    self.left_motor.setVelocity(left)
    self.right_motor.setVelocity(right)

# Examples:
set_velocity(0.3, 0.0)    # Forward
set_velocity(-0.2, 0.0)   # Backward
set_velocity(0.0, 0.5)    # Spin left
set_velocity(0.0, -0.5)   # Spin right
```

---

**Status:** ✅ Production ready  
**Behavior:** Reactive obstacle avoidance  
**See also:** Level 2.1 - Touch Sensors
