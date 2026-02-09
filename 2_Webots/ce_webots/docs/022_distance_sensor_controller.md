# 022 Distance Sensor Controller

**Level:** 2.2 - Exteroceptive Range Sensing  
**Type:** Proactive autonomous controller  
**Purpose:** Navigate smoothly without collisions using distance sensors

## Overview

The distance sensor controller demonstrates **proactive obstacle avoidance** using IR/Ultrasonic sensors. Unlike touch sensors that react after collision, distance sensors detect obstacles BEFORE contact and smoothly adjust speed and direction.

**Key Features:**
- 5 DistanceSensors (front, front-left, front-right, left, right)
- Configurable parameters (speed, distance thresholds)
- Beautiful real-time visual feedback with color-coded sensor bars
- Adaptive turning based on available space
- Wall-following behavior for navigation
- Smooth collision-free movement
- No physical contact required

**Files:**
- Controller: `ce_webots/022_distance_sensor_controller.py`
- World: `worlds/022_distance_sensor.wbt`
- Entry point: `ros2 run ce_webots 022_distance_sensor_controller`

**Mission:** Navigate through obstacles automatically, slowing down as they approach

---

## Quick Start

### Launch

```bash
# Terminal 1 - Start Webots
webots ~/ros2_ws/src/ce_webots/worlds/022_distance_sensor.wbt

# Terminal 2 - Run autonomous navigation
ros2 run ce_webots 022_distance_sensor_controller
```

### Expected Behavior

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
- 🟢 Green bars: Safe distance (> 0.5m)
- 🟡 Yellow bars: Warning zone (0.2m - 0.5m)
- 🔴 Red bars: Danger zone (< 0.2m)
- Real-time action status with emojis
- Configuration display on startup

## Configuration Parameters

All behavior parameters are configurable at the top of the class:

```python
class AutonomousWalker(Node):
    def __init__(self):
        super().__init__('autonomous_walker')
        
        # === Configuration Parameters ===
        self.MAX_SPEED = 0.8           # Maximum wheel speed (rad/s)
        self.AVOID_DIST = 0.35          # Obstacle avoidance distance threshold (meters)
        self.SIDE_ADJUST_DIST = 0.2    # Side wall adjustment distance (meters)
        self.TURN_SPEED_RATIO = 0.8    # Speed ratio when turning (0.0 - 1.0)
```

**Parameter Details:**

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `MAX_SPEED` | 0.8 rad/s | 0.1 - 2.0 | Maximum forward wheel velocity |
| `AVOID_DIST` | 0.35 m | 0.15 - 1.0 | Distance threshold to trigger turning |
| `SIDE_ADJUST_DIST` | 0.2 m | 0.1 - 0.5 | Distance for wall-following adjustments |
| `TURN_SPEED_RATIO` | 0.8 | 0.3 - 1.0 | Speed multiplier when turning |

**Tuning Tips:**
- **Increase `AVOID_DIST`** for more cautious navigation
- **Decrease `MAX_SPEED`** if robot moves too fast
- **Adjust `TURN_SPEED_RATIO`** for sharper (higher) or gentler (lower) turns
- **Modify `SIDE_ADJUST_DIST`** to change wall-following sensitivity

---

## How It Works

### 1. Initialize Distance Sensors

```python
# Get 5 distance sensor devices
self.ds_front = self.robot.getDevice('ds_front')
self.ds_front_left = self.robot.getDevice('ds_front_left')
self.ds_front_right = self.robot.getDevice('ds_front_right')
self.ds_left = self.robot.getDevice('ds_left')
self.ds_right = self.robot.getDevice('ds_right')

# Enable sensors (required!)
self.ds_front.enable(self.timestep)
self.ds_front_left.enable(self.timestep)
self.ds_front_right.enable(self.timestep)
self.ds_left.enable(self.timestep)
self.ds_right.enable(self.timestep)
```

**Sensor placement:**
```
        Front (0°)
           ↑
    FL ↗   |   ↖ FR
      45°      45°
        
Left ←  🤖  → Right
  90°         90°
```

### 2. Read and Convert Sensor Values

```python
def value_to_distance(value):
    """Convert sensor value to distance in meters"""
    if value < 50:
        return float('inf')  # No obstacle detected
    elif value > 1000:
        return 0.05  # Very close (min range)
    else:
        # Linear interpolation from lookup table
        # value 50→1000 maps to distance 1.0→0.05
        return 1.0 - ((value - 50) / (1000 - 50)) * (1.0 - 0.05)

# Read all sensors
distances = {
    'front': value_to_distance(self.ds_front.getValue()),
    'front_left': value_to_distance(self.ds_front_left.getValue()),
    'front_right': value_to_distance(self.ds_front_right.getValue()),
    'left': value_to_distance(self.ds_left.getValue()),
    'right': value_to_distance(self.ds_right.getValue())
}
```

**Lookup table in world file:**
```vrml
DistanceSensor {
  lookupTable [
    0    0    0     # No obstacle → 0
    0.05 50   0     # 5cm → 50
    1.0  1000 0     # 1m → 1000
  ]
}
```

### 3. Navigation Logic

```python
def run(self):
    while self.robot.step(self.timestep) != -1:
        # 1. Read all sensor values (in meters)
        d_front = self.get_distance('ds_front')
        d_front_left = self.get_distance('ds_front_left')
        d_front_right = self.get_distance('ds_front_right')
        d_left = self.get_distance('ds_left')
        d_right = self.get_distance('ds_right')

        # 2. Display sensor data with beautiful formatting
        self._print_sensor_status(d_front, d_left, d_right, d_front_left, d_front_right)

        # 3. Decision making logic
        left_speed = self.MAX_SPEED
        right_speed = self.MAX_SPEED
        status = "🟢 FORWARD"
        
        # CASE 1: Obstacle detected - Turn away
        if d_front < self.AVOID_DIST or d_front_left < self.AVOID_DIST or d_front_right < self.AVOID_DIST:
            # Decide turn direction (go to more open side)
            if d_left > d_right:
                # Left side more open -> Turn left
                left_speed = -self.MAX_SPEED * self.TURN_SPEED_RATIO
                right_speed = self.MAX_SPEED * self.TURN_SPEED_RATIO
                status = "🔴 ⬅️  TURN LEFT"
            else:
                # Right side more open -> Turn right
                left_speed = self.MAX_SPEED * self.TURN_SPEED_RATIO
                right_speed = -self.MAX_SPEED * self.TURN_SPEED_RATIO
                status = "🔴 ➡️  TURN RIGHT"
        
        # CASE 2: Path clear - Check for wall following adjustments
        else:
            if d_left < self.SIDE_ADJUST_DIST:
                # Too close to left wall -> Adjust right
                left_speed += 0.1
                right_speed -= 0.1
                status = "🟡 ADJUST RIGHT"
            elif d_right < self.SIDE_ADJUST_DIST:
                # Too close to right wall -> Adjust left
                left_speed -= 0.1
                right_speed += 0.1
                status = "🟡 ADJUST LEFT"
        
        # Display action and command motors
        print(f"  ➤ Action: {status}")
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
```

**Decision Tree:**
```
Read Sensors
    |
    ├─ Front sensor < AVOID_DIST?
    │   YES ─> Turn to more open side (LEFT/RIGHT)
    │   NO  ─┐
    │        |
    │        ├─ Left sensor < SIDE_ADJUST_DIST?
    │        │   YES ─> Adjust right
    │        │   NO  ─┐
    │        │        |
    │        │        ├─ Right sensor < SIDE_ADJUST_DIST?
    │        │        │   YES ─> Adjust left
    │        │        │   NO  ─> Forward
```

```python
def calculate_navigation(self, distances):
    # Find closest front obstacle
    min_front = min([
        distances['front'],
        distances['front_left'],
        distances['front_right']
    ])
    
    # CASE 1: VERY CLOSE - Emergency stop and turn
    if min_front < STOP_DISTANCE:  # < 0.15m
        # Determine turn direction
        if distances['left'] > distances['right']:
            turn_direction = 1  # Turn left (more space)
        else:
            turn_direction = -1  # Turn right
        
        left_speed = -TURN_SPEED * turn_direction
        right_speed = TURN_SPEED * turn_direction
        state = "AVOIDING"
    
    # CASE 2: APPROACHING - Gradual slowdown
    elif min_front < SLOW_DISTANCE:  # 0.15m - 0.4m
        # Adaptive speed: closer = slower
        speed_factor = (min_front - 0.15) / (0.4 - 0.15)
        speed_factor = max(0.2, min(1.0, speed_factor))  # 20%-100%
        forward_speed = MAX_SPEED * speed_factor
        
        # Add steering based on side sensors
        steering = 0.0
        if distances['left'] < distances['right']:
            steering = -0.3  # Steer right
        elif distances['right'] < distances['left']:
            steering = 0.3   # Steer left
        
        left_speed = (forward_speed + steering) / WHEEL_RADIUS
        right_speed = (forward_speed - steering) / WHEEL_RADIUS
        state = "CAUTIOUS"
    
    # CASE 3: CLEAR PATH - Full speed
    else:  # > 0.4m
        forward_speed = MAX_SPEED  # 0.4 m/s
        
        # Minor steering adjustments
        steering = 0.0
        if distances['left'] < SAFE_DISTANCE:
            steering = -0.1  # Slight right
        if distances['right'] < SAFE_DISTANCE:
            steering = 0.1   # Slight left
        
        left_speed = (forward_speed + steering) / WHEEL_RADIUS
        right_speed = (forward_speed - steering) / WHEEL_RADIUS
        state = "EXPLORING"
    
    return left_speed, right_speed, state
```

---

## Beautiful Visual Output

### Sensor Display

The controller includes a beautiful real-time sensor visualization:

```python
def _print_sensor_status(self, front, left, right, front_left, front_right):
    """Print beautiful formatted sensor readings"""
    def get_bar(distance, max_dist=1.0):
        """Generate a visual bar representing distance"""
        bar_length = int((distance / max_dist) * 20)
        bar_length = min(20, max(0, bar_length))
        
        if distance < 0.2:
            color = "🔴"  # Danger - very close
            bar = "█" * bar_length
        elif distance < 0.5:
            color = "🟡"  # Warning - close
            bar = "▓" * bar_length
        else:
            color = "🟢"  # Safe - far
            bar = "░" * bar_length
        
        return f"{color} {bar:<20} {distance:.2f}m"
```

**Color Coding:**
- 🔴 **Red** (< 0.2m): Danger zone - immediate action needed
- 🟡 **Yellow** (0.2m - 0.5m): Warning zone - caution required
- 🟢 **Green** (> 0.5m): Safe zone - clear path

**Bar Types:**
- `█` Solid blocks: Very close obstacles
- `▓` Dense blocks: Moderate distance
- `░` Light blocks: Far/safe distance

### Startup Banner

Displays configuration on startup:

```
================================================================================
🤖 AUTONOMOUS WALKER - Configuration
================================================================================
  Max Speed:           0.8 rad/s
  Avoid Distance:      0.35 m
  Side Adjust Dist:    0.2 m
  Turn Speed Ratio:    0.8
================================================================================
```

### Action Status

Real-time action indicators:
- 🟢 **FORWARD** - Moving straight ahead
- 🔴 **⬅️ TURN LEFT** - Avoiding obstacle, turning left
- 🔴 **➡️ TURN RIGHT** - Avoiding obstacle, turning right  
- 🟡 **ADJUST LEFT** - Minor correction, moving slightly left
- 🟡 **ADJUST RIGHT** - Minor correction, moving slightly right

---

## Navigation Behaviors

### 1. Obstacle Avoidance (Primary)

**Trigger:** Any front sensor < `AVOID_DIST` (default: 0.35m)

**Action:**
```python
# Compare left vs right clearance
if d_left > d_right:
    # Turn left (more space on left)
    left_speed = -MAX_SPEED * TURN_SPEED_RATIO
    right_speed = MAX_SPEED * TURN_SPEED_RATIO
else:
    # Turn right (more space on right)
    left_speed = MAX_SPEED * TURN_SPEED_RATIO
    right_speed = -MAX_SPEED * TURN_SPEED_RATIO
```

**Example:**
```
Front obstacle detected at 0.28m
Left clearance: 0.45m
Right clearance: 0.82m
→ Turn RIGHT (more space)
```

### 2. Wall Following (Secondary)

**Trigger:** Side sensor < `SIDE_ADJUST_DIST` (default: 0.2m) AND no front obstacle

**Action:**
```python
if d_left < SIDE_ADJUST_DIST:
    # Too close to left wall
    left_speed += 0.1   # Speed up left wheel
    right_speed -= 0.1  # Slow down right wheel
    # → Robot veers right
elif d_right < SIDE_ADJUST_DIST:
    # Too close to right wall
    left_speed -= 0.1   # Slow down left wheel
    right_speed += 0.1  # Speed up right wheel
    # → Robot veers left
```

**Purpose:** Smooth navigation along walls without collision

---

## Adaptive Speed Control

### Distance Thresholds

```python
SAFE_DISTANCE = 0.8   # > 0.8m: Full speed, no worries
SLOW_DISTANCE = 0.4   # 0.4-0.8m: Start slowing down
STOP_DISTANCE = 0.15  # < 0.15m: Emergency stop and turn
```

### Speed Calculation

```
Distance  | Speed Factor | Actual Speed | State
----------|--------------|--------------|----------
> 0.8m    | 100%         | 0.40 m/s     | EXPLORING
0.6m      | 69%          | 0.28 m/s     | CAUTIOUS
0.4m      | 38%          | 0.15 m/s     | CAUTIOUS
0.2m      | 7%           | 0.03 m/s     | CAUTIOUS
< 0.15m   | 0%           | 0.00 m/s     | AVOIDING
```

**Formula:**
```python
speed_factor = (distance - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
speed_factor = max(0.2, min(1.0, speed_factor))  # Clamp 20%-100%
actual_speed = MAX_SPEED * speed_factor
```

---

## Multi-Sensor Fusion

### Sensor Roles

**Primary (front sensors):**
- Front center: Straight-ahead detection
- Front-left: Left diagonal obstacles
- Front-right: Right diagonal obstacles
- **Determines:** Speed and emergency stops

**Secondary (side sensors):**
- Left: Wall/obstacle on left
- Right: Wall/obstacle on right
- **Determines:** Steering adjustments

### Decision Priority

```
1. Emergency avoidance (any front < 0.15m)
   → Immediate stop and turn
   
2. Adaptive slowing (any front < 0.4m)
   → Gradual deceleration + steering
   
3. Steering adjustments (side sensors)
   → Minor course corrections
   
4. Full speed ahead (all clear)
   → Maximum speed, straight path
```

### Example Scenario

**Situation:** Approaching corner obstacle

```
Sensors:
  front: 0.3m        → Trigger slowdown
  front_left: 0.2m   → Closest sensor
  left: 0.5m         → Some space on left
  right: 1.2m        → More space on right

Decision:
  Speed: 0.3m → 60% speed factor → 0.24 m/s
  Steering: Steer right (away from left obstacle)
  
Result: Smooth curve to the right while slowing down
```

---

## Code Structure

```python
class AutonomousWalker(Node):
    # Configuration Parameters
    MAX_SPEED = 0.8              # Maximum wheel speed (rad/s)
    AVOID_DIST = 0.35            # Obstacle avoidance threshold (m)
    SIDE_ADJUST_DIST = 0.2       # Wall following threshold (m)
    TURN_SPEED_RATIO = 0.8       # Turn speed multiplier
    
    def __init__(self):
        super().__init__('autonomous_walker')
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Setup motors
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Setup sensors
        self.sensors = {}
        sensor_names = ['ds_front', 'ds_front_left', 'ds_front_right', 
                       'ds_left', 'ds_right']
        for name in sensor_names:
            self.sensors[name] = self.robot.getDevice(name)
            self.sensors[name].enable(self.timestep)
        
        # Start execution
        self.run()
    
    def get_distance(self, sensor_name):
        """Convert raw value (0-1000) to meters"""
        value = self.sensors[sensor_name].getValue()
        return value / 1000.0
    
    def _print_sensor_status(self, front, left, right, front_left, front_right):
        """Print beautiful formatted sensor readings with color-coded bars"""
        # Creates visual representation of sensor data
        # Color codes: 🔴 danger, 🟡 warning, 🟢 safe
        # Bar styles: █ solid, ▓ dense, ░ light
        pass
    
    def run(self):
        """Main control loop"""
        # Display configuration
        # Read sensors continuously
        # Make navigation decisions
        # Command motors
        # Display status
        pass
```

---

## ROS2 Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/distance_sensors` | `Float32MultiArray` | 3 Hz | Distance readings (5 sensors) |
| `/robot_state` | `std_msgs/String` | On change | Current navigation state |
| `/obstacle_warning` | `std_msgs/Bool` | On event | Close obstacle alert |

### Monitoring

```bash
# Watch distance readings
ros2 topic echo /distance_sensors

# Output:
data:
- 0.85      # front
- 1.2       # front_left
- 0.92      # front_right
- inf       # left (no obstacle)
- 1.5       # right
---

# Watch state changes
ros2 topic echo /robot_state

# Output:
data: 'EXPLORING'
---
data: 'CAUTIOUS'
---
data: 'AVOIDING'
---
```

---

## Troubleshooting

### Robot still collides with obstacles

**Causes:**
- `AVOID_DIST` too small
- `MAX_SPEED` too high
- Sensor not detecting properly

**Fixes:**
```python
self.AVOID_DIST = 0.5    # Increase safety margin
self.MAX_SPEED = 0.6     # Reduce speed
```

### Robot gets stuck in corners

**Cause:** Both left and right sides blocked equally

**Solution:** Add timeout counter:
```python
# In __init__
self.stuck_counter = 0

# In run loop
if status.startswith("🔴 TURN"):
    self.stuck_counter += 1
    if self.stuck_counter > 50:  # Stuck for 50 iterations
        # Force reverse
        left_speed = -self.MAX_SPEED * 0.3
        right_speed = -self.MAX_SPEED * 0.3
else:
    self.stuck_counter = 0
```

### Sensors show no readings

**Check:**
1. Are sensors enabled in `__init__`?
2. Are sensor names correct in world file?
3. Is `timestep` set correctly?

**Verify:**
```python
# Add debug output
raw_value = self.sensors['ds_front'].getValue()
self.get_logger().info(f'Raw sensor value: {raw_value}')
```

### Output doesn't display properly

**Issue:** Terminal doesn't support Unicode/emojis

**Solution:** Use simple ASCII version:
```python
def get_bar(distance, max_dist=1.0):
    bar_length = int((distance / max_dist) * 20)
    if distance < 0.2:
        color = "[RED]"
        bar = "#" * bar_length
    elif distance < 0.5:
        color = "[YEL]"
        bar = "=" * bar_length
    else:
        color = "[GRN]"
        bar = "." * bar_length
    return f"{color} {bar:<20} {distance:.2f}m"
```

---

## Exercises

### Beginner

**Exercise 1: Adjust Safety Margins**

Modify the configuration parameters:
```python
self.AVOID_DIST = 0.5        # More cautious
self.SIDE_ADJUST_DIST = 0.3  # Earlier wall adjustments
```

Question: How does this affect navigation behavior?

**Exercise 2: Change Robot Speed**

```python
self.MAX_SPEED = 1.2  # Faster
self.MAX_SPEED = 0.4  # Slower
```

Question: At what speed does collision avoidance start to fail?

**Exercise 3: Tune Turning Aggressiveness**

```python
self.TURN_SPEED_RATIO = 0.3   # Gentle turns
self.TURN_SPEED_RATIO = 1.0   # Sharp turns
```

Observe the difference in turning behavior.

### Intermediate

**Exercise 4: Add Gradual Slowdown**

Instead of constant speed, slow down as obstacles approach:

```python
# In run loop, before decision logic
min_front = min(d_front, d_front_left, d_front_right)

if min_front < self.AVOID_DIST:
    # Obstacle close - turn as before
    pass
else:
    # Path clear - adjust speed based on distance
    if min_front < 0.7:
        speed_factor = min_front / 0.7
        left_speed = self.MAX_SPEED * speed_factor
        right_speed = self.MAX_SPEED * speed_factor
        status = f"🟡 SLOWING ({speed_factor*100:.0f}%)"
```

**Exercise 5: Add Sensor Averaging**

Reduce sensor noise with running average:

```python
# In __init__
self.sensor_history = {
    'ds_front': [],
    'ds_front_left': [],
    'ds_front_right': [],
    'ds_left': [],
    'ds_right': []
}
self.HISTORY_SIZE = 5

def get_distance_averaged(self, sensor_name):
    """Get averaged distance from sensor"""
    current = self.get_distance(sensor_name)
    
    # Add to history
    self.sensor_history[sensor_name].append(current)
    if len(self.sensor_history[sensor_name]) > self.HISTORY_SIZE:
        self.sensor_history[sensor_name].pop(0)
    
    # Return average
    return sum(self.sensor_history[sensor_name]) / len(self.sensor_history[sensor_name])
```

**Exercise 6: Wall Following Mode**

Implement consistent wall following:

```python
TARGET_WALL_DISTANCE = 0.3  # Desired distance from wall
WALL_FOLLOW_KP = 0.5        # Proportional gain

# In run loop
if d_left < 0.8:  # Wall detected on left
    error = d_left - TARGET_WALL_DISTANCE
    steering_adjustment = error * WALL_FOLLOW_KP
    
    left_speed = self.MAX_SPEED + steering_adjustment
    right_speed = self.MAX_SPEED - steering_adjustment
    status = f"🟢 WALL FOLLOW (error: {error:.2f}m)"
```

### Advanced

**Exercise 7: Predictive Obstacle Avoidance**

Calculate time-to-collision and act earlier:

```python
def calculate_time_to_collision(distance, speed):
    """Calculate seconds until collision"""
    if speed <= 0:
        return float('inf')
    return distance / speed

# In run loop
robot_speed = 0.05  # Approximate linear speed in m/s
ttc_front = calculate_time_to_collision(d_front, robot_speed)

if ttc_front < 3.0:  # Less than 3 seconds to collision
    # Start avoiding early
    pass
```

**Exercise 8: Add ROS2 Publishers**

Publish sensor data and robot state:

```python
from std_msgs.msg import Float32MultiArray, String

# In __init__
self.sensor_pub = self.create_publisher(
    Float32MultiArray, '/distance_sensors', 10)
self.status_pub = self.create_publisher(
    String, '/robot_status', 10)

# In run loop
sensor_msg = Float32MultiArray()
sensor_msg.data = [d_front, d_front_left, d_front_right, d_left, d_right]
self.sensor_pub.publish(sensor_msg)

status_msg = String()
status_msg.data = status
self.status_pub.publish(status_msg)
```

**Exercise 9: Custom Visual Display Themes**

Create different visual themes:

```python
# Minimal theme
def get_bar_minimal(distance):
    if distance < 0.2:
        return f"! {distance:.2f}m"
    elif distance < 0.5:
        return f"~ {distance:.2f}m"
    else:
        return f"  {distance:.2f}m"

# Matrix theme
def get_bar_matrix(distance):
    bar_length = int((distance / 1.0) * 10)
    bar = "█" * bar_length + "░" * (10 - bar_length)
    return f"[{bar}] {distance:.2f}m"
```

---

## Comparison: Touch vs Distance

| Feature | 021 (Touch) | 022 (Distance) |
|---------|-------------|----------------|
| **Detection** | Contact required | No contact needed |
| **Response** | After collision | Before collision |
| **Speed** | Constant until hit | Adaptive to proximity |
| **Path** | Bouncing, jerky | Smooth, flowing |
| **Efficiency** | Low (many collisions) | High (no collisions) |
| **Safety** | Requires impact | Prevents impact |
| **Real Use** | Emergency backup | Primary navigation |

**Evolution:** Contact sensing → Range sensing → Vision (next levels!)

---

## Related Controllers

**Same Level:**
- [021_touch_sensor_controller.md](021_touch_sensor_controller.md) - Reactive (after contact)

**Previous Level:**
- [012_keyboard_with_distance.md](012_keyboard_with_distance.md) - Manual with encoders

**Next Level:**
- Level 3.1 - IMU (coming soon)

---

## Reference

### Distance Sensor Specifications

| Parameter | Value | Unit |
|-----------|-------|------|
| Type | DistanceSensor (IR) | - |
| Min Range | 0.05 | m |
| Max Range | 1.0 | m |
| Count | 5 | sensors |
| Update Rate | 32 | ms |
| Accuracy | ±0.01 | m (simulation) |

### Navigation Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `MAX_SPEED` | 0.8 rad/s | 0.1 - 2.0 | Maximum wheel velocity |
| `AVOID_DIST` | 0.35 m | 0.15 - 1.0 | Obstacle avoidance threshold |
| `SIDE_ADJUST_DIST` | 0.2 m | 0.1 - 0.5 | Wall following threshold |
| `TURN_SPEED_RATIO` | 0.8 | 0.3 - 1.0 | Turn speed multiplier |

### Robot States

| State | Symbol | Trigger | Behavior |
|-------|--------|---------|----------|
| FORWARD | 🟢 | All clear | Full speed ahead |
| TURN LEFT | 🔴 ⬅️ | Front obstacle, left more open | Rotate left |
| TURN RIGHT | 🔴 ➡️ | Front obstacle, right more open | Rotate right |
| ADJUST LEFT | 🟡 | Right wall too close | Slight left correction |
| ADJUST RIGHT | 🟡 | Left wall too close | Slight right correction |

### Sensor Placement

```
Sensor          | Angle | Position
----------------|-------|------------------
ds_front        | 0°    | 0.2m forward
ds_front_left   | 45°   | 0.18m forward, 0.08m left
ds_front_right  | -45°  | 0.18m forward, 0.08m right
ds_left         | 90°   | Center, 0.11m left
ds_right        | -90°  | Center, 0.11m right
```

### Formula: Adaptive Speed

```python
# Linear scaling between thresholds
speed_factor = (distance - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
speed_factor = max(0.2, min(1.0, speed_factor))
actual_speed = MAX_SPEED * speed_factor

# Example:
# distance = 0.3m
# speed_factor = (0.3 - 0.15) / (0.4 - 0.15) = 0.6
# actual_speed = 0.4 * 0.6 = 0.24 m/s
```

---

**Status:** ✅ Production ready  
**Behavior:** Proactive obstacle avoidance  
**See also:** Level 2.2 - Distance Sensors
