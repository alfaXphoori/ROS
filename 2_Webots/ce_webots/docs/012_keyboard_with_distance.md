# 012 Keyboard with Distance

**Level:** 1.1 - Proprioceptive Sensing  
**Type:** Manual control with feedback  
**Purpose:** Interactive driving with real-time distance tracking

## Overview

The keyboard distance controller combines manual control with encoder feedback, showing **live distance measurements** as you drive. Unlike the autonomous mission controller, this lets you explore while learning how encoders measure movement.

**Key Features:**
- Keyboard teleoperation (like 002)
- Real-time distance display (total + per wheel)
- Distance reset capability
- Non-blocking input for smooth control
- Encoder stabilization (prevents NaN)

**Files:**
- Controller: `ce_webots/012_keyboard_with_distance.py`
- World: `worlds/011_wheel_encoder.wbt`
- Entry point: `ros2 run ce_webots 012_keyboard_with_distance`

---

## Quick Start

### Launch

```bash
# Terminal 1 - Start Webots
webots ~/ros2_ws/src/ce_webots/worlds/011_wheel_encoder.wbt

# Terminal 2 - Run manual controller
ros2 run ce_webots 012_keyboard_with_distance
```

### Interface

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

## How It Works

### 1. Initialize Encoders

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
- Doesn't block simulation
- Checks if input available
- Continues if no key pressed
- Smooth encoder updates

### 3. Update Distance Continuously

```python
def update_distance(self):
    # Read current positions
    left_pos = self.left_encoder.getValue()
    right_pos = self.right_encoder.getValue()
    
    # Validate readings (NaN protection)
    if not isinstance(left_pos, (int, float)):
        return
    
    # Calculate deltas
    delta_left = left_pos - self.prev_left_encoder
    delta_right = right_pos - self.prev_right_encoder
    
    # Convert to distance (ABSOLUTE value for total)
    dist_left = abs(delta_left * WHEEL_RADIUS)
    dist_right = abs(delta_right * WHEEL_RADIUS)
    
    # Sanity check (ignore large jumps > 1m)
    if dist_left < 1.0 and dist_right < 1.0:
        self.distance_left += dist_left
        self.distance_right += dist_right
        self.total_distance += (dist_left + dist_right) / 2.0
    
    # Update previous values
    self.prev_left_encoder = left_pos
    self.prev_right_encoder = right_pos
```

### 4. Live Display

```python
def print_status(self):
    # Direction indicator
    if self.current_linear > 0:
        cmd_str = "↑ Forward"
    elif self.current_linear < 0:
        cmd_str = "↓ Backward"
    elif self.current_angular > 0:
        cmd_str = "↺ Turn Left"
    elif self.current_angular < 0:
        cmd_str = "↻ Turn Right"
    else:
        cmd_str = "⏸ Stopped"
    
    # Print with carriage return (overwrites line)
    print(f"\r📏 Distance: {self.total_distance:6.3f}m | "
          f"L: {self.distance_left:5.3f}m | "
          f"R: {self.distance_right:5.3f}m | "
          f"{cmd_str:12s}", end='', flush=True)
```

### 5. Reset Capability

```python
def reset_distance(self):
    """Reset distance counter to zero"""
    self.total_distance = 0.0
    self.distance_left = 0.0
    self.distance_right = 0.0
    self.get_logger().info('\n📏 Distance reset to 0.000m')
```

---

## Key Differences from Mission Controller

| Feature | 011 (Mission) | 012 (Keyboard) |
|---------|---------------|----------------|
| **Control** | Autonomous | Manual |
| **Distance Tracking** | Signed (±) | Absolute (abs) |
| **Purpose** | Reach target | Exploration |
| **Stopping** | Automatic | User-controlled |
| **Reset** | No | Yes (R key) |
| **Display** | Periodic logs | Live continuous |

### Why Absolute Distance?

```python
# Mission controller (signed)
delta_distance = (dist_left + dist_right) / 2.0  # Can be negative
distance_traveled += delta_distance  # 5.4m forward, then -5.4m back = 0m

# Keyboard controller (absolute)
dist_left = abs(delta_left * WHEEL_RADIUS)  # Always positive
total_distance += (dist_left + dist_right) / 2.0  # 5.4m forward, then 5.4m back = 10.8m total movement
```

**Reason:** Users want to know *total movement*, not net displacement!

---

## Controls

### Movement Keys

| Key | Action | Linear | Angular |
|-----|--------|--------|---------|
| W | Forward | +0.5 m/s | 0 |
| S | Backward | -0.5 m/s | 0 |
| A | Turn left | 0 | +0.5 rad/s |
| D | Turn right | 0 | -0.5 rad/s |
| X | Stop | 0 | 0 |

### Special Keys

| Key | Action |
|-----|--------|
| R | Reset distance counter |
| H | Show help |
| ESC | Exit program |

---

## Encoder Stabilization

### The NaN Problem

```python
# ❌ Without stabilization
left_pos = self.left_encoder.getValue()  # First read = NaN or 0.0 (unstable)
delta = left_pos - self.prev  # NaN - 0.0 = NaN
distance += delta * radius    # NaN propagates!
```

### The Solution

```python
# ✅ With stabilization
# Wait 5 simulation steps for sensors to initialize
for _ in range(5):
    self.robot.step(self.timestep)

# NOW read initial values
self.prev_left_encoder = self.left_encoder.getValue()  # Stable!
```

**Why 5 steps?**
- Webots sensors need time to initialize
- First few readings may be unstable
- 5 steps = 160ms (sufficient for all sensors)

---

## Code Structure

```python
class KeyboardWithDistance(Node):
    def __init__(self):
        # Initialize robot and encoders
        self.robot = Robot()
        self.left_encoder = self.robot.getDevice('left_wheel_sensor')
        self.right_encoder = self.robot.getDevice('right_wheel_sensor')
        
        # Enable encoders
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Stabilize (5 steps)
        for _ in range(5):
            self.robot.step(self.timestep)
        
        # Initialize tracking
        self.prev_left_encoder = self.left_encoder.getValue()
        self.prev_right_encoder = self.right_encoder.getValue()
        
        # Distance counters
        self.total_distance = 0.0
        self.distance_left = 0.0
        self.distance_right = 0.0
    
    def run(self):
        import select
        
        while rclpy.ok():
            # Update distance from encoders
            self.update_distance()
            
            # Print live status
            self.print_status()
            
            # Check for keyboard input (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    break
                elif key in 'wWsSaAdDxX':
                    self.process_movement(key)
                elif key in 'rR':
                    self.reset_distance()
            
            # Step simulation
            self.robot.step(self.timestep)
```

---

## Usage Examples

### Measure a Square

```
1. Press R - reset distance
2. Press W - drive forward
3. Count: 1... 2... (watch distance reach ~1.0m)
4. Press X - stop
5. Note exact distance traveled
6. Repeat for 4 sides
7. Total distance ≈ 4m (perimeter of square)
```

### Test Wheel Slip

```
1. Press R - reset
2. Press W - forward 2m
3. Press A - spin left 3 seconds
4. Note: Left wheel negative distance, right positive!
5. Total distance = abs(left) + abs(right) / 2
```

### Encoder Calibration

```
1. Measure real distance (e.g., 1m tape on floor)
2. Press R - reset
3. Drive exactly to tape end
4. Compare encoder reading to actual
5. Calculate error percentage
```

---

## Troubleshooting

### Distance shows NaN

**Cause:** Encoders read before stabilization

**Fix:** Already implemented - 5-step wait before first read

### Distance jumps randomly

**Cause:** Large delta values from sensor glitch

**Fix:** Already implemented - sanity check:
```python
if dist_left < 1.0 and dist_right < 1.0:
    # Accept reading
else:
    # Ignore (sensor spike)
```

### Left/right distances very different

**Normal when:**
- Turning (wheels move at different speeds)
- Spinning (opposite directions)

**Problem if:**
- Driving straight but difference > 5%
- Could indicate wheel slip or calibration issue

### Distance doesn't reset

**Check:** Did you press 'r' or 'R'?

**Debug:**
```python
# Add logging in reset function
def reset_distance(self):
    self.get_logger().info('RESET CALLED')  # Verify function runs
    self.total_distance = 0.0
    ...
```

---

## Exercises

### Beginner

**Exercise 1: Measure Room Dimensions**

```
1. Reset distance
2. Drive along wall
3. Stop at corner
4. Record distance = wall length
5. Repeat for all 4 walls
```

**Exercise 2: Odometry Accuracy Test**

```
1. Mark starting position
2. Drive square pattern (4 × 1m sides)
3. Return to start
4. Ideal total: 4.0m
5. Check: Did encoders show ~4.0m?
```

### Intermediate

**Exercise 3: Add Speed Display**

Show current velocity:

```python
# Calculate from encoder deltas
dt = self.timestep / 1000.0  # Convert to seconds
velocity = delta_distance / dt

print(f"Velocity: {velocity:.2f} m/s")
```

**Exercise 4: Individual Wheel Display**

Show left/right speeds separately:

```python
left_vel = (left_pos - prev_left) * WHEEL_RADIUS / dt
right_vel = (right_pos - prev_right) * WHEEL_RADIUS / dt

print(f"L: {left_vel:.2f} m/s | R: {right_vel:.2f} m/s")
```

### Advanced

**Exercise 5: Path Recording**

Record and replay path:

```python
self.path = []  # List of (x, y, theta)

# While driving, calculate position
x += delta_distance * cos(theta)
y += delta_distance * sin(theta)
theta += delta_rotation

self.path.append((x, y, theta))

# Later: replay by following recorded waypoints
```

**Exercise 6: Wheel Slip Detection**

Detect when wheels slip:

```python
# Compare commanded vs measured
commanded_distance = velocity * time_elapsed
measured_distance = encoder_distance

slip = (commanded_distance - measured_distance) / commanded_distance * 100
if slip > 10:
    self.get_logger().warn(f'Wheel slip: {slip:.1f}%')
```

---

## Comparison: Signed vs Absolute Distance

### Autonomous Mission (Signed)

```python
# Forward 5m
distance = +5.0

# Backward 3m  
distance = +5.0 - 3.0 = +2.0  # Net displacement

# Use case: "Where am I relative to start?"
```

### Manual Control (Absolute)

```python
# Forward 5m
distance = 5.0

# Backward 3m
distance = 5.0 + 3.0 = 8.0  # Total movement

# Use case: "How far have I driven total?"
```

**Both are correct** - different questions!

---

## Related Controllers

**Same Level:**
- [011_wheel_encoder_mission.md](011_wheel_encoder_mission.md) - Autonomous version

**Previous Level:**
- [002_keyboard_teleop.md](002_keyboard_teleop.md) - No encoder feedback

**Next Level:**
- [021_touch_sensor_controller.md](021_touch_sensor_controller.md) - Add external sensing

---

## Reference

### Display Format

```
📏 Distance: TTT.TTTm | L: LL.LLLm | R: RR.RRRm | ↑ Forward
           ↑              ↑            ↑            ↑
       Total (avg)    Left wheel   Right wheel   Current command
```

### Sanity Checks

```python
# Ignore unrealistic values
MAX_SINGLE_STEP_DISTANCE = 1.0  # meters

if dist_left > MAX_SINGLE_STEP_DISTANCE:
    # Sensor glitch - ignore
    return
```

### Non-Blocking Input

```python
import select

# Check stdin without blocking
ready = select.select([sys.stdin], [], [], 0)[0]

if ready:
    key = sys.stdin.read(1)  # Key available
else:
    # No key pressed - continue
    pass
```

---

**Status:** ✅ Production ready  
**Use Case:** Manual exploration with distance feedback  
**See also:** Level 1.1 - Wheel Encoders
