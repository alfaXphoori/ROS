# 💥 021 - Touch Sensor Controller

> **Level 2.1 - Exteroceptive Contact Sensing | Reactive Obstacle Avoidance with Bumpers**

---

## 📌 Overview

Level 2.1: Touch Sensor / Bumper Controller

Reactive behavior using TouchSensor (bumper type).
Robot explores environment and reacts to physical contact.

Mission: Navigate forward, detect obstacles with bumper sensor,
         and execute escape maneuvers when collision is detected.

This demonstrates:
- TouchSensor API (type "bumper")
- Reactive behaviors (stimulus → response)
- Simple obstacle avoidance
- State machine for behavior control

### ✨ Key Features

- 🎯 3 TouchSensors (front, left, right bumpers)
- 🔄 State machine: EXPLORING → BACKING_UP → TURNING → repeat
- 📊 Collision detection and counting
- ⚡ Immediate reactive response to contact
- 📡 ROS2 event publishing
- 🎨 Real-time status display

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `021_touch_sensor_controller.py` | Reactive controller script |
| `021_touch_sensor.wbt` | Webots world file |


---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/021_touch_sensor.wbt
```

### Step 2️⃣: Run Reactive Controller

```bash
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

**Watch:** Robot bumps obstacles, backs up, turns away, and continues exploring!

---

## 🔧 How It Works

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

TouchSensors return values > 0 when contact is detected:

```python
def read_bumpers(self):
    """Read all touch sensor values"""
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

### 3. State Machine

```python
STATE_EXPLORING = "EXPLORING"
STATE_BACKING_UP = "BACKING_UP"
STATE_TURNING = "TURNING"

def run_behavior(self):
    while True:
        bumpers = self.read_bumpers()
        
        if self.state == STATE_EXPLORING:
            if bumpers['any']:
                self.collision_count += 1
                self.state = STATE_BACKING_UP
                self.backup_timer = 0
            else:
                self.move_forward()
        
        elif self.state == STATE_BACKING_UP:
            if self.backup_timer < BACKUP_DURATION:
                self.move_backward()
                self.backup_timer += 1
            else:
                self.state = STATE_TURNING
                self.turn_timer = 0
        
        elif self.state == STATE_TURNING:
            if self.turn_timer < TURN_DURATION:
                # Determine which way to turn
                if bumpers['front'] and bumpers['left']:
                    self.turn_right()
                elif bumpers['front'] and bumpers['right']:
                    self.turn_left()
                else:
                    self.turn_left()  # Default: turn left
                self.turn_timer += 1
            else:
                self.state = STATE_EXPLORING
```

---

## 📊 Sensor Information

### TouchSensor Specification

| Property | Value |
|----------|-------|
| **Type** | Webots TouchSensor device |
| **Return Value** | > 0 when contact detected |
| **Contact Force** | Proportional to applied force |
| **Position** | Front, left, right bumpers |
| **Sensitivity** | Configurable in world file |

### Sensor Placement

```
           Front Bumper
               🎯
              ╱ ╲
             ╱   ╲
            ╱     ╲
    Left 🎯       🎯 Right
      Bumper    Bumper
        ╱   🤖   ╲
       ╱          ╲
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ TouchSensor integration
- ✅ Contact detection principles
- ✅ State machine design patterns
- ✅ Reactive behavior programming
- ✅ Collision response strategies
- ✅ Bump-and-backup behaviors

---

## 📚 Sensor Knowledge: TouchSensor (Bumper)

### 🎯 How It Works

**TouchSensor** detects direct contact. When touching an object, it outputs **> 0**:

```
Normal State:       Sensor Value = 0.0  ✅ No contact
Contact Detected:   Sensor Value = 1.5  ⚠️ In contact
Strong Impact:      Sensor Value = 5.0  🔴 Heavy hit
```

### 📊 Specifications

| Property | Value | Note |
|---------|-------|------|
| **Type** | Binary Contact Sensor | On/Off detection |
| **Reading** | 0 = No contact | False |
| | > 0 = Contact | True |
| **Response Speed** | Instant (~1ms) | Immediate when touching |
| **Detection Range** | ~1-2cm | Must be close |
| **Accuracy** | ±5% | Good enough |
| **Force Dependency** | ❌ Affected | Value changes with force |

### 💡 Usage Tips

**✅ Do:**
- Obstacle detection (reactive)
- Impact detection
- Event-based behaviors
- Multiple bumpers for direction

**❌ Avoid:**
- Using for distance measurement
- Forgetting to enable() sensor
- Not handling multiple collisions

### ⚠️ Limitations

```
1. Reactive Only
   ❌ Detects AFTER collision
   ✓ Use distance sensor for BEFORE
   
2. Contact Force Variation
   - Value changes with force
   - Only threshold > 0 works
   
3. Limited Coverage
   - Need multiple bumpers
   - 1 bumper = 1 direction only
```

### 🔧 Best Practices

```python
# Multiple bumpers for full coverage
bumpers = {
    'front': robot.getDevice('bumper_front'),
    'left': robot.getDevice('bumper_left'),
    'right': robot.getDevice('bumper_right'),
}

# Enable all
for bumper in bumpers.values():
    bumper.enable(timestep)

# Simple boolean check
collision_detected = bumpers['front'].getValue() > 0

# Direction detection
front_hit = bumpers['front'].getValue() > 0
left_hit = bumpers['left'].getValue() > 0
right_hit = bumpers['right'].getValue() > 0
```

---

## 🔍 Reactive vs Proactive

### Strategy 1: Always Turn Left (Predictable)
```python
def escape_collision(self):
    self.move_backward()  # Back away first
    self.turn_left()      # Then turn left
```
**Pros:** Predictable, simple  
**Cons:** May get stuck in corners with walls on left

### Strategy 2: Turn Away from Impact (Intelligent)
```python
def escape_collision(self, bumpers):
    self.move_backward()
    if bumpers['left']:
        self.turn_right()    # Turn away from left collision
    elif bumpers['right']:
        self.turn_left()     # Turn away from right collision
    else:
        self.turn_left()     # Default
```
**Pros:** Adapts to collision direction  
**Cons:** Slightly more complex

---

## 📝 Customization

### Modify Timing Parameters

```python
BACKUP_DURATION = 50      # timesteps to back up
TURN_DURATION = 75        # timesteps to turn
FORWARD_SPEED = 5.0       # rad/s
TURN_SPEED = 4.0          # rad/s
```

### Add New Bumpers

```python
# In initialization
self.bumper_rear = self.robot.getDevice('bumper_rear')
self.bumper_rear.enable(self.timestep)

# In read_bumpers
bumpers['rear'] = self.bumper_rear.getValue() > 0
```

### Implement Different Escape Patterns

```python
def escape_collision(self):
    # Spin in place
    self.spin_left(angle=180)
    
    # OR: Do a complex maneuver
    self.move_backward(2.0)
    self.turn_right(angle=45)
    self.move_forward(1.0)
```

---

## 📚 Related Resources

- 📖 [Webots TouchSensor](https://cyberbotics.com/doc/reference/touchsensor)
- 🤖 [State Machines in Robotics](https://en.wikipedia.org/wiki/Finite-state_machine)
- 🎯 [Reactive Control Systems](https://en.wikipedia.org/wiki/Reactive_planning)
- 👀 [022 Distance Sensor Controller](../022_distance_sensor_controller/) (Proactive alternative)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **No collision detection** | Ensure sensors enabled; check world file |
| **Stuck in loop** | Increase backup/turn duration; adjust speeds |
| **Overly aggressive** | Reduce turn speed; increase backup duration |
| **Not turning far enough** | Increase TURN_DURATION or turn speed |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
