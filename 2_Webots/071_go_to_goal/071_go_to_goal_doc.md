# 071 Go to Goal Documentation

## Overview

**Level 7.1: Go to Goal with Obstacle Avoidance and Stuck Recovery**

This ROS2 controller implements advanced autonomous navigation with waypoint-based goal seeking, intelligent obstacle avoidance, and automatic stuck recovery mechanisms. The robot navigates to multiple waypoints in sequence while avoiding obstacles and recovering from challenging situations.

**Key Achievement**: Autonomous multi-waypoint navigation with robust obstacle avoidance and stuck recovery in complex environments.

---

## Features

### 1. **Odometry-Based Localization**
- Calculates robot position (X, Y, Theta) from wheel encoder data
- Uses IMU (Inertial Measurement Unit) for accurate heading correction
- Implements differential drive kinematics for 2-wheel robots
- Starts with initial pose facing North (90°)

### 2. **Waypoint Navigation**
- Visits multiple predefined waypoints in sequence
- Returns to home position after completing the circuit
- Two-phase navigation: TURN to face goal, then MOVE toward it
- Precise arrival detection (within 15cm of target)

### 3. **Advanced Obstacle Avoidance**
- Multi-level decision system (Emergency/Critical/Mild/Wall/Clear)
- Uses 052 SLAM algorithm for obstacle detection
- Stop-before-action pattern prevents erratic movements
- Precise 120° turns for emergency situations

### 4. **Stuck Detection and Recovery**
- Automatic detection of stuck situations
- Multiple recovery strategies: reverse, turn left, turn right
- Commits to recovery actions until completion
- Prevents reactive loops during recovery

### 5. **Lidar-Based Perception**
- 360° environment scanning
- Three-sector monitoring (Front, Left, Right, Back)
- Configurable safety distances for different threat levels

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GoToGoal Node                            │
├─────────────────────────────────────────────────────────────┤
│ Inputs:                                                     │
│  • Wheel Encoders (left/right)                             │
│  • IMU (Roll, Pitch, Yaw)                                  │
│  • Lidar (360° range scan)                                 │
├─────────────────────────────────────────────────────────────┤
│ Processing:                                                 │
│  1. Odometry Update (encoder + IMU → pose)                 │
│  2. Obstacle Detection (lidar → front/left/right/back)     │
│  3. Navigation State Machine (TURN → MOVE → REACHED)       │
│  4. Obstacle Avoidance (stop → action → recovery)          │
├─────────────────────────────────────────────────────────────┤
│ Outputs:                                                    │
│  • Motor Velocities (left/right wheels)                    │
│  • Terminal Dashboard (position, status, sensors)          │
└─────────────────────────────────────────────────────────────┘
```

---

## Configuration Parameters

### Robot Physical Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| `WHEEL_RADIUS` | 0.08 m | Radius of drive wheels |
| `AXLE_LENGTH` | 0.24 m | Distance between left/right wheels |

### Speed Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| `MAX_SPEED` | 4.0 rad/s | Maximum wheel velocity |
| `TURN_SPEED_RATIO` | 0.5 | Speed multiplier during turns |
| `SLOW_SPEED_RATIO` | 0.5 | Speed multiplier for cautious movement |

### Distance Thresholds
| Parameter | Value | Behavior |
|-----------|-------|----------|
| `EMERGENCY_DISTANCE` | 0.3 m | Triggers 120° turn or reverse |
| `CRITICAL_DISTANCE` | 0.4 m | Triggers in-place turn maneuver |
| `SAFE_DISTANCE` | 0.7 m | Triggers mild steering avoidance |
| `MAX_LIDAR_RANGE` | 4.0 m | Maximum valid lidar reading |

### Navigation Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| Arrival Threshold | 0.15 m | Distance to consider goal reached |
| Turning Threshold | 0.1 rad (~6°) | Angle error for turn → move transition |
| Course Correction | 0.5 rad (~29°) | Angle error to trigger re-turn |
| Wait After Arrival | 50 iterations | Pause duration at each waypoint |

---

## Core Algorithms

### 1. Odometry Calculation

**Differential Drive Kinematics:**

```python
# Step 1: Calculate wheel displacements
dl = Δencoder_left × WHEEL_RADIUS
dr = Δencoder_right × WHEEL_RADIUS

# Step 2: Calculate robot displacement
d_center = (dl + dr) / 2.0  # Linear distance

# Step 3: Update pose
theta = IMU.yaw  # Use IMU for accurate heading
x += d_center × cos(theta)
y += d_center × sin(theta)
```

**Key Insight**: IMU provides absolute heading, eliminating cumulative drift from encoder-based theta calculations.

---

### 2. Waypoint Navigation

**State Machine:**

```
        ┌─────────────────────────┐
        │    START / NEW GOAL     │
        └───────────┬─────────────┘
                    │
                    ▼
        ┌─────────────────────────┐
        │   TURN (face target)    │
        │   |angle_diff| < 0.1    │
        └───────────┬─────────────┘
                    │
                    ▼
        ┌─────────────────────────┐
        │   MOVE (toward goal)    │◄────┐ (if angle_diff > 0.5)
        │   distance < 0.15m      │     │
        └───────────┬─────────────┴─────┘
                    │
                    ▼
        ┌─────────────────────────┐
        │  REACHED (wait & next)  │
        │   wait 50 iterations    │
        └───────────┬─────────────┘
                    │
                    └──► Next waypoint
```

**Angle and Distance Calculation:**

```python
# Calculate vector to goal
dx = goal_x - robot_x
dy = goal_y - robot_y
distance = sqrt(dx² + dy²)

# Calculate required heading
target_angle = atan2(dy, dx)
angle_diff = normalize(target_angle - robot_theta)

# Normalize angle to [-π, π]
while angle > π:  angle -= 2π
while angle < -π: angle += 2π
```

---

### 3. Obstacle Detection (052 SLAM Algorithm)

**Three-Sector Monitoring:**

```python
# Divide 360° lidar into sectors
n = len(ranges)
front_sector = ranges[center ± width]  # center = n/2
left_sector  = ranges[center ± width]  # center = n/4
right_sector = ranges[center ± width]  # center = 3n/4
back_sector  = ranges[0 ± width] + ranges[n ± width]

# Find minimum distance in each sector
min_front = min(front_sector)
min_left  = min(left_sector)
min_right = min(right_sector)
min_back  = min(back_sector)
```

**Sector Width**: 1/12 of total points (~30° coverage per sector)

---

### 4. Obstacle Avoidance Logic

**Hierarchical Decision System:**

```python
# Priority 1: EMERGENCY (< 0.3m front)
if min_front < EMERGENCY_DISTANCE:
    if min_back > EMERGENCY_DISTANCE:
        → STOP → REVERSE (15 steps)
    else:
        → STOP → TURN 120° (precise angle)

# Priority 2: CRITICAL (< 0.4m front)        
elif min_front < CRITICAL_DISTANCE:
    → STOP → TURN to open side (30 steps)
    
# Priority 3: MILD AVOIDANCE (< 0.7m front)
elif min_front < SAFE_DISTANCE:
    → STEER toward open side (no stop)
    
# Priority 4: SIDE WALLS (< 0.4m left/right)
elif min_left < CRITICAL_DISTANCE:
    → STEER RIGHT
elif min_right < CRITICAL_DISTANCE:
    → STEER LEFT
    
# Priority 5: CLEAR PATH
else:
    → CONTINUE NAVIGATION
```

---

### 5. Stop-Before-Action Pattern

**Prevents erratic movements by committing to actions:**

```python
Phase 1: STOP (5 iterations)
    ↓
Phase 2: EXECUTE ACTION
    • BACK: 15 iterations
    • TURN (timer): 30 iterations
    • TURN (precise): Until angle reached
    ↓
Phase 3: RETURN TO NAVIGATION
```

**Precise 120° Turn (Emergency Only):**
```python
target_angle = 120° (radians)
turn_start = current_theta

while True:
    angle_turned = |current_theta - turn_start|
    if angle_turned >= target_angle:
        break
    else:
        continue_turning()
```

---

## Waypoint Configuration

### Default Waypoints

| Point | Coordinates | Color | Description |
|-------|-------------|-------|-------------|
| A | (2.0, 0.8) | Red | Northeast quadrant |
| B | (0.0, -2.2) | Green | South center |
| C | (-2.2, -0.5) | Blue | Southwest quadrant |
| D | (-1.2, 1.8) | Yellow | Northwest quadrant |
| Home | (0.0, 0.0) | - | Starting position |

**Navigation Pattern**: A → B → C → D → Home → (repeat)

### Customizing Waypoints

Edit the `self.goals` list in the `__init__` method:

```python
self.goals = [
    {'x': X_COORD, 'y': Y_COORD, 'name': 'Description'},
    # Add more waypoints as needed
]
```

**Initial Pose**: `x=0.0, y=0.0, theta=1.5708` (facing North/+Y direction)

---

## Navigation Behavior

### Normal Operation

1. **Turn Phase**
   - Rotate in place until facing goal (within 0.1 rad)
   - Turn speed: 2.0 rad/s
   - Direction: Shortest angular path

2. **Move Phase**
   - Move forward while maintaining heading
   - Speed: Proportional to distance (min 1.0, max 4.0 rad/s)
   - Continuous course correction (2× angle error)
   - Re-enter Turn phase if angle error > 0.5 rad

3. **Arrival**
   - Stop when within 15cm of goal
   - Wait 50 iterations (~1.5 seconds)
   - Advance to next waypoint

### Obstacle Avoidance Override

**Navigation is suspended when obstacles detected:**

- Obstacle avoidance takes priority over goal navigation
- Robot commits to avoidance action until completion
- Navigation resumes after obstacle cleared
- State machine (TURN/MOVE/REACHED) preserved during avoidance

---

## Dashboard Display

**Real-time terminal output:**

```
==================================================
   🎯 GO TO GOAL + AVOIDANCE (v7.1)
==================================================
 Position: X=1.23 Y=0.45
 Heading:  87.3°
--------------------------------------------------
 LIDAR: Front=2.15m | Left=1.82m | Right=3.45m
--------------------------------------------------
 TARGET:   Point A (Red) (2.0, 0.8)
 STATUS:   ⬆️ Moving (0.89m)
==================================================
```

### Status Indicators

| Icon | Meaning | Condition |
|------|---------|-----------|
| 🔄 Turning | Rotating to face goal | State = TURN |
| ⬆️ Moving | Moving toward goal | State = MOVE |
| ✅ ARRIVED! | Reached waypoint | distance < 0.15m |
| 🚨 EMERGENCY! | Critically close obstacle | front < 0.3m |
| 🚧 STOP! | Preparing avoidance action | Entering stop phase |
| ⚠️ REVERSING! | Backing away from obstacle | Recovery action |
| 🔄 Avoiding | Turning to avoid obstacle | Recovery action |
| 🛑 Blocked | Obstacle in path | Mild avoidance active |
| ⚠️ Wall Left/Right | Side obstacle detected | Wall following |

---

## Usage

### Running the Controller

```bash
# Terminal 1: Launch Webots simulation
ros2 launch ce_webots_launch robot_launch.py

# Terminal 2: Run the go-to-goal controller
ros2 run ce_webots 071_go_to_goal
```

### Expected Behavior

1. Robot starts at origin (0,0) facing North
2. Turns to face first waypoint (Point A)
3. Navigates while avoiding obstacles
4. Arrives at Point A, waits briefly
5. Continues to Points B, C, D, and Home
6. Loops indefinitely through all waypoints

---

## Technical Implementation

### Key Methods

#### `update_odometry()`
- Calculates position from encoder deltas
- Uses IMU for absolute heading
- Updates `self.pose` dict

#### `detect_obstacle(ranges)`
- Analyzes lidar data in sectors
- Returns (min_front, min_left, min_right)
- Filters invalid readings (inf)

#### `avoid_obstacle(ranges)`
- Implements 052 SLAM avoidance algorithm
- Manages stop timer and action timer
- Handles precise angle turns for emergency
- Returns (left_speed, right_speed, status) or None

#### `navigate_to_goal(ranges)`
- Main navigation controller
- Checks obstacles first (priority)
- Implements TURN/MOVE/REACHED state machine
- Returns motor speeds and status

#### `normalize_angle(angle)`
- Wraps angles to [-π, π] range
- Essential for shortest-path turning

#### `print_dashboard()`
- Clears terminal and displays current state
- Shows position, heading, sensors, target, status

---

## State Variables

### Pose Tracking
- `self.pose`: Dictionary with 'x', 'y', 'theta'
- `self.last_enc_l/r`: Previous encoder readings

### Navigation State
- `self.current_goal_idx`: Index in goals list
- `self.state`: "TURN", "MOVE", or "REACHED"
- `self.wait_counter`: Iterations at waypoint

### Obstacle Avoidance State
- `self.stuck_timer`: Countdown for timed actions
- `self.stuck_action`: "NONE", "LEFT", "RIGHT", "BACK", "STOP"
- `self.stop_timer`: Countdown for stop phase
- `self.target_turn_angle`: For precise angle turns (radians)
- `self.turn_start_angle`: Starting angle when turn begins

---

## Differences from 052 SLAM Controller

| Feature | 052 SLAM | 071 Go to Goal |
|---------|----------|----------------|
| **Primary Goal** | Map exploration | Waypoint navigation |
| **ROS2 Topics** | Publishes /scan, /map, /odom | No ROS2 publishing |
| **Mapping** | Generates occupancy grid | No mapping |
| **Navigation** | Random exploration | Goal-directed |
| **Emergency Turn** | Timer-based | Precise 120° angle |
| **Stop Pattern** | Immediate action | Stop-before-action |
| **Keyboard Control** | Toggle manual/auto | Not implemented |
| **State Machine** | Exploration only | TURN/MOVE/REACHED |

---

## Tuning Guide

### For Faster Navigation
```python
self.MAX_SPEED = 6.0           # Increase max speed
forward_speed = min(6.0, distance * 3.0)  # More aggressive acceleration
```

### For Safer Navigation
```python
self.SAFE_DISTANCE = 1.0       # Increase safety margin
self.EMERGENCY_DISTANCE = 0.5  # Earlier emergency response
```

### For Tighter Turns
```python
turn_speed = 3.0  # In navigate_to_goal() TURN state
```

### For More Precise Arrivals
```python
if distance < 0.10:  # Reduce from 0.15m
```

### For Longer Waiting at Waypoints
```python
if self.wait_counter > 100:  # Increase from 50
```

---

## Common Issues and Solutions

### Issue: Robot oscillates at goal
**Cause**: Arrival threshold too small  
**Solution**: Increase `distance < 0.15` to `0.20`

### Issue: Robot gets stuck in corners
**Cause**: Recovery actions too short  
**Solution**: Increase `self.stuck_timer` values (15 → 25, 30 → 40)

### Issue: Robot doesn't avoid obstacles
**Cause**: Distance thresholds too small  
**Solution**: Increase SAFE_DISTANCE and CRITICAL_DISTANCE

### Issue: Robot turns too slowly
**Cause**: Turn speed too low  
**Solution**: Increase `turn_speed` in TURN state

### Issue: Robot overshoots goals
**Cause**: Speed too high near goal  
**Solution**: Modify speed calculation:
```python
forward_speed = min(self.MAX_SPEED, distance * 1.5)  # Reduce from 2.0
```

---

## Future Enhancements

### Potential Improvements

1. **Dynamic Obstacle Avoidance**
   - Predict moving obstacle paths
   - Implement velocity obstacles algorithm

2. **Path Planning**
   - Integrate A* or RRT for optimal paths
   - Plan around known obstacles

3. **Adaptive Speed Control**
   - Slow down in narrow passages
   - Speed up in open areas

4. **ROS2 Integration**
   - Publish navigation status
   - Subscribe to external goal commands
   - Publish TF transforms

5. **Multi-Robot Coordination**
   - Share maps between robots
   - Collision avoidance with other robots

6. **Recovery Strategies**
   - Backtrack along previous path
   - Request human intervention if truly stuck

---

## Related Controllers

- **052_slam_controller.py**: SLAM-based exploration (foundation for obstacle avoidance)
- **051_lidar_controller.py**: Basic lidar obstacle avoidance
- **031_imu_controller.py**: IMU sensor integration
- **011_wheel_encoder_mission.py**: Encoder-based odometry basics

---

## References

### Concepts Used
- **Differential Drive Kinematics**: Two-wheel robot motion model
- **Odometry**: Position estimation from wheel motion
- **Lidar Sectoring**: Divide scan into regions for decision-making
- **State Machine**: Structured control flow (TURN/MOVE/REACHED)
- **Reactive Control**: Immediate response to sensor data
- **Behavior Priority**: High-priority behaviors override low-priority

### Algorithms
- **052 SLAM Obstacle Avoidance**: Multi-level distance thresholds
- **Stop-Before-Action**: Prevent reactive oscillations
- **Precise Angle Turns**: IMU-based angle control for 120° maneuvers
- **Waypoint Following**: Turn-then-move approach

---

## Author Notes

**Version**: 7.1  
**Date**: February 2026  
**Framework**: ROS2 + Webots  

This controller demonstrates advanced autonomous navigation by combining:
- Odometry for self-localization
- Waypoint navigation for task-oriented behavior
- Robust obstacle avoidance for safety
- Stuck recovery for resilience

The stop-before-action pattern and precise angle turns represent improvements over the 052 SLAM controller, providing smoother and more predictable navigation behavior.

---

**End of Documentation**
