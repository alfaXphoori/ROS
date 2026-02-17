# 🎯 071 - Go to Goal

> **Level 7.1 - Advanced Navigation | Waypoint-Based Autonomous Navigation with Obstacle Avoidance**

---

## 📌 Overview

Level 7.1: Go to Goal (Navigation + Obstacle Avoidance + Stuck Recovery)
=========================================================================
Navigate to waypoints with advanced obstacle avoidance and stuck recovery

Features:
1. Odometry: Calculate position (X, Y, Theta) from encoders and IMU
2. Go-to-Goal: Navigate to specified waypoints using angle/distance control
3. Obstacle Avoidance: 052 SLAM algorithm (Emergency/Critical/Mild/Wall/Clear)
4. Stuck Detection: Automatically detect and recover from stuck situations
5. Waypoint Loop: Visit multiple points in sequence

Author: AI Assistant

### ✨ Key Features

- 📍 Multi-waypoint navigation
- 🎯 Precise goal seeking
- 🧩 State machine architecture (TURN → MOVE → REACHED)
- 🚧 Multi-level obstacle avoidance
- 💪 Stuck detection and recovery
- 🧭 Odometry-based localization
- 📊 Real-time status dashboard
- 📡 ROS2 integration (Odometry, TF, Markers)
- 🔄 Return to home feature

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `071_go_to_goal.py` | Waypoint navigation controller |
| `071_go_goal.wbt` | Webots world file |


---

## 🚀 How to Run This Lab

### Prerequisites

- ✅ Webots installed
- ✅ ROS 2 Jazzy installed and sourced
- ✅ **RViz2 installed**
- ✅ Labs 031 (IMU), 051 (LIDAR), 052 (SLAM) completed
- ✅ Workspace built and sourced

### Running Steps

#### Terminal 1: Launch Webots Simulation

```bash
webots ~/ros2_ws/src/ce_webots/worlds/071_go_goal.wbt
```

**Environment features:**
- Large arena with obstacles
- Multiple waypoints to navigate
- Complex obstacle configurations
- Testing ground for autonomous navigation

#### Terminal 2: Run Go-to-Goal Navigator

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Run the waypoint navigator
ros2 run ce_webots 071_go_to_goal
```

**What to observe:**
- Autonomous waypoint navigation
- Multi-level obstacle avoidance
- State machine transitions (TURN → MOVE → REACHED)
- Stuck detection and recovery
- Progress toward each waypoint
- Real-time position and heading

#### Terminal 3: Launch RViz2

```bash
# Launch RViz2
rviz2
```

### 📊 RViz Configuration for Go-to-Goal Navigation

#### Step 1: Set Fixed Frame

- Set **Fixed Frame** to `odom`

#### Step 2: Add LaserScan Display

1. Click **Add** → **LaserScan**
2. Configure:
   - **Topic:** `/scan`
   - **Size (m):** `0.05`
   - **Color:** Red or Yellow
   - **Style:** Flat Squares

#### Step 3: Add Odometry Display

1. Click **Add** → **Odometry**
2. Configure:
   - **Topic:** `/odom`
   - **Keep:** `500` (show path history)
   - **Position Tolerance:** `0.1`
   - **Angle Tolerance:** `0.1`
   - **Shape:** Arrow
   - **Color:** Blue
   - **Shaft Length:** `0.3`
   - **Head Length:** `0.1`

#### Step 4: Add Waypoint Markers

1. Click **Add** → **Marker**
2. Configure:
   - **Topic:** `/waypoint_markers`

OR

1. Click **Add** → **MarkerArray**
2. Configure:
   - **Topic:** `/waypoint_markers`

This displays:
- Green spheres for pending waypoints
- Yellow sphere for current target
- Blue sphere for completed waypoints
- Red sphere for current robot position

#### Step 5: Add Path Display (Optional)

1. Click **Add** → **Path**
2. Configure:
   - **Topic:** `/planned_path` (if published)
   - **Color:** Green
   - **Alpha:** `0.5`

#### Step 6: Add Map (if available)

1. Click **Add** → **Map**
2. Configure:
   - **Topic:** `/map` (if SLAM-based mapping is enabled)

### Save RViz Configuration

```bash
File → Save Config As → ~/ros2_ws/src/ce_webots/config/071_go_to_goal.rviz
```

Load next time:
```bash
rviz2 -d ~/ros2_ws/src/ce_webots/config/071_go_to_goal.rviz
```

### Real-Time Navigation Dashboard

```
================================================================================
              🎯 GO TO GOAL NAVIGATION CONTROLLER
================================================================================

📍 CURRENT POSITION & WAYPOINTS
   Position: (1.245, 2.376) | Heading: 45.3°
   
   Waypoint 1: (0, 0)    [████████████  ] 35% MOVING
   Waypoint 2: (5, 0)    [░░░░░░░░░░░░  ] 0% PENDING
   Waypoint 3: (5, 5)    [░░░░░░░░░░░░  ] 0% PENDING
   Waypoint 4: (0, 5)    [░░░░░░░░░░░░  ] 0% PENDING
   Home:      (0, 0)    [░░░░░░░░░░░░  ] 0% PENDING

🧭 NAVIGATION STATE
   Mode:      MOVING
   Status:    🎯 Moving toward (5, 0)
   Distance to Goal: 3.45 m
   Heading Error: -12.3°
   
🚧 OBSTACLE DETECTION
   Front:     0.85 m 🟢 (Safe)
   Left:      0.42 m 🟡 (Warning)
   Right:     0.68 m 🟢 (Safe)
   Back:      2.10 m 🟢 (Clear)
   
🔧 STATUS
   Stuck Detection: ✅ OK
   Last Movement: 0.23m (recent)
   Recovery Attempts: 0

================================================================================
```

### What You Should See in RViz

**LaserScan (Red/Yellow):**
- 360° obstacle detection
- Updates in real-time as robot moves
- Shows walls and obstacles

**Odometry Trail (Blue):**
- Path history showing where robot has been
- Arrow shows current position and heading
- Should align with planned waypoints

**Waypoint Markers:**
- **Green spheres:** Future waypoints (pending)
- **Yellow sphere:** Current target waypoint
- **Blue spheres:** Completed waypoints
- **Red sphere:** Current robot position

**Navigation Behavior:**
- Robot turns toward waypoint
- Moves forward while avoiding obstacles
- Reaches waypoint and moves to next

### Monitoring Topics

```bash
# Terminal 4 (optional): Monitor navigation
ros2 topic list

# Monitor odometry
ros2 topic echo /odom

# Check waypoint status
ros2 topic echo /current_waypoint

# View LIDAR scan
ros2 topic echo /scan

# Monitor navigation state
ros2 topic echo /nav_state

# Check waypoint markers
ros2 topic echo /waypoint_markers

# Monitor TF transforms
ros2 run tf2_ros tf2_echo odom base_link
```

### Understanding the Navigation States

**State Machine:**

1. **TURN:** Rotate toward waypoint
   - Adjusts heading until aligned (within tolerance)
   - Uses IMU for accurate angle control
   - Threshold: typically ±5°

2. **MOVE:** Move toward waypoint
   - Drives forward while monitoring obstacles
   - Adjusts path based on LIDAR data
   - Reduces speed when close to goal

3. **REACHED:** Waypoint achieved
   - Stops at waypoint
   - Marks waypoint as completed
   - Advances to next waypoint
   - Threshold: typically 0.2m

**Obstacle Avoidance Levels:**
- **Emergency** (< 0.15m): Hard brake, reverse
- **Critical** (< 0.25m): Sharp turn away
- **Mild** (< 0.40m): Gentle adjustment
- **Wall Follow:** Maintain distance from walls
- **Clear:** Full speed ahead

### Interactive Controls

The system runs autonomously, but you can monitor progress:
- Watch Terminal 2 for status updates
- Observe RViz for visual confirmation
- Check waypoint completion progress

### Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot doesn't move** | Check if waypoints are defined in code |
| **Overshoots waypoints** | Reduce speed or increase tolerance |
| **Gets stuck** | Verify stuck detection is enabled, check obstacle clearance |
| **Wrong heading** | Check IMU calibration and coordinate system |
| **Crashes into obstacles** | Decrease speed or increase safety distances |
| **RViz markers not visible** | Check `/waypoint_markers` topic is publishing |
| **Odometry drift** | Normal for encoder-only - consider adding GPS or visual odometry |

### Waypoint Configuration

To modify waypoints, edit the controller code:

```python
# Example waypoint list
waypoints = [
    (0, 0),    # Home
    (5, 0),    # Point 1
    (5, 5),    # Point 2
    (0, 5),    # Point 3
    (0, 0),    # Return home
]
```

### Performance Metrics

Monitor these metrics to evaluate navigation:
- **Success Rate:** Percentage of waypoints reached
- **Path Efficiency:** Actual distance / optimal distance
- **Time to Goal:** How long to reach each waypoint
- **Stuck Events:** Number of stuck detections
- **Collision Avoidance:** Near-miss count

### Advanced Experiments

1. **Complex Paths:**
   - Add more waypoints
   - Create figure-8 patterns
   - Navigate through narrow passages

2. **Dynamic Obstacles:**
   - Add moving obstacles
   - Test reactive avoidance
   - Implement dynamic replanning

3. **Optimization:**
   - Tune PID parameters
   - Adjust safety thresholds
   - Optimize speed profiles

4. **Integration:**
   - Combine with global path planning
   - Add visual odometry
   - Implement loop closure

---

## 🔧 How It Works

### 1. Initialize Navigation System

```python
# Robot sensors
self.left_encoder = self.robot.getDevice('left_wheel_sensor')
self.right_encoder = self.robot.getDevice('right_wheel_sensor')
self.imu = self.robot.getDevice('inertial_unit')
self.lidar = self.robot.getDevice('lidar')

# Enable all sensors
for sensor in [self.left_encoder, self.right_encoder, self.imu, self.lidar]:
    sensor.enable(self.timestep)

# Navigation parameters
self.WHEEL_RADIUS = 0.08
self.AXLE_LENGTH = 0.24
self.MAX_SPEED = 4.0

# Waypoints
self.waypoints = [
    {'x': 5, 'y': 0, 'name': 'East'},
    {'x': 5, 'y': 5, 'name': 'Northeast'},
    {'x': 0, 'y': 5, 'name': 'North'},
    {'x': 0, 'y': 0, 'name': 'Home'},
]

# Start position
self.x = 0.0
self.y = 0.0
self.theta = 1.57  # Facing North (90°)
```

### 2. Odometry Update

```python
def update_odometry(self):
    """Update position from encoders and IMU"""
    
    # Read wheel encoders
    left_pos = self.left_encoder.getValue()
    right_pos = self.right_encoder.getValue()
    
    # Calculate distances
    left_dist = (left_pos - self.prev_left_pos) * self.WHEEL_RADIUS
    right_dist = (right_pos - self.prev_right_pos) * self.WHEEL_RADIUS
    
    # Differential drive kinematics
    delta_s = (left_dist + right_dist) / 2
    delta_theta = (right_dist - left_dist) / self.AXLE_LENGTH
    
    # Get absolute heading from IMU
    quat = self.imu.getQuaternion()
    self.theta = self.quaternion_to_yaw(quat)
    
    # Update position
    if delta_s != 0:
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)
    
    # Save for next iteration
    self.prev_left_pos = left_pos
    self.prev_right_pos = right_pos
```

### 3. State Machine Navigation

```python
STATE_TURN = "TURN"       # Face goal
STATE_MOVE = "MOVE"       # Move toward goal
STATE_REACHED = "REACHED" # Goal achieved

def navigate_to_waypoint(self, goal):
    """State machine for goal-seeking"""
    
    # Calculate goal direction
    dx = goal['x'] - self.x
    dy = goal['y'] - self.y
    goal_distance = math.sqrt(dx**2 + dy**2)
    goal_heading = math.atan2(dy, dx)
    
    # Calculate angle error
    angle_error = goal_heading - self.theta
    # Normalize to [-π, π]
    while angle_error > math.pi:
        angle_error -= 2 * math.pi
    while angle_error < -math.pi:
        angle_error += 2 * math.pi
    
    # State machine
    if self.state == STATE_TURN:
        # Turn toward goal
        if abs(angle_error) < 0.1:  # ~6° tolerance
            self.state = STATE_MOVE
        else:
            self.turn_toward_goal(angle_error)
    
    elif self.state == STATE_MOVE:
        # Move forward
        if goal_distance < 0.15:  # ~15cm accuracy
            self.state = STATE_REACHED
            self.stop()
        elif abs(angle_error) > 0.5:
            # Lost heading, turn again
            self.state = STATE_TURN
        else:
            self.move_forward()
            # Adjust heading while moving
            if abs(angle_error) > 0.05:
                self.adjust_heading(angle_error)
    
    elif self.state == STATE_REACHED:
        # Goal reached, wait then go to next
        if self.reached_timeout > 50:
            return True  # Signal to go to next waypoint
```

### 4. Obstacle Avoidance

```python
def check_obstacles_and_adjust(self):
    """Multi-level obstacle detection"""
    
    # Read LIDAR
    distances = self.lidar.getRangeImage()
    
    # Analyze sectors
    front = min(distances[85:95])
    left = min(distances[75:165])
    right = min(distances[195:285])
    
    # Multi-level response
    if front < 0.2 and left < 0.25 and right < 0.25:
        # EMERGENCY: Trapped!
        return BACK_UP_AND_TURN_120()
    
    elif front < 0.3:
        # CRITICAL: Can't move forward
        if left > right:
            return TURN_LEFT()
        else:
            return TURN_RIGHT()
    
    elif front < 0.5:
        # WARNING: Slow down and adjust
        return SLOW_AND_STEER_AROUND()
    
    elif left < 0.25 or right < 0.25:
        # MILD: Adjust sides
        if left < right:
            return STEER_RIGHT()
        else:
            return STEER_LEFT()
    
    else:
        # CLEAR: Continue
        return None
```

### 5. Stuck Detection & Recovery

```python
def detect_stuck(self):
    """Check if robot is not making progress"""
    
    # Store position every 50 steps
    if self.timestep_counter % 50 == 0:
        distance_moved = math.sqrt(
            (self.x - self.last_check_x)**2 +
            (self.y - self.last_check_y)**2
        )
        
        if distance_moved < 0.05:  # Moved < 5cm
            return True
    
    return False

def recover_from_stuck(self):
    """Execute recovery sequence"""
    
    recovery_steps = [
        (self.BACK_UP, 100),      # Back up for 100 steps
        (self.TURN_LEFT, 150),    # Turn left for 150 steps
        (self.MOVE_FORWARD, 100), # Try forward
    ]
    
    for action, duration in recovery_steps:
        for _ in range(duration):
            action()
            if not self.detect_stuck():
                return  # Recovered!
```

---

## 🗺️ Waypoint Navigation Concept

```
Grid Layout (10m × 10m):
  
  (0,5)───────────(5,5)
    │               │
    │     🤖        │
    │    (2,3)      │
    │               │
  (0,0)───────────(5,0)
  
Path:
  1. Start at (0,0), face North
  2. Move to (5,0) - East
  3. Move to (5,5) - Northeast corner
  4. Move to (0,5) - North
  5. Return to (0,0) - Home
```

---

## 📚 Sensor Knowledge: Odometry + Compass Navigation

### 🎯 How It Works

**Autonomous Navigation** uses Odometry (position from wheels) + IMU Compass (heading) to navigate to target waypoints:

```
Navigation System Components:

1. ODOMETRY (Where am I?)
   ├─ Read wheel encoders
   ├─ Calculate Δx, Δy, Δθ
   └─→ Estimate absolute position

2. COMPASS (Which direction?)
   ├─ IMU heading measurement
   ├─ Convert quaternion → yaw
   └─→ Accurate angle (fixes drift)

3. GOAL TRACKING
   ├─ Read target waypoint (x, y)
   ├─ Calculate bearing to goal
   ├─ Compare with current heading
   └─→ Generate steering commands

4. OBSTACLE DETECTION
   ├─ LIDAR scan 360°
   ├─ Check front sector
   └─→ Avoid collision

Navigation Loop:
  
  while not at_goal:
      pos_x, pos_y = odometry.get_position()
      heading = imu.get_heading()
      
      goal_bearing = atan2(
          goal_y - pos_y,
          goal_x - pos_x
      )
      
      angle_error = goal_bearing - heading
      
      if distance_to_goal < 0.2:
          break  # Reached!
      
      set_motor_speeds(angle_error)
```

### 📊 Specifications

| Property | Value | Notes |
|---------|-------|-------|
| **Odometry Drift** | ~5% per meter | Accumulates over distance |
| **IMU Heading Drift** | ~0.1°/min | Gyro drift over time |
| **Update Rate** | ~10 Hz | Main control loop |
| **LIDAR Detection** | 360° @ 0.05-2.0m | Obstacle avoidance |
| **Navigation Tolerance** | ±0.1m | Goal reached threshold |
| **Max Speed** | 1.0 m/s | Forward velocity |
| **Max Turn Rate** | 1.0 rad/s | Angular velocity |

### 💡 Usage Tips

**✅ Do:**
- Multi-waypoint missions
- Replanning around obstacles
- Loop closure correction
- Odometry + LIDAR fusion
- Dynamic obstacle avoidance

**❌ Avoid:**
- Trust odometry alone (use LIDAR)
- Ignore compass drift
- Too tight goal tolerance
- Not checking for stuck robot
- Straight-line paths only

### ⚠️ Limitations

```
1. Odometry Drift (Cumulative Error)
   
   Error Growth:
   ├─ After 10m: ±0.5m error
   ├─ After 50m: ±2.5m error
   ├─ After 100m: ±5m error
   
   Causes:
   ├─ Wheel slippage (carpet, slopes)
   ├─ Encoder resolution limits
   ├─ Uneven loading
   ├─ Surface friction changes
   └─→ Solution: LIDAR landmarks!

2. IMU Heading Drift
   ├─ Gyro integration error
   ├─ ~0.1-0.5°/minute
   ├─ 10 minutes = 1-5° error
   └─→ Solution: Loop closure detection

3. Obstacle Avoidance Conflicts
   ├─ Local obstacles block direct path
   ├─ Need path replanning
   ├─ Potential dead-ends
   └─→ Solution: RRT/A* planning

4. Dynamic Obstacles
   ├─ Moving people invalidate map
   ├─ Sensor lag = delayed detection
   └─→ Solution: Predict motion

5. Stuck Robot Detection
   ├─ Motor commands sent but no movement
   ├─ Can loop endlessly
   └─→ Solution: Timeout + backtrack
```

### 🔧 Autonomous Navigation Algorithm

```python
import numpy as np
from math import atan2, sqrt, cos, sin

class AutonomousNavigator:
    
    def __init__(self, wheel_radius=0.05, wheel_distance=0.1):
        """Initialize navigator with odometry"""
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0  # Heading
        
        # Wheel parameters
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        
        # State tracking
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.reached = False
        self.stuck_count = 0
    
    def update_odometry(self, left_encoder, right_encoder,
                       left_prev, right_prev):
        """Update position based on encoders"""
        
        # Calculate deltas
        left_delta = left_encoder - left_prev
        right_delta = right_encoder - right_prev
        
        # Convert to distances
        left_dist = left_delta * self.wheel_radius
        right_dist = right_delta * self.wheel_radius
        
        # Calculate forward and turning
        forward = (left_dist + right_dist) / 2.0
        turning = (right_dist - left_dist) / self.wheel_distance
        
        # Update pose
        if abs(turning) < 0.001:  # Straight line
            self.pose_x += forward * cos(self.pose_theta)
            self.pose_y += forward * sin(self.pose_theta)
        else:  # Curved path
            radius = forward / turning
            self.pose_theta += turning
            
            self.pose_x += radius * (sin(self.pose_theta) 
                                    - sin(self.pose_theta - turning))
            self.pose_y -= radius * (cos(self.pose_theta) 
                                    - cos(self.pose_theta - turning))
    
    def correct_heading(self, imu_heading):
        """Correct heading drift using IMU compass"""
        
        # Replace heading with more accurate IMU value
        self.pose_theta = imu_heading
    
    def set_goal(self, goal_x, goal_y):
        """Set navigation target"""
        
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.reached = False
    
    def calculate_control(self, lidar_ranges):
        """Calculate motor speeds to reach goal"""
        
        # 1. Calculate distance to goal
        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        distance = sqrt(dx**2 + dy**2)
        
        # 2. Calculate bearing to goal
        goal_bearing = atan2(dy, dx)
        
        # 3. Calculate angle error
        angle_error = goal_bearing - self.pose_theta
        
        # Normalize to [-π, π]
        while angle_error > np.pi:
            angle_error -= 2 * np.pi
        while angle_error < -np.pi:
            angle_error += 2 * np.pi
        
        # 4. Check if at goal
        if distance < 0.1:  # 10cm tolerance
            self.reached = True
            return 0.0, 0.0
        
        # 5. Check for obstacles ahead
        front_min = min(lidar_ranges[0:45] + 
                       lidar_ranges[315:360])
        
        if front_min < 0.3:  # 30cm ahead
            # Obstacle! Stop forward motion
            forward_speed = 0.0
            
            # Turn to avoid
            if lidar_ranges[90] > lidar_ranges[270]:
                angle_error = 0.3  # Turn left
            else:
                angle_error = -0.3  # Turn right
        else:
            # Clear path
            if distance > 1.0:
                forward_speed = 0.8  # Fast
            elif distance > 0.3:
                forward_speed = 0.3  # Slow down
            else:
                forward_speed = 0.1  # Very slow
        
        # 6. Proportional steering control
        turn_gain = 0.5
        turn_speed = angle_error * turn_gain
        turn_speed = np.clip(turn_speed, -0.5, 0.5)
        
        # 7. Calculate motor speeds
        left_speed = forward_speed - turn_speed
        right_speed = forward_speed + turn_speed
        
        # Clamp to valid range
        left_speed = np.clip(left_speed, -1.0, 1.0)
        right_speed = np.clip(right_speed, -1.0, 1.0)
        
        return left_speed, right_speed
    
    def check_stuck(self, left_speed, right_speed, actual_forward):
        """Detect if robot is stuck"""
        
        # If commands sent but no motion
        if abs(left_speed) > 0.1 and abs(actual_forward) < 0.01:
            self.stuck_count += 1
            
            if self.stuck_count > 50:  # ~5 seconds stuck
                return True
        else:
            self.stuck_count = 0
        
        return False
    
    def plan_escape(self):
        """Backtrack if stuck"""
        
        # Move backward and turn
        self.pose_x -= 0.5 * cos(self.pose_theta)
        self.pose_y -= 0.5 * sin(self.pose_theta)
        self.pose_theta += np.pi / 4  # Turn 45°
        
        self.stuck_count = 0
```

### 📊 Navigation State Machine

```
┌─────────────────────┐
│  IDLE               │
│ Waiting for goal    │
└─────┬───────────────┘
      │ set_goal(x, y)
      ↓
┌─────────────────────┐
│  NAVIGATING         │
│ Moving to goal      │
├─────────┬───────────┤
│ • Check odometry    │
│ • Correct heading   │
│ • Detect obstacles  │
│ • Plan path         │
└─────┬───────┬───────┘
      │       │
   ┌──┘       └──┐
   │             │
   ↓             ↓
REACHED      STUCK?
   │             │
   ↓             ↓
┌──────────┐  ┌──────────────┐
│  SUCCESS │  │  ESCAPE      │
└──────────┘  │  Backtrack+  │
              │  Replan      │
              └──────┬───────┘
                     │
                     └──→ NAVIGATING
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ State machine design
- ✅ Waypoint-based navigation
- ✅ Odometry integration
- ✅ Goal-seeking algorithms
- ✅ Multi-level obstacle avoidance
- ✅ Stuck detection and recovery
- ✅ Real-time status monitoring
- ✅ Advanced ROS2 integration

---

## 📊 Navigation Parameters

```python
# Speed control
MAX_SPEED = 4.0         # rad/s
TURN_SPEED_RATIO = 0.5  # Slower when turning
SLOW_SPEED_RATIO = 0.5  # Emergency slow mode

# Distance thresholds
EMERGENCY_DISTANCE = 0.3    # Backup and turn
CRITICAL_DISTANCE = 0.4     # Turn in place
SAFE_DISTANCE = 0.7         # Safe for forward
MAX_LIDAR_RANGE = 4.0       # Trust range

# Navigation tolerances
ARRIVAL_THRESHOLD = 0.15    # Goal reached
TURNING_THRESHOLD = 0.1     # Done turning (radians)
COURSE_CORRECTION = 0.5     # Re-turn if error > this
```

---

## 📝 Customization

### Add Custom Waypoints

```python
self.waypoints = [
    {'x': 2, 'y': 0, 'name': 'First'},
    {'x': 2, 'y': 2, 'name': 'Second'},
    {'x': 0, 'y': 2, 'name': 'Third'},
]
```

### Adjust Navigation Precision

```python
# Very precise navigation
ARRIVAL_THRESHOLD = 0.05     # 5cm accuracy
TURNING_THRESHOLD = 0.05     # ±3° heading error

# Quick (rough) navigation
ARRIVAL_THRESHOLD = 0.5      # 50cm accuracy
TURNING_THRESHOLD = 0.3      # ±17° ok
```

### Implement Path Planning

```python
def compute_path(self, start, goal):
    """A* or Dijkstra path planning"""
    # More sophisticated than direct waypoints
    # Can navigate around complex obstacles
    pass
```

---

## 📚 Related Resources

- 📖 [Path Planning Algorithms](https://en.wikipedia.org/wiki/Motion_planning)
- 🎯 [State Machines](https://en.wikipedia.org/wiki/Finite-state_machine)
- 📍 [Navigation Stack](https://docs.ros.org/en/nav2_tutorials/)
- 🗺️ [Occupancy Grid Mapping](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)
- 🤖 [ROS 2 Navigation](https://github.com/ros-planning/navigation2)
- 👀 [052 SLAM Controller](../052_slam_controller/)
- 👀 [022 Distance Sensor](../022_distance_sensor_controller/)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot doesn't reach goal** | Increase navigation time; check odometry |
| **Gets stuck frequently** | Reduce thresholds; improve obstacle detection |
| **Overshoots goals** | Reduce MAX_SPEED; increase ARRIVAL_THRESHOLD |
| **Erratic behavior** | Check LIDAR alignment; verify odometry |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
