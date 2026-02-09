# Level 5.1 - LIDAR Mapping & Obstacle Detection

## Overview

This level introduces **LIDAR (Light Detection and Ranging)** - a revolutionary sensor technology that enables robots to "see" their environment in 360 degrees. Unlike cameras that capture images, LIDAR measures distances to create precise spatial maps.

**Key Concepts:**
- 360-degree environment scanning
- Distance measurement array (360 points)
- Sector-based navigation strategy
- Autonomous obstacle avoidance
- Real-time spatial awareness

## What is LIDAR?

LIDAR works by:
1. Emitting laser pulses in all directions
2. Measuring the time it takes for light to bounce back
3. Calculating distance: `distance = (speed_of_light × time) / 2`
4. Creating a "point cloud" of the environment

**Applications:**
- Self-driving cars (Tesla, Waymo, Cruise)
- Warehouse robots (Amazon, Fetch Robotics)
- Drones and aerial mapping
- Autonomous vacuum cleaners (advanced models)
- Archaeological site scanning

## Hardware Setup

### LIDAR Configuration
```python
Specifications:
- Type: 2D Scanning LIDAR
- Resolution: 360 points (1° per measurement)
- Field of View: 360° (full circle)
- Range: 0.05m to 2.0m
- Update Rate: Real-time (every timestep)
- Position: Center top of robot (0, 0, 0.12)
- Orientation: Aligned with robot front (point 0 = North)
```

### Point Indexing System

LIDAR data is organized as an array where each index corresponds to an angle:

```
              North (Front)
                Point 0
                   ↑
                   |
    Point 270 ←---🤖---→ Point 90
        West       |        East
                   |
                   ↓
               Point 180
                 South
```

**Key Indices:**
- Point 0: Front (0°, North)
- Point 90: Right (90°, East)
- Point 180: Back (180°, South)
- Point 270: Left (270°, West)

## Sector-Based Navigation

Instead of processing all 360 points individually, we divide the environment into logical sectors:

### Sector Definitions

```python
SECTORS:
1. Front: Points 85-95 (±5° from straight ahead)
2. Front-Left: Points 60-120 (60° cone on left-front)
3. Front-Right: Points 300-360 (60° cone on right-front)
4. Left: Points 135-225 (90° left side)
5. Right: Points 0-45 + 315-360 (90° right side)
6. Back: Points 225-315 (90° behind)
```

### Why Sectors?

**Advantages:**
- Simplifies decision making (6 values instead of 360)
- Focuses on relevant directions
- Reduces computational load
- Mirrors human spatial awareness

**Decision Logic:**
```python
if front_sector < CRITICAL_DISTANCE:
    # Emergency! Back up immediately
    
elif front_sector < SAFE_DISTANCE:
    # Obstacle ahead, choose direction to turn
    if front_left > front_right:
        turn_left()
    else:
        turn_right()
        
else:
    # Path clear, go forward
    # Adjust slightly if too close to side walls
```

## Configuration Parameters

```python
# LIDAR Settings
LIDAR_POINTS = 360          # Number of measurement points
LIDAR_FOV = 2 * math.pi     # 360 degrees in radians
LIDAR_MAX_RANGE = 2.0       # Maximum detection (meters)
PUBLISH_RATE = 10           # Data update frequency (Hz)

# Navigation Settings
BASE_SPEED = 2.0            # Normal forward speed (rad/s)
TURN_SPEED = 1.5            # Rotation speed (rad/s)
MAX_SPEED = 6.28            # Speed limit (rad/s)

# Safety Zones
SAFE_DISTANCE = 0.6         # Start avoiding at this distance
CRITICAL_DISTANCE = 0.3     # Emergency stop distance

# Manual Control
MANUAL_FORWARD_SPEED = 3.0  # Manual mode forward speed
MANUAL_TURN_SPEED = 2.0     # Manual mode turn speed
```

## Algorithm Breakdown

### 1. Data Processing Pipeline

```
LIDAR Raw Data (360 float values)
    ↓
Filter invalid readings (inf, nan, negative)
    ↓
Group into 6 sectors
    ↓
Find minimum distance in each sector
    ↓
Decision logic based on sector distances
    ↓
Motor commands (left_speed, right_speed)
```

### 2. Sector Analysis

For each sector, we find the **minimum distance** (closest obstacle):

```python
def _get_sector_min(range_data, indices):
    valid_ranges = []
    for idx in indices:
        r = range_data[idx]
        # Filter: positive, within range, not infinity/nan
        if 0 < r < LIDAR_MAX_RANGE and not math.isinf(r) and not math.isnan(r):
            valid_ranges.append(r)
    
    return min(valid_ranges) if valid_ranges else LIDAR_MAX_RANGE
```

**Why minimum?** 
- We care about the closest obstacle (most dangerous)
- One close object is enough to trigger avoidance
- Maximum would ignore nearby threats

### 3. Navigation Strategy

**Three-Tier Response:**

```python
# Tier 1: CRITICAL - Emergency maneuver
if front < 0.3m:
    back_up_and_turn_away()

# Tier 2: WARNING - Avoid obstacle
elif front < 0.6m:
    turn_toward_open_space()

# Tier 3: SAFE - Minor adjustments
else:
    go_forward()
    adjust_for_corridor_centering()
```

### 4. Corridor Centering

When navigating hallways, the robot stays centered:

```python
if left_distance < 0.6m and right_distance > 0.6m:
    # Too close to left wall
    left_wheel_speed = BASE_SPEED * 1.2   # Speed up left
    right_wheel_speed = BASE_SPEED * 0.8  # Slow down right
    # Result: Gentle turn away from wall
```

## Dashboard Visualization

### Radar Display

```
┌───  LIDAR SECTOR ANALYSIS ───┐
│                              │
│      ↖ FL  ↑ F   FR ↗        │
│      0.85  1.20  0.78        │
│                              │
│   L ← 0.92    🤖   0.65 → R  │
│                              │
│             1.45             │
│              ↓ B             │
└──────────────────────────────┘
```

**Interpretation:**
- Front (F): 1.20m - Clear
- Front-Right (FR): 0.78m - Close, may need to veer left
- Right (R): 0.65m - Within safe zone
- All other sectors: Safe

### Status Indicators

```
✅ Clear path - Closest obstacle: 1.20m
⚡ WARNING: Obstacle at 0.45m
⚠️  CRITICAL: Obstacle at 0.25m!
```

### Range Bar

Visual representation of front distance:
```
Front Distance: [████████████████░░░░░░░░░░] 1.20m
                 |<---- Clear ---->|<-Empty->|
```

## Running the Controller

```bash
# Terminal 1: Launch Webots simulation
webots ~/ros2_ws/src/ce_webots/worlds/051_lidar.wbt

# Terminal 2: Run ROS2 controller
source ~/ros2_ws/install/setup.bash
ros2 run ce_webots 051_lidar_controller

# Optional - Visualize in RViz2
ros2 run rviz2 rviz2
# Add LaserScan display, topic: /lidar_scan
# Add PointCloud display, topic: /lidar_point_cloud
```

## ROS2 Integration

### Published Topics

**1. LaserScan Message**
```python
Topic: /lidar_scan
Type: sensor_msgs/LaserScan
Rate: 10 Hz

Fields:
- angle_min: 0.0 (start angle)
- angle_max: 6.28319 (2π radians)
- angle_increment: 0.01745 (1 degree)
- range_min: 0.1m
- range_max: 2.0m
- ranges: [360 float values]
```

**2. PointCloud Message**
```python
Topic: /lidar_point_cloud
Type: sensor_msgs/PointCloud
Rate: 10 Hz

Contains:
- header (timestamp, frame_id)
- points: [(x, y, z) coordinates]
```

### Coordinate Transformation

LIDAR provides polar coordinates (angle, distance), converted to Cartesian (x, y):

```python
for i, distance in enumerate(range_data):
    angle = i * (2π / 360)
    x = distance * cos(angle)
    y = distance * sin(angle)
    points.append((x, y, 0))
```

## Keyboard Controls

| Key | Action | Mode |
|-----|--------|------|
| W | Move forward | Manual |
| S | Move backward | Manual |
| A | Turn left | Manual |
| D | Turn right | Manual |
| SPACE | Toggle AUTO/MANUAL | Both |
| ESC | Exit program | Both |

**Modes:**
- **AUTO**: Robot uses LIDAR to navigate autonomously
- **MANUAL**: User controls with WASD keys

## Common Issues & Solutions

### Problem: Robot gets stuck in corners

**Symptoms:** Spins in place, can't find exit

**Cause:** Front, left, and right all blocked

**Solutions:**
1. Improve back-up logic - reverse longer distances
2. Add random exploration when stuck
3. Increase TURN_SPEED for faster escapes
4. Implement wall-following behavior

```python
# Enhanced stuck detection
if front < 0.6 and left < 0.6 and right < 0.6:
    # Back up until back sensor clears
    while back < 1.0:
        move_backward()
    # Then turn toward most open direction
```

### Problem: LIDAR shows all infinity values

**Symptoms:** All sectors show max range, no obstacles detected

**Solutions:**
1. Check LIDAR is enabled: `lidar.enable(timestep)`
2. Verify LIDAR name matches: `getDevice('lidar')`
3. Confirm obstacles have boundingObject defined
4. Check LIDAR maxRange >= arena diagonal (4x4 arena → need ~5.6m)

### Problem: Robot ignores obstacles on one side

**Symptoms:** Crashes into left/right walls but avoids front

**Causes & Fixes:**
1. **Sector indices wrong** - Verify point 0 = front after rotation
2. **LIDAR rotation misaligned** - Check robot rotation matches LIDAR
3. **Sector ranges incorrect** - Print raw data to debug

```python
# Debug: Print all sector ranges
for name, distance in sector_distances.items():
    print(f"{name}: {distance:.2f}m")
```

### Problem: Oscillation in corridors

**Symptoms:** Zigzags between walls, unstable centering

**Solutions:**
1. Reduce centering adjustment multipliers (1.2 → 1.1, 0.8 → 0.9)
2. Add deadband: only adjust if difference > threshold
3. Use exponential smoothing on distance readings

```python
# Deadband approach
left_right_diff = abs(left_dist - right_dist)
if left_right_diff > 0.15:  # Only adjust if significant
    apply_centering_correction()
```

### Problem: Front sensor shows 2.0m but obstacle visible

**Symptoms:** Robot crashes despite clear sensor readings

**Causes:**
1. Obstacle below LIDAR height (LIDAR at 0.12m, obstacle at 0.05m)
2. Obstacle material doesn't reflect laser (black, absorptive)
3. Obstacle too thin (smaller than laser beam width)

**Fix:** Ensure all obstacles:
- Extend above LIDAR height
- Have reflective materials (PBRAppearance with non-zero roughness)
- Minimum width ~0.1m

## Performance Tuning

### For Tight Spaces (Small Arena):
```python
BASE_SPEED = 1.5           # Slower for reaction time
SAFE_DISTANCE = 0.5        # Start avoiding earlier
CRITICAL_DISTANCE = 0.25   # Shorter emergency zone
```

### For Open Environments:
```python
BASE_SPEED = 3.0           # Faster cruising
SAFE_DISTANCE = 0.8        # More space to maneuver
TURN_SPEED = 2.0           # Quicker turns
```

### For Narrow Corridors:
```python
# Enable aggressive centering
CENTERING_GAIN = 1.5       # Strong wall avoidance
CORRIDOR_WIDTH = 0.8       # Trigger centering threshold
```

## Exercises

### Exercise 1: Sector Visualization
**Goal:** Understand LIDAR coordinate system

1. Print raw LIDAR data for points [0, 90, 180, 270]
2. Place obstacles at known positions (1m north, 1m east, etc.)
3. Verify corresponding LIDAR points show correct distances
4. Draw a top-down map on paper based on LIDAR data

### Exercise 2: Custom Sector Strategy
**Goal:** Implement different navigation behaviors

Create three strategies:
1. **Wall Follower:** Always keep left wall at 0.5m distance
2. **Explorer:** Prioritize least-visited directions
3. **Escape Artist:** Always head toward largest open space

Compare their effectiveness in different environments.

### Exercise 3: Mapping
**Goal:** Build a simple occupancy grid

1. Create a 2D array representing arena (40x40 cells)
2. For each LIDAR reading, mark occupied cells
3. Track robot position using IMU/odometry
4. Build cumulative map over time
5. Display map in terminal using ASCII characters

```python
# Example output
Map:
█████████████████████
█                   █
█    ▓              █
█         🤖        █
█              ▓    █
█                   █
█████████████████████

█ = walls, ▓ = obstacles, 🤖 = robot
```

### Exercise 4: Obstacle Classification
**Goal:** Identify object shapes from LIDAR

**Hint:** Analyze cluster patterns
- Small circular object: 10-15 continuous close readings
- Flat wall: 60+ readings at similar distance
- Corner: Sharp distance change (0.5m → 2.0m in 2 points)

```python
def classify_obstacle(ranges, start_idx, end_idx):
    readings = ranges[start_idx:end_idx]
    span = end_idx - start_idx
    
    if span > 60:
        return "WALL"
    elif span < 15:
        return "SMALL_OBJECT"
    else:
        return "LARGE_OBJECT"
```

### Exercise 5: Multi-Robot Coordination
**Goal:** Avoid other moving robots

**Challenge:** Add second robot to arena, modify LIDAR controller to:
1. Detect moving obstacles (changing distance over time)
2. Predict collision paths
3. Yield to other robots
4. Maintain minimum separation distance

## Advanced Topics

### 1. SLAM (Simultaneous Localization and Mapping)

Basic concept:
```python
# Pseudo-code for simple SLAM
while robot_active:
    lidar_scan = get_lidar_data()
    robot_pose = estimate_position(imu, odometry)
    
    map = update_map(map, lidar_scan, robot_pose)
    path = plan_path(map, robot_pose, goal)
    
    execute_motion(path)
```

Popular SLAM algorithms:
- GMapping (Grid-based)
- Cartographer (Google)
- RTAB-Map (RGB-D + LIDAR)
- LaMa (Fast 2D SLAM)

### 2. Particle Filter Localization

Use LIDAR to localize in known map:

```python
# 1. Scatter particles (possible robot positions)
particles = generate_random_poses(1000)

# 2. For each particle, simulate LIDAR scan
for particle in particles:
    expected_scan = simulate_lidar(map, particle.pose)
    weight = compare_scans(expected_scan, actual_scan)
    particle.weight = weight

# 3. Resample based on weights
particles = weighted_resample(particles)

# 4. Estimate pose as weighted average
robot_pose = mean(particles)
```

### 3. Dynamic Window Approach (DWA)

Advanced navigation considering robot dynamics:

```python
# 1. Generate velocity samples
for v in velocity_range:
    for w in angular_velocity_range:
        # 2. Simulate trajectory
        trajectory = predict_path(v, w, time_horizon)
        
        # 3. Score trajectory
        score = 0
        score += distance_to_goal(trajectory.end)
        score -= proximity_to_obstacles(trajectory)
        score -= deviation_from_path(trajectory)
        
        # 4. Choose best
        if score > best_score:
            best_v, best_w = v, w
```

### 4. Scan Matching

Align consecutive LIDAR scans to estimate motion:

```python
def estimate_motion(scan_t0, scan_t1):
    """
    Find transformation (dx, dy, dtheta) that best aligns scans
    """
    best_match = None
    best_error = float('inf')
    
    for dx in range(-0.5, 0.5, 0.01):
        for dy in range(-0.5, 0.5, 0.01):
            for dtheta in range(-30, 30, 1):
                transformed = transform_scan(scan_t1, dx, dy, dtheta)
                error = compute_distance(scan_t0, transformed)
                
                if error < best_error:
                    best_error = error
                    best_match = (dx, dy, dtheta)
    
    return best_match
```

### 5. Frontier Exploration

Autonomous exploration of unknown environments:

```python
# 1. Build occupancy grid from LIDAR
# 2. Find frontiers (boundary between known/unknown)
# 3. Choose nearest frontier as goal
# 4. Navigate to frontier
# 5. Repeat until entire area mapped

def find_frontiers(occupancy_grid):
    frontiers = []
    for cell in occupancy_grid:
        if cell.is_free() and cell.has_unknown_neighbor():
            frontiers.append(cell)
    return cluster_frontiers(frontiers)
```

## Mathematics Background

### Polar to Cartesian Conversion

Given LIDAR reading at angle θ and distance r:

$$
x = r \cos(\theta)
$$
$$
y = r \sin(\theta)
$$

### Distance Between Points

Euclidean distance between two LIDAR points:

$$
d = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
$$

### Sector Minimum Algorithm

For sector $S$ with indices $[i_{start}, i_{end}]$:

$$
d_{min} = \min_{i \in S} \{ r_i \mid r_i > 0 \land r_i < r_{max} \}
$$

### Obstacle Avoidance Vector

Repulsive force from obstacle:

$$
\vec{F}_{avoid} = K \cdot \frac{1}{d^2} \cdot \hat{d}
$$

Where:
- $K$ = avoidance gain constant
- $d$ = distance to obstacle
- $\hat{d}$ = unit vector away from obstacle

## Real-World Comparison

### This Simulation vs Real LIDAR

| Feature | Simulation | Real LIDAR (Hokuyo UST-10LX) |
|---------|-----------|------------------------------|
| Range | 2m | 10m |
| Resolution | 360 points (1°) | 1080 points (0.25°) |
| Frequency | 10 Hz | 40 Hz |
| Accuracy | Perfect | ±30mm |
| Cost | Free | $1,000-$5,000 |
| Noise | None | Gaussian noise, outliers |
| Weather | N/A | Affected by rain, fog, dust |

### Commercial LIDAR Examples

1. **Velodyne VLP-16** (Puck)
   - 16 layers, 360° horizontal
   - 100m range, $4,000
   - Used in: Autonomous cars, drones

2. **Sick TiM561**
   - 2D, 10m range, $1,500
   - Used in: AGVs, warehouse robots

3. **Ouster OS1-64**
   - 64 layers, 120m range
   - Solid-state, $12,000
   - Used in: Robotaxis, mining

## Next Steps

After mastering LIDAR basics:

1. **Level 5.2** - 3D LIDAR / Point Cloud Processing
2. **Level 5.3** - LIDAR + Camera Fusion
3. **Level 6** - Navigation Stack (ROS2 Nav2)
4. **Level 7** - SLAM and Mapping

## References

- **ROS2 Navigation:** https://navigation.ros.org/
- **LIDAR Principles:** https://en.wikipedia.org/wiki/Lidar
- **sensor_msgs/LaserScan:** https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
- **GMapping SLAM:** https://openslam.org/gmapping.html
- **Dynamic Window Approach:** Fox et al., 1997

---

**Experiment boldly!** LIDAR is the foundation of modern autonomous navigation. Understanding it opens doors to self-driving cars, delivery robots, and advanced robotics! 🤖🔦
