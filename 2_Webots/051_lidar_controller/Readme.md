# 🔴 051 - LIDAR Controller

> **Level 5.1 - Range Sensing | 360° Environment Mapping and Obstacle Detection**

---

## 📌 Overview

The **LIDAR Controller** introduces **360-degree environment scanning**. LIDAR (Light Detection and Ranging) measures distances in all directions to create precise spatial maps. Unlike cameras that capture 2D images, LIDAR creates distance arrays enabling sophisticated obstacle avoidance and mapping—your robot now has 360° vision!

### ✨ Key Features

- 🔴 360° environment scanning (360 distance points)
- 🗺️ Sector-based navigation strategy
- 📊 Real-time distance array visualization
- 🧭 Obstacle avoidance in all directions
- 📈 Multi-level threat assessment
- 🎯 Smart navigation decisions
- 📡 ROS2 LaserScan publishing
- 🎨 Real-time status display

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `051_lidar_controller.py` | LIDAR navigation controller |
| `051_lidar.wbt` | Webots world file |
| `051_lidar_controller.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/051_lidar.wbt
```

### Step 2️⃣: Run LIDAR Controller

```bash
ros2 run ce_webots 051_lidar_controller
```

### Real-Time Visualization

```
================================================================================
                    360° LIDAR SCAN (Real-time)
================================================================================

                        Front (0°)
                            ↑
           ╱─────────────────────────────────╲
          ╱  120° ┊        ┊        ┊ 60°     ╲
         ╱        ╱════════════════╲          ╲
        │        │    🟢 0.95m      │          │
        │ 150°   │                  │ 30°      │ West
        │        │    (Safe)        │          │ (270°)
        │────────┤      🤖          ├──────────│
        │ 210°   │                  │ 330°     │
        │        │    (Obstacles)   │          │
        │        ╲════════════════╱           │
         ╲        ╲ 120° ┊ ┊ 60° ╱           ╱
          ╲        ╲─────────────╱            ╱
           ╲_____________________________────╱

        Front: 🟢 0.95m (Safe)
        Left:  🟡 0.32m (Warning)
        Right: 🟢 0.88m (Safe)
        Back:  🟢 1.50m (Safe)

================================================================================
```

---

## 🔧 How It Works

### 1. Initialize LIDAR

```python
# Get LIDAR device
self.lidar = self.robot.getDevice('lidar')
self.lidar.enable(self.timestep)

# LIDAR specifications
self.NUM_POINTS = 360      # 1° resolution (360 points)
self.MAX_RANGE = 2.0       # meters
```

### 2. Read LIDAR Data

```python
def read_lidar(self):
    """Get 360-point distance array from LIDAR"""
    # Returns array of 360 distances (one per degree)
    distances = self.lidar.getRangeImage()
    
    return distances  # [d0°, d1°, d2°, ... d359°]
```

### 3. Sector-Based Navigation

Instead of processing all 360 points individually, divide into sectors:

```python
def analyze_sectors(self, distances):
    """Organize LIDAR data into directional sectors"""
    
    sectors = {
        'front': min(distances[85:95]),      # ±5° from front
        'front_left': min(distances[60:120]),   # 60° cone left
        'front_right': min(distances[240:300]), # 60° cone right
        'left': min(distances[75:165]),      # 90° left side
        'right': min(distances[195:285]),    # 90° right side
        'back': min(distances[135:225]),     # 90° behind
    }
    
    return sectors
```

### 4. Point Indexing System

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

Key Indices:
  0°:   Front
  90°:  Right
  180°: Back
  270°: Left
```

### 5. Multi-Level Decision Logic

```python
def navigate(self, sectors):
    """Choose action based on proximity threats"""
    
    # LEVEL 1: Emergency - Multiple obstacles
    if (sectors['front'] < 0.2 and 
        sectors['front_left'] < 0.25 and 
        sectors['front_right'] < 0.25):
        return BACK_UP_TURN()
    
    # LEVEL 2: Critical - Front obstacle
    elif sectors['front'] < 0.3:
        if sectors['left'] > sectors['right']:
            return TURN_LEFT()
        else:
            return TURN_RIGHT()
    
    # LEVEL 3: Safe - Move forward
    else:
        if sectors['left'] < 0.5 or sectors['right'] < 0.5:
            return ADJUST_SIDES()
        else:
            return MOVE_FORWARD()
```

---

## 📊 LIDAR Information

### Sensor Specification

| Property | Value |
|----------|-------|
| **Type** | 2D Scanning LIDAR |
| **Resolution** | 360 points (1° per point) |
| **FOV** | 360° (full circle) |
| **Range** | 0.05m to 2.0m |
| **Update Rate** | Real-time |

### Distance Interpretation

```
Distance < 0.20m:  🔴 DANGER (immediate threat)
Distance 0.20-0.35: 🟠 WARNING (slow/turn)
Distance 0.35-0.60: 🟡 CAUTION (adjust course)
Distance > 0.60m:  🟢 SAFE (proceed)
```

---

## 📚 Sensor Knowledge: LIDAR (Light Detection and Ranging)

### 🎯 How It Works

**LIDAR** emits infrared laser rays in all 360° and measures the return time:

```
LIDAR 360° Scan:
    ↑ 90°
    │
    │  point_1  point_2  point_3
    │    ↓        ↓        ↓
270°←───●─────────●─────────●──→ 90°
    │    ↓        ↓        ↓
    │ 
    ↓ 270°

Scanning Pattern:
  • 360 rays emitted per scan
  • 1° angular resolution
  • Range: 0.05-2.0 meters
  • Rotation: ~10 Hz
  
Data Structure:
  ranges[] = [d0, d1, d2, ..., d359]
  
  index = 0:   0° (robot front)
  index = 90:  90° (robot left)
  index = 180: 180° (robot back)
  index = 270: 270° (robot right)
```

### 📊 Specifications

| คุณสมบัติ | ค่า | หมายเหตุ |
|---------|-----|--------|
| **จำนวน Rays** | 360 | One per degree |
| **ความละเอียด** | 1° | Angular resolution |
| **ช่วงวัด** | 0.05-2.0 m | Typical range |
| **ความแม่นยำ** | ±5 cm | ±5% error |
| **Scan Rate** | ~10 Hz | 100 ms/scan |
| **Max Range** | 2.0 m | Beyond = ∞ |
| **Dead Zone** | 0.05 m | Too close |

### 💡 Usage Tips

**✅ ทำได้:**
- 360° obstacle detection
- Sector-based analysis
- Distance mapping
- Wall following
- SLAM applications

**❌ ที่ควรหลีกเลี่ยง:**
- Treating all rays equally
- Ignoring angular information
- Using raw distances without filtering
- Processing every ray (sample instead)
- Not considering max range

### ⚠️ Limitations

```
1. Dead Zone Near Robot
   ├─ 0.05m minimum range
   ├─ Closer objects = not detected
   └─→ Solution: Use bumper sensor

2. Only Range Data (No Texture)
   ├─ Can't distinguish objects
   ├─ Shiny surfaces = reflection issues
   └─→ Solution: Combine with camera

3. Angular Resolution
   ├─ 1° = coarse for small objects
   ├─ 360 rays total
   └─→ Solution: Use multiple scans

4. Reflection Issues
   ├─ Glass/mirrors = no reflection
   ├─ Dark objects = poor reflection
   └─→ Solution: Sensor fusion

5. Slow Rotation
   ├─ ~10 Hz scan rate
   ├─ Not real-time for high-speed robots
   └─→ Solution: Predict movements
```

### 🔧 LIDAR Data Analysis

```python
import numpy as np

def lidar_scan_analysis(ranges, num_rays=360):
    """Analyze LIDAR scan for obstacles"""
    
    # Remove invalid readings (>max_range)
    valid_ranges = np.array(ranges)
    valid_ranges[valid_ranges > 2.0] = 2.0  # Cap at max
    
    # 1. Front sector analysis (0° to 45°)
    front_rays = list(range(0, 45)) + list(range(315, 360))
    front_distances = [valid_ranges[i] for i in front_rays]
    front_min = np.min(front_distances)
    front_avg = np.mean(front_distances)
    
    # 2. Left sector analysis (45° to 135°)
    left_rays = list(range(45, 135))
    left_distances = [valid_ranges[i] for i in left_rays]
    left_min = np.min(left_distances)
    
    # 3. Back sector analysis (135° to 225°)
    back_rays = list(range(135, 225))
    back_distances = [valid_ranges[i] for i in back_rays]
    back_min = np.min(back_distances)
    
    # 4. Right sector analysis (225° to 315°)
    right_rays = list(range(225, 315))
    right_distances = [valid_ranges[i] for i in right_rays]
    right_min = np.min(right_distances)
    
    return {
        'front_min': front_min,
        'front_avg': front_avg,
        'left_min': left_min,
        'back_min': back_min,
        'right_min': right_min,
    }

def detect_walls(ranges, threshold=0.3):
    """Detect nearby walls/obstacles"""
    
    valid_ranges = np.array(ranges)
    valid_ranges[valid_ranges > 2.0] = np.inf
    
    # Find rays with close obstacles
    obstacles = np.where(valid_ranges < threshold)[0]
    
    if len(obstacles) == 0:
        return None
    
    # Find continuous segments
    diffs = np.diff(obstacles)
    breaks = np.where(diffs > 1)[0]
    
    segments = []
    start_idx = 0
    for break_idx in breaks:
        segments.append(obstacles[start_idx:break_idx+1])
        start_idx = break_idx + 1
    segments.append(obstacles[start_idx:])
    
    # Convert to angle ranges
    walls = []
    for segment in segments:
        start_angle = segment[0]
        end_angle = segment[-1]
        center_angle = np.mean(segment)
        
        walls.append({
            'start': start_angle,
            'end': end_angle,
            'center': center_angle,
            'width': len(segment),
        })
    
    return walls

def calculate_safe_direction(ranges, threat_distance=0.5):
    """Find safest direction to move"""
    
    analysis = lidar_scan_analysis(ranges)
    
    # Score each direction
    sectors = {
        'front': analysis['front_avg'],
        'left': analysis['left_min'],
        'right': analysis['right_min'],
    }
    
    # Find safest
    safest = max(sectors.items(), key=lambda x: x[1])
    
    return safest[0], safest[1]  # direction, distance

# LIDAR Scan Visualization
def print_lidar_map(ranges, ascii_width=40):
    """ASCII visualization of LIDAR scan"""
    
    print("═" * (ascii_width + 2))
    print("║ LIDAR SCAN (Top View) " + " " * (ascii_width - 21) + "║")
    print("╠" + "═" * ascii_width + "╣")
    
    # Convert ranges to ASCII
    max_range = 2.0
    center = ascii_width // 2
    
    # Simple visualization
    for i in range(0, 360, 45):
        distance = ranges[i] if i < len(ranges) else 0
        distance = min(distance, max_range)
        bar_length = int((distance / max_range) * (ascii_width // 2))
        
        direction_names = ['▲ Front', '◀ Left', '▼ Back', '▶ Right']
        direction = direction_names[i // 90]
        
        print(f"║ {direction}: {'█' * bar_length} {distance:.2f}m")
    
    print("╚" + "═" * ascii_width + "╝")
```

### 📊 Sector-Based Obstacle Detection

```
         FRONT (0°)
            ▲
            │
  FRONT    │ FRONT
   LEFT    │  RIGHT
    │      │      │
    └──────●──────┘  ← Robot
           │
        BACK (180°)

Sector Mapping:
  [0°-45°]:     Front-Right
  [45°-135°]:   Left
  [135°-225°]:  Back
  [225°-315°]:  Right  
  [315°-360°]:  Front-Left
  
Analysis Table:
┌─────────┬──────────┬──────────┐
│ Sector  │ Min Dist │ Decision │
├─────────┼──────────┼──────────┤
│ Front   │  1.2 m   │ Proceed  │
│ Left    │  0.8 m   │ OK       │
│ Back    │  2.0 m   │ Clear    │
│ Right   │  0.4 m   │ Obstacle │
└─────────┴──────────┴──────────┘
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ LIDAR sensor principles and operation
- ✅ 360° distance array interpretation
- ✅ Sector-based decision making
- ✅ Multi-threat obstacle avoidance
- ✅ Spatial awareness in robotics
- ✅ Real-time environment mapping
- ✅ ROS2 LaserScan message format

---

## 📈 Sector Analysis Example

**Scenario: Narrow Corridor**

```
LIDAR Readings:
  Front (0°):        0.40m 🟡
  Front-Left (70°):  0.18m 🔴
  Left (90°):        0.15m 🔴
  Front-Right (290°): 0.50m 🟡
  Right (270°):      0.55m 🟡
  Back (180°):       2.00m 🟢

Decision:
  ✗ Can't go straight (0.40m, too close)
  ✗ Can't turn left (wall at 0.15m)
  ✓ Turn right (0.55m available)
  ✓ Move forward-right
```

---

## 📝 Customization

### Adjust Safety Distances

```python
EMERGENCY_DISTANCE = 0.2    # Trigger backup
CRITICAL_DISTANCE = 0.3     # Trigger turn
SAFE_DISTANCE = 0.6         # Safe to move
```

### Change Sector Angles

```python
# Wider front sector (±15° instead of ±5°)
sectors['front'] = min(distances[75:105])

# Narrow left/right sectors
sectors['left'] = min(distances[80:100])
sectors['right'] = min(distances[260:280])
```

### Add Wall-Following Behavior

```python
def wall_follow(self, sectors):
    """Keep consistent distance from wall"""
    target_wall_distance = 0.3
    
    if sectors['left'] < target_wall_distance:
        return MOVE_RIGHT()  # Back away from wall
    elif sectors['left'] > target_wall_distance + 0.1:
        return MOVE_LEFT()   # Get closer to wall
    else:
        return MOVE_FORWARD()  # Maintain distance
```

---

## 📚 Related Resources

- 📖 [Webots LIDAR](https://cyberbotics.com/doc/reference/lidar)
- 🔗 [LIDAR Technology](https://en.wikipedia.org/wiki/Lidar)
- 🗺️ [2D Mapping Concepts](https://en.wikipedia.org/wiki/Occupancy_grid_mapping)
- 🤖 [ROS 2 LaserScan Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)
- 👀 [022 Distance Sensor](../022_distance_sensor_controller/) (5-sensor alternative)
- 👀 [052 SLAM Controller](../052_slam_controller/) (Advanced mapping)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **No LIDAR readings** | Verify enable() called; check world file |
| **All distances = 0** | Check LIDAR orientation; verify range |
| **Robot doesn't avoid** | Lower thresholds; check sector definitions |
| **Erratic behavior** | Increase sector sizes; reduce sensitivity |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
