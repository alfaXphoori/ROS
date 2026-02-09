# 061 Depth Camera (RGB-D) Documentation

## 🎯 Why Learn This?

**Problem with 2D Lidar:**
> Lidar sees only "lines" (2D) in a single horizontal plane. If there's a table with thin legs but a wide tabletop, Lidar might see through the legs and collide with the table surface!

**Solution: Depth Camera (RGB-D)**
> Depth cameras (like Kinect, RealSense) see "3D images" (Point Clouds), allowing the robot to understand object shapes and measure distance for **every pixel** in the image.

---

## Key Concepts

### Lidar vs Depth Camera Comparison

| Feature | 2D Lidar | Depth Camera (RGB-D) |
|---------|----------|---------------------|
| **Perception** | Single horizontal plane | Full 3D field of view |
| **Data Type** | 360° distance array | 3D point cloud (X, Y, Z) |
| **Table Detection** | ❌ May miss tabletops | ✅ Sees tabletop AND legs |
| **Object Shape** | ❌ Cannot determine | ✅ Can reconstruct 3D shape |
| **Color Info** | ❌ No color | ✅ RGB color per point |
| **Height Awareness** | ❌ Fixed height | ✅ Multiple height levels |
| **Use Cases** | Floor mapping, walls | Manipulation, 3D mapping |

### Real-World Example: The Table Problem

```
Side View:
                ┌─────────────┐  ← Tabletop (height: 0.75m)
                │             │
          Lidar ─────→──────  │  ← Lidar beam (height: 0.2m)
                │             │     Passes under table!
                │             │
                │             │
                └─┘         └─┘  ← Narrow table legs
                
Robot Result: ❌ COLLISION with tabletop

With Depth Camera:
📷 Sees BOTH legs and tabletop → Avoids collision ✅
```

---

## Architecture

```
┌───────────────────────────────────────────────────────────┐
│           DepthCameraController Node                      │
├───────────────────────────────────────────────────────────┤
│ Inputs:                                                   │
│  • RGB Camera → Color images                             │
│  • Depth Camera (RangeFinder) → Distance per pixel       │
│  • Wheel Encoders → Odometry                             │
│  • Keyboard → Manual control                             │
├───────────────────────────────────────────────────────────┤
│ Processing:                                               │
│  1. Depth Image → Point Cloud Conversion                 │
│  2. 3D Obstacle Analysis (by height zones)               │
│  3. 3D-Aware Navigation Logic                            │
│  4. TF Tree Broadcasting                                 │
├───────────────────────────────────────────────────────────┤
│ Outputs:                                                  │
│  • /camera/depth/points (PointCloud2)                    │
│  • /camera/rgb/image_raw (Image)                         │
│  • /camera/depth/image_raw (Image)                       │
│  • /camera/depth/camera_info (CameraInfo)                │
│  • /odom (Odometry)                                      │
│  • TF Transforms                                         │
└───────────────────────────────────────────────────────────┘
```

---

## Features

### 1. **RGB-D Camera System**
- **RGB Camera**: Captures color images (640x480)
- **Depth Camera**: Measures distance for each pixel (range: 0.1m - 5.0m)
- **Synchronized**: Both cameras aligned for accurate color mapping

### 2. **Point Cloud Generation**
Converts 2D depth image + RGB image into 3D point cloud:

```python
For each pixel (u, v) in depth image:
    depth = depth_image[v][u]
    
    # Convert pixel to 3D point using camera intrinsics
    X = (u - cx) × depth / fx
    Y = (v - cy) × depth / fy
    Z = depth
    
    # Get color from RGB image
    R, G, B = rgb_image[v][u]
    
    point_cloud.add(X, Y, Z, R, G, B)
```

### 3. **3D Obstacle Detection**
Analyzes obstacles by **height zones**:

| Zone | Height Range | Example Objects |
|------|--------------|-----------------|
| **Ground** | < 0.05m | Floor, shadows |
| **Low** | 0.05 - 0.3m | Table legs, chair legs, pets |
| **Mid** | 0.3 - 0.5m | Chairs, low shelves |
| **High** | 0.5 - 1.5m | Tabletops, counters, shelves |
| **Ceiling** | > 1.5m | Ceiling, overhead lights |

### 4. **3D-Aware Navigation**
Navigation logic that understands vertical obstacles:

```python
if tabletop_detected_ahead:
    turn_away()  # Even if legs are far!
    
elif low_obstacle_ahead:
    can_drive_under()  # If robot is shorter
    
elif overhead_clearance_low:
    duck_or_turn()  # Avoid hitting camera
```

### 5. **ROS2 Visualization**
- **PointCloud2**: Display 3D colored points in RViz
- **RGB Image**: Raw color feed
- **Depth Image**: Grayscale distance visualization
- **CameraInfo**: Calibration data for reconstruction

---

## Configuration Parameters

### Robot Parameters
```python
WHEEL_RADIUS = 0.08      # meters
AXLE_LENGTH = 0.24       # meters
MAX_SPEED = 4.0          # rad/s
```

### Distance Thresholds
```python
EMERGENCY_DISTANCE = 0.3  # Stop immediately
SAFE_DISTANCE = 1.0       # Slow down zone
HEIGHT_MIN = 0.05         # Ignore ground
HEIGHT_MAX = 1.5          # Ignore ceiling
```

### Camera Parameters
```python
CAMERA_FOV_H = 90°        # Horizontal field of view
CAMERA_FOV_V = 60°        # Vertical field of view
RESOLUTION = 640 × 480    # Image size
MAX_RANGE = 5.0 m         # Maximum depth
MIN_RANGE = 0.1 m         # Minimum depth
```

---

## Core Algorithms

### 1. Pinhole Camera Model

Convert pixel coordinates `(u, v)` to 3D point `(X, Y, Z)`:

```python
# Camera intrinsic parameters
fx = width / (2 × tan(FOV_H / 2))
fy = height / (2 × tan(FOV_V / 2))
cx = width / 2   # Principal point X
cy = height / 2  # Principal point Y

# Back-projection
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
Z = depth[v][u]
```

**Matrix Form:**
$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = 
\begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
\begin{bmatrix} X/Z \\ Y/Z \\ 1 \end{bmatrix}
$$

### 2. Point Cloud Generation

**Pseudocode:**
```python
point_cloud = []

for v in range(0, height, step):
    for u in range(0, width, step):
        depth = depth_image[v][u]
        
        if depth is invalid:
            continue
        
        # 3D coordinates in camera frame
        X = (u - cx) × depth / fx
        Y = (v - cy) × depth / fy
        Z = depth
        
        # Color from RGB image
        R, G, B = rgb_image[v][u]
        
        # Transform to world frame if needed
        point_world = transform(X, Y, Z, camera_pose)
        
        point_cloud.append((X, Y, Z, R, G, B))
```

**Optimization:** Downsample by processing every 4th pixel (step=4) to reduce computational load.

### 3. Height-Based Obstacle Analysis

```python
def analyze_obstacles(point_cloud):
    zones = {'front': ∞, 'left': ∞, 'right': ∞, 
             'center_low': ∞, 'center_high': ∞}
    
    for (X, Y, Z, R, G, B) in point_cloud:
        # Calculate height above ground
        height = camera_height - Y  # Y points down
        
        # Skip ground and ceiling
        if height < HEIGHT_MIN or height > HEIGHT_MAX:
            continue
        
        # Classify obstacle
        is_low = height < 0.3     # Table legs
        is_high = height > 0.5    # Table tops
        
        # Spatial zones (X=right, Z=forward)
        if |X| < 0.3:  # Center
            zones['front'] = min(zones['front'], Z)
            
            if is_low:
                zones['center_low'] = min(zones['center_low'], Z)
            if is_high:
                zones['center_high'] = min(zones['center_high'], Z)
        
        elif X < -0.2:  # Left
            zones['left'] = min(zones['left'], Z)
        
        elif X > 0.2:   # Right
            zones['right'] = min(zones['right'], Z)
    
    return zones
```

### 4. 3D Navigation Logic

```python
def navigate_3d(zones):
    # CRITICAL: Table top detected (even with open legs)
    if zones['center_high'] < EMERGENCY_DISTANCE:
        turn_away_from_table()
        return
    
    # EMERGENCY: Close obstacle
    if zones['front'] < EMERGENCY_DISTANCE:
        spin_to_open_side()
        return
    
    # CAUTION: Obstacle in safe zone
    if zones['front'] < SAFE_DISTANCE:
        steer_around_obstacle()
        return
    
    # CLEAR: Full speed
    drive_forward()
```

**Key Advantage:** Detects tabletops even when table legs are far apart!

---

## ROS2 Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/camera/depth/points` | `PointCloud2` | ~15 Hz | 3D colored point cloud |
| `/camera/rgb/image_raw` | `Image` | ~15 Hz | RGB color image (BGRA8) |
| `/camera/depth/image_raw` | `Image` | ~15 Hz | Depth image (MONO16, mm) |
| `/camera/depth/camera_info` | `CameraInfo` | ~15 Hz | Camera calibration |
| `/odom` | `Odometry` | ~30 Hz | Robot position estimate |

### TF Tree

```
map
 └─ odom (static)
     └─ base_link (odometry)
         └─ camera_link (+0.15m forward, +0.3m up)
             └─ camera_depth_optical_frame (rotated)
```

**Optical Frame Convention:**
- X-axis → Right
- Y-axis → Down
- Z-axis → Forward (into scene)

---

## Usage

### 1. Setup Webots World

Add RGB-D camera to robot in Webots:

```
Robot {
  children [
    Camera {
      name "camera_rgb"
      width 640
      height 480
      near 0.1
      far 5.0
    }
    RangeFinder {
      name "camera_depth"
      width 640
      height 480
      minRange 0.1
      maxRange 5.0
    }
  ]
}
```

### 2. Build Package

```bash
cd /home/admin/ros2_ws
colcon build --packages-select ce_webots
source install/setup.bash
```

### 3. Run Controller

```bash
# Launch Webots with depth camera controller
ros2 launch ce_webots_launch depth_camera_world.launch.py
```

### 4. Visualize in RViz

```bash
rviz2

# Add displays:
# - PointCloud2 → /camera/depth/points (colored by RGB)
# - Image → /camera/rgb/image_raw
# - Image → /camera/depth/image_raw
# - TF → Show camera_link
```

### 5. Controls

- **[W]** - Forward
- **[S]** - Backward
- **[A]** - Turn Left
- **[D]** - Turn Right
- **[M]** - Toggle Auto Navigation
- **[Q]** - Quit

---

## Dashboard Output

```
============================================================
 📷 RGB-D CAMERA | Mode: AUTO
============================================================
 Position: X=1.23m  Y=-0.45m
------------------------------------------------------------
 3D Obstacle Detection:
   Front:  2.34m  |  Left: 3.12m  |  Right: 2.89m
   Low (legs):   3.50m  🦵
   High (table): 1.20m  🪑
------------------------------------------------------------
 ⚠️  TABLE TOP DETECTED! (Lidar would miss this)
------------------------------------------------------------
 Controls: [W/A/S/D] Move | [M] Toggle Auto Mode | [Q] Quit
```

---

## Technical Details

### Point Cloud Format (PointCloud2)

**Fields:**
- `x` (FLOAT32) - Meter (camera frame)
- `y` (FLOAT32) - Meter (camera frame)
- `z` (FLOAT32) - Meter (camera frame)
- `rgb` (UINT32) - Packed RGB (24-bit color)

**RGB Packing:**
```python
rgb_uint32 = (R << 16) | (G << 8) | B
```

**Point Step:** 16 bytes (4 floats × 4 bytes)

### Coordinate Frames

**Camera Frame (Standard ROS Convention):**
```
     Y (down)
     |
     |____X (right)
    /
   Z (forward into scene)
```

**Base Link Frame (Robot):**
```
     X (forward)
     |
     |____Y (left)
    /
   Z (up)
```

**Transform:** Camera frame is rotated -90° around X, then -90° around Z relative to base_link.

### Performance Optimization

**Downsampling:**
```python
step = 4  # Process every 4th pixel
# 640×480 = 307,200 pixels
# With step=4: 160×120 = 19,200 points
# Reduction: 94% fewer points, 16× faster
```

**Publishing Rate Control:**
```python
if iteration % 2 == 0:
    publish_pointcloud()  # 15 Hz instead of 30 Hz
```

---

## Advantages Over 2D Lidar

### 1. **Table Detection** ✅
- **Lidar:** Sees only at one height → Misses tabletops
- **Depth Camera:** Sees full 3D structure → Detects tops and legs

### 2. **Object Recognition** ✅
- **Lidar:** Distance only
- **Depth Camera:** Shape + Color → Can classify objects

### 3. **Overhang Detection** ✅
- **Lidar:** Cannot see above/below scan plane
- **Depth Camera:** Sees vertical obstacles

### 4. **Small Object Detection** ✅
- **Lidar:** May miss thin objects between rays
- **Depth Camera:** Dense pixel grid catches small objects

### 5. **Manipulation** ✅
- **Lidar:** No shape info → Cannot grasp
- **Depth Camera:** 3D shape → Can plan grasps

---

## Limitations

### 1. **Range**
- Typical max range: 3-5m (vs Lidar: 10-30m)
- Minimum range: ~0.1m (blind spot)

### 2. **Lighting Conditions**
- Infrared-based depth: Fails in sunlight
- RGB: Poor in darkness
- Solution: Use ToF (Time-of-Flight) cameras

### 3. **Reflective Surfaces**
- Mirrors, glass, shiny metal: Invalid depth
- Water surfaces: Absorption errors

### 4. **Computational Cost**
- Point clouds: 100K-1M points per frame
- Processing: GPU recommended for real-time

### 5. **Field of View**
- Typical: 60°-90° (vs Lidar: 360°)
- Solution: Rotate camera or use multiple cameras

---

## Common Issues & Solutions

### Issue 1: Empty Point Cloud
**Symptoms:** No points in RViz  
**Causes:**
1. Camera not enabled in Webots
2. Depth range too narrow
3. TF frame mismatch

**Solution:**
```python
# Check depth camera setup
self.depth_camera.enable(self.timestep)

# Verify depth range
print(f"Range: {self.min_range} to {self.max_range}m")

# Check frame_id in PointCloud2
pointcloud.header.frame_id = 'camera_depth_optical_frame'
```

### Issue 2: Point Cloud Misaligned
**Symptoms:** Points don't match RGB image  
**Cause:** Camera intrinsics incorrect

**Solution:**
```python
# Recalculate from FOV
fx = width / (2.0 × tan(CAMERA_FOV_H / 2.0))
fy = height / (2.0 × tan(CAMERA_FOV_V / 2.0))
```

### Issue 3: RViz Shows Noise
**Symptoms:** Random floating points  
**Cause:** Invalid depth values not filtered

**Solution:**
```python
if depth == float('inf') or depth > max_range or depth < min_range:
    continue  # Skip this point
```

### Issue 4: Low Frame Rate
**Symptoms:** Laggy visualization  
**Solutions:**
```python
# 1. Increase downsample step
step = 8  # Instead of 4

# 2. Reduce publish rate
if iteration % 5 == 0:
    publish_pointcloud()

# 3. Reduce resolution in Webots
Camera { width 320 height 240 }
```

---

## Extending the Controller

### 1. **Add Object Segmentation**
```python
from sklearn.cluster import DBSCAN

# Cluster point cloud into objects
clustering = DBSCAN(eps=0.1, min_samples=10)
labels = clustering.fit_predict(points[:, :3])  # X,Y,Z only

for object_id in set(labels):
    object_points = points[labels == object_id]
    classify_object(object_points)
```

### 2. **Implement Semantic Segmentation**
```python
# Use deep learning for pixel-wise classification
from torchvision.models.segmentation import deeplabv3_resnet50

model = deeplabv3_resnet50(pretrained=True)
semantic_labels = model(rgb_image)

# Combine with depth
for label in ['table', 'chair', 'person']:
    mask = semantic_labels == LABEL_ID[label]
    object_pointcloud = pointcloud[mask]
```

### 3. **Add 3D Object Detection**
```python
# Use PointNet++ or VoxelNet for 3D detection
import torch
from pointnet2 import PointNet2

detector = PointNet2(num_classes=10)
bounding_boxes = detector(pointcloud)  # [(x,y,z,w,h,d,class), ...]
```

### 4. **Implement SLAM with RGB-D**
```python
# Use ORB-SLAM3 or ElasticFusion
from orbslam3 import RGBD_SLAM

slam = RGBD_SLAM()
for rgb, depth in camera_stream:
    pose = slam.track(rgb, depth)
    map_3d = slam.get_map()
```

---

## Real-World Applications

### 1. **Indoor Navigation**
- Detect doorways, stairs, obstacles
- Understand room layout
- Avoid furniture

### 2. **Object Manipulation**
- Grasp planning (shape + distance)
- Bin picking (separate overlapping objects)
- Assembly tasks

### 3. **Human-Robot Interaction**
- Gesture recognition (hand tracking)
- Person following
- Social distancing

### 4. **3D Mapping**
- Create textured 3D models
- Virtual reality environments
- AR/VR applications

### 5. **Inspection**
- Measure object dimensions
- Detect defects (3D surface analysis)
- Quality control

---

## References

### Hardware
- [Intel RealSense](https://www.intelrealsense.com/)
- [Microsoft Kinect](https://developer.microsoft.com/en-us/windows/kinect/)
- [Orbbec Cameras](https://www.orbbec.com/)

### ROS2 Packages
- [depth_image_proc](http://wiki.ros.org/depth_image_proc) - Depth processing
- [image_geometry](http://wiki.ros.org/image_geometry) - Camera models
- [pcl_ros](http://wiki.ros.org/pcl_ros) - Point cloud processing

### Algorithms
- Point Cloud Library (PCL): https://pointclouds.org/
- Open3D: http://www.open3d.org/
- OpenCV RGBD Module: https://docs.opencv.org/4.x/d2/d3a/group__rgbd.html

### Papers
- *KinectFusion: Real-time Dense Surface Mapping* (Newcombe et al., 2011)
- *ElasticFusion: Dense SLAM Without A Pose Graph* (Whelan et al., 2015)

---

## Summary

**What We Learned:**
1. ✅ RGB-D cameras provide **3D perception** (vs 2D lidar)
2. ✅ Point clouds encode **geometry + color**
3. ✅ Height-based analysis detects **tabletops**, **overhangs**
4. ✅ Camera intrinsics enable **2D ↔ 3D conversion**
5. ✅ ROS2 PointCloud2 enables **visualization** and **processing**

**Key Takeaway:**
> Depth cameras unlock 3D understanding, enabling robots to navigate complex environments with vertical obstacles that 2D sensors cannot perceive.

**Next Steps:**
- Level 7: Sensor Fusion (Lidar + Camera + IMU)
- Advanced: Deep Learning for 3D Object Detection
- Advanced: RGB-D SLAM for dense 3D mapping

---

**End of Documentation**
