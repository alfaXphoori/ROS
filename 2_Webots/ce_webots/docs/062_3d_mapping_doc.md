# Level 6.2: 3D Mapping with Point Cloud Generation

## Overview

**062_3d_mapping.py** demonstrates RGB-D based 3D mapping by converting depth camera data into colored point clouds. This controller combines depth sensing with RGB imagery to create rich 3D representations of the environment, suitable for advanced perception tasks like object recognition, navigation planning, and scene understanding.

**Key Concept**: Unlike 2D LIDAR that only captures a horizontal slice of the environment, RGB-D cameras capture full 3D structure with color information, enabling detection of obstacles at different heights (e.g., tables, overhangs, stairs).

---

## Learning Objectives

After completing this level, you will understand:
1. **Pinhole Camera Model**: How to convert 2D depth images into 3D point clouds
2. **RGB-D Fusion**: Combining depth and color data into XYZRGB point clouds
3. **Camera Intrinsics**: Using focal length and principal point for projection
4. **Point Cloud Publishing**: ROS2 PointCloud2 message structure
5. **3D Visualization**: Configuring RViz2 for point cloud display

---

## Hardware Components

### Sensors
- **camera_rgb** (Camera): 128x128 BGRA images at 1.3 radians FOV
- **camera_depth** (RangeFinder): 128x128 depth values (0.1m - 3.0m range)
- **lidar** (Lidar): 360-degree scanning for reference
- **imu** (InertialUnit): Orientation tracking (roll, pitch, yaw)
- **left_encoder** / **right_encoder**: Wheel rotation sensors

### Actuators
- **left_motor** / **right_motor**: Differential drive wheels

### Physical Parameters
```python
WHEEL_RADIUS = 0.08  # meters
AXLE_LENGTH = 0.24   # meters
MAX_SPEED = 10.0     # rad/s
```

---

## Features

### 1. Point Cloud Generation (`publish_pointcloud()`)

Converts depth + RGB data into 3D colored point cloud using **pinhole camera model**:

#### Algorithm Steps

**Step 1: Camera Intrinsic Parameters**
```python
width = 128, height = 128
fov = 1.3 radians (≈ 74.5°)
f_x = f_y = width / (2 * tan(fov/2))  # Focal length
c_x = width / 2  # Principal point X
c_y = height / 2 # Principal point Y
```

**Step 2: Depth Image Processing**
- Reshape depth array to (height, width)
- Filter valid range: 0.1m < depth < 3.0m
- Create coordinate meshgrid for all pixels

**Step 3: 3D Reprojection**
```
For each pixel (u, v) with depth Z:
  X = (u - c_x) * Z / f_x
  Y = (v - c_y) * Z / f_y
  Z = depth_value
```

This converts 2D image coordinates to 3D camera coordinates.

**Step 4: Color Extraction**
- Extract RGB values from BGRA image
- Pack into single 32-bit integer: `rgb = (R << 16) | (G << 8) | B`
- Store as float32 for PointCloud2 compatibility

**Step 5: Point Cloud Assembly**
- Create structured NumPy array with fields: `x`, `y`, `z`, `rgb`
- Serialize to bytes
- Publish as `sensor_msgs/PointCloud2`

#### Performance Optimization
- Uses NumPy vectorization (no Python loops)
- Boolean masking for filtering
- Efficient memory layout (structured arrays)

---

### 2. Manual Control

**Keyboard Controls**:
- **W** / **↑**: Move forward (5.0 rad/s both wheels)
- **S** / **↓**: Move backward (-5.0 rad/s both wheels)
- **A** / **←**: Turn left (differential: -2.5, +2.5)
- **D** / **→**: Turn right (differential: +2.5, -2.5)
- **M**: Toggle Auto Spin mode

---

### 3. Auto Spin Mode

When enabled (press **M**):
- Robot rotates in place: `left = -2.0`, `right = +2.0`
- Scans 360° environment for complete point cloud coverage
- Ideal for static scene reconstruction

**Use Case**: Position robot in center of room, enable auto spin, capture complete 3D model

---

### 4. Robot Visualization (`publish_robot_markers()`)

Publishes 3D markers to visualize robot body in RViz:

| Part | Type | Size (x,y,z) | Color | Position |
|------|------|--------------|-------|----------|
| Body | CUBE | 0.4×0.2×0.1 | Blue | (0,0,0) |
| Lidar | CYLINDER | 0.16×0.16×0.06 | Black | (0,0,0.12) |
| Camera | CUBE | 0.06×0.1×0.06 | Gray | (0.15,0,0.12) |

All markers published to `/robot_marker` topic.

---

## ROS2 Integration

### Published Topics

| Topic | Message Type | Frame | Rate | Description |
|-------|-------------|-------|------|-------------|
| `/camera/points` | PointCloud2 | camera_depth_link | ~31Hz | Colored 3D point cloud |
| `/camera/rgb/image_raw` | Image | - | ~31Hz | RGB image (unused) |
| `/camera/depth/image_raw` | Image | - | ~31Hz | Depth image (unused) |
| `/scan` | LaserScan | lidar_link | ~31Hz | 2D laser scan |
| `/odom` | Odometry | odom → base_link | ~31Hz | Robot odometry |
| `/robot_marker` | Marker | base_link | ~31Hz | Robot 3D model |

### TF Tree

```
map
 └─ odom (static identity)
     └─ base_link (x, y, theta from odometry)
         ├─ camera_depth_link (x=0.15, z=0.12, rotated -90° pitch)
         └─ lidar_link (z=0.12)
```

**Camera Rotation**:
- Quaternion: `(-0.5, 0.5, -0.5, 0.5)`
- Aligns camera Z-axis with robot X-axis (forward)
- Necessary for correct point cloud orientation

---

## Point Cloud Data Structure

### PointCloud2 Message Format

```python
msg.header.frame_id = "camera_depth_link"
msg.width = N  # Number of points
msg.height = 1  # Unorganized cloud

msg.fields = [
    PointField(name='x', offset=0, datatype=FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=FLOAT32, count=1)
]

msg.point_step = 16 bytes  # 4 fields × 4 bytes
msg.row_step = 16 * N
msg.is_dense = True  # No invalid points (already filtered)
```

### RGB Encoding

Color stored as **single float32** with bit packing:
```
Bits 24-31: Red (0-255)
Bits 16-23: Green (0-255)
Bits 8-15:  Blue (0-255)
Bits 0-7:   Unused (0)
```

**RViz automatically unpacks this encoding when displaying point clouds.**

---

## Odometry System

### Encoder-Based Position Tracking

```python
# Wheel distances
dl = (encoder_left_current - encoder_left_previous) * WHEEL_RADIUS
dr = (encoder_right_current - encoder_right_previous) * WHEEL_RADIUS

# Robot center movement
distance = (dl + dr) / 2.0
d_theta = (dr - dl) / AXLE_LENGTH

# Update pose
pose['x'] += distance * cos(theta)
pose['y'] += distance * sin(theta)
pose['theta'] += d_theta  # IMU overrides if available
```

### IMU Integration

If IMU yaw is valid (not NaN), it **overrides** encoder-calculated theta:
```python
rpy = self.imu.getRollPitchYaw()
if rpy and not math.isnan(rpy[2]):
    self.pose['theta'] = rpy[2]
```

**Benefit**: Eliminates angular drift from wheel slippage.

---

## Usage Instructions

### 1. Launch Simulation

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ce_webots_launch slam.launch.py world:=062_3d_map.wbt
```

### 2. Start Controller

```bash
# Terminal 2
source install/setup.bash
ros2 run ce_webots 062_3d_mapping
```

### 3. Configure RViz2

```bash
# Terminal 3
rviz2
```

**Required Configuration**:

1. **Fixed Frame**: Set to `map`
2. **Add PointCloud2 Display**:
   - Click "Add" → "By Topic" → `/camera/points`
   - Set Size: 0.01 - 0.03
   - Set Style: Points or Flat Squares
   - Color Transformer: RGB8 (to show colors)
3. **Add Marker Display**:
   - Topic: `/robot_marker`
   - Shows robot body in 3D
4. **Add TF Display** (Optional):
   - Shows coordinate frames

### 4. Test Point Cloud

**Manual Scan**:
1. Use **W/A/S/D** to drive robot around
2. Observe point cloud updating in RViz
3. Notice table tops, walls, objects with color

**360° Scan**:
1. Position robot in open area
2. Press **M** to enable Auto Spin
3. Let robot complete full rotation
4. Point cloud shows complete room

---

## Configuration Parameters

### Camera Parameters

```python
# Depth Camera
WIDTH = 128 pixels
HEIGHT = 128 pixels
FOV = 1.3 radians (≈ 74.5°)
MIN_RANGE = 0.1 meters
MAX_RANGE = 3.0 meters
UPDATE_RATE = 32 ms (≈ 31 Hz)
```

**Trade-offs**:
- **Higher resolution** (e.g., 256×256): More points, lower FPS, higher CPU
- **Lower resolution** (e.g., 64×64): Fewer points, higher FPS, less detail

### Control Parameters

```python
FORWARD_SPEED = 5.0   # rad/s (manual forward/back)
TURN_SPEED = 2.5      # rad/s (manual turn)
AUTO_SPIN_SPEED = 2.0 # rad/s (auto spin mode)
```

---

## Mathematical Foundations

### Pinhole Camera Model

The fundamental equation for 3D reprojection:

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = 
\frac{1}{Z} K \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
$$

Where $K$ is the camera intrinsic matrix:

$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

**Inverse (used in code)**:

$$
\begin{bmatrix} X \\ Y \\ Z \end{bmatrix} = 
Z \begin{bmatrix}
\frac{u - c_x}{f_x} \\
\frac{v - c_y}{f_y} \\
1
\end{bmatrix}
$$

### Focal Length Calculation

From field of view:

$$
f_x = f_y = \frac{w}{2 \tan(\text{FOV}/2)}
$$

For $w=128$, $\text{FOV}=1.3$:

$$
f_x = \frac{128}{2 \times \tan(0.65)} \approx 84.2 \text{ pixels}
$$

---

## Visualization Tips

### RViz Point Cloud Settings

**Best Quality**:
- Size: 0.02
- Style: Flat Squares
- Color Transformer: RGB8
- Decay Time: 0 (instant update)

**Performance Mode** (for large clouds):
- Size: 0.01
- Style: Points
- Subsample: 2-4

### Common Issues

**Problem**: Point cloud not visible  
**Solution**: 
- Check Fixed Frame = `map`
- Verify topic `/camera/points` is publishing
- Set Size > 0.01

**Problem**: Colors wrong  
**Solution**: Set Color Transformer to **RGB8** (not Intensity/Axis)

**Problem**: Point cloud rotated wrong  
**Solution**: Verify `camera_depth_link` TF has correct quaternion

**Problem**: Too many points (lag)  
**Solution**: Reduce camera resolution or increase MIN_RANGE

---

## Differences from Level 6.1 (Depth Camera)

| Feature | 061_depth_camera | 062_3d_mapping |
|---------|------------------|----------------|
| Primary Output | RGB/Depth Images | Point Cloud |
| Use Case | Human visualization | Robot perception |
| RViz Display | Image viewer | PointCloud2 |
| Data Format | 2D arrays | 3D XYZRGB points |
| Auto Mode | None | Spin scan |
| Markers | None | Robot body model |

**Level 6.1** focused on publishing raw sensor data.  
**Level 6.2** processes that data into actionable 3D maps.

---

## Advanced Topics

### Point Cloud Filtering

The code applies basic filtering:
```python
valid_mask = (depth > 0.1) & (depth < 3.0)
```

**Possible Enhancements**:
- Statistical outlier removal
- Voxel grid downsampling
- Plane segmentation (RANSAC)
- Pass-through filters (crop regions)

### Integration with SLAM

Point clouds can be used for:
1. **3D Mapping**: Build volumetric occupancy grids
2. **Loop Closure**: Match point clouds for drift correction
3. **Obstacle Detection**: Identify 3D obstacles (tables, overhangs)
4. **Object Recognition**: Match against 3D model database

---

## Troubleshooting

### Robot doesn't move in auto mode
- Verify auto_mode flag toggled: Check dashboard shows "🤖 AUTO SPIN"
- Check motor velocities printed in debug mode

### Point cloud looks distorted
- Verify camera FOV matches actual hardware (1.3 rad)
- Check focal length calculation
- Ensure depth range (0.1-3.0m) matches RangeFinder settings

### High CPU usage
- Reduce camera resolution in world file
- Increase timestep (slower simulation)
- Disable unused publishers (rgb_pub, depth_pub)

### TF tree broken
- Verify all transforms published every frame
- Check quaternion normalization (sum of squares = 1)
- Use `ros2 run tf2_tools view_frames` to debug

---

## Performance Notes

### Computational Complexity

**Point Cloud Generation**: $O(w \times h)$ where $w=128, h=128$
- Total pixels: 16,384
- After filtering (~70% valid): ~11,000 points
- Processing time: ~5-10ms (NumPy optimized)

**Bottleneck**: Serialization to PointCloud2 bytes

### Optimization Strategies

1. **Vectorization**: Use NumPy array operations (already done)
2. **Downsampling**: Reduce camera resolution
3. **Region of Interest**: Only process center of image
4. **Publishing Rate**: Publish at 10Hz instead of 31Hz

---

## Next Steps

After mastering Level 6.2, consider:

1. **Level 6.3**: 3D Navigation with point cloud obstacle avoidance
2. **Level 7**: Sensor Fusion (combine LIDAR + RGB-D)
3. **Level 8**: Object Recognition from point clouds
4. **Advanced**: SLAM with loop closure using point cloud matching

---

## Code Reference

### Key Functions

| Function | Purpose | Output |
|----------|---------|--------|
| `publish_pointcloud()` | Generate 3D map | PointCloud2 message |
| `update_odometry()` | Track position | pose dict |
| `publish_tf()` | Coordinate transforms | TF tree |
| `publish_robot_markers()` | Visualization | Marker messages |
| `print_dashboard()` | Status display | Terminal output |

### Dependencies

```python
import rclpy                    # ROS2 Python client
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np              # Fast array math
import math                     # Trigonometry
from controller import Robot    # Webots interface
```

---

## Summary

**062_3d_mapping.py** demonstrates:
✅ RGB-D data fusion into colored point clouds  
✅ Pinhole camera model for 3D reprojection  
✅ Efficient NumPy-based processing  
✅ ROS2 PointCloud2 publishing  
✅ Complete odometry and TF system  
✅ Auto spin mode for 360° scanning  
✅ 3D robot visualization in RViz  

**Key Takeaway**: Point clouds provide rich 3D+color data that enables advanced perception tasks impossible with 2D LIDAR alone (e.g., detecting table tops, overhanging obstacles, textured surfaces).

---

**Author**: AI Assistant  
**Version**: 1.0  
**Date**: February 2026  
**License**: Educational Use
