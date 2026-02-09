# 🌐 062 - 3D Mapping

> **Level 6.2 - Point Cloud Mapping | Rich 3D Environment Reconstruction**

---

## 📌 Overview

The **3D Mapping Controller** converts RGB-D camera data into **rich 3D point clouds** for advanced perception tasks. Unlike traditional 2D LIDAR maps, this creates detailed 3D representations enabling object recognition, navigation planning, and scene understanding—your robot now understands full 3D environments!

### ✨ Key Features

- 📹 RGB-D image fusion (color + depth)
- ☁️ Real-time point cloud generation
- 🎨 Colored point cloud (XYZRGB format)
- 🔄 Pinhole camera model implementation
- 📊 3D visualization support
- 🧭 Robot odometry tracking
- 📡 ROS2 PointCloud2 publishing
- 🎮 Manual + Auto spin modes
- 💾 Point cloud structure optimization

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `062_3d_mapping.py` | 3D mapping controller |
| `062_3d_map.wbt` | Webots world file |
| `062_3d_mapping_doc.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/062_3d_map.wbt
```

### Step 2️⃣: Run 3D Mapper

```bash
ros2 run ce_webots 062_3d_mapping
```

### Manual Control

```
⌨️  CONTROLS:
  W / ↑ - Forward
  S / ↓ - Backward
  A / ← - Turn Left
  D / → - Turn Right
  M - Toggle Auto Spin
  Q / ESC - Quit

🎯 Watch as the robot rotates and the point cloud builds!
```

---

## 🔧 How It Works

### 1. Initialize RGB-D System

```python
# RGB camera
self.camera_rgb = self.robot.getDevice('camera_rgb')
self.camera_rgb.enable(self.timestep)

# Depth camera
self.camera_depth = self.robot.getDevice('camera_depth')
self.camera_depth.enable(self.timestep)

# Specifications
self.WIDTH = 128
self.HEIGHT = 128
self.FOV = 1.3  # radians
self.RANGE_MIN = 0.1  # meters
self.RANGE_MAX = 3.0  # meters
```

### 2. Camera Intrinsic Calculation

```python
def setup_camera_intrinsics(self):
    """Calculate focal length and principal point"""
    
    # Focal length from FOV
    f_x = f_y = self.WIDTH / (2 * math.tan(self.FOV / 2))
    
    # Principal point at image center
    c_x = self.WIDTH / 2
    c_y = self.HEIGHT / 2
    
    self.K = {
        'fx': f_x,
        'fy': f_y,
        'cx': c_x,
        'cy': c_y
    }
```

### 3. Point Cloud Generation

```python
def generate_pointcloud(self):
    """Convert RGB-D to colored 3D point cloud"""
    
    # Get RGB and depth images
    rgb_data = self.camera_rgb.getImage()
    depth_data = self.camera_depth.getRangeImage()
    
    # Convert to NumPy arrays
    rgb = np.frombuffer(rgb_data, np.uint8)
    rgb = rgb.reshape((self.HEIGHT, self.WIDTH, 4))[:, :, :3]  # BGRA→BGR
    
    depth = np.array(depth_data)
    depth = depth.reshape((self.HEIGHT, self.WIDTH))
    
    # Filter valid depth range
    valid_mask = (depth >= self.RANGE_MIN) & (depth <= self.RANGE_MAX)
    
    # Create coordinate grids
    u, v = np.meshgrid(
        np.arange(self.WIDTH),
        np.arange(self.HEIGHT)
    )
    
    # Backproject to 3D using pinhole camera model
    X = (u - self.K['cx']) * depth / self.K['fx']
    Y = (v - self.K['cy']) * depth / self.K['fy']
    Z = depth
    
    # Extract RGB
    R = rgb[:, :, 2]  # Correct BGR→RGB
    G = rgb[:, :, 1]
    B = rgb[:, :, 0]
    
    # Pack into PointCloud2 structure
    points_array = np.zeros(
        (self.HEIGHT * self.WIDTH,),
        dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'f4')]
    )
    
    points_array['x'] = X.flatten()
    points_array['y'] = Y.flatten()
    points_array['z'] = Z.flatten()
    
    # Pack RGB into single float (for ROS2 compatibility)
    rgb_packed = (R.flatten() << 16) | (G.flatten() << 8) | B.flatten()
    points_array['rgb'] = rgb_packed.astype('f4')
    
    return points_array, valid_mask
```

### 4. Publish Point Cloud

```python
def publish_pointcloud(self, points):
    """Publish PointCloud2 message to ROS2"""
    
    # Create PointCloud2 message
    header = Header()
    header.stamp = self.get_clock().now().to_msg()
    header.frame_id = 'camera_rgb_optical_frame'
    
    # PointCloud2 fields
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    
    cloud = PointCloud2(
        header=header,
        height=1,  # Unorganized
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=16,  # 4 × 4 bytes
        row_step=len(points) * 16,
        data=points.tobytes(),
        is_dense=True
    )
    
    self.pointcloud_publisher.publish(cloud)
```

---

## 📊 Point Cloud Format

```
PointCloud2 Structure (Unorganized):
  
  Each point:  X (f4) | Y (f4) | Z (f4) | RGB (f4)
               4 bytes   4 bytes   4 bytes   4 bytes
               ─────────────────────────────────────
               total: 16 bytes per point
  
  For 128×128 image:
    307,200 points × 16 bytes = 4.9 MB
```

---

## 🔄 Odometry Integration

```python
def update_robot_pose(self):
    """Track robot position for odometry"""
    
    # Read encoders
    left_pos = self.left_encoder.getValue()
    right_pos = self.right_encoder.getValue()
    
    # Calculate distance
    left_dist = (left_pos - self.prev_left) * self.WHEEL_RADIUS
    right_dist = (right_pos - self.prev_right) * self.WHEEL_RADIUS
    delta_s = (left_dist + right_dist) / 2
    
    # Update position
    self.x += delta_s * math.cos(self.theta)
    self.y += delta_s * math.sin(self.theta)
    
    # Publish odometry
    self.publish_odometry()
```

---

## 📚 Sensor Knowledge: Point Cloud & 3D Mapping

### 🎯 How It Works

**Point Cloud** is a collection of 3D points (X, Y, Z) representing environment surfaces:

```
Point Cloud Generation:

Step 1: Capture RGB-D Image
   RGB Image: (640×480) color per pixel
   Depth Map: (640×480) distance per pixel

Step 2: Backproject Each Pixel
   For pixel (x, y) with depth z:
   X = (x - cx) * z / fx
   Y = (y - cy) * z / fy
   Z = z
   
   Result: One 3D point (X, Y, Z)

Step 3: Accumulate Over Time
   Frame 1: 307,200 points
   Frame 2: 307,200 points + robot moved
   Frame 3: 307,200 points + robot moved
   ...
   
   Total: Millions of points = 3D map!

Point Cloud Structure:
  points_cloud[i] = (x, y, z, r, g, b)
  
  Example:
  ├─ Point 0: (-0.5,  0.2, 1.0, 255, 0, 0)  # Red
  ├─ Point 1: (-0.4,  0.3, 1.1, 255, 0, 0)  # Red
  ├─ Point 2: ( 0.0,  0.0, 0.8,   0, 255, 0)  # Green
  └─ Point N: ( 5.0,  3.0, 2.5,   0, 0, 255)  # Blue
```

### 📊 Specifications

| คุณสมบัติ | ค่า | หมายเหตุ |
|---------|-----|--------|
| **Points per Frame** | 307,200 | 640×480 RGB-D |
| **Capture Rate** | ~30 Hz | 33ms per frame |
| **Accumulation** | Real-time | As robot moves |
| **Max Points** | Millions | Depends on memory |
| **Color per Point** | RGB 8-bit | (R, G, B) 0-255 |
| **Coordinates** | 32-bit float | (X, Y, Z) meters |
| **Memory per Point** | ~16 bytes | XYZ (12) + RGB (3) + padding |

### 💡 Usage Tips

**✅ ทำได้:**
- Complete 3D environment mapping
- Object localization in 3D space
- Voxel grid representation
- Surface reconstruction
- Obstacle avoidance planning

**❌ ที่ควรหลีกเลี่ยง:**
- Keep ALL points (memory explosion)
- No filtering/downsampling
- Processing without GPU
- Ignoring coordinate transformation
- Not handling outliers

### ⚠️ Limitations

```
1. Memory Explosion
   
   Point Count Growth:
   ├─ 1 frame: 307K points = ~5 MB
   ├─ 10 frames: 3.07M points = ~50 MB
   ├─ 100 frames: 30.7M points = ~500 MB
   ├─ 1000 frames: 307M points = ~5 GB ⚠️
   
   Solutions:
   ├─ Voxel grid decimation
   ├─ Spatial downsampling
   ├─ Sliding window (keep recent)
   └─ GPU point cloud processing

2. Computational Load
   ├─ ICP (Iterative Closest Point) is slow
   ├─ 100M points = seconds to process
   └─→ Use hierarchical approaches

3. Outliers & Noise
   ├─ Sensor noise = spurious points
   ├─ Dynamic objects = transients
   ├─ Shadows = empty regions
   └─→ Statistical outlier removal

4. Coordinate Transformations
   ├─ Each frame: different camera pose
   ├─ Must transform to world coordinates
   ├─ Errors accumulate
   └─→ Use SLAM for correction

5. Occlusions
   ├─ Blocked views = missing regions
   ├─ Can't see inside objects
   └─→ Multiple viewpoints needed
```

### 🔧 Point Cloud Processing

```python
import numpy as np
from scipy.spatial import KDTree

class PointCloud3D:
    
    def __init__(self, points_xyz, colors_rgb=None):
        """Initialize point cloud
        
        Args:
            points_xyz: (N, 3) array of 3D coordinates
            colors_rgb: (N, 3) array of RGB values (0-255)
        """
        self.points = np.array(points_xyz, dtype=np.float32)
        self.colors = np.array(colors_rgb, dtype=np.uint8) if colors_rgb is not None else None
        self._kdtree = None
    
    def build_kdtree(self):
        """Build KD-tree for fast nearest neighbor search"""
        self._kdtree = KDTree(self.points)
    
    def remove_outliers(self, k=10, std_ratio=1.0):
        """Statistical outlier removal (covariance analysis)"""
        
        if self._kdtree is None:
            self.build_kdtree()
        
        # Find k nearest neighbors for each point
        distances, indices = self._kdtree.query(
            self.points, k=k+1
        )
        
        # Compute mean and std of distances
        mean_dist = np.mean(distances, axis=1)
        std_dist = np.std(distances, axis=1)
        threshold = np.mean(mean_dist) + std_ratio * np.std(mean_dist)
        
        # Keep points below threshold
        mask = mean_dist < threshold
        
        self.points = self.points[mask]
        if self.colors is not None:
            self.colors = self.colors[mask]
        
        self._kdtree = None  # Rebuild needed
        
        return np.sum(mask)  # Points removed
    
    def downsample_voxel(self, voxel_size=0.01):
        """Voxel grid downsampling"""
        
        # Compute voxel indices
        voxel_indices = np.floor(
            self.points / voxel_size
        ).astype(np.int32)
        
        # Create lookup table
        voxel_dict = {}
        for i, idx in enumerate(voxel_indices):
            key = tuple(idx)
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(i)
        
        # Keep one point per voxel (centroid)
        new_points = []
        new_colors = []
        
        for indices_in_voxel in voxel_dict.values():
            # Compute voxel centroid
            centroid = np.mean(
                self.points[indices_in_voxel], axis=0
            )
            new_points.append(centroid)
            
            # Use color from first point
            if self.colors is not None:
                new_colors.append(self.colors[indices_in_voxel[0]])
        
        self.points = np.array(new_points, dtype=np.float32)
        if self.colors is not None:
            self.colors = np.array(new_colors, dtype=np.uint8)
        
        self._kdtree = None
        
        return len(new_points)  # Remaining points

    def transform(self, rotation_matrix, translation_vector):
        """Apply rigid transformation (rotation + translation)"""
        
        # (X', Y', Z') = R * (X, Y, Z) + T
        self.points = (rotation_matrix @ self.points.T).T + translation_vector

    def estimate_normals(self, k=10):
        """Estimate surface normals (tangent plane)"""
        
        if self._kdtree is None:
            self.build_kdtree()
        
        # Find k nearest neighbors
        distances, indices = self._kdtree.query(
            self.points, k=k
        )
        
        normals = []
        
        for i, neighbor_indices in enumerate(indices):
            # Get neighbor points
            neighbors = self.points[neighbor_indices]
            
            # Compute covariance matrix
            centered = neighbors - neighbors[0]
            cov = centered.T @ centered
            
            # Smallest eigenvector = normal
            eigenvalues, eigenvectors = np.linalg.eigh(cov)
            normal = eigenvectors[:, 0]  # Smallest eigenvalue
            
            normals.append(normal)
        
        return np.array(normals)
    
    def crop(self, x_range, y_range, z_range):
        """Crop point cloud to region of interest"""
        
        mask = (
            (self.points[:, 0] >= x_range[0]) & 
            (self.points[:, 0] <= x_range[1]) &
            (self.points[:, 1] >= y_range[0]) & 
            (self.points[:, 1] <= y_range[1]) &
            (self.points[:, 2] >= z_range[0]) & 
            (self.points[:, 2] <= z_range[1])
        )
        
        self.points = self.points[mask]
        if self.colors is not None:
            self.colors = self.colors[mask]
        
        return np.sum(mask)  # Remaining points
```

### 📊 3D Mapping Pipeline

```
RGB-D Stream
    ↓
Estimate Camera Pose (SLAM)
    ↓
Transform Points to World Frame
    ↓
Accumulate Points (global cloud)
    ↓
Downsample (voxel grid)
    ↓
Remove Outliers
    ↓
Loop Closure Detection
    ↓
Map Optimization
    ↓
Surface Reconstruction (Poisson)
    ↓
Export (PLY/PCD format)
    ↓
Visualization/Navigation
```

### 📊 Voxel Grid Representation

```
Alternative: Occupancy Voxel Grid

Traditional Point Cloud:
└─ Millions of points
   ├─ High precision
   ├─ High memory
   └─ Slow processing

Voxel Grid (3D pixels):
├─ Fixed grid cells
├─ Each cell: occupied/free
├─ Fast ray casting
├─ Memory efficient
└─ Example: 10×10×10 @ 0.1m = complete 1m³ room

Voxel States:
  0: Unknown (not observed)
  1: Free (observed empty)
  2: Occupied (observed obstacle)

Benefits:
✓ Fast collision checking
✓ Easy navigation planning
✓ Bounded memory
✓ GPU acceleration
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ Point cloud generation from RGB-D
- ✅ Pinhole camera model application
- ✅ Color-depth image fusion
- ✅ PointCloud2 message format
- ✅ Efficient point packing
- ✅ Odometry integration
- ✅ ROS2 TF broadcasting
- ✅ 3D visualization in RViz2

---

## 📐 Pinhole Camera Model Math

```
2D Image Coordinates (u, v) → 3D World (X, Y, Z)

Camera Frame (at focal point):
  X = (u - cx) × Z / fx
  Y = (v - cy) × Z / fy
  Z = depth (from RangeFinder)

Where:
  fx, fy = Focal lengths (pixels)
  cx, cy = Principal point (pixels)
  u, v = Image coordinates (pixels)
  Z = Depth value (meters)
```

---

## 📝 Customization

### Lower Resolution (Faster)

```python
self.WIDTH = 64
self.HEIGHT = 64
# Results in 16× fewer points
```

### Increase Range

```python
self.RANGE_MAX = 5.0  # Detect further objects
```

### Auto Spin Configuration

```python
AUTO_SPIN_SPEED = 2.0  # rad/s
AUTO_SPIN_PERIOD = 6.28  # Full rotation (~6 seconds)
```

### Filter by Color

```python
# Detect only red objects
red_mask = (rgb[:,:,2] > 150) & (rgb[:,:,0] < 50)
valid_mask &= red_mask
```

---

## 📚 Related Resources

- 📖 [Point Cloud Library](https://pointclouds.org/)
- 🔗 [Pinhole Camera](https://en.wikipedia.org/wiki/Pinhole_camera_model)
- 📐 [3D Geometry](https://en.wikipedia.org/wiki/3D_computer_graphics)
- 🤖 [ROS 2 PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)
- 📺 [RViz2 Point Cloud Visualization](https://github.com/ros2/rviz)
- 👀 [061 Depth Camera](../061_depth_camera_controller/)
- 👀 [052 SLAM](../052_slam_controller/)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **No point cloud** | Verify cameras enabled; check publisher connection |
| **Distorted cloud** | Recalibrate intrinsics; check FOV setting |
| **Performance slow** | Reduce resolution; downsample points |
| **Memory issues** | Process fewer points; stream instead of store |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
