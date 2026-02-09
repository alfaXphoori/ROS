# 📹 061 - Depth Camera Controller

> **Level 6.1 - RGB-D Sensors | 3D Point Cloud and Advanced Perception**

---

## 📌 Overview

The **Depth Camera (RGB-D) Controller** demonstrates **3D vision**. Unlike 2D LIDAR that sees only a horizontal line, depth cameras capture full 3D structures with color information—enabling detection of obstacles at different heights like tables, shelves, and overhangs!

### ✨ Key Features

- 📷 RGB Camera for color images (640x480)
- 📏 Depth Camera for distance per pixel
- 🌈 Point cloud generation (XYZRGB)
- 🎯 3D obstacle detection at multiple heights
- 📊 Pinhole camera model implementation
- 🔄 Real-time 3D-aware navigation
- 📡 ROS2 PointCloud2 publishing
- 🎨 Real-time 3D visualization

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `061_depth_camera_controller.py` | Depth perception controller |
| `061_depth_camera.wbt` | Webots world file |
| `061_depth_camera_doc.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Start Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/061_depth_camera.wbt
```

### Step 2️⃣: Run Depth Camera Controller

```bash
ros2 run ce_webots 061_depth_camera_controller
```

---

## 🔍 Lidar vs Depth Camera

### The Table Problem 🪑

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

### Comparison Table

| Feature | 2D LIDAR | Depth Camera (RGB-D) |
|---------|----------|---------------------|
| **Perception** | Single horizontal plane | Full 3D field of view |
| **Data Type** | 360° distance array | 3D point cloud (X, Y, Z) |
| **Table Detection** | ❌ May miss tabletops | ✅ Sees complete 3D shape |
| **Object Shape** | ❌ Cannot determine | ✅ Can reconstruct |
| **Color Info** | ❌ No color | ✅ RGB per point |
| **Height Awareness** | ❌ Fixed height | ✅ Multiple height levels |

---

## 🔧 How It Works

### 1. Initialize RGB-D Cameras

```python
# RGB camera
self.camera_rgb = self.robot.getDevice('camera_rgb')
self.camera_rgb.enable(self.timestep)

# Depth camera (RangeFinder in Webots)
self.camera_depth = self.robot.getDevice('camera_depth')
self.camera_depth.enable(self.timestep)

# Specifications
self.WIDTH = 640
self.HEIGHT = 480
self.FOV = 1.3  # radians (≈ 74.5°)
```

### 2. Pinhole Camera Model

```python
def calculate_camera_intrinsics(self):
    """Compute camera intrinsic parameters"""
    
    # Field of View conversion
    focal_length = self.WIDTH / (2 * math.tan(self.FOV / 2))
    
    # Principal point (image center)
    cx = self.WIDTH / 2
    cy = self.HEIGHT / 2
    
    return {
        'fx': focal_length,  # Focal length X
        'fy': focal_length,  # Focal length Y
        'cx': cx,            # Principal point X
        'cy': cy,            # Principal point Y
    }
```

### 3. Point Cloud Generation

```python
def generate_pointcloud(self, rgb_image, depth_image):
    """Convert RGB + Depth to 3D colored point cloud"""
    
    # Get intrinsics
    fx = self.intrinsics['fx']
    fy = self.intrinsics['fy']
    cx = self.intrinsics['cx']
    cy = self.intrinsics['cy']
    
    # Create coordinate meshgrid
    u_grid, v_grid = np.meshgrid(
        np.arange(self.WIDTH),
        np.arange(self.HEIGHT)
    )
    
    # Reshape depth image
    depth = depth_image.reshape((self.HEIGHT, self.WIDTH))
    
    # Backproject to 3D (pinhole camera model)
    X = (u_grid - cx) * depth / fx
    Y = (v_grid - cy) * depth / fy
    Z = depth
    
    # Extract RGB (convert BGRA to RGB)
    B = rgb_image[:, :, 0]
    G = rgb_image[:, :, 1]
    R = rgb_image[:, :, 2]
    
    # Pack into PointCloud2 structure
    points = np.zeros((self.HEIGHT * self.WIDTH,),
                     dtype=[('x', 'f4'), ('y', 'f4'), 
                           ('z', 'f4'), ('rgb', 'u4')])
    
    points['x'] = X.flatten()
    points['y'] = Y.flatten()
    points['z'] = Z.flatten()
    
    # Pack RGB into single 32-bit integer
    points['rgb'] = (R.flatten() << 16 | 
                    G.flatten() << 8 | 
                    B.flatten())
    
    return points
```

### 4. 3D Obstacle Analysis

```python
def analyze_3d_obstacles(self, pointcloud):
    """Detect obstacles at different heights"""
    
    # Divide into height zones
    ground_level = pointcloud[pointcloud['z'] < 0.1]      # Floor
    leg_level = pointcloud[(0.1 < pointcloud['z']) & 
                          (pointcloud['z'] < 0.5)]  # Table legs
    table_level = pointcloud[(0.5 < pointcloud['z']) & 
                            (pointcloud['z'] < 1.0)]  # Table tops
    above_table = pointcloud[pointcloud['z'] > 1.0]      # Overhangs
    
    return {
        'ground': len(ground_level),
        'legs': len(leg_level),
        'table': len(table_level),
        'above': len(above_table),
    }
```

---

## 📊 Point Cloud Structure

```
PointCloud2 Message:
  Header:
    frame_id: 'camera_rgb_optical_frame'
    timestamp: <current time>
  
  Point Format (N points):
    ┌─────────────────┐
    │ Point 0         │
    │  x: 0.123 m     │
    │  y: -0.045 m    │
    │  z: 0.895 m     │
    │  rgb: 0xFF0000  │  ← Red (R=255, G=0, B=0)
    ├─────────────────┤
    │ Point 1         │
    │  x: 0.124 m     │
    │  y: -0.044 m    │
    │  z: 0.896 m     │
    │  rgb: 0xFF0001  │
    └─────────────────┘
    ... (307,200 points total for 640×480)
```

---

## 📚 Sensor Knowledge: Depth Camera (RGB-D)

### 🎯 How It Works

**RGB-D Camera** sends RGB (color) + Depth (distance) per point:

```
RGB-D Sensor Output:
┌───────────────────┬──────────────┐
│  RGB Channel      │ Depth Channel │
├───────────────────┼──────────────┤
│ B, G, R: 0-255    │ Z: 0.0-6.0m   │
│ (Color)           │ (Distance)    │
└───────────────────┴──────────────┘

Data Structure:
  image_rgb[y,x] = (B, G, R)        # Color
  image_depth[y,x] = distance (m)   # Depth

3D Point Cloud Generation:
  
  For each pixel (x, y):
    1. Read RGB color → (B, G, R)
    2. Read depth → z_distance
    3. Backproject to 3D:
       X = (x - cx) * z / fx
       Y = (y - cy) * z / fy
       Z = z
    4. Store point: (X, Y, Z, R, G, B)

Result: 640×480 = 307,200 colored 3D points!
```

### 📊 Specifications

| คุณสมบัติ | ค่า | หมายเหตุ |
|---------|-----|--------|
| **Resolution** | 640×480 | RGB + Depth |
| **Depth Range** | 0.1-6.0 m | Typical range |
| **Depth Accuracy** | ±1-2% of range | At 1m: ±1-2cm |
| **RGB Format** | BGRA | 4 bytes/pixel |
| **Depth Format** | Float32 | Per pixel |
| **FPS** | ~30 Hz | 33 ms/frame |
| **Total Data** | ~1.2 MB/frame | RGB + Depth |

### 💡 Usage Tips

**✅ ทำได้:**
- 3D point cloud generation
- Object recognition + grasping
- Collision detection
- Plane fitting (tables, walls)
- Semantic segmentation

**❌ ที่ควรหลีกเลี่ยง:**
- Trust all depth readings (filter noise)
- Process full resolution (downsample)
- Ignoring camera calibration
- Using depth without RGB context
- Not handling shadows

### ⚠️ Limitations

```
1. Depth Measurement Errors
   
   IR Structured Light Issues:
   ├─ Shadow regions = no data
   ├─ Transparent surfaces = fails
   ├─ Shiny surfaces = reflections
   ├─ Sunlight interference = noise
   └─→ Solutions:
       • Temporal filtering
       • Morphological operations
       • Multi-frame averaging

2. Limited Range
   ├─ 0.1m minimum (too close)
   ├─ 6.0m maximum (far objects)
   └─→ For far objects: use LIDAR

3. Computational Load
   ├─ 307,200 pixels × 30 Hz
   ├─ Point cloud processing heavy
   └─→ Downsample or GPU acceleration

4. Camera Calibration Dependency
   ├─ Must know: fx, fy, cx, cy
   ├─ Incorrect calib = wrong 3D points
   └─→ Calibrate with checkerboard!

5. Ambient Light Sensitivity
   ├─ IR emitter competes with sun
   ├─ Outdoor use limited
   └─→ Works best indoors
```

### 🔧 RGB-D Point Cloud Generation

```python
import numpy as np
import cv2

class RGBDProcessor:
    
    def __init__(self, focal_x=525, focal_y=525, 
                 center_x=320, center_y=240):
        """Camera intrinsics (typically from calibration)"""
        self.fx = focal_x   # Focal length X
        self.fy = focal_y   # Focal length Y
        self.cx = center_x  # Principal point X
        self.cy = center_y  # Principal point Y
    
    def depth_to_3d(self, x, y, depth):
        """Backproject single pixel to 3D"""
        
        # Pinhole camera model
        X = (x - self.cx) * depth / self.fx
        Y = (y - self.cy) * depth / self.fy
        Z = depth
        
        return np.array([X, Y, Z])
    
    def rgbd_to_pointcloud(self, image_rgb, image_depth):
        """Convert RGB-D image to 3D point cloud"""
        
        height, width = image_depth.shape
        points_xyz = []
        colors_rgb = []
        
        for y in range(height):
            for x in range(width):
                depth = image_depth[y, x]
                
                # Skip invalid depth
                if depth <= 0.0 or depth > 6.0:
                    continue
                
                # Backproject to 3D
                point_3d = self.depth_to_3d(x, y, depth)
                points_xyz.append(point_3d)
                
                # Get color
                b, g, r = image_rgb[y, x, :3]
                colors_rgb.append([r, g, b])
        
        return np.array(points_xyz), np.array(colors_rgb)
    
    def filter_depth(self, depth_image, method='median'):
        """Clean depth data (remove noise)"""
        
        if method == 'median':
            # Median filter (good for noise)
            filtered = cv2.medianBlur(
                (depth_image * 1000).astype(np.uint16), 
                ksize=5
            )
            return filtered.astype(np.float32) / 1000.0
        
        elif method == 'bilateral':
            # Bilateral filter (smooth while preserving edges)
            filtered = cv2.bilateralFilter(
                (depth_image * 1000).astype(np.uint16),
                d=5, sigmaColor=50, sigmaSpace=50
            )
            return filtered.astype(np.float32) / 1000.0
        
        elif method == 'morphological':
            # Close holes; remove noise
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (5, 5)
            )
            binary = (depth_image > 0).astype(np.uint8)
            binary = cv2.morphologyEx(
                binary, cv2.MORPH_CLOSE, kernel
            )
            filtered = depth_image.copy()
            filtered[binary == 0] = 0  # Keep zeros
            return filtered

    def detect_planes(self, points_xyz, threshold=0.05):
        """Detect large planar surfaces (tables, walls)"""
        
        from sklearn.linear_model import RANSACRegressor
        
        if len(points_xyz) < 10:
            return None
        
        # RANSAC plane fitting
        # Plane equation: aX + bY + cZ = d
        # Fit: Z = aX + bY + c
        
        X = points_xyz[:, :2]  # Use X, Y
        Z = points_xyz[:, 2]    # Predict Z
        
        ransac = RANSACRegressor(random_state=42)
        ransac.fit(X, Z)
        
        # Extract plane coefficients
        a, b = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_
        
        # Normal vector: (a, b, -1) normalized
        normal = np.array([a, b, -1])
        normal = normal / np.linalg.norm(normal)
        
        return {
            'normal': normal,
            'distance': c,
            'inliers': ransac.inlier_mask_,
        }

    def segment_object(self, points_xyz, plane):
        """Remove table/ground; isolate objects"""
        
        # Filter points far from plane
        threshold = 0.05  # 5cm above plane
        
        distances = []
        for point in points_xyz:
            # Distance = |ax + by + cz - d|
            dist = abs(np.dot(plane['normal'], point) 
                      - plane['distance'])
            distances.append(dist)
        
        distances = np.array(distances)
        object_points = points_xyz[distances > threshold]
        
        return object_points
```

### 📊 Camera Calibration Parameters

```
Pinhole Camera Model:

Camera Matrix (K):
┌         ┐
│ fx  0  cx │
│  0  fy cy │
│  0  0   1 │
└         ┘

Where:
  fx, fy = focal length (pixels)
  cx, cy = principal point (image center)

Example Typical Values:
  fx = 525 px   (depth camera)
  fy = 525 px
  cx = 320 px   (image width / 2)
  cy = 240 px   (image height / 2)

Backprojection Formula:
  X = (x - cx) * Z / fx
  Y = (y - cy) * Z / fy
  Z = Z (from sensor)

Example (pixel [640, 480] at 1m):
  X = (640 - 320) * 1.0 / 525 = 0.61 m
  Y = (480 - 240) * 1.0 / 525 = 0.46 m
  Z = 1.0 m
  
  Point = (0.61, 0.46, 1.0) meters
```

### 📊 RGB-D Processing Pipeline

```
Raw RGB-D Input
        ↓
Temporal Filtering (multi-frame average)
        ↓
Spatial Filtering (median/bilateral)
        ↓
Backproject to 3D (point cloud)
        ↓
Plane Fitting (find table/floor)
        ↓
Object Segmentation (above plane)
        ↓
Clustering (identify objects)
        ↓
Feature Extraction (size, shape)
        ↓
Object Recognition (ML model)
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ RGB-D sensor principles
- ✅ Pinhole camera model
- ✅ Camera intrinsic parameters
- ✅ Depth image backprojection
- ✅ Point cloud generation
- ✅ 3D coordinate systems
- ✅ Multi-level obstacle detection
- ✅ ROS2 PointCloud2 format

---

## 📐 Camera Intrinsics Explanation

### Focal Length (f)

```
f = width / (2 × tan(FOV/2))

Example:
  Width = 640 pixels
  FOV = 1.3 radians (74.5°)
  f = 640 / (2 × tan(1.3/2))
  f ≈ 517 pixels
```

### Principal Point (cx, cy)

```
The principal point is the center of the image where the
optical axis intersects the image plane.

For most cameras:
  cx = width / 2
  cy = height / 2

Example:
  Width = 640, Height = 480
  cx = 320, cy = 240
```

---

## 📝 Customization

### Filter by Height Range

```python
# Only detect obstacles above table level
high_obstacles = pointcloud[pointcloud['z'] > 0.5]
```

### Adjust FOV

```python
# Wider field of view
self.FOV = 1.5  # ~86°

# Narrower (telephoto)
self.FOV = 0.8  # ~46°
```

### Color Filtering

```python
# Detect only red objects
red_points = pointcloud[(pointcloud['rgb'] >> 16) > 200]
```

---

## 📚 Related Resources

- 📖 [Webots RangeFinder](https://cyberbotics.com/doc/reference/rangefinder)
- 🔗 [Pinhole Camera Model](https://en.wikipedia.org/wiki/Pinhole_camera_model)
- 📐 [Camera Calibration](https://en.wikipedia.org/wiki/Camera_resectioning)
- 🌈 [Point Cloud Processing](https://pointclouds.org/)
- 🤖 [ROS 2 PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)
- 👀 [062 3D Mapping](../062_3d_mapping/) (Advanced point cloud)
- 👀 [041 Camera Controller](../041_camera_controller/) (RGB only)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **No depth data** | Verify RangeFinder enabled; check range |
| **Point cloud distorted** | Recalibrate intrinsics; check FOV |
| **Obstacles missed** | Adjust height thresholds; increase resolution |
| **Performance slow** | Reduce point cloud; downsample image |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
