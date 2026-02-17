# 📷 041 - Camera Controller (Ball Chaser)

> **Level 4.1 - Visual Sensors | RGB Camera Object Tracking and Autonomous Chasing**

---

## 📌 Overview

RGB Camera Controller - Color Detection
========================================

Script demonstrating RGB Camera usage for:
- Capturing color images from robot camera
- Processing raw pixel data (BGRA format)
- Center pixel color detection
- Multi-color object detection
- Publishing camera images to ROS2

Features:
- Real-time color detection at center pixel
- Visual RGB channel bars
- Configurable color thresholds
- Keyboard control for navigation
- ROS2 image publishing

### ✨ Key Features

- 📷 RGB Camera integration (640x480 BGRA format)
- 🔍 Color-based object detection (Color Thresholding)
- 📊 Real-time ball position tracking
- 🎯 Autonomous ball chasing behavior
- 🎮 Manual + Auto modes
- 🖼️ Camera image visualization
- 📈 Centroid-based object localization
- 📡 ROS2 image_raw publishing

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `041_camera_controller.py` | Ball chaser controller script |
| `041_camera.wbt` | Webots world file |


---

## 🚀 How to Run This Lab

### Prerequisites

- ✅ Webots installed
- ✅ ROS 2 Jazzy installed and sourced
- ✅ Basic understanding of computer vision concepts
- ✅ Workspace built and sourced

### Running Steps

#### Terminal 1: Launch Webots Simulation

```bash
webots ~/ros2_ws/src/ce_webots/worlds/041_camera.wbt
```

**Environment features:**
- Arena with colored objects (red, green, blue balls)
- Robot with forward-facing camera
- Good lighting for color detection
- Open space for ball chasing

#### Terminal 2: Run Camera Controller

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Run the ball chaser
ros2 run ce_webots 041_camera_controller
```

**What to observe:**
- Real-time camera feed processing
- Ball detection status (detected/not detected)
- Ball position indicator (left/center/right)
- Autonomous chasing behavior in AUTO mode
- Manual control available with WASD keys

### Interactive Controls

- **M** - Toggle between AUTO and MANUAL modes
- **W** - Move forward (manual mode)
- **A** - Turn left (manual mode)
- **S** - Move backward (manual mode)
- **D** - Turn right (manual mode)
- **Q** - Quit

### Real-Time Dashboard

```
============================================================
   📷  CAMERA CONTROLLER - Ball Chaser
============================================================

📸 CAMERA INFO
   Resolution: 640x480
   Target: Red objects (BGR: [150, 0, 0]-[255, 100, 100])

🎯 DETECTION STATUS
   Status: ✅ RED BALL DETECTED!
   Position: 0.65 [━━━━━━━━━━━━━━━━━━━🔴━━━━━━━━━━]
   L ←                              → R

------------------------------------------------------------
🎮 MODE: AUTO
📊 STATUS: ➡️  Ball on right (pos: 0.65)
------------------------------------------------------------
⌨️  CONTROLS: WASD=Manual | M=Auto Mode | Q=Quit
============================================================
```

### Understanding the Display

- **Position Bar:** Shows where ball appears in image (0.0 = left, 0.5 = center, 1.0 = right)
- **AUTO Mode:** Robot automatically chases the ball
- **MANUAL Mode:** User controls robot with keyboard

### Autonomous Behavior

When ball is detected:
1. **Ball on left** (pos < 0.4): Turn left
2. **Ball centered** (0.4 ≤ pos ≤ 0.6): Move forward
3. **Ball on right** (pos > 0.6): Turn right
4. **Ball not detected**: Stop and search

### Monitoring Topics

```bash
# Terminal 3 (optional): View camera image
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/image_raw

# Monitor ball detection
ros2 topic echo /ball_detected

# Monitor ball position
ros2 topic echo /ball_position

# List all camera topics
ros2 topic list | grep camera
```

### Experiment Ideas

1. **Change Target Color:**
   - Modify BGR thresholds to track green or blue objects
   - Test with multiple colored objects

2. **Improve Detection:**
   - Add blob size filtering
   - Implement morphological operations (erosion/dilation)
   - Use HSV color space instead of BGR

3. **Advanced Behaviors:**
   - Stop at certain distance from ball
   - Circle around the ball
   - Track multiple objects simultaneously

### Troubleshooting

| Issue | Solution |
|-------|----------|
| **No ball detected** | Check color thresholds match object color in simulation |
| **Detection too sensitive** | Tighten BGR threshold ranges |
| **Robot doesn't chase** | Verify AUTO mode is enabled (press M) |
| **Camera image blank** | Check if camera is enabled and resolution is correct |
| **Position always 0.5** | No color match - adjust thresholds |

### Color Detection Tips

**BGR Format in Webots:**
- **Red Ball:** BGR [0, 0, 150-255]
- **Green Ball:** BGR [0, 150-255, 0]
- **Blue Ball:** BGR [150-255, 0, 0]

**Threshold Tuning:**
```python
# Tight threshold (precise but may miss)
min_color = [200, 0, 0]  # Pure blue
max_color = [255, 50, 50]

# Loose threshold (detects more but less accurate)
min_color = [150, 0, 0]
max_color = [255, 100, 100]
```

---

## 🔧 How It Works

### 1. Initialize Camera

```python
# Get camera device
self.camera = self.robot.getDevice('camera')
self.camera.enable(self.timestep)

# Camera specifications
self.WIDTH = 640
self.HEIGHT = 480
self.FORMAT = "BGRA"  # Blue, Green, Red, Alpha

# Define target color (red ball)
# Webots uses BGR format!
self.TARGET_COLOR_MIN = [150, 0, 0]      # Dark red
self.TARGET_COLOR_MAX = [255, 100, 100]  # Bright red
```

### 2. Capture and Process Image

```python
def get_camera_image(self):
    """Capture image from camera"""
    image_data = self.camera.getImage()
    
    # Convert to NumPy array for processing
    image = np.frombuffer(image_data, np.uint8)
    image = image.reshape((self.HEIGHT, self.WIDTH, 4))  # BGRA
    
    return image
```

### 3. Color Detection Algorithm

```python
def detect_red_ball(self, image):
    """Find red object in image"""
    # Extract BGR channels (ignore Alpha)
    bgr = image[:, :, :3]
    
    # Create mask for red color
    # Thresholding: min < pixel < max
    mask = (
        (bgr[:, :, 0] >= self.TARGET_COLOR_MIN[0]) & 
        (bgr[:, :, 0] <= self.TARGET_COLOR_MAX[0]) &  # B
        (bgr[:, :, 1] >= self.TARGET_COLOR_MIN[1]) & 
        (bgr[:, :, 1] <= self.TARGET_COLOR_MAX[1]) &  # G
        (bgr[:, :, 2] >= self.TARGET_COLOR_MIN[2]) & 
        (bgr[:, :, 2] <= self.TARGET_COLOR_MAX[2])    # R
    )
    
    # Count pixels
    pixel_count = np.sum(mask)
    
    if pixel_count > self.DETECTION_THRESHOLD:
        return True, mask
    else:
        return False, None
```

### 4. Calculate Ball Position

```python
def find_ball_centroid(self, mask):
    """Find center of ball using centroid"""
    # Get coordinates of red pixels
    y_coords, x_coords = np.where(mask)
    
    if len(x_coords) == 0:
        return None
    
    # Calculate centroid (center of mass)
    centroid_x = np.mean(x_coords)
    centroid_y = np.mean(y_coords)
    
    # Normalize to 0.0-1.0 range
    normalized_x = centroid_x / self.WIDTH
    normalized_y = centroid_y / self.HEIGHT
    
    return {
        'x': normalized_x,
        'y': normalized_y,
        'pixel_x': centroid_x,
        'pixel_y': centroid_y
    }
```

### 5. Chase Behavior

```python
def chase_ball(self, ball_pos):
    """Turn and move toward ball"""
    center = 0.5  # Image center
    error = ball_pos['x'] - center
    
    # Proportional control
    turn_power = error * self.TURN_GAIN
    
    # Set motor speeds
    left_speed = self.FORWARD_SPEED - turn_power
    right_speed = self.FORWARD_SPEED + turn_power
    
    # Clamp to valid range
    left_speed = np.clip(left_speed, -self.MAX_SPEED, self.MAX_SPEED)
    right_speed = np.clip(right_speed, -self.MAX_SPEED, self.MAX_SPEED)
    
    return left_speed, right_speed
```

---

## 📊 Image Processing Pipeline

```
Original Image (640x480 BGRA)
        ↓
Extract BGR Channels
        ↓
Apply Color Threshold
    [150,0,0] < BGR < [255,100,100]
        ↓
Binary Mask (Red = 255, Other = 0)
        ↓
Find Connected Components
        ↓
Calculate Centroid (Center of Mass)
        ↓
Normalize Position (0.0-1.0)
        ↓
Generate Control Commands
```

---

## 🎨 Color Detection Guide

### Working with Different Colors

**Red Ball (Current):**
```python
MIN = [150, 0, 0]      # Dark red
MAX = [255, 100, 100]  # Bright red
```

**Blue Ball:**
```python
MIN = [200, 0, 0]      # Dark blue (B channel high)
MAX = [255, 100, 50]
```

**Green Ball:**
```python
MIN = [0, 100, 0]      # Dark green (G channel high)
MAX = [100, 255, 100]
```

**Yellow Ball:**
```python
MIN = [0, 100, 100]    # Red + Green
MAX = [100, 255, 255]
```

### Finding Correct Color Range

1. Save a frame with the target object
2. Use color picker tool (Python PIL, OpenCV GUI)
3. Note the BGR values (remember: BGR not RGB!)
4. Set MIN to dark version, MAX to bright version

---

## 📚 Sensor Knowledge: RGB Camera

### 🎯 How It Works

**RGB Camera** sends BGRA (Blue-Green-Red-Alpha) image 640x480 pixels every frame:

```
Image Format: BGRA (8 bits each channel)
┌─────────────┬───────┐
│ Resolution  │640x480│
├─────────────┼───────┤
│ FPS         │ ~64   │
├─────────────┼───────┤
│ Format      │ BGRA  │
├─────────────┼───────┤
│ Bytes/pixel │  4    │
└─────────────┴───────┘

Color Detection Flow:
  1. Capture image (640x480)
  2. Filter by HSV range
  3. Find contours
  4. Calculate centroid (ball center)
  5. Control robot toward centroid
```

### 📊 Specifications

| Property | Value | Notes |
|---------|-------|-------|
| **Resolution** | 640x480 pixels | VGA standard |
| **Field of View** | ~60° | Typical FOV |
| **Format** | BGRA | 4 bytes/pixel |
| **Update Rate** | ~64 Hz | 15.6 ms/frame |
| **Focus** | Fixed (Infinity) | No focusing needed |
| **Color Depth** | 8-bit (0-255) | Per channel |
| **Data Rate** | 30.72 MB/s | Bandwidth needed |

### 💡 Usage Tips

**✅ Do:**
- Color filtering (HSV > RGB)
- Ball/object tracking
- Line detection
- Contour analysis
- Real-time control loop

**❌ Avoid:**
- Use Raw RGB (HSV is better)
- Process full resolution image
- Don't downsample before processing
- Don't do edge detection first

### ⚠️ Limitations

```
1. Color Space Issues
   └─ RGB vs BGR: Webots uses BGR!
      │
      └─→ Use HSV: Better for lighting changes
          H: 0-180 (Hue/color)
          S: 0-255 (Saturation/purity)
          V: 0-255 (Brightness)

2. Lighting Dependency
   • Shadows = color shift
   • Direct sun = washout
   • Solution: Use HSV range (not fixed RGB)

3. FOV Limitation
   • 60° FOV = limited detection
   • Ball must be in center area
   • Edges may miss objects

4. Processing Speed
   • 640x480 = 307,200 pixels
   • Takes time to process
   • Solution: Downsample or use ROI
   
5. No Depth Information
   • Can't measure distance to ball
   • Use camera + distance sensor for proximity
```

### 🔧 HSV Color Space Best Practices

```python
# HSV is BETTER than RGB for color detection!
#
# Why? Because:
#   H (Hue)        = Color (invariant to lighting)
#   S (Saturation) = Color purity
#   V (Value)      = Brightness (changes with lighting)

# Example: Red ball detection
def detect_red_with_hsv(image_bgra):
    """Detect red using HSV color space"""
    
    import cv2
    import numpy as np
    
    # 1. Convert BGRA → BGR
    bgr = cv2.cvtColor(image_bgra, cv2.COLOR_BGRA2BGR)
    
    # 2. Convert BGR → HSV
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    
    # 3. Define HSV range for RED
    #    Red wraps around: 0-10 and 170-180
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # 4. Create mask
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 5. Find contours
    contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    
    if not contours:
        return None, mask
    
    # 6. Get largest contour (ball)
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    
    if M["m00"] == 0:
        return None, mask
    
    # 7. Calculate centroid
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    
    return (cx, cy), mask
```

### 📊 Centroid to Motor Control

```
Camera Frame: 640x480
             0     320    640
           ┌─────────────────┐
         0 │        △        │  ← Ball at (320, 150)
           │      / | \      │
           │     /  |  \     │
       240 │────┼───●───┼────│  ← Center (320, 240)
           │     \  |  /     │
           │      \ | /      │
       480 └─────────────────┘

Position → Speed:
  cx < 280:  Turn LEFT  (negative yaw)
  cx > 360:  Turn RIGHT (positive yaw)
  cx = 320:  Go FORWARD (straight)
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ Camera sensor integration
- ✅ Image capture and format conversion
- ✅ Color space basics (BGR vs RGB)
- ✅ Thresholding for object detection
- ✅ Centroid calculation
- ✅ Proportional steering control
- ✅ ROS2 image publishing

---

## 📝 Customization

### Track Different Color

```python
# For blue objects
self.TARGET_COLOR_MIN = [200, 0, 0]
self.TARGET_COLOR_MAX = [255, 100, 50]
```

### Adjust Aggressiveness

```python
# Gentle turning
self.TURN_GAIN = 0.5

# Aggressive turning
self.TURN_GAIN = 1.5
```

### Change Detection Threshold

```python
# More lenient (detects partial objects)
self.DETECTION_THRESHOLD = 200

# More strict (only large objects)
self.DETECTION_THRESHOLD = 2000
```

---

## 📚 Related Resources

- 📖 [Webots Camera](https://cyberbotics.com/doc/reference/camera)
- 🔗 [OpenCV Color Detection](https://docs.opencv.org/master/d7/d1d/tutorial_js_colorspaces.html)
- 🎨 [Color Space (BGR vs RGB)](https://en.wikipedia.org/wiki/RGB_color_model)
- 📐 [Centroid Calculation](https://en.wikipedia.org/wiki/Centroid)
- 🤖 [ROS 2 Image Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Ball not detected** | Verify ball is red; adjust color range |
| **Erratic tracking** | Increase DETECTION_THRESHOLD; reduce noise |
| **Robot spins in place** | Decrease TURN_GAIN; reduce lighting changes |
| **Image too dark/bright** | Check world lighting; adjust in camera settings |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
