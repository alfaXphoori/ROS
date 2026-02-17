# 🖤 042 - Line Follower Controller

> **Level 4.2 - Visual Sensors | Autonomous Line Following with Camera Vision**

---

## 📌 Overview


Level 4.2: Line Follower (Camera)
=================================
Robot follows a black line on a white floor using a camera

Logic:
1. Read image from camera (Grayscale)
2. Crop only the bottom part (ROI - Region of Interest) to see the line near the robot
3. Find the position of dark pixels (black pixels)
4. Calculate the centroid of the line
5. Calculate Error = (image center - line centroid)
6. Adjust left/right motor speeds to keep Error at 0 (P-Controller)

Author: AI Assistant

### ✨ Key Features

- 📷 Bottom-region image analysis
- 🔍 Brightness-based line detection
- 📍 Centroid calculation for line position
- 🎯 Proportional steering control
- 📊 Real-time visual feedback
- 🎨 Visual line tracking display
- ⚡ Fast processing (low-resolution optimized)
- 📈 Adaptive speed control

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `042_line_follower_controller.py` | Line following controller |
| `042_line_follower.wbt` | Webots world with track |


---

## 🚀 How to Run This Lab

### Prerequisites

- ✅ Webots installed
- ✅ ROS 2 Jazzy installed and sourced
- ✅ Understanding of Lab 041 (camera basics)
- ✅ Workspace built and sourced

### Running Steps

#### Terminal 1: Launch Webots Simulation

```bash
webots ~/ros2_ws/src/ce_webots/worlds/042_line_follower.wbt
```

**Environment features:**
- Track with black line on white/gray floor
- Curved and straight sections
- Various track complexities (sharp turns, gentle curves)
- Robot with downward-facing camera

#### Terminal 2: Run Line Follower

```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Run the line follower
ros2 run ce_webots 042_line_follower_controller
```

**What to observe:**
- Robot automatically follows the black line
- Real-time line position feedback
- Adaptive speed control (slower on sharp turns)
- Motor speed adjustments visible in display
- Smooth navigation through curves

### Automatic Operation

The line follower runs autonomously:
- No keyboard control needed
- Continuously adjusts left/right motors
- Uses proportional control for smooth following
- Handles curves and straight sections

### Real-Time Dashboard

```
============================================================
   🖤 LINE FOLLOWER CONTROLLER
============================================================

📷 CAMERA VIEW (160x120 bottom half):
   ░░░░░░░░░░░░░░░░░
   ░░░░░░░░░░░░░░░░░
   ░░░░░░░░░░░░░░░░░
   ░░░░░░░░░░░░░░░░░  ← Light gray floor
   ░░░░░░░░░░░░░░░░░

   ░░░░░░░░░░░░░░░░░
   ░░░░░▓▓▓▓▓▓░░░░░░  ← Black line detected
   ░░░░░▓▓▓▓▓▓░░░░░░
   ░░░░░▓▓▓▓▓▓░░░░░░

────────────────────────────────────────────────────────
🎯 LINE POSITION: 0.52 [━━━━━━━━━🟤━━━━━━━━]
📊 LINE WIDTH: 10 pixels
⚡ MOTOR CONTROL: L=4.2 rad/s | R=3.8 rad/s
────────────────────────────────────────────────────────
```

### Understanding the Control

**Line Position:**
- **0.0-0.4:** Line on left → increase right motor speed
- **0.4-0.6:** Line centered → both motors equal speed
- **0.6-1.0:** Line on right → increase left motor speed

**Proportional Control:**
```
error = image_center - line_centroid
left_speed = base_speed - (Kp × error)
right_speed = base_speed + (Kp × error)
```

### Monitoring Topics

```bash
# Terminal 3 (optional): View camera ROI
ros2 run rqt_image_view rqt_image_view
# Select: /camera/line_roi

# Monitor line position
ros2 topic echo /line/position

# Monitor control values
ros2 topic echo /pid/error

# Check camera feed
ros2 topic list | grep camera
```

### Tuning Parameters

Key parameters to adjust performance:

1. **Proportional Gain (Kp):**
   - **Higher (3.0-5.0):** Aggressive, fast response, may oscillate
   - **Lower (1.0-2.0):** Gentle, smooth, may lose line on sharp turns
   - **Optimal:** ~2.5-3.5 for most tracks

2. **Base Speed:**
   - **Higher:** Faster but harder to control
   - **Lower:** More stable but slower
   - **Adaptive:** Reduce speed when error is high

3. **Brightness Threshold:**
   - **Higher (200+):** Only detects very dark pixels
   - **Lower (100-150):** Detects more but may include shadows
   - **Auto-adjust:** Calculate threshold from image statistics

### Experiment Ideas

1. **Add PID Control:**
   - Implement derivative term (reduce oscillation)
   - Add integral term (eliminate steady-state error)

2. **Advanced Features:**
   - Detect intersection and make decisions
   - Handle broken lines or gaps
   - Speed up on straight sections
   - Add line width monitoring

3. **Improve Robustness:**
   - Add edge detection (Canny/Sobel)
   - Implement Hough line detection
   - Use color-based segmentation

### Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot loses line** | Reduce speed or increase Kp |
| **Oscillates on straight** | Decrease Kp or add derivative term |
| **Can't handle sharp turns** | Reduce base speed or add adaptive speed |
| **No line detected** | Adjust brightness threshold |
| **Wrong ROI** | Check camera placement and crop settings |

### Line Follower Algorithm

```
1. Capture camera image (160x120)
2. Crop bottom half (ROI near robot)
3. Convert to grayscale
4. Threshold: dark pixels (< threshold) = line
5. Calculate weighted average (centroid)
6. Compute error = center - centroid
7. Apply P-control: adjust motor speeds
8. Repeat at camera frame rate
```

### Performance Metrics

- **Track Completion Time:** How fast can it complete a lap?
- **Position Error:** Average deviation from line center
- **Oscillation:** Frequency of left-right corrections
- **Recovery Time:** Time to recover after losing line

---

## 🔧 How It Works

### 1. Initialize Camera (Bottom-Facing)

```python
# Get camera device
self.camera = self.robot.getDevice('camera')
self.camera.enable(self.timestep)

# Camera specs (low-resolution for speed)
self.WIDTH = 160
self.HEIGHT = 120
self.FOV = 1.0  # radians
self.FOCUS_DISTANCE = 0.15  # meters from ground
```

### 2. Image Processing Pipeline

```python
def detect_line(self, image):
    """Extract line from camera image"""
    # Step 1: Extract bottom half (robot can't see ground ahead)
    bottom_half = image[60:120, :, :]
    
    # Step 2: Convert BGRA to brightness
    bgr = bottom_half[:, :, :3]
    brightness = (bgr[:, :, 0] + bgr[:, :, 1] + bgr[:, :, 2]) / 3
    
    # Step 3: Threshold - black line (brightness < 100)
    line_mask = brightness < 100
    
    # Step 4: Find black pixels
    black_pixels = np.where(line_mask)
    
    if len(black_pixels[1]) == 0:
        return None
    
    # Step 5: Calculate centroid
    centroid_x = np.mean(black_pixels[1])
    position = centroid_x / self.WIDTH
    
    return position  # 0.0=left, 1.0=right
```

### 3. Proportional Steering Control

```python
def calculate_steering(self, line_position):
    """Convert line position to motor speeds"""
    center = 0.5  # Middle of image
    error = line_position - center
    
    # Proportional gain (tuning parameter!)
    TURN_GAIN = 0.8
    
    # Turn adjustment
    turn_power = error * TURN_GAIN
    
    # Base speed
    base_speed = 4.0
    
    # Differential speed
    left_speed = base_speed - turn_power
    right_speed = base_speed + turn_power
    
    return left_speed, right_speed
```

---

## 🖼️ Image Processing Examples

### Example 1: Line Center (Go Straight)

```
Image:    [░░░░░░░░▓▓▓▓▓▓░░░░░░░]
Position: 0.5 (center)
Error:    0.0

Motor Control:
  Left:  4.0 rad/s  (base speed)
  Right: 4.0 rad/s  (base speed)
  Result: ➡️ STRAIGHT
```

### Example 2: Line Right (Turn Right)

```
Image:    [░░░░░░░░░░░░░░░▓▓▓▓░░]
Position: 0.7 (right side)
Error:    +0.2

Motor Control:
  Adjustment: 0.2 × 0.8 = 0.16
  Left:  4.16 rad/s  (faster)
  Right: 3.84 rad/s  (slower)
  Result: ↘️ TURN RIGHT
```

### Example 3: Line Left (Turn Left)

```
Image:    [░░▓▓▓▓░░░░░░░░░░░░░░░]
Position: 0.15 (left side)
Error:    -0.35

Motor Control:
  Adjustment: -0.35 × 0.8 = -0.28
  Left:  3.72 rad/s  (slower)
  Right: 4.28 rad/s  (faster)
  Result: ↙️ TURN LEFT
```

---

## 📐 Brightness Calculation

For each pixel in BGRA format:

```python
brightness = (Blue + Green + Red) / 3

Why not use Alpha?
- Alpha represents transparency, not light intensity
- RGB values represent actual color brightness
```

---

## 📚 Sensor Knowledge: Camera (Line Detection)

### 🎯 How It Works

**Camera-based Line Detection** uses ROI (Region of Interest) at the bottom to analyze black lines:

```
Image Capture & Processing:
  1. Capture full 640x480 image
  2. Extract bottom ROI (e.g., rows 300-480)
  3. Convert to grayscale
  4. Apply brightness threshold
  5. Find line centroid (center of black pixels)
  6. Calculate steering error
  7. Generate motor commands

ROI Concept:
┌────────────────────────┐
│  Full Image 640x480    │
│  ┌──────────────────┐  │
│  │ Lines 0-300      │  │ Ignore top (not needed)
│  │ (sky/obstacles)  │  │
│  └──────────────────┘  │
│  ┌──────────────────┐  │
│  │ Lines 300-480    │  │← ROI: Only analyze here!
│  │ (floor + line)   │  │
│  └──────────────────┘  │
└────────────────────────┘
```

### 📊 Specifications

| Property | Value | Notes |
|---------|-------|-------|
| **Full Resolution** | 640x480 | Standard VGA |
| **ROI Height** | ~180 pixels | Bottom portion |
| **ROI Width** | 640 pixels | Full width |
| **Effective Pixels** | 115,200 | Reduced processing |
| **Threshold** | 0-100 (Dark) | Brightness cutoff |
| **Update Rate** | ~64 Hz | 15.6 ms/frame |
| **Processing Time** | ~2-5 ms | ROI optimized |

### 💡 Usage Tips

**✅ Do:**
- Use ROI to reduce processing load
- Apply brightness thresholding
- Centroid-based line detection
- Dynamic threshold adjustment
- Multi-segment line analysis

**❌ Avoid:**
- Process entire image (slow)
- Use color detection (B/W faster)
- Static thresholds in changing light
- Ignore shadows on line
- Too narrow ROI (miss line turns)

### ⚠️ Limitations

```
1. Lighting Sensitivity
   ├─ Dark line needs: threshold < 100
   ├─ Shadows shift: brightness changes
   │  └─→ Solution: Normalize brightness
   └─  Solution: Adaptive thresholding

2. Line Width Assumption
   • Assumes fixed line width
   • Thick lines = centroid at center ✓
   • Thin lines (1px) = noisy ✗
   • Solution: Dilate/morphology operations

3. Curve Limitation
   • Sharp curves > camera FOV
   • May lose line at turns
   • Solution: Slow down; increase ROI height

4. No Lookahead
   • Only sees current line position
   • Can't anticipate next turn
   • Solution: Use machine learning (LSTM)
   
5. Processing vs Speed Trade-off
   • Full resolution = accurate but slow
   • Low resolution = fast but inaccurate
   • ROI = best balance
```

### 🔧 Line Detection Algorithm

```python
import cv2
import numpy as np

def detect_line_centroid(image_bgra, roi_start=300, roi_end=480):
    """Detect black line and return centroid x-position"""
    
    # 1. Convert BGRA → Grayscale
    gray = cv2.cvtColor(image_bgra, cv2.COLOR_BGRA2GRAY)
    
    # 2. Extract ROI (bottom portion for line detection)
    roi = gray[roi_start:roi_end, :]  # All width, bottom rows
    
    # 3. Apply threshold (black line detection)
    #    Bright = 255, Black line = 0
    #    We want pixels < threshold
    _, binary = cv2.threshold(roi, 100, 255, cv2.THRESH_BINARY_INV)
    # Result: Black pixels → 255, Others → 0
    
    # 4. Find line centroid
    moments = cv2.moments(binary)
    
    if moments["m00"] == 0:
        return None, None, binary  # No line found
    
    # Center of mass calculation
    centroid_x = int(moments["m10"] / moments["m00"])
    centroid_y = int(moments["m01"] / moments["m00"]) + roi_start
    
    # 5. Normalize to 0.0-1.0 range
    normalized_x = centroid_x / 640.0
    
    return normalized_x, (centroid_x, centroid_y), binary

def calculate_line_error(centroid_x, image_width=640):
    """Calculate steering error"""
    
    center = image_width / 2.0
    error = (centroid_x - center) / center  # Range: -1.0 to +1.0
    
    return error  # Negative = left, Positive = right

def adaptive_threshold_line(roi, method='otsu'):
    """Adaptive thresholding for changing lighting"""
    
    if method == 'otsu':
        # Otsu's automatic threshold
        _, binary = cv2.threshold(roi, 0, 255, 
                                   cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    elif method == 'adaptive':
        # Adaptive threshold (local brightness)
        binary = cv2.adaptiveThreshold(roi, 255, 
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 21, 5)
    
    return binary

# Morphological operations for line cleanup
def clean_line_image(binary):
    """Clean up noise in line detection"""
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    
    # Morphological closing: remove small black dots
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    
    # Morphological opening: remove small white noise
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
    
    return cleaned
```

### 📊 Line Following Control Loop

```
Centroid Detection
        ↓
Calculate Error (center - centroid)
        ↓
Error = 0?        No Steering needed ✓
   │
   └─→ Error > 0: Line is LEFT  (turn left)
        │
        └─→ turn_power = -error * gain
        
   └─→ Error < 0: Line is RIGHT (turn right)
        │
        └─→ turn_power = -error * gain

Apply Proportional Control
        ↓
left_speed = base_speed - turn_power
right_speed = base_speed + turn_power
        ↓
Send to motors
```

---

## 🎓 Learning Outcomes

After using this controller, you'll understand:

- ✅ Camera-based line detection
- ✅ Image ROI (Region of Interest) selection
- ✅ Brightness/grayscale thresholding
- ✅ Centroid-based position calculation
- ✅ Proportional control tuning
- ✅ Real-time image processing
- ✅ Closed-loop visual servoing

---

## 📝 Customization

### Adjust Line Detection Sensitivity

```python
# Current (black line on gray floor)
line_mask = brightness < 100

# Detect dark objects
line_mask = brightness < 80

# Detect only very dark
line_mask = brightness < 50
```

### Tune Steering Gain

```python
# Gentle steering (follows smooth curves)
TURN_GAIN = 0.5

# Normal steering (current)
TURN_GAIN = 0.8

# Aggressive steering (quick curves)
TURN_GAIN = 1.2
```

### Change Camera Region

```python
# Look at middle section instead of bottom
middle_region = image[45:75, :, :]

# Look at larger bottom area
large_bottom = image[30:120, :, :]
```

### Track Different Colors

```python
# White line on dark floor
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))

# Yellow line
yellow_mask = cv2.inRange(hsv, (20, 100, 100), (40, 255, 255))
```

---

## 🔍 Advanced: HSV Thresholding

For better robustness to lighting changes, use HSV instead of brightness:

```python
# Convert to HSV
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

# Create mask for black (low saturation, low value)
lower_black = (0, 0, 0)
upper_black = (180, 255, 50)

mask = cv2.inRange(hsv, lower_black, upper_black)
```

**Advantages:**
- Robust to lighting variations
- Better separation of color and brightness
- More reliable in changing light

---

## 📚 Related Resources

- 📖 [Webots Camera](https://cyberbotics.com/doc/reference/camera)
- 🔗 [OpenCV Line Detection](https://docs.opencv.org/master/da/d22/tutorial_py_canny.html)
- 📐 [Proportional Control](https://en.wikipedia.org/wiki/Proportional_control)
- 🎨 [HSV Color Space](https://en.wikipedia.org/wiki/HSL_and_HSV)
- 🤖 [ROS 2 Image Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)
- 👀 [041 Camera Controller](../041_camera_controller/) (Color detection)

---

## ⚠️ Troubleshooting

| Issue | Solution |
|-------|----------|
| **Robot doesn't detect line** | Verify line brightness < 100; check camera orientation |
| **Follows line erratically** | Reduce TURN_GAIN; apply more smoothing |
| **Can't follow curves** | Increase TURN_GAIN; speed up processing |
| **Lost line halfway** | Increase ROI size; decrease threshold |

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
