# Level 4.1: RGB Camera Controller - Ball Chaser 📷

## Overview
Welcome to **Level 4: Visual Sensors**! This controller demonstrates the use of **RGB Camera** for computer vision-based robot control. The robot can now "see" its environment, detect colored objects, and autonomously track and chase targets using real-time image processing.

## Learning Objectives
- Understand camera sensor principles and image capture
- Process RGB images using NumPy
- Implement color-based object detection (Color Thresholding)
- Calculate object position from pixel coordinates
- Design autonomous tracking behavior
- Publish camera images to ROS2 for visualization
- Integrate vision with motion control

## Sensor Information

### RGB Camera
- **Type**: Webots `Camera` device
- **Resolution**: 640x480 pixels (VGA)
- **Format**: BGRA (Blue, Green, Red, Alpha)
- **Field of View**: Wide-angle lens
- **ROS2 Message**: `sensor_msgs/Image`
- **Applications**:
  - Object detection and tracking
  - Line following
  - Color-based navigation
  - Visual servoing
  - QR code/marker detection

### Image Coordinate System
```
    0 ───────────── width (640) → X
    │
    │     Camera View
    │
height (480)
    ↓
    Y
```

### Color Spaces
- **Webots Output**: BGRA (Blue, Green, Red, Alpha channels)
- **OpenCV Standard**: BGR (Blue, Green, Red)
- **RGB**: Most intuitive (Red, Green, Blue)
- **HSV**: Better for color detection (Hue, Saturation, Value)

## Configuration Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MAX_SPEED` | 0.6 rad/s | Maximum forward speed |
| `TURN_SPEED` | 0.4 rad/s | Rotation speed |
| `TARGET_COLOR_MIN` | [150, 0, 0] | Red lower bound (BGR) |
| `TARGET_COLOR_MAX` | [255, 100, 100] | Red upper bound (BGR) |
| `DETECTION_THRESHOLD` | 500 pixels | Minimum pixels for detection |
| `PUBLISH_RATE` | 10 Hz | ROS2 image publishing rate |

## Features

### 1. Real-Time Dashboard
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

### 2. Color Detection Algorithm
The controller uses **color thresholding** to detect red objects:

```python
# Step 1: Define color range
lower_red = [150, 0, 0]   # Minimum BGR values
upper_red = [255, 100, 100]  # Maximum BGR values

# Step 2: Create binary mask
mask = (image >= lower_red) & (image <= upper_red)

# Step 3: Count detected pixels
if pixel_count > THRESHOLD:
    ball_detected = True
```

### 3. Object Position Tracking
Calculates normalized position (0-1 range):
```python
# Find all red pixels
red_pixels = where(mask == 255)

# Calculate centroid (average position)
center_x = mean(red_pixels[column])

# Normalize: 0=left edge, 0.5=center, 1=right edge
position = center_x / image_width
```

### 4. Autonomous Ball Chasing
Proportional control system:
```python
error = ball_position - 0.5  # Target is center (0.5)

if |error| < 0.1:
    # Ball centered → Move forward
    left_speed = right_speed = MAX_SPEED
    
elif error > 0:
    # Ball on right → Turn right
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED * (1 - error*2)
    
else:
    # Ball on left → Turn left
    left_speed = MAX_SPEED * (1 + error*2)
    right_speed = MAX_SPEED
```

### 5. Dual Control Modes

**AUTO Mode** (Default):
- Robot autonomously searches and chases red ball
- Rotates when no ball detected
- Tracks and approaches detected ball

**MANUAL Mode** (Press WASD):
- Full keyboard control override
- Press **M** to return to AUTO mode

## World Setup

The world includes:
- **🔴 Red Ball** (target): Sphere at (1.5, 0, 0.15)
- **🟢 Green Cube**: Box at (-1.5, 1.5, 0.1)
- **🔵 Blue Cylinder**: At (-1.5, -1.5, 0.15)
- **🤖 CameraBot**: Green robot with front-mounted camera
- **Arena**: 6x6 meter enclosed space

## ROS2 Integration

### Published Topics

| Topic | Message Type | Rate | Content |
|-------|--------------|------|---------|
| `/camera/image_raw` | `sensor_msgs/Image` | 10 Hz | Raw BGR camera image |
| `/camera/image_processed` | `sensor_msgs/Image` | 10 Hz | Binary mask (detection result) |
| `/camera/detection` | `std_msgs/String` | 10 Hz | Detection status string |

### Message Examples

**Image Message Structure**:
```python
header:
  stamp: current_time
  frame_id: "camera_link"
height: 480
width: 640
encoding: "bgr8"  # or "mono8" for mask
data: [byte array of pixel data]
```

**Detection Message**:
```
"RED_BALL_DETECTED,position=0.65"
# or
"NO_DETECTION"
```

## Running the Controller

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ce_webots --symlink-install
source install/setup.bash
```

### 2. Launch Webots
```bash
webots ~/ros2_ws/src/ce_webots/worlds/041_camera.wbt
```

### 3. Run the Controller
In a new terminal:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ce_webots 041_camera_controller
```

### 4. Visualize Camera Feed (Optional)
Use RViz2 or rqt_image_view:
```bash
# Install if needed
sudo apt install ros-jazzy-rqt-image-view

# View raw camera
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View processed mask
ros2 run rqt_image_view rqt_image_view /camera/image_processed
```

### 5. Monitor Detection Status
```bash
ros2 topic echo /camera/detection
```

## Controls

| Key | Action |
|-----|--------|
| **M** | Switch to AUTO mode (ball chasing) |
| **W** | Manual: Move forward |
| **S** | Manual: Move backward |
| **A** | Manual: Turn left |
| **D** | Manual: Turn right |
| **Q** | Quit program |

## Expected Behavior

### AUTO Mode Sequence
1. **Searching Phase**: Robot rotates slowly, scanning for red object
2. **Detection**: When red pixels > threshold, ball is detected
3. **Positioning**: Robot calculates ball position in image
4. **Tracking**: Robot adjusts heading to center ball in view
5. **Approach**: When centered, robot moves forward toward ball
6. **Repeat**: Continuous tracking as ball moves

### Visual Indicators
- **Position Bar**: Shows ball location in camera view
  - 🔴 marker indicates detected position
  - `L ←` to `→ R` shows left-right range
- **Status Messages**: 
  - 🔍 Searching when no detection
  - ⬅️ / ➡️ Turning toward ball
  - 🎯 Moving forward when centered

## Exercises

### 🎯 Exercise 1: Color Tuning
**Task**: Adjust color detection parameters for better accuracy
```python
# Try different color ranges
self.TARGET_COLOR_MIN = [140, 0, 0]    # Wider red range
self.TARGET_COLOR_MAX = [255, 120, 120]

# Adjust sensitivity
self.DETECTION_THRESHOLD = 300  # Lower = more sensitive
```
Test in different lighting conditions.

### 🎯 Exercise 2: Multi-Color Detection
**Task**: Detect green cube instead of red ball
```python
# Green color range (BGR)
self.TARGET_COLOR_MIN = [0, 150, 0]
self.TARGET_COLOR_MAX = [100, 255, 100]
```
Observe how robot behavior changes with different targets.

### 🎯 Exercise 3: Enhanced Tracking
**Task**: Improve proportional control
```python
# In control_ball_chaser():
turn_correction = error * 3.0  # Stronger correction
distance_estimate = detected_pixel_count  # Estimate distance

# Stop when very close
if distance_estimate > 5000:  # Large in view = close
    stop_motors()
```

### 🎯 Exercise 4: Obstacle Avoidance
**Task**: Combine camera with distance sensors
1. Add distance sensors from Level 022
2. Create hybrid controller:
   - Use camera to find ball
   - Use distance sensors to avoid obstacles
   - Navigate around obstacles toward target

### 🎯 Exercise 5: Multiple Object Tracking
**Task**: Track multiple colored objects
```python
# Detect all three objects
red_mask = detect_color([150,0,0], [255,100,100])
green_mask = detect_color([0,150,0], [100,255,100])
blue_mask = detect_color([0,0,150], [100,100,255])

# Choose closest or largest
target = select_best_target([red, green, blue])
```

## Advanced Applications

### 1. Line Following
Detect black line on white floor:
```python
# Black color range
lower_black = [0, 0, 0]
upper_black = [50, 50, 50]

# Process only bottom half of image
line_region = image[height//2:, :]

# Calculate line center
line_center = find_line_centroid(line_region)

# Steering control
error = line_center - width/2
```

### 2. QR Code Detection
Use OpenCV for marker detection:
```python
import cv2

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect QR codes
detector = cv2.QRCodeDetector()
data, points, _ = detector.detectAndDecode(gray)

if data:
    print(f"QR Code: {data}")
```

### 3. Face/Object Recognition
Use pre-trained models:
```python
# OpenCV Haar Cascades
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
faces = face_cascade.detectMultiScale(gray, 1.1, 4)

# Or use deep learning models
# YOLO, SSD, Faster R-CNN for object detection
```

### 4. Visual Servoing
Precise positioning using vision:
```python
# PID controller for precise centering
error_integral = 0
error_derivative = 0

error = target_position - center
error_integral += error * dt
error_derivative = (error - prev_error) / dt

control = Kp*error + Ki*error_integral + Kd*error_derivative
```

### 5. SLAM with Vision
Combine camera with odometry:
```python
# Extract visual features
features = detect_ORB_features(image)

# Match with previous frame
matches = match_features(prev_features, current_features)

# Estimate camera motion
translation, rotation = estimate_motion(matches)

# Update robot pose
pose = update_pose(pose, translation, rotation)
```

## Common Issues & Solutions

### Issue 1: No Detection
**Symptoms**: Ball never detected, always searching
**Solutions**:
- Lower `DETECTION_THRESHOLD` (try 200)
- Widen color range (increase MAX, decrease MIN)
- Check lighting in simulation
- Verify ball color in world file

### Issue 2: False Detections
**Symptoms**: Detects non-target objects
**Solutions**:
- Narrow color range (more restrictive bounds)
- Increase `DETECTION_THRESHOLD`
- Add size filtering (area constraints)
- Use HSV color space for better discrimination

### Issue 3: Jittery Movement
**Symptoms**: Robot oscillates around target
**Solutions**:
- Increase center tolerance (make `< 0.1` larger)
- Add smoothing to position estimate
- Reduce turn correction multiplier
- Implement low-pass filter on position

### Issue 4: Camera Images Not Publishing
**Symptoms**: No images in ROS2 topics
**Solutions**:
- Verify camera is enabled: `camera.enable(timestep)`
- Check topic names: `ros2 topic list`
- Confirm publish rate is not too high
- Test with: `ros2 topic hz /camera/image_raw`

### Issue 5: Slow Processing
**Symptoms**: Low frame rate, laggy response
**Solutions**:
- Reduce camera resolution in world file
- Lower publish rate
- Optimize image processing (vectorized operations)
- Use region of interest (process partial image)

## Color Detection Tips

### Understanding BGR vs RGB
```python
# Webots/OpenCV uses BGR (Blue, Green, Red)
red_bgr = [0, 0, 255]      # Pure red
green_bgr = [0, 255, 0]    # Pure green  
blue_bgr = [255, 0, 0]     # Pure blue

# Human intuition uses RGB
red_rgb = [255, 0, 0]      # Pure red
```

### Color Range Selection
To find good color ranges:
1. Capture sample image with red ball
2. Use color picker tool to get exact BGR values
3. Add tolerance: ±50 for each channel
4. Test and adjust based on results

### Lighting Sensitivity
Color detection affected by:
- **Ambient light**: Brighter = higher values
- **Shadows**: Darker = lower values
- **Reflections**: Can cause false positives
- **Solution**: Use HSV color space (less sensitive to lighting)

## Performance Metrics

### Detection Accuracy
```python
# True Positive: Correctly detected ball
# False Positive: Detected non-ball as ball
# False Negative: Missed actual ball

accuracy = correct_detections / total_frames
```

### Tracking Precision
```python
# Measure deviation from target center
average_error = mean(abs(ball_position - 0.5))

# Good tracking: < 0.1 (within 10% of image)
```

### Response Time
```python
# Time from detection to centered
response_time = time_centered - time_first_detected

# Typical: 1-3 seconds for 90° turn
```

## Physics & Math Background

### Image Processing Fundamentals
```
Pixel: Basic unit of image (single point)
Image: 2D array of pixels (height × width)
Color Channel: Separate red/green/blue intensities
Thresholding: Binary classification (in-range or not)
```

### Centroid Calculation
```
Given N pixels at positions (x₁,y₁), (x₂,y₂), ..., (xₙ,yₙ):

Centroid_x = (x₁ + x₂ + ... + xₙ) / N
Centroid_y = (y₁ + y₂ + ... + yₙ) / N
```

### Proportional Control
```
Output = K_p × Error

Where:
- Error = Desired_Position - Actual_Position
- K_p = Proportional gain (tuning parameter)
- Larger K_p = Faster response, but may oscillate
```

## Key Takeaways

✅ Cameras enable **visual perception** of the environment  
✅ **Color thresholding** is simple but effective for object detection  
✅ **Centroid calculation** determines object position in image  
✅ **Proportional control** creates smooth tracking behavior  
✅ ROS2 **Image messages** allow vision system visualization  
✅ Combining **vision + motion** enables autonomous behaviors  
✅ **Manual override** useful for testing and debugging  
✅ Color detection sensitive to **lighting conditions**

## Next Steps

After mastering camera control:
- **Level 4.2**: Line Following (detect and follow black line)
- **Level 4.3**: Object Recognition (identify specific objects)
- **Level 5**: LIDAR sensors for mapping
- **Level 6**: Sensor Fusion (combine camera + LIDAR + IMU)
- **Advanced**: Visual SLAM, Deep Learning object detection

---

**Happy Vision! 📷🤖🎯**
