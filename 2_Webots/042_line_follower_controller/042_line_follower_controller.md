# Level 4.2 - Line Following with Camera Vision

## Overview

This level introduces **line following** using computer vision - a fundamental skill in autonomous robotics. The robot uses a downward-facing camera to detect a black line on the floor and automatically steers to follow it.

**Key Concepts:**
- Bottom-region image analysis
- Brightness-based line detection
- Centroid calculation for line position
- Proportional steering control
- Real-time visual feedback

## Hardware Setup

### Camera Configuration
```python
Camera Specifications:
- Resolution: 160x120 pixels (low resolution for fast processing)
- Field of View: 1.0 radians (~57 degrees)
- Position: Front of robot, 0.15m above floor
- Orientation: Tilted down ~28 degrees (0.5 radians)
- Update Rate: 10 Hz
```

### Track Design
- **Line Color:** Black (brightness < 100)
- **Floor Color:** Light gray (RGB 0.8, 0.8, 0.8)
- **Line Width:** ~15cm
- **Track Shape:** Oval with straight sections and curved ends

## Line Detection Algorithm

### 1. Image Processing Pipeline

```
Camera Image (BGRA format)
    ↓
Extract Bottom Half (rows 60-120)
    ↓
Calculate Brightness for each pixel
    ↓
Threshold Detection (brightness < 100 = black)
    ↓
Find all black pixels
    ↓
Calculate Centroid (average X position)
    ↓
Convert to Position (0.0 = left edge, 1.0 = right edge)
    ↓
Proportional Control
```

### 2. Brightness Calculation

For each pixel in BGRA format:
```python
brightness = (blue + green + red) / 3
```

Why not use Alpha channel?
- Alpha (A) represents transparency, not brightness
- RGB values represent actual color intensity

### 3. Centroid Method

The centroid represents the "center of mass" of the detected line:

```python
centroid_x = sum(all_black_pixel_x_positions) / count(black_pixels)
position = centroid_x / image_width  # Normalize to 0.0-1.0
```

**Example:**
- Image width: 160 pixels
- Black pixels at X positions: [75, 76, 77, 78, 79, 80]
- Centroid: (75+76+77+78+79+80) / 6 = 77.5
- Position: 77.5 / 160 = 0.484 (slightly left of center)

## Control Strategy

### Proportional Steering

```python
center_position = 0.5  # Middle of image
error = position - center_position
turn_adjustment = error * TURN_GAIN

left_speed = BASE_SPEED - turn_adjustment
right_speed = BASE_SPEED + turn_adjustment
```

**How it works:**
- Line at center (0.5): error = 0 → robot goes straight
- Line to the right (0.7): error = +0.2 → turn right (slow left wheel)
- Line to the left (0.3): error = -0.2 → turn left (slow right wheel)

### TURN_GAIN Parameter

Controls how aggressively the robot turns:
- **Low gain (1.0):** Gentle turns, may drift off on sharp curves
- **Medium gain (2.0):** Balanced, default setting
- **High gain (4.0):** Sharp turns, may oscillate

## Configuration Parameters

```python
# Camera Settings
PUBLISH_RATE = 10        # Hz - camera update frequency
CAMERA_WIDTH = 160       # pixels
CAMERA_HEIGHT = 120      # pixels

# Line Detection
LINE_THRESHOLD = 100     # Brightness threshold (0-255)
                        # Pixels darker than this = line

# Control Parameters
BASE_SPEED = 3.0        # rad/s - forward speed when centered
TURN_GAIN = 2.0         # Steering sensitivity multiplier
MAX_SPEED = 6.28        # rad/s - maximum wheel speed

# Search Behavior
SEARCH_LEFT_SPEED = 0.5  # Speed when line lost
SEARCH_RIGHT_SPEED = -0.5
```

## Code Structure

### Main Components

1. **Camera Processing** (`process_camera_image`)
   - Converts BGRA bytes to brightness values
   - Analyzes bottom half of image only
   - Detects black pixels below threshold
   - Calculates line position

2. **Motor Control** (`calculate_motor_speeds`)
   - Computes steering error
   - Applies proportional control
   - Handles line-lost condition

3. **Keyboard Override**
   - W: Forward
   - S: Backward
   - A: Turn left
   - D: Turn right
   - Space: Toggle Auto/Manual mode

4. **Dashboard Display**
   - Visual line position indicator (40-char bar)
   - Pixel count (line width indicator)
   - Motor speeds (L/R)
   - Mode indicator (AUTO/MANUAL)

## Running the Controller

```bash
# Terminal 1: Launch simulation
webots ~/ros2_ws/src/ce_webots/worlds/042_line_follower.wbt

# Terminal 2: Run controller
source ~/ros2_ws/install/setup.bash
ros2 run ce_webots 042_line_follower_controller
```

## Common Issues & Troubleshooting

### Problem: Robot loses the line on curves

**Symptoms:** Works on straight sections but drifts off at turns

**Solutions:**
1. Increase TURN_GAIN (try 3.0-4.0)
2. Reduce BASE_SPEED (try 2.0) for more reaction time
3. Increase LINE_THRESHOLD if lighting is dim
4. Check camera tilt - should see ~2m ahead on floor

### Problem: Robot oscillates (zigzags) on straight line

**Symptoms:** Continuous left-right swaying

**Solutions:**
1. Decrease TURN_GAIN (try 1.5)
2. Reduce PUBLISH_RATE (try 5 Hz) for smoother control
3. Add derivative control (see Advanced section)

### Problem: "Line Lost! Searching..." constantly displayed

**Symptoms:** No black pixels detected

**Solutions:**
1. Verify line color is black in world file: `baseColor 0 0 0`
2. Increase LINE_THRESHOLD (try 120-150) if floor is dark
3. Check camera position - should point at floor
4. Ensure robot starts on the line

### Problem: Robot always turns one direction

**Symptoms:** Continuous rotation even when centered

**Solutions:**
1. Check wheel motors are correctly assigned (left/right)
2. Verify image coordinate system (x=0 is left edge)
3. Test with manual controls (WASD) to verify motors work

## Performance Tuning

### For Smooth Tracking:
```python
BASE_SPEED = 2.5
TURN_GAIN = 2.0
LINE_THRESHOLD = 100
PUBLISH_RATE = 10
```

### For Fast Racing:
```python
BASE_SPEED = 4.0
TURN_GAIN = 3.5
LINE_THRESHOLD = 90
PUBLISH_RATE = 20  # Requires changing time.sleep in main loop
```

### For Wide Lines (>20cm):
```python
LINE_THRESHOLD = 110
TURN_GAIN = 1.5  # Less aggressive on wide targets
```

### For Narrow Lines (<10cm):
```python
LINE_THRESHOLD = 90
TURN_GAIN = 3.0  # More aggressive on thin targets
```

## Exercises

### Exercise 1: Threshold Tuning
**Goal:** Understand brightness thresholding

1. Print average brightness values for line and floor
2. Find optimal LINE_THRESHOLD for your track
3. Test with different floor colors (white, gray, dark gray)
4. What happens if threshold is too high? Too low?

### Exercise 2: Region Selection
**Goal:** Optimize image analysis region

**Current:** Bottom half only (rows 60-120)

1. Try analyzing bottom quarter only (rows 90-120)
2. Try analyzing full image (rows 0-120)
3. Try analyzing a horizontal strip at specific height
4. Which performs best on curves? Why?

### Exercise 3: PID Control
**Goal:** Implement advanced control

**Current:** P (Proportional) only

Add Integral and Derivative terms:
```python
# Add these variables
self.integral_error = 0
self.previous_error = 0
self.dt = 1.0 / PUBLISH_RATE

# In calculate_motor_speeds:
error = position - 0.5

# Integral term (accumulates error over time)
self.integral_error += error * self.dt

# Derivative term (rate of change)
derivative = (error - self.previous_error) / self.dt
self.previous_error = error

# PID formula
Kp = 2.0   # Proportional gain
Ki = 0.1   # Integral gain
Kd = 0.5   # Derivative gain

turn = Kp * error + Ki * self.integral_error + Kd * derivative
```

**Expected results:**
- Faster settling on straight sections
- Less oscillation
- Better curve handling

### Exercise 4: Speed Adaptation
**Goal:** Slow down on curves, speed up on straights

**Hint:** Use line position stability as curve indicator

```python
# Detect curves by checking how much line moves
position_change = abs(position - self.last_position)

if position_change > 0.1:  # Sharp curve
    base_speed = 2.0
else:  # Straight section
    base_speed = 4.0
```

### Exercise 5: Multi-Line Handling
**Goal:** Follow specific line when multiple lines present

**Challenge:** Modify world to have 2 parallel lines, track only the right one

**Hint:** Modify centroid to only count pixels in right half of image

## Advanced Topics

### 1. Edge Detection
Instead of finding line center, detect line edges:

```python
# Find leftmost and rightmost black pixels
left_edge = min(black_pixel_x_positions)
right_edge = max(black_pixel_x_positions)
line_center = (left_edge + right_edge) / 2
```

**Advantages:** More robust to gaps in line

### 2. Weighted Centroid
Give more weight to pixels closer to robot:

```python
# Pixels at bottom of image (closer) weighted more
weighted_sum = 0
total_weight = 0

for (x, y) in black_pixels:
    weight = y  # Higher row number = closer to robot
    weighted_sum += x * weight
    total_weight += weight

centroid = weighted_sum / total_weight
```

### 3. Intersection Detection
Detect T-junctions or crossings:

```python
# Count black pixels in top half too
top_black_count = count_black_pixels(rows 0-60)
bottom_black_count = count_black_pixels(rows 60-120)

if top_black_count > threshold and bottom_black_count > threshold:
    # Intersection detected!
```

### 4. Color Line Following
Track colored lines instead of black:

```python
def detect_colored_line(image, target_color_range):
    """
    target_color_range: [(r_min, r_max), (g_min, g_max), (b_min, b_max)]
    """
    colored_pixels = []
    for pixel in image:
        r, g, b, a = pixel
        if (r_min <= r <= r_max and 
            g_min <= g <= g_max and 
            b_min <= b <= b_max):
            colored_pixels.append(pixel)
    return colored_pixels
```

### 5. Predictive Following
Use line angle to anticipate turns:

```python
# Fit a line through detected pixels
# Calculate line angle
# Adjust steering based on predicted path
```

## Physics & Math Background

### Image Coordinate System
```
(0,0) -------- X -------- (160,0)
  |                          |
  Y                          Y
  |                          |
(0,120) --------------- (160,120)
```
- Origin (0,0) = Top-Left
- X increases rightward
- Y increases downward

### Centroid Formula
For discrete points:
$$
\text{centroid}_x = \frac{1}{N} \sum_{i=1}^{N} x_i
$$

Where N = number of black pixels

### Proportional Control Transfer Function
$$
u(t) = K_p \cdot e(t)
$$

Where:
- $u(t)$ = control output (turn adjustment)
- $K_p$ = proportional gain (TURN_GAIN)
- $e(t)$ = error (position - setpoint)

## Real-World Applications

1. **Automated Guided Vehicles (AGVs)**
   - Warehouse robots following magnetic or painted lines
   - Same algorithm, different sensors (magnetic sensors vs camera)

2. **Road Lane Keeping**
   - Self-driving cars use similar principles
   - More complex: detect two lines (lane edges), keep between them

3. **Agricultural Robots**
   - Follow crop rows for harvesting/spraying
   - Uses color difference between crops and soil

4. **Assembly Line Tracking**
   - Products follow colored tape through factory
   - Multiple lines for different product types

## Next Steps

After mastering line following:
1. **Level 4.3** - Object Detection & Tracking
2. **Level 4.4** - Color-Based Navigation (follow colored markers)
3. **Level 5.1** - LIDAR Mapping
4. **Multi-Sensor Fusion** - Combine camera + IMU + distance sensors

## References

- **PID Control:** https://en.wikipedia.org/wiki/PID_controller
- **Image Moments:** https://en.wikipedia.org/wiki/Image_moment
- **Computer Vision:** OpenCV documentation - Image Thresholding
- **ROS2 Camera:** sensor_msgs/Image message type

---

**Practice Makes Perfect!** Try different tracks, speeds, and parameters to truly understand the algorithm. The best learning comes from experimentation! 🤖
