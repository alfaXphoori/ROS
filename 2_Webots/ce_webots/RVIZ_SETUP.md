# Running SLAM with RViz2

## Quick Start

### Terminal 1: Launch Webots Simulation
```bash
cd ~/ros2_ws
webots src/ce_webots/worlds/052_slam.wbt
```

### Terminal 2: Run the Controller
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ce_webots 052_slam_controller
```

### Terminal 3: Launch RViz2
```bash
cd ~/ros2_ws
source install/setup.bash
rviz2
```

## RViz2 Configuration

Once RViz2 opens:

### 1. Set Fixed Frame
- In the left panel under "Global Options"
- Set `Fixed Frame` to: **odom**

### 2. Add Displays

#### Add LaserScan:
1. Click **Add** button (bottom left)
2. Select **By topic** tab
3. Find `/scan` → **LaserScan**
4. Click OK
5. Adjust settings:
   - Size (m): 0.05
   - Color: Red or your choice

#### Add Map (Occupancy Grid):
1. Click **Add** button
2. Select **By topic** tab
3. Find `/map` → **OccupancyGrid**
4. Click OK

#### Add Robot Model (TF):
1. Click **Add** button
2. Select **By display type** tab
3. Select **TF**
4. Click OK

#### Add Odometry Path:
1. Click **Add** button
2. Select **By topic** tab
3. Find `/odom` → **Odometry**
4. Click OK
5. Under "Odometry" settings:
   - Keep Arrows: 100
   - Arrow Scale: 0.2

### 3. Save Configuration
- File → Save Config As...
- Save to: `~/ros2_ws/src/ce_webots/rviz/slam_config.rviz`

## Published Topics

The controller publishes:
- `/scan` (sensor_msgs/LaserScan) - Lidar data
- `/map` (nav_msgs/OccupancyGrid) - SLAM map
- `/odom` (nav_msgs/Odometry) - Robot odometry
- TF transforms: `odom` → `base_link` → `lidar_link`

## Controls

- **W/A/S/D** - Manual movement
- **M** - Toggle Auto/Manual mode

## Troubleshooting

**No data in RViz?**
- Check topics: `ros2 topic list`
- Check data: `ros2 topic echo /scan --once`

**TF errors?**
- Verify: `ros2 run tf2_tools view_frames`

**Map not showing?**
- Map updates every 0.5 seconds
- Move robot to build the map
