# **🎯 ROS 2 TF2 Lab Exercises**

Hands-on practice with coordinate transformations for real robot applications.

---

## **📌 Project Title**

TF2 Transform System - From Static Frames to Dynamic Robot Applications

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides practical exercises for mastering ROS 2 TF2 (Transform Library 2). You'll progress from basic static transforms to complex dynamic robot systems with multiple sensors and moving parts.

**Prerequisites:** Complete `09_TF2/Readme.md` first for foundational concepts!

**Duration:** ~2 hours  
**Level:** Intermediate  
**What You Already Know:** Publishers, Subscribers, Services, Actions, Launch Files

---

## **🎯 Learning Objectives**

By completing these exercises, you will:
- ✅ Broadcast static transforms for fixed robot components
- ✅ Create dynamic transforms for moving parts
- ✅ Listen to and use transforms in your nodes
- ✅ Perform coordinate transformations
- ✅ Work with quaternions and Euler angles
- ✅ Debug transform trees
- ✅ Visualize frames in RViz2
- ✅ Build complete robot TF systems

---

## **📊 Lab Architecture**

### **Progressive Complexity**

```
Exercise 1: Static Transforms          Exercise 2: Dynamic Transforms
─────────────────────────────          ──────────────────────────────
Simple fixed frames                    Moving frames (wheels, joints)
base_link → laser                      Real-time transform updates
Static broadcaster                     Dynamic broadcaster
Launch file integration                Time-synchronized transforms

Exercise 3: Transform Math             Exercise 4: Complete Robot
──────────────────────────             ──────────────────────────
Coordinate conversions                 Full robot TF tree
Quaternion calculations                Multi-sensor integration
Transform lookup timing                RViz2 visualization
Frame tree debugging                   Production-ready system
```

---

## **📚 Exercise Overview**

| Exercise | Title | Duration | Difficulty | Focus |
|----------|-------|----------|------------|-------|
| 1 | Static Transforms | 30 min | Beginner | Fixed frames, static broadcaster |
| 2 | Dynamic Transforms | 30 min | Intermediate | Moving frames, dynamic updates |
| 3 | Transform Math & Tools | 30 min | Intermediate | Coordinate math, debugging |
| 4 | Real Robot Application | 30 min | Advanced | Complete system integration |

---

## **Exercise 1: Static Transforms (Beginner) 📍**

### **📋 Objective**

Learn to broadcast and listen to static transforms - the foundation of any robot's coordinate system.

### **🎯 What You'll Learn**

- Create static transform broadcasters
- Define fixed relationships between frames
- Use `static_transform_publisher` node
- Configure static transforms in launch files
- Listen to static transforms

### **💡 Real-World Use Case**

**Scenario:** Mobile robot with fixed laser scanner

Your robot has:
- **base_link** - Robot's center point
- **laser_frame** - Laser scanner mounted 0.1m forward, 0.2m up
- **camera_link** - Camera mounted 0.15m forward, 0.25m up

These sensors never move relative to the robot, so use static transforms!

### **📝 Step 1: Create Static Transform Broadcaster**

**File:** `src/exercise_1/static_tf_broadcaster.py`

```python
#!/usr/bin/env python3
"""
Static TF2 Broadcaster
Publishes fixed transforms for robot sensors
"""

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class StaticFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        
        # Create static broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Broadcast static transforms
        self.broadcast_laser_transform()
        self.broadcast_camera_transform()
        
        self.get_logger().info('Static transforms published!')
    
    def broadcast_laser_transform(self):
        """Broadcast base_link -> laser_frame transform"""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        
        # Translation (x, y, z)
        t.transform.translation.x = 0.1  # 10cm forward
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2  # 20cm up
        
        # Rotation (quaternion) - no rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info('Published base_link → laser_frame')
    
    def broadcast_camera_transform(self):
        """Broadcast base_link -> camera_link transform"""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Translation
        t.transform.translation.x = 0.15  # 15cm forward
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.25  # 25cm up
        
        # Rotation - camera tilted down 15 degrees
        # Convert degrees to radians
        pitch = math.radians(-15.0)
        
        # Quaternion for pitch rotation (around Y axis)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(pitch / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(pitch / 2.0)
        
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info('Published base_link → camera_link')


def main(args=None):
    rclpy.init(args=args)
    node = StaticFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **📝 Step 2: Create Static Transform Listener**

**File:** `src/exercise_1/static_tf_listener.py`

```python
#!/usr/bin/env python3
"""
Static TF2 Listener
Listens to static transforms and prints them
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class StaticFrameListener(Node):
    def __init__(self):
        super().__init__('static_tf_listener')
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timer to check transforms
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Static TF Listener started')
    
    def timer_callback(self):
        """Check and print available transforms"""
        try:
            # Get transform from base_link to laser_frame
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'laser_frame',
                now
            )
            
            self.get_logger().info(
                f'Laser Transform - '
                f'Translation: [{trans.transform.translation.x:.3f}, '
                f'{trans.transform.translation.y:.3f}, '
                f'{trans.transform.translation.z:.3f}]'
            )
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not get laser transform: {ex}')
        
        try:
            # Get transform from base_link to camera_link
            trans_cam = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',
                now
            )
            
            self.get_logger().info(
                f'Camera Transform - '
                f'Translation: [{trans_cam.transform.translation.x:.3f}, '
                f'{trans_cam.transform.translation.y:.3f}, '
                f'{trans_cam.transform.translation.z:.3f}]'
            )
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not get camera transform: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = StaticFrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### **📝 Step 3: Create Launch File**

**File:** `src/exercise_1/robot_frames.launch.py`

```python
#!/usr/bin/env python3
"""
Launch file for static transform demonstration
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([  # type: ignore
        # Static transform broadcaster node
        Node(
            package='ce_robot_tf',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster',
            output='screen'
        ),
        
        # Static transform listener node
        Node(
            package='ce_robot_tf',
            executable='static_tf_listener',
            name='static_tf_listener',
            output='screen'
        ),
        
        # Alternative: Use built-in static_transform_publisher
        # Publish odom -> base_link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
    ])
```

### **🧪 Step 4: Test Static Transforms**

**Build and run:**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_tf --symlink-install
source install/setup.bash

# Run the launch file
ros2 launch ce_robot_tf robot_frames.launch.py
```

**Verify transforms:**

```bash
# Terminal 1: Check TF topics
ros2 topic echo /tf_static

# Terminal 2: Echo specific transform
ros2 run tf2_ros tf2_echo base_link laser_frame

# Terminal 3: View frame tree
ros2 run tf2_tools view_frames.py
# Opens frames.pdf

# Terminal 4: Visualize in RViz2
rviz2
# Add: Displays → TF
# Set Fixed Frame: base_link
```

**Expected output:**

```
[static_tf_broadcaster]: Published base_link → laser_frame
[static_tf_broadcaster]: Published base_link → camera_link
[static_tf_listener]: Laser Transform - Translation: [0.100, 0.000, 0.200]
[static_tf_listener]: Camera Transform - Translation: [0.150, 0.000, 0.250]
```

---

### **💡 Key Concepts Learned**

1. **StaticTransformBroadcaster** - Publish fixed transforms
2. **TransformStamped** - Transform message structure
3. **Transform buffer** - Store and lookup transforms
4. **Frame hierarchy** - Parent → child relationships
5. **Quaternions** - Represent rotations (w, x, y, z)

---

## **🎯 Exercise 1 Challenges**

### **Challenge 1: Add More Sensors**

Add these sensors to your robot:
- GPS antenna: 0.0m forward, 0.0m sideways, 0.3m up
- IMU sensor: at robot center (0, 0, 0) but rotated 45° around Z-axis

### **Challenge 2: Create Robot Arm**

Create a simple 2-joint robot arm:
```
base_link → shoulder (0.1m up) → elbow (0.2m forward) → gripper (0.15m forward)
```

---

*Continue to Exercise 2 for dynamic transforms...*

---

## **📊 Summary**

After Exercise 1, you should understand:
- ✅ How to broadcast static transforms
- ✅ Transform message structure
- ✅ Frame naming conventions
- ✅ Using TF2 tools for debugging
- ✅ Visualizing frames in RViz2

**Next:** Exercise 2 - Dynamic Transforms for moving robot parts! 🚀
