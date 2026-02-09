# 🤖 001 - Simple Robot Controller

> **Tutorial 00 - Basic Control | Robot-side Webots Controller**

---

## 📌 Overview

The **Simple Robot Controller** is the essential bridge between **ROS 2** and **Webots simulation**. It runs inside the Webots environment as an external controller and translates ROS 2 velocity commands into motor movements using differential drive kinematics.

### ✨ Key Features

- ✅ Subscribes to `/cmd_vel` topic for motion commands
- 🔄 Converts Twist messages to wheel speeds
- ⚙️ Implements differential drive kinematics
- 🔁 Runs continuously in Webots simulation loop

### 📂 Files in This Directory

| File | Purpose |
|------|---------|
| `001_simple_robot_controller.py` | Main controller script |
| `001_basic_control.wbt` | Webots world file |
| `001_simple_robot_controller.md` | Detailed documentation |

---

## 🚀 Quick Start

### Step 1️⃣: Launch Webots

```bash
webots ~/ros2_ws/src/ce_webots/worlds/001_basic_control.wbt
```

### Step 2️⃣: Run the Controller

```bash
ros2 run ce_webots 001_simple_robot_controller
```

**Expected Output:**
```
[INFO] [simple_robot_controller]: Simple Robot Controller Started
[INFO] [simple_robot_controller]: Waiting for /cmd_vel commands...
```

### Step 3️⃣: Send Test Command

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

🎯 The robot should move forward at **0.5 m/s**!

---

## 🔧 How It Works

### 1. Initialize Webots Robot

The controller connects to Webots and retrieves the robot instance with the simulation timestep (typically 32ms):

```python
from controller import Robot

self.robot = Robot()
self.timestep = int(self.robot.getBasicTimeStep())
```

### 2. Get Motor Devices

Motors are configured for velocity control by setting position to infinity:

```python
self.left_motor = self.robot.getDevice('left_motor')
self.right_motor = self.robot.getDevice('right_motor')

# Set to velocity control mode
self.left_motor.setPosition(float('inf'))
self.right_motor.setPosition(float('inf'))
self.left_motor.setVelocity(0.0)
self.right_motor.setVelocity(0.0)
```

### 3. Subscribe to ROS 2 Commands

The controller listens for velocity commands on the `/cmd_vel` topic:

```python
from geometry_msgs.msg import Twist

self.subscription = self.create_subscription(
    Twist,
    '/cmd_vel',
    self.cmd_vel_callback,
    10
)
```

### 4. Convert Twist to Wheel Speeds

Differential drive kinematics converts linear and angular velocities to individual wheel speeds:

```python
def cmd_vel_callback(self, msg):
    # Extract velocities
    linear = msg.linear.x    # m/s
    angular = msg.angular.z  # rad/s
    
    # Differential drive kinematics
    # wheel_speed = (linear ± angular * wheel_distance/2) / wheel_radius
```

---

## � วิธีการสร้าง .wbt File (How to Create World Files)

### 🎯 .wbt File คืออะไร?

**.wbt (Webots World)** ไฟล์เป็น VRML-based format ที่กำหนด:
- 🤖 Robot models และตำแหน่ง
- 🌍 Environment (walls, obstacles, ground)
- 💡 Lights และ camera viewpoint
- ⚙️ Physics parameters
- 🔧 Controller connections

### 📝 วิธี 1: สร้างใหม่ผ่าน GUI (Recommended)

**Step 1: เปิด Webots และ New World**
```
File → New → New World
```

**Step 2: Add Robot**
```
Add → Robot → TurtleBot3
(หรือ DifferentialWheels, E-puck)
```

**Step 3: Position Robot**
```
ใน Scene Tree:
  • คลิกขวา TurtleBot3
  • Edit → Translation/Rotation
  • ปรับตำแหน่ง
```

**Step 4: Add Ground/Obstacles**
```
Add → Solid → Box/Cylinder
  → ตั้งค่า size และ position
  → ทำให้ static (ถ้าต้องการ)
```

**Step 5: Save**
```
File → Save As → my_world.wbt
```

### 💻 วิธี 2: สร้างโดยตรง (Text Editor)

**ไฟล์ .wbt พื้นฐาน:**

```proto
#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/robots/universal_robots/ur/protos/UR10e.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/factory/walls/protos/Wall.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/released/projects/objects/backgrounds/protos/TexturedBackground.proto"

WorldInfo {
  title "My Robot World"
  ERP 0.6
  basicTimeStep 32
  FPS 60
}

Viewpoint {
  position 0 1.5 2
  orientation 0 1 0 1.2
}

TexturedBackground {
  texture "noon"
}

PointLight {
  intensity 1
  location 0 1 0
}

DEF GROUND Solid {
  translation 0 0 0
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.2 0.8 0.2
      }
      geometry Box {
        size 10 0.1 10
      }
    }
  ]
  boundingObject Box {
    size 10 0.1 10
  }
  locked TRUE
}

DEF ROBOT UR10e {
  translation 0 0 0
  controller "my_controller"
  controllerArgs ["arg1", "arg2"]
}

DEF WALL Wall {
  translation 2 0 0
  size 0.1 1 10
}

DEF OBSTACLE Solid {
  translation 1 0.5 1
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.8 0.2 0.2
      }
      geometry Box {
        size 0.5 1 0.5
      }
    }
  ]
  boundingObject Box {
    size 0.5 1 0.5
  }
}
```

### 📊 .wbt File Structure

```
.wbt File Anatomy:

HEADER
├─ #VRML_SIM R2023b utf8    ← Version
└─ EXTERNPROTO              ← External robot definitions

WORLD SETTINGS
├─ WorldInfo {}             ← Physics, timestep
├─ Viewpoint {}             ← Camera view
└─ PointLight/SpotLight     ← Lighting

ENVIRONMENT
├─ TexturedBackground       ← Sky/background
├─ Ground/Walls            ← Static objects
└─ Obstacles               ← Collision objects

ROBOTS & CONTROLLERS
├─ DEF ROBOT {...}         ← Robot instance
├─ controller "name"        ← Points to controller
└─ controllerArgs ["args"]  ← Pass parameters
```

### 🔧 Common .wbt Components

**1. Robot Definition**
```proto
DEF MY_ROBOT Solid {
  translation 0 0.5 0
  rotation 0 0 1 0
  children [
    # Visual shape
    Shape {
      appearance PBRAppearance {
        baseColor 0.1 0.1 0.9
      }
      geometry Box {
        size 0.4 0.4 0.2
      }
    }
  ]
  boundingObject Box {
    size 0.4 0.4 0.2
  }
  physics Physics {}
  controller "001_simple_robot_controller"
}
```

**2. Static Ground**
```proto
DEF GROUND Solid {
  translation 0 0 0
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.5 0.5 0.5
      }
      geometry Box {
        size 100 0.1 100
      }
    }
  ]
  boundingObject Box {
    size 100 0.1 100
  }
  locked TRUE  # ← Can't move!
}
```

**3. Obstacles (Movable Box)**
```proto
DEF OBSTACLE Solid {
  translation 2 0.5 0
  rotation 0 0 1 0.5
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.8 0.2 0.2
        metalness 0.5
        roughness 0.2
      }
      geometry Box {
        size 0.5 1.0 0.5
      }
    }
  ]
  boundingObject Box {
    size 0.5 1.0 0.5
  }
  physics Physics {
    density 500  # kg/m³
  }
}
```

**4. Walls (Simple)**
```proto
DEF WALL_1 Solid {
  translation 0 0.5 5
  children [
    Shape {
      geometry Box {
        size 0.2 1 10
      }
    }
  ]
  boundingObject Box {
    size 0.2 1 10
  }
  locked TRUE
}
```

**5. Lighting**
```proto
# Point Light (omnidirectional)
PointLight {
  intensity 1.0
  location 0 2 0
  radius 100
}

# Spot Light (directional)
SpotLight {
  intensity 0.8
  location 0 3 0
  direction 0 -1 0
  beamWidth 0.5
  cutOffAngle 0.7
}
```

### 📊 Coordinate System & Units

```
Webots Coordinate System:
       Y (Up)
       │  Z (Back)
       │ /
       │/
───────O──── X (Right)

Units: All in METERS (m)

Example Positions:
  translation 0 0 0     ← At origin
  translation 1 0.5 2   ← X=1m, Y=0.5m, Z=2m
  translation -1 0 -1   ← Negative values OK
```

### 🔀 Rotations (Quaternion Format)

```proto
rotation <axis_x> <axis_y> <axis_z> <angle_rad>

Examples:
  rotation 0 1 0 0      ← No rotation
  rotation 0 1 0 1.57   ← 90° around Y-axis
  rotation 1 0 0 3.14   ← 180° around X-axis
  rotation 0 0 1 -1.57  ← -90° around Z-axis
```

### 💡 Tips & Best Practices

**✅ ทำได้:**
- ใช้ DEF ชื่อให้ meaningful
- ตั้งค่า physics สำหรับ dynamic objects
- ใช้ PBRAppearance สำหรับ realistic rendering
- เพิ่ม Comment (#) อธิบาย

**❌ ที่ควรหลีกเลี่ยง:**
```proto
# ❌ DON'T: Impossible settings
locked TRUE              # แต่มี physics!
translation "not_a_number"

# ✅ DO: Clear & correct
locked TRUE              # Static, no physics
physics Physics {}       # Only dynamic objects
```

### 📥 Load External Robot

**Option 1: Use EXTERNPROTO (Recommended)**
```proto
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/clearpath/turtlebot3/protos/Turtlebot3Burger.proto"

DEF ROBOT Turtlebot3Burger {
  translation 0 0 0
  controller "my_controller"
}
```

**Option 2: Reference Local File**
```proto
EXTERNPROTO "../robots/MyRobot.proto"

DEF MY_ROBOT MyRobot {
  translation 0 0 0
  controller "controller_name"
}
```

### 🔍 Debugging .wbt Files

**Problem: Robot not moving?**
```
Check:
1. ✅ controller "name" matches actual file
2. ✅ physics Physics {} exists
3. ✅ locked FALSE or no locked field
```

**Problem: Simulation too slow?**
```
Optimize:
1. Reduce basicTimeStep (32 → 64)
2. Remove unnecessary physics
3. Simplify complex geometries
```

**Problem: Syntax Error?**
```
Use Webots built-in validator:
Tools → View → Scene Tree Inspector
      → Verify all nodes
```

### 📝 Template: Quick World Creation

```proto
#VRML_SIM R2023b utf8

WorldInfo {
  basicTimeStep 32
}

Viewpoint {
  position 0 1 2
  orientation 0 1 0 1.2
}

PointLight {
  intensity 1
  location 0 1 0
}

# Ground
Solid {
  translation 0 0 0
  boundingObject Box { size 10 0.1 10 }
  locked TRUE
}

# Your Robot
DEF ROBOT Solid {
  translation 0 0.5 0
  children [
    Shape {
      geometry Box { size 0.3 0.3 0.3 }
    }
  ]
  boundingObject Box { size 0.3 0.3 0.3 }
  physics Physics {}
  controller "001_simple_robot_controller"
}
```

---

## �📊 Topic Reference

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | Input velocity commands |
| `/robot/odom` | `nav_msgs/Odometry` | Robot odometry output |
| `/tf` | `tf2_msgs/TFMessage` | Transform frames |

---

## 🎓 Learning Outcomes

After completing this tutorial, you will understand:

- ✅ How to create a Webots external controller
- ✅ ROS 2 subscription patterns
- ✅ Differential drive kinematics
- ✅ Motor control in simulation
- ✅ Webots-ROS 2 integration basics

---

## 📚 Related Resources

- 📖 [Webots Documentation](https://cyberbotics.com/doc/)
- 🔗 [ROS 2 Geometry Twist Message](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
- 🤖 [Differential Drive Kinematics](https://en.wikipedia.org/wiki/Differential_wheeled_robot)
- 🔍 [ce_webots Repository](../../)

---

## 📝 Notes

> **Tip:** To modify robot parameters (wheel size, motor specs, etc.), edit the configuration in the world file `001_basic_control.wbt` or the controller script.

> **Common Issues:** 
> - Ensure Webots is running before launching the ROS controller
> - Check that `/cmd_vel` topic is publishing with correct message format

---

## **👤 Authors**

- 🚀 [@alfaXphoori](https://www.github.com/alfaXphoori)

---

<div align="center">

**Made with ❤️ for the ROS 2 Community**

</div>
