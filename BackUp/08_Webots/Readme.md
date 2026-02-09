# 🤖 **Webots Robot Simulator with ROS 2**

## **📌 Project Title**

Webots Robot Simulation Package: ce_webots

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **📋 Overview**

This module demonstrates the integration of Webots robot simulator with ROS 2 Jazzy. The `ce_webots` package provides a complete example of:

- Setting up a ROS 2 package for Webots simulation
- Creating a simple differential drive robot in Webots world files
- Developing Webots controllers to control simulated robots
- Launching Webots simulation from ROS 2 launch files
- Understanding the basic structure for sim-to-real robotics development

The package includes a simple differential drive robot that moves forward for 3 seconds and stops, demonstrating fundamental concepts of robot simulation and control.

---

## **✅ Prerequisites**

- ROS 2 Jazzy Jalisco installed and configured
- Webots R2025a or later installed
- Ubuntu 24.04 LTS (recommended)
- VS Code with ROS extensions (optional)
- Basic understanding of Python 3
- Familiarity with ROS 2 package structure

---

## **� Package Structure**

The `ce_webots` package follows standard ROS 2 Python package structure:

```
ce_webots/
├── package.xml              # Package manifest with dependencies
├── setup.py                 # Python package setup and entry points
├── setup.cfg                # Installation configuration
├── resource/
│   └── ce_webots           # Package marker file
├── ce_webots/
│   └── __init__.py         # Python package marker
├── controllers/
│   └── robot_controller/
│       └── robot_controller.py  # Webots controller script
├── launch/
│   └── robot_launch.py     # ROS 2 launch file for simulation
├── worlds/
│   └── simple_robot.wbt    # Webots world file with robot model
└── test/
    ├── test_copyright.py   # Copyright test
    ├── test_flake8.py      # Code style test
    └── test_pep257.py      # Docstring test
```

---

## **📊 Architecture Diagram**

### **System Overview**

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Host System (Ubuntu 24.04)                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │            Webots Simulator (R2025a)                       │   │
│  │  ┌──────────────────────────────────────────────────────┐  │   │
│  │  │  Physics Engine (ODE)                                │  │   │
│  │  │  • Timestep: 16ms (62.5 Hz)                          │  │   │
│  │  │  • Gravity, friction, collisions                     │  │   │
│  │  └──────────────────────────────────────────────────────┘  │   │
│  │                          ↓↑                                │   │
│  │  ┌──────────────────────────────────────────────────────┐  │   │
│  │  │  simple_robot.wbt (World File)                       │  │   │
│  │  │  ┌────────────────────────────────────────────────┐  │  │   │
│  │  │  │  Robot (Differential Drive)                    │  │  │   │
│  │  │  │  ├─ Body: 0.2m × 0.1m × 0.05m                  │  │  │   │
│  │  │  │  ├─ Left Motor  (RotationalMotor)              │  │  │   │
│  │  │  │  ├─ Right Motor (RotationalMotor)              │  │  │   │
│  │  │  │  ├─ Left Wheel  (0.04m radius)                 │  │  │   │
│  │  │  │  ├─ Right Wheel (0.04m radius)                 │  │  │   │
│  │  │  │  └─ Ball Caster (rear support)                 │  │  │   │
│  │  │  └────────────────────────────────────────────────┘  │  │   │
│  │  │                                                      │  │   │
│  │  │  Environment: 2m × 2m Arena                          │  │   │
│  │  └──────────────────────────────────────────────────────┘  │   │
│  │                          ↓↑                                │   │
│  │  ┌──────────────────────────────────────────────────────┐  │   │
│  │  │  robot_controller.py (Webots Controller)             │  │   │
│  │  │  • Python script running inside Webots               │  │   │
│  │  │  • Reads sensor values                               │  │   │
│  │  │  • Sends motor commands                              │  │   │
│  │  │  • Main control loop (timestep sync)                 │  │   │
│  │  └──────────────────────────────────────────────────────┘  │   │
│  └────────────────────────────────────────────────────────────┘   │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐   │
│  │              ROS 2 Jazzy Middleware Layer                  │   │
│  │  ┌──────────────────────────────────────────────────────┐  │   │
│  │  │  robot_launch.py (Launch File)                       │  │   │
│  │  │  • Starts Webots with world file                     │  │   │
│  │  │  • ExecuteProcess action                             │  │   │
│  │  │  • Mode: realtime                                    │  │   │
│  │  └──────────────────────────────────────────────────────┘  │   │
│  └────────────────────────────────────────────────────────────┘   │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

### **File Installation Flow**

```
┌──────────────────────────────────────────────────────────────┐
│         Package Build & Installation Process                 │
└──────────────────────────────────────────────────────────────┘

Source Tree                    Installed Location
━━━━━━━━━━━                    ━━━━━━━━━━━━━━━━━━

ce_webots/
├── package.xml       ─────→  install/ce_webots/share/ce_webots/
│                              package.xml
│
├── setup.py          ─────→  (Used during build, not installed)
│
├── launch/
│   └── robot_launch.py ───→  install/ce_webots/share/ce_webots/
│                              launch/robot_launch.py
│
├── worlds/
│   └── simple_robot.wbt ──→  install/ce_webots/share/ce_webots/
│                              worlds/simple_robot.wbt
│
├── controllers/
│   └── robot_controller/
│       └── robot_        ──→  install/ce_webots/share/ce_webots/
│          controller.py       controllers/robot_controller/
│                              robot_controller.py
│
└── resource/
    └── ce_webots      ─────→  install/ce_webots/share/ament_index/
                               resource_index/packages/ce_webots



---

## **🎯 Key Components**

### **1. Package Configuration**

#### **package.xml**
Defines package dependencies:
- `rclpy` - ROS 2 Python client library
- `geometry_msgs` - Standard geometry message types
- Build type: `ament_python`

#### **setup.py**
Configures installation of:
- Launch files to `share/ce_webots/launch/`
- World files to `share/ce_webots/worlds/`
- Controllers to `share/ce_webots/controllers/robot_controller/`

### **2. Robot Controller**

[robot_controller.py](08_Webots/src/ce_webots/controllers/robot_controller/robot_controller.py) is a Webots Python controller that:

**Key Features:**
- Initializes the Webots Robot API
- Gets timestep from world file (16ms)
- Accesses left and right motors by device name
- Sets motors to velocity control mode (position = infinity)
- Implements simple timed behavior:
  - Moves forward at 2.0 rad/s for 3 seconds
  - Stops after elapsed time
  
**Code Structure:**
```python
from controller import Robot

def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    left_motor = robot.getDevice('left_motor')
    right_motor = robot.getDevice('right_motor')
    
    left_motor.setPosition(float('inf'))  # Velocity control
    right_motor.setPosition(float('inf'))
    
    # Control loop runs until simulation stops
    while robot.step(timestep) != -1:
        # Motor control logic here
```

### **3. Webots World File**

[simple_robot.wbt](08_Webots/src/ce_webots/worlds/simple_robot.wbt) defines the simulation environment:

**World Components:**
- **WorldInfo**: Basic timestep = 16ms (62.5 Hz physics update)
- **TexturedBackground**: Skybox and lighting
- **RectangleArena**: 2m × 2m floor boundary
- **Robot**: Differential drive robot

**Robot Structure:**
```
Robot
├── BODY: 0.2m × 0.1m × 0.05m blue box
├── WHEEL_LEFT: HingeJoint with RotationalMotor
│   ├── Motor name: "left_motor"
│   ├── Radius: 0.04m, Height: 0.02m
│   └── Position: (0.05, 0.06, 0) from body
├── WHEEL_RIGHT: HingeJoint with RotationalMotor
│   ├── Motor name: "right_motor"
│   ├── Radius: 0.04m, Height: 0.02m
│   └── Position: (0.05, -0.06, 0) from body
└── BALL_WHEEL: Passive caster
    ├── Sphere radius: 0.02m
    ├── Mass: 0.05kg
    └── Position: (-0.05, 0, -0.02) rear center
```

**Physics Properties:**
- Robot body mass: 1 kg
- Wheel physics enabled (friction, inertia)
- Ball caster for stability

### **4. Launch File**

[robot_launch.py](08_Webots/src/ce_webots/launch/robot_launch.py) launches Webots with the world file:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('ce_webots')
    world_file = os.path.join(package_dir, 'worlds', 'simple_robot.wbt')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['webots', '--mode=realtime', world_file],
            output='screen'
        )
    ])
```

**Launch Parameters:**
- `--mode=realtime`: Runs simulation at real-time speed
- World file path: Retrieved from installed package location
- Output to screen for debugging

---

## **🛠 Installation and Setup**

### **Step 1: Install Webots**

Download and install Webots R2025a or later:

```bash
# Method 1: Snap (Recommended for Ubuntu)
sudo snap install webots

# Method 2: Official Website
# Visit https://cyberbotics.com/
# Download and install Webots R2025a for your platform
# Webots is automatically added to PATH by snap
# No additional configuration needed
```

**For .deb Package Installation:**
```bash
# Add to ~/.bashrc
export WEBOTS_HOME=/usr/local/webots
export PATH=$WEBOTS_HOME:$PATH
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH
```

# Verify Webots installation
webots --version
```

**Expected output:** `Webots R2025a` or later

### **Step 2: Build the Package**

Navigate to your ROS 2 workspace and build the ce_webots package:

```bash
# Navigate to workspace root (adjust path if needed)
cd ~/ros2_ws

# Build the ce_webots package
colcon build --packages-select ce_webots

# Source the workspace
source install/setup.bash
```

**Expected output:**
```
Starting >>> ce_webots
Finished <<< ce_webots [0.50s]

Summary: 1 package finished [0.70s]
```

### **Step 3: Verify Package Installation
    
    # Distance sensor (front)
    DistanceSensor {
      translation 0.15 0 0
      rotation 0 0 1 0
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 1 0 0
            metalness 0
          }
          geometry Cylinder {
            height 0.02
            radius 0.01
          }
        }
      ]
      name "front_sensor"
      lookupTable [
        0 0 0
        0.5 500 0
        1.0 1000 0
      ]
      type "infra-red"
      numberOfRays 1
    }
  ]
  boundingObject Box {
    size 0.3 0.2 0.1
  }
  physics Physics {
    density -1
    mass 1
  }
  controller "my_robot_controller"
  supervisor FALSE
}

# Add an obstacle
Solid {
  translation 1 0 0.1
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.8 0.2 0.2
        metalness 0
      }
      geometry Box {
        size 0.3 0.3 0.2
      }
    }
  ]
  boundingObject Box {
    size 0.3 0.3 0.2
  }
}
```

**What this world contains:**
- ✅ Differential drive robot (2 wheels)
- ✅ Two motors: `left_motor`, `right_motor`
- ✅ Two encoders: `left_encoder`, `right_encoder`
- ✅ Distance sensor: `front_sensor`
- ✅ 5m x 5m arena with walls
- ✅ One obstacle for testing

### **3.2 Robot Proto Description (Optional)**

Create a custom robot proto file for reusable robot definitions:

```bash
cd ~/ros2_ws/src/ce_robot/protos
touch my_robot.proto
```

Paste this PROTO definition:

```
#VRML_SIM R2025a utf8

PROTO MyRobot [
  field SFVec3f translation 0 0 0
  field SFRotation rotation 0 0 1 0
]
{
  Robot {
    translation IS translation
    rotation IS rotation
    children [
      Shape {
        geometry Box { size 0.2 0.2 0.1 }
      }
      HingeJoint {
        jointParameters HingeJointParameters {
          axis 0 1 0
          anchor 0 0 0
        }
        device RotationalMotor { name "motor1" }
        endPoint Solid {
          translation 0.15 0 0
          geometry Sphere { radius 0.05 }
        }
      }
    ]
    controller "robot_controller"
  }
}
```

---

## **💻 Step 4: Create Robot Controller with ROS 2 Integration**

### **4.1 Complete Differential Drive Controller**

**IMPORTANT:** Webots requires controllers to be in a directory with the same name as the controller.

Create the controller directory and file:

```bash
cd ~/ros2_ws/src/ce_robot/controllers
mkdir -p my_robot_controller
cd my_robot_controller
touch my_robot_controller.py
chmod +x my_robot_controller.py
```

**Directory structure should be:**
```
controllers/
└── my_robot_controller/
    └── my_robot_controller.py
```

Edit `controllers/my_robot_controller/my_robot_controller.py`:

```python
#!/usr/bin/env python3
"""
Webots Robot Controller with ROS 2 Integration
Reads distance sensor and controls differential drive motors
"""

from controller import Robot, Motor, DistanceSensor, PositionSensor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import math

class WebotsRobotController(Node):
    def __init__(self):
        super().__init__('webots_robot_controller')
        
        # Initialize Webots Robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Get motor devices
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # Set motors to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Get encoder sensors

Check that the package was installed correctly:

```bash
# List package files
ros2 pkg prefix ce_webots

# Check for launch files
ls $(ros2 pkg prefix ce_webots)/share/ce_webots/launch/

# Check for world files
ls $(ros2 pkg prefix ce_webots)/share/ce_webots/worlds/

# Check for controllers
ls $(ros2 pkg prefix ce_webots)/share/ce_webots/controllers/
```

---

## **🚀 Running the Simulation**

### **Launch Webots with the Robot**

Use the provided launch file to start Webots:

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch Webots with the simple robot world
ros2 launch ce_webots robot_launch.py
```

**What happens:**
1. Webots simulator launches in realtime mode
2. The `simple_robot.wbt` world file loads
3. The differential drive robot appears in the arena
4. The `robot_controller` automatically starts
5. Robot moves forward at 2.0 rad/s for 3 seconds
6. Robot stops after 3 seconds
7. Console shows controller output

**Expected Console Output:**
```
Robot controller started!
Moving forward for 3 seconds...
Robot stopped.
```

**Controls in Webots:**
- **Mouse drag**: Rotate view
- **Mouse wheel**: Zoom in/out
- **Right-click drag**: Pan view
- **Play button**: Start/pause simulation
- **Reset button**: Reset simulation to initial state

---

## **📚 Understanding the Code**

### **Robot Controller Breakdown**

The [robot_controller.py](08_Webots/src/ce_webots/controllers/robot_controller/robot_controller.py) demonstrates:

**1. Robot Initialization**
```python
robot = Robot()
timestep = int(robot.getBasicTimeStep())
```
- Creates Robot instance (Webots API)
- Gets simulation timestep from world file (16ms)

**2. Device Access**
```python
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
```
- Accesses devices by name (must match world file)
- Device names defined in VRML structure

**3. Velocity Control Mode**
```python
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
```
- `float('inf')` enables velocity control
- Without this, motors use position control

**4. Main Control Loop**
```python
while robot.step(timestep) != -1:
    # Control logic
```
- `robot.step()` advances simulation by one timestep
- Returns -1 when simulation terminates
- Must be called to update sensor readings

**5. Motor Commands**
```python
left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)
```
- Sets angular velocity in rad/s
- Positive = forward, negative = backward

### **World File Breakdown**

The [simple_robot.wbt](08_Webots/src/ce_webots/worlds/simple_robot.wbt) defines:

**1. World Properties**
```vrml
WorldInfo {
  basicTimeStep 16
}
```
- Physics update rate: 16ms (62.5 Hz)
- Affects simulation accuracy and speed

**2. Robot Structure**
```vrml
Robot {
  translation 0 0 0.05
  controller "robot_controller"
}
```
- Initial position: (0, 0, 0.05) meters
- Links to controller by name

**3. HingeJoint Motors**
```vrml
HingeJoint {
  device [
    RotationalMotor {
      name "left_motor"
    }
  ]
}
```
- Creates rotational joint with motor
- Motor name accessible in controller

**4. Physics Properties**
```vrml
physics Physics {
  density -1
  mass 1
}
```
- `density -1` means use explicit mass
- Mass affects inertia and momentum

---

## **🔧 Customization and Experiments**

### **Experiment 1: Change Robot Behavior**

Modify the controller to move in different patterns:

**Circle Motion:**
```python
# In robot_controller.py
left_motor.setVelocity(1.5)   # Slower left wheel
right_motor.setVelocity(2.5)  # Faster right wheel
```

**Back and Forth:**
```python
if time_counter < 3000:
    left_motor.setVelocity(2.0)
    right_motor.setVelocity(2.0)
elif time_counter < 6000:
    left_motor.setVelocity(-2.0)
    right_motor.setVelocity(-2.0)
else:
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
```

### **Experiment 2: Modify Robot Dimensions**

Edit [simple_robot.wbt](08_Webots/src/ce_webots/worlds/simple_robot.wbt):

**Larger Robot Body:**
```vrml
geometry Box {
  size 0.3 0.15 0.08  # Increased from 0.2 0.1 0.05
}
```

**Bigger Wheels:**
```vrml
geometry Cylinder {
  height 0.04
  radius 0.06  # Increased from 0.04
}
```

### **Experiment 3: Change Arena Size**

```vrml
RectangleArena {
  floorSize 4 4  # Increased from 2 2
}
```

---

## **🐛 Troubleshooting**

### **Issue: Webots doesn't start**

**Solution:**
```bash
# Check if Webots is installed
which webots

# Try running directly
webots --version

# If snap installation, check status
snap list | grep webots
```

### **Issue: Controller not found**

**Error:** `Warning: "robot_controller" controller not found`

**Solution:**
1. Check controller name matches in world file and directory
2. Verify controller file exists:
   ```bash
   ls $(ros2 pkg prefix ce_webots)/share/ce_webots/controllers/robot_controller/
   ```
3. Rebuild package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ce_webots --symlink-install
   ```

### **Issue: Robot falls through floor**

**Solution:**
- Add `boundingObject` to robot body in world file
- Add `physics Physics {}` to robot
- Check robot is positioned above floor (z > 0)

### **Issue: Motors don't respond**

**Solution:**
1. Verify device names match:
   - World file: `name "left_motor"`
   - Controller: `robot.getDevice('left_motor')`
2. Check velocity control mode is set:
   ```python
   motor.setPosition(float('inf'))
   ```
3. Ensure control loop is running:
   ```python
   while robot.step(timestep) != -1:
   ```

---

## **📖 Key Concepts Summary**

### **Webots Fundamentals**

| Concept | Description |
|---------|-------------|
| **World File (.wbt)** | VRML file defining environment, robots, and objects |
| **Controller** | Python/C++ script controlling robot behavior |
| **Device** | Robot component (motor, sensor) accessed by name |
| **Timestep** | Simulation update interval (ms) |
| **Physics** | Enables realistic physical simulation |

### **ROS 2 Integration Patterns**

| Pattern | Purpose |
|---------|---------|
| **Pure Webots Controller** | Webots-only code (this example) |
| **ROS 2 Node + Webots** | Webots driver publishes to ROS 2 topics |
| **webots_ros2_driver** | Official bridge for full integration |
| **Launch File** | Start Webots and ROS 2 nodes together |

### **Robot Components**

| Component | World File | Controller Access |
|-----------|------------|-------------------|
| Motor | `RotationalMotor { name "..." }` | `robot.getDevice('...')` |
| Sensor | `PositionSensor { name "..." }` | `robot.getDevice('...').enable()` |
| Camera | `Camera { name "..." }` | `robot.getDevice('...').enable()` |
| LiDAR | `Lidar { name "..." }` | `robot.getDevice('...').enable()` |

---

## **🎓 Next Steps**

1. **Add Sensors**: Modify world file to include distance sensors or cameras
2. **ROS 2 Integration**: Create ROS 2 nodes that communicate with Webots
3. **Teleoperation**: Add keyboard/joystick control via ROS 2 topics
4. **Navigation**: Implement obstacle avoidance using sensor data
5. **Multi-Robot**: Add multiple robots to the simulation
6. **Advanced Controllers**: Use PID control for smooth motion

---

## **📚 Additional Resources**

- [Webots Documentation](https://cyberbotics.com/doc/guide/index)
- [Webots ROS 2 Interface](https://github.com/cyberbotics/webots_ros2)
- [VRML97 Reference](https://www.web3d.org/documents/specifications/14772/V2.0/)
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)

---

## **✅ Learning Outcomes**

After completing this module, you have:

- ✅ Set up a complete ROS 2 package for Webots simulation
- ✅ Created a differential drive robot in a Webots world file
- ✅ Developed a Python controller for robot behavior
- ✅ Launched Webots from ROS 2 launch files
- ✅ Understood robot-simulator communication patterns
- ✅ Applied basic physics principles in virtual environments

**Next Module:** Advanced ROS 2 + Webots integration with topics and services

---
    
    # Get launch configurations
    world_file = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    
    # Start Webots simulator
    webots_process = ExecuteProcess(
        cmd=['webots', '--mode=realtime', world_file],
        output='screen',
        shell=False
    )
    
    # The robot controller is automatically started by Webots
    # based on the 'controller' field in the Robot definition in the .wbt file
    # No need to launch it as a separate ROS 2 node here
    
    return LaunchDescription([
        world_file_arg,
        mode_arg,
        webots_process,
    ])
```

**Launch File Features:**
- ✅ **Configurable World File** - Pass world file as argument
- ✅ **Simulation Mode** - Choose realtime, fast, or pause
- ✅ **Auto-start Controller** - Webots starts the controller specified in the .wbt file
- ✅ **Simplified Architecture** - Controller runs inside Webots process

---

## **🔧 Step 6: Build and Configure Package**

### **6.1 Update setup.py**

Edit `~/ros2_ws/src/ce_robot/setup.py` to include new files:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'ce_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*_launch.py')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.wbt')),
        # Install controller files (including subdirectories)
        (os.path.join('share', package_name, 'controllers'), 
            glob('controllers/*/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@ksu.ac.th',
    description='CE Robot ROS 2 Webots Integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Webots controllers are NOT ROS 2 executables
            # They are run by Webots itself from the .wbt file
        ],
    },
)
```

### **6.2 Build the Package**

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
```

**Expected Output:**
```
Starting >>> ce_robot
Finished <<< ce_robot [2.5s]

Summary: 1 package finished [2.6s]
```

---

## **✅ Step 7: Launch and Test Simulation**

### **7.1 Launch Webots Simulation**

**Terminal 1 - Start Simulation:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ce_robot webots_sim_launch.py
```

**Expected Output:**
```
[INFO] [webots_controller]: 🤖 Webots Robot Controller initialized
[INFO] [webots_controller]:    Timestep: 32ms
[INFO] [webots_controller]:    Wheel radius: 0.05m
[INFO] [webots_controller]:    Wheel separation: 0.26m
[INFO] [webots_controller]: 🚀 Starting robot control loop...
```

You should see:
- ✅ Webots window opens with the robot in the arena
- ✅ Robot controller node starts successfully
- ✅ ROS 2 topics are publishing

### **7.2 Verify ROS 2 Integration**

**Terminal 2 - Check Topics:**
```bash
ros2 topic list
```

**Expected Topics:**
```
/cmd_vel           # Command velocity input
/distance_sensor   # Distance sensor data
/odom              # Odometry output
/parameter_events
/rosout
```

### **7.3 Test Robot Control**

**Terminal 3 - Send Movement Commands:**

```bash
# Move forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Rotate in place at 1.0 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"

# Move in a circle
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### **7.4 Monitor Sensor Data**

**Terminal 4 - Watch Sensors:**

```bash
# Monitor distance sensor (watch for obstacles)
ros2 topic echo /distance_sensor

# Monitor robot position (odometry)
ros2 topic echo /odom
```

### **7.5 Visualize System**

**Use ROS 2 Visualization Tools:**

```bash
# View node/topic graph
rqt_graph

# Plot sensor data over time
rqt_plot /distance_sensor/range

# 3D visualization (requires additional configuration)
rviz2
```

---

## **🎯 Step 8: Practice Exercises**

### **Exercise 1: Wall Following**

Create a simple wall-following behavior:

```python
# Add to controller
def wall_follow_callback(self):
    distance = self.distance_sensor.getValue()
    
    if distance < 300:  # Too close to wall
        # Turn away from wall
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = -0.5
    elif distance > 700:  # Too far from wall
        # Turn towards wall
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.3
    else:  # Good distance
        # Move straight
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0
```

### **Exercise 2: Obstacle Avoidance**

Implement simple obstacle avoidance:

```python
def obstacle_avoidance(self):
    distance = self.distance_sensor.getValue()
    
    twist = Twist()
    
    if distance < 400:  # Obstacle ahead
        self.get_logger().warn('⚠️  Obstacle! Turning...')
        twist.linear.x = 0.0
        twist.angular.z = 1.0  # Turn in place
    else:
        twist.linear.x = 0.5  # Move forward
        twist.angular.z = 0.0
    
    # Publish to cmd_vel
    self.cmd_vel_pub.publish(twist)
```

### **Exercise 3: Add More Sensors**

Modify the world file to add a camera or LiDAR sensor, then publish the data to ROS 2 topics.

---

## **📝 Key Concepts**

### **Simulation vs Reality Comparison**

| Aspect | Simulation (Webots) | Real Robot |
|--------|-------------------|-----------|
| **Development Speed** | 100x faster | 1x (reference) |
| **Safety** | Complete (no risks) | Risk of damage/injury |
| **Physics** | Realistic simulation | Ground truth |
| **Sensor Noise** | Configurable & consistent | Unpredictable variation |
| **Debugging** | Easy inspection | Difficult observation |
| **Cost** | Free software | Hardware expensive |
| **Reliability** | 100% repeatable | Variable conditions |

### **Webots Architecture Components**

- **Physics Engine** - Realistic physics simulation (ODE-based)
- **Rendering Engine** - Real-time 3D visualization
- **Controllers** - C++/Python/Java robot control programs
- **Sensors** - Simulated camera, LiDAR, IMU, encoders, touch sensors
- **Actuators** - Moinstallation fails with "Unable to locate package"**
**Cause:** Repository not configured correctly or package not available for Ubuntu 24.04
**Solution:**
```bash
# Use snap installation instead
sudo snap install webots

# Or download .deb from official site
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt install -y ./webots_2025a_amd64.deb
```
**Prevention:** Use snap for Ubuntu 24.04 (Noble) - it's the most reliable method

### **Issue: Webots simulator crashes on launch**
**Cause:** GPU acceleration not available or OpenGL issues
**Solution:**
```bash
# Check Webots version and help
webots --version
webots --help

# Run with debug output
webots --stdout --stderr my_world.wbt

# Try software rendering mode
webots --mode=fast my_world.wbt
```
**Prevention:** Ensure graphics drivers are installed and use headless mode for servers

### **Issue: "GPG key error" or repository 404 error**
**Cause:** Old apt repository instructions using deprecated focal release
**Solution:**
```bash
# Remove old repository if added
sudo rm /etc/apt/sources.list.d/webots.list

# Use snap installation instead
sudo snap install webots
```
**Prevention:** Use snap installation for modern Ubuntu versions (22.04+)
- **Parameter Server** - Dynamic simulation parameter adjustment
- **Timestep Synchronization** - Deterministic simulation-ROS 2 timing

### **Sim2Real Transfer Principles**

1. **Physics Accuracy** - Match simulation parameters to real robot
2. **Sensor Realism** - Add noise and delays matching real sensors
3. **Timing Consistency** - Maintain real-time constraints
4. **Gradual Complexity** - Test simple behaviors first, then complex
5. **Domain Randomization** - Vary simulation parameters to improve robustness

---

## **⚠️ Troubleshooting**

### **Issue: Build fails with "NameError: name 'os' is not defined"**
**Cause:** Missing `import os` in setup.py file
**Solution:**
```bash
# Edit setup.py and ensure these imports are at the top:
cd ~/ros2_ws/src/ce_robot
nano setup.py
```
Make sure the file starts with:
```python
from setuptools import setup
import os
from glob import glob
```
Then rebuild:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```
**Prevention:** Always verify imports when creating setup.py files

### **Issue: Launch fails with "could not open file" and "executable not found"**
**Cause:** World file doesn't exist or controller executable not built properly
**Solution:**
```bash
# 1. Verify the world file exists
ls ~/ros2_ws/src/ce_robot/worlds/my_first_robot.wbt

# 2. If missing, create it following Step 3.1 instructions

# 3. Verify controller file exists
ls ~/ros2_ws/src/ce_robot/controllers/my_robot_controller.py

# 4. Check setup.py has correct entry_points:
grep -A 5 "entry_points" ~/ros2_ws/src/ce_robot/setup.py

# 5. Rebuild with verbose output
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install --event-handlers console_direct+

# 6. Verify installation
ls install/ce_robot/lib/ce_robot/
ls install/ce_robot/share/ce_robot/worlds/

# 7. Try launching with absolute path
ros2 launch ce_robot webots_sim_launch.py world:=/home/admin/ros2_ws/src/ce_robot/worlds/my_first_robot.wbt
```
**Prevention:** Always create all required files before building and ensure setup.py correctly installs them

### **Issue: "Could not find the controller directory" - Webots uses generic controller**
**Cause:** Webots expects controllers to be in a directory with the same name as the controller file
**Solution:**
```bash
# Correct directory structure:
cd ~/ros2_ws/src/ce_robot/controllers

# If you have: controllers/my_robot_controller.py (WRONG)
# Move it to: controllers/my_robot_controller/my_robot_controller.py (CORRECT)

# Fix the structure:
mkdir -p my_robot_controller
mv my_robot_controller.py my_robot_controller/

# Verify structure:
ls -la my_robot_controller/
# Should show: my_robot_controller.py

# Update setup.py to use 'controllers/*/*.py' pattern instead of 'controllers/*.py'
# Then rebuild:
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```
**Prevention:** Always create controllers in their own directory: `controllers/controller_name/controller_name.py`

### **Issue: WARNING about duplicate 'solid' names in world file**
**Cause:** Multiple Solid nodes in the world file have the same default name
**Solution:**
Edit the world file and give unique names to each wheel solid:
```bash
cd ~/ros2_ws/src/ce_robot/worlds
nano my_first_robot.wbt
```
Find the left wheel endPoint Solid and add `name "left_wheel_solid"`:
```vrml
endPoint Solid {
  name "left_wheel_solid"
  translation -0.15 -0.13 0
  ...
}
```
Find the right wheel endPoint Solid and add `name "right_wheel_solid"`:
```vrml
endPoint Solid {
  name "right_wheel_solid"
  translation -0.15 0.13 0
  ...
}
```
**Prevention:** Always give unique names to all Solid nodes in your world file

### **Issue: "ModuleNotFoundError: No module named 'rclpy'" in controller**
**Cause:** Webots is using system Python instead of ROS 2 Python environment with rclpy installed
**Solution:**

**Option 1: Set WEBOTS_PYTHON environment variable (Recommended)**
```bash
# Find your ROS 2 Python path
which python3
# Should show: /usr/bin/python3 or similar

# Add to ~/.bashrc to make permanent
echo 'export WEBOTS_PYTHON_EXECUTABLE=/usr/bin/python3' >> ~/.bashrc
source ~/.bashrc

# Or set temporarily for current session
export WEBOTS_PYTHON_EXECUTABLE=/usr/bin/python3

# Verify ROS 2 packages are available
python3 -c "import rclpy; print('rclpy found!')"
```

**Option 2: Install ROS 2 packages in system Python**
```bash
# Install rclpy and other ROS 2 packages for system Python
pip3 install rclpy geometry-msgs sensor-msgs nav-msgs

# Or use ROS 2 sourced environment
source /opt/ros/jazzy/setup.bash
```

**Option 3: Add ROS 2 setup to controller script**
Add this at the top of your controller file (before imports):
```python
#!/usr/bin/env python3
import sys
import os

# Add ROS 2 Python packages to path
ros_python_path = '/opt/ros/jazzy/lib/python3.12/site-packages'
if ros_python_path not in sys.path:
    sys.path.insert(0, ros_python_path)

# Now import ROS 2 packages
import rclpy
from rclpy.node import Node
# ... rest of imports
```

After applying the fix, restart Webots:
```bash
# Close Webots completely
pkill webots

# Launch again
ros2 launch ce_robot webots_sim_launch.py
```

**Prevention:** Always ensure Webots uses Python environment with ROS 2 packages installed, or set WEBOTS_PYTHON_EXECUTABLE

### **Issue: "ModuleNotFoundError: No module named 'ce_robot.controllers'"**
**Cause:** The setup.py incorrectly tries to create a ROS 2 executable from the Webots controller. Webots controllers are executed BY Webots itself, not as standalone ROS 2 nodes.
**Solution:**

**IMPORTANT:** Remove the controller from entry_points in setup.py. The correct setup.py should be:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'ce_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*_launch.py')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.wbt')),
        # Install controller files (for Webots to find)
        (os.path.join('share', package_name, 'controllers'), 
            glob('controllers/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@ksu.ac.th',
    description='CE Robot ROS 2 Webots Integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # DO NOT add Webots controllers here!
            # Controllers are executed by Webots, not as ROS 2 executables
        ],
    },
)
```

Then update the launch file to NOT start the controller as a ROS 2 node:

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare arguments
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.expanduser('~/ros2_ws/src/ce_robot/worlds/my_first_robot.wbt'),
        description='Full path to Webots world file'
    )
    
    world_file = LaunchConfiguration('world')
    
    # Start Webots simulator (controller starts automatically from .wbt file)
    webots_process = ExecuteProcess(
        cmd=['webots', '--mode=realtime', world_file],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        world_file_arg,
        webots_process,
    ])
```

Rebuild and test:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
source install/setup.bash
ros2 launch ce_robot webots_sim_launch.py
```

**Prevention:** Webots controllers are embedded Python scripts run BY the Webots simulator, not standalone ROS 2 executables. The controller field in the Robot definition in the .wbt file tells Webots which script to run.

### **Issue: Webots simulator crashes on launch**
**Cause:** GPU acceleration not available or OpenGL issues
**Solution:**
```bash
webots --help
webots --stdout --stderr my_world.wbt
```
**Prevention:** Use headless mode for server deployments

### **Issue: ROS 2 nodes cannot connect to Webots**
**Cause:** ROS 2 middleware not properly initialized or topic mismatch
**Solution:**
```bash
export ROS_DOMAIN_ID=0
ros2 daemon status
ros2 topic list
```
**Prevention:** Ensure both Webots and ROS 2 use same domain ID

### **Issue: Sensor data shows unrealistic values**
**Cause:** Sensor simulation parameters incorrect (range, resolution)
**Solution:** Check world file sensor configuration:
```bash
grep -A 5 "DistanceSensor" my_world.wbt
```
**Prevention:** Validate sensor parameters against real sensor specifications

### **Issue: Simulation runs too fast or too slow**
**Cause:** Timestep mismatch or CPU throttling
**Solution:**
```bash
# Check physics timestep in world file
grep "basicTimeStep" my_world.wbt
# Should be 32ms (0.032s) for typical robots
```
**Prevention:** Use consistent timestep across all simulation configurations

---

## **📚 Resources**

- [Webots Official Documentation](https://www.cyberbotics.com/doc) - Complete reference
- [Webots ROS 2 Integration](https://github.com/cyberbotics/webots_ros2) - Official ROS 2 bridge
- [Webots Tutorials](https://www.cyberbotics.com/doc/guide/tutorials) - Step-by-step guides
- [ROS 2 Simulation Guide](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots.html) - ROS 2 integration guide
- [VRML Reference](https://www.web3d.org/vrml97-amendments) - World file format specification

---

## **✅ Verification Checklist**

Your Webots simulation lab is complete when you can verify:

**Installation & Setup:**
- [ ] Webots simulator installed: `webots --version` shows version
- [ ] ROS 2 Webots bridge installed: `ros2 pkg list | grep webots`
- [ ] Environment variables set in `~/.bashrc`
- [ ] Workspace directories created: `worlds/`, `controllers/`, `launch/`

**World & Robot:**
- [ ] World file created: `worlds/my_first_robot.wbt`
- [ ] World file loads in Webots without errors
- [ ] Robot appears in simulation with 2 wheels and 1 sensor
- [ ] Robot has proper physics (not floating or falling through floor)

**Controller & ROS 2:**
- [ ] Controller created: `controllers/my_robot_controller.py`
- [ ] Controller is executable: `chmod +x`
- [ ] Package builds successfully: `colcon build`
- [ ] Launch file created: `launch/webots_sim_launch.py`
- [ ] Launch file starts both Webots and controller

**ROS 2 Integration:**
- [ ] Topics visible: `ros2 topic list` shows `/cmd_vel`, `/distance_sensor`, `/odom`
- [ ] Controller subscribes to `/cmd_vel` successfully
- [ ] Distance sensor publishes to `/distance_sensor`
- [ ] Odometry publishes to `/odom`
- [ ] `rqt_graph` shows proper node connections

**Robot Control:**
- [ ] Robot moves forward with: `ros2 topic pub /cmd_vel ...`
- [ ] Robot rotates with angular velocity commands
- [ ] Robot stops when commanded
- [ ] Distance sensor detects obstacle when robot approaches it
- [ ] Odometry updates as robot moves

**Advanced Features:**
- [ ] Can monitor sensor data: `ros2 topic echo /distance_sensor`
- [ ] Can visualize topics: `rqt_plot /distance_sensor/range`
- [ ] Simulation runs at consistent 32ms timestep
- [ ] No lag or crashes during operation

---

## **🚀 Next Steps & Advanced Topics**

After mastering Webots simulation basics, expand your skills:

### **Immediate Next Steps:**

1. **Add More Sensors**
   - Camera sensor for vision processing
   - LiDAR for mapping and navigation
   - IMU for orientation tracking
   - GPS for absolute positioning

2. **Implement Autonomous Behaviors**
   - Obstacle avoidance algorithm
   - Wall following behavior
   - Goal-seeking navigation
   - Simple SLAM implementation

3. **Multi-Robot Simulation**
   - Add multiple robots to world file
   - Implement robot-to-robot communication
   - Coordinate fleet behavior
   - Test formation control

### **Advanced Modules:**

1. **09_Hardware_Integration** - Connect real sensors and actuators
   - GPIO control for motors and sensors
   - I2C/SPI communication protocols
   - Real-time performance tuning
   - Bridge simulation to real hardware

2. **10_Navigation_Stack** - Implement ROS 2 Nav2
   - Map building with SLAM
   - Path planning algorithms
   - Costmap configuration
   - Recovery behaviors

3. **11_Real_Robot_Deployment** - Deploy to physical robots
   - Sim2real transfer techniques
   - Handle real-world sensor noise
   - Safety and error recovery
   - Performance optimization

### **Practice Projects:**

**Project 1: Warehouse Robot**
- Navigate between waypoints
- Avoid dynamic obstacles
- Report battery status
- Return to charging station

**Project 2: Line Following Robot**
- Add color sensor to world
- Detect line on floor
- Follow path autonomously
- Handle intersections

**Project 3: Mapping Robot**
- Add LiDAR sensor
- Build map of environment
- Localize within map
- Navigate to goals

---

**✅ Simulation Foundation Complete! Ready for Advanced Robotics!** 🚀🤖✨

**Key Achievements:**
- ✅ Webots simulation environment configured
- ✅ Differential drive robot created and controlled
- ✅ ROS 2 integration with sensors and actuators
- ✅ Launch file orchestration mastered
- ✅ Odometry and sensor data publishing
- ✅ Foundation for advanced robotics development
