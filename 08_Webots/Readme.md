# 🤖 **Webots Robot Simulator & ROS 2 Integration**

## **📌 Project Title**

Mastering Virtual Robotics: Webots Simulator Integration with ROS 2 Jazzy

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **📋 Overview**

This comprehensive module teaches professional virtual robotics development using Webots simulator with ROS 2 integration. You will learn to:

- Install and configure Webots robot simulator on Ubuntu 24.04
- Understand robot simulation fundamentals and physics engines
- Integrate ROS 2 with Webots for realistic testing environments
- Create launch files that orchestrate simulator startup and node communication
- Develop robot controllers that read sensors and control actuators in simulation
- Test complex multi-sensor robotic systems safely before physical deployment
- Understand simulation-to-reality (sim2real) transfer principles

This foundation enables rapid development, testing, and validation of ROS 2 applications in a safe virtual environment before deploying to physical robots.

---

## **✅ Prerequisites**

- ROS 2 Jazzy Jalisco installed and configured (from 00_Install module)
- Ubuntu 24.04 LTS with 4GB+ free disk space
- VS Code with ROS extensions configured
- Understanding of ROS 2 nodes, topics, and launch files (from 01-07 modules)
- Basic Python 3 knowledge for controller development
- Display server configured (for Webots GUI)

---

## **📊 Robot Simulation Architecture**

```
┌──────────────────────────────────────────────────────────┐
│        Ubuntu 24.04 LTS Virtual Machine                  │
├──────────────────────────────────────────────────────────┤
│  ┌────────────────────────────────────────────────────┐  │
│  │  Webots Simulator (Physics Engine & Rendering)     │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │ Simulated Robot                              │  │  │
│  │  │ ├─ Body (Physics)                            │  │  │
│  │  │ ├─ Joints & Motors                           │  │  │
│  │  │ └─ Simulated Sensors                         │  │  │
│  │  │    ├─ Camera, LiDAR, IMU                     │  │  │
│  │  │    └─ Encoders, Proximity Sensors            │  │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
│              ↕ Socket/ROS 2 Bridge                       │
│  ┌────────────────────────────────────────────────────┐  │
│  │  ROS 2 Jazzy Middleware (DDS)                      │  │
│  │  ┌──────────────────────────────────────────────┐  │  │
│  │  │ Controller Nodes                             │  │  │
│  │  │ ├─ Motor Command Publisher                   │  │  │
│  │  │ ├─ Sensor Data Subscriber                    │  │  │
│  │  │ └─ State Estimator Node                      │  │  │
│  │  └──────────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────┘  │
├──────────────────────────────────────────────────────────┤
│  Launch System: Multi-node orchestration                 │
│  Build System: Colcon with simulation targets            │
└──────────────────────────────────────────────────────────┘
```

---

## **🎯 Learning Objectives**

After completing this module, you will be able to:

- [ ] Understand Webots simulator architecture and physics engine capabilities
- [ ] Install Webots from official repositories and configure ROS 2 bridge
- [ ] Create Webots `.proto` robot descriptions and world files
- [ ] Write robot controller Python scripts for simulated robots
- [ ] Develop launch files that start Webots simulator and ROS 2 nodes
- [ ] Publish motor commands from ROS 2 topics to Webots actuators
- [ ] Subscribe to Webots sensor data through ROS 2 topics
- [ ] Implement multi-sensor data fusion (camera, LiDAR, IMU, encoders)
- [ ] Debug robot behavior using Webots inspector and ROS 2 CLI tools
- [ ] Understand simulation parameters (physics timestep, gravity, friction)
- [ ] Apply sim2real transfer principles for real robot deployment
- [ ] Test complex robotic behaviors safely in simulation

---

## **🛠 Step 1: Install and Configure Webots Simulator**

### **1.1 Install Webots (Choose ONE method)**

#### **Method 1: Snap Installation (Recommended for Ubuntu 24.04)**

This is the easiest and most reliable method for Ubuntu 24.04:

```bash
# Install Webots via snap
sudo snap install webots

# Verify installation
webots --version
```

**Expected output:** `Webots R2025a` (or newer)

#### **Method 2: Download from Official Website**

If snap is not available or you prefer the official package:

```bash
# Download Webots R2025a for Ubuntu
cd ~/Downloads
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb

# Install the package
sudo apt install -y ./webots_2025a_amd64.deb

# Install dependencies if needed
sudo apt-get install -f

# Verify installation
which webots
webots --version
```

#### **Method 3: Build from Source (Advanced)**

For the latest development version:

```bash
# Install build dependencies
sudo apt-get update
sudo apt-get install -y git cmake g++ libavcodec-dev libavformat-dev \
    libswscale-dev libglu1-mesa-dev libglib2.0-dev libfreeimage-dev \
    libfreetype6-dev libxml2-dev libzzip-dev libssl-dev libboost-dev \
    libgd-dev libssh-dev libzip-dev libreadline-dev libassimp-dev \
    libpci-dev

# Clone and build
git clone --recurse-submodules https://github.com/cyberbotics/webots.git
cd webots
make -j$(nproc)

# Add to PATH
echo 'export WEBOTS_HOME=~/webots' >> ~/.bashrc
echo 'export PATH=$WEBOTS_HOME:$PATH' >> ~/.bashrc
source ~/.bashrc
```

**Choose Method 1 (Snap) for simplicity and reliability on Ubuntu 24.04.**

### **1.2 Install Webots ROS 2 Bridge**

Install the official Webots ROS 2 package for Jazzy:

```bash
# Update package list
sudo apt-get update

# Install Webots ROS 2 interface
sudo apt-get install -y ros-jazzy-webots-ros2

# If the package is not available, you can build from source:
# cd ~/ros2_ws/src
# git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git
# cd ~/ros2_ws
# rosdep install --from-paths src --ignore-src -r -y
# colcon build --packages-select webots_ros2
```

Verify ROS 2 integration:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep webots
```

**Expected output:**
```
webots_ros2_driver
webots_ros2_msgs
webots_ros2_control
```

### **1.3 Configure Environment Variables**

Update `.bashrc` to include Webots paths (adjust based on installation method):

```bash
nano ~/.bashrc
```

**For Snap Installation:**
```bash
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

**For Source Build:**
```bash
# Add to ~/.bashrc
export WEBOTS_HOME=~/webots
export PATH=$WEBOTS_HOME:$PATH
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH
```

Reload terminal:

```bash
source ~/.bashrc
```

**Verify Webots is accessible:**

```bash
which webots
webots --version
```

If installed via snap, the path will be `/snap/bin/webots`.

---

## **🚀 Step 2: Create Workspace Structure**

### **2.1 Setup Project Directories**

Create the necessary folder structure:

```bash
cd ~/ros2_ws/src/ce_robot
mkdir -p worlds controllers launch config protos
```

**Directory Structure:**
```
ce_robot/
├── controllers/          # Robot controller Python scripts
├── worlds/              # Webots world files (.wbt)
├── protos/              # Custom robot PROTO definitions
├── launch/              # ROS 2 launch files
├── config/              # Configuration files
└── ce_robot/            # Python package for ROS 2 nodes
```

---

## **🎯 Step 3: Create Your First Simulation World**

### **3.1 Complete Differential Drive Robot World**

**Current Directory Structure:**
```
~/ros2_ws/src/ce_robot/
├── ce_robot/              # Python package
├── controllers/           # Robot controller scripts (created in Step 2)
├── worlds/                # Webots world files (created in Step 2)
│   └── my_first_robot.wbt # ← We'll create this file now
├── protos/                # Custom PROTO files (created in Step 2)
├── launch/                # ROS 2 launch files (created in Step 2)
├── config/                # Configuration files (created in Step 2)
├── package.xml
├── setup.py
└── resource/
```

Create `worlds/my_first_robot.wbt`:

```bash
cd ~/ros2_ws/src/ce_robot/worlds
touch my_first_robot.wbt
```

Paste this complete world file:

```vrml
#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"

WorldInfo {
  info "My First ROS 2 Robot Simulation"
  title "Webots ROS 2 Lab"
  basicTimeStep 32
}

Viewpoint {
  orientation -0.5 0.5 0.707 1.8
  position 0 0 5
}

TexturedBackgroundLight {
}

RectangleArena {
  floorSize 5 5
  wallHeight 0.5
}

# Simple differential drive robot
Robot {
  translation 0 0 0.05
  children [
    # Robot body
    Shape {
      appearance PBRAppearance {
        baseColor 0.2 0.5 0.8
        metalness 0
      }
      geometry Box {
        size 0.3 0.2 0.1
      }
    }
    
    # Left wheel
    HingeJoint {
      jointParameters HingeJointParameters {
        position 0
        axis 0 1 0
        anchor -0.15 -0.13 0
      }
      device [
        RotationalMotor {
          name "left_motor"
          maxVelocity 10
        }
        PositionSensor {
          name "left_encoder"
        }
      ]
      endPoint Solid {
        translation -0.15 -0.13 0
        rotation 1 0 0 1.5708
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.1 0.1 0.1
              metalness 0
            }
            geometry Cylinder {
              height 0.04
              radius 0.05
            }
          }
        ]
        boundingObject Cylinder {
          height 0.04
          radius 0.05
        }
        physics Physics {
          density -1
          mass 0.1
        }
      }
    }
    
    # Right wheel
    HingeJoint {
      jointParameters HingeJointParameters {
        position 0
        axis 0 1 0
        anchor -0.15 0.13 0
      }
      device [
        RotationalMotor {
          name "right_motor"
          maxVelocity 10
        }
        PositionSensor {
          name "right_encoder"
        }
      ]
      endPoint Solid {
        translation -0.15 0.13 0
        rotation 1 0 0 1.5708
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.1 0.1 0.1
              metalness 0
            }
            geometry Cylinder {
              height 0.04
              radius 0.05
            }
          }
        ]
        boundingObject Cylinder {
          height 0.04
          radius 0.05
        }
        physics Physics {
          density -1
          mass 0.1
        }
      }
    }
    
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

Create `controllers/my_robot_controller.py`:

```bash
cd ~/ros2_ws/src/ce_robot/controllers
touch my_robot_controller.py
chmod +x my_robot_controller.py
```

Paste this complete controller implementation:

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
        self.left_encoder = self.robot.getDevice('left_encoder')
        self.right_encoder = self.robot.getDevice('right_encoder')
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Get distance sensor
        self.distance_sensor = self.robot.getDevice('front_sensor')
        self.distance_sensor.enable(self.timestep)
        
        # Robot physical parameters
        self.wheel_radius = 0.05  # meters
        self.wheel_separation = 0.26  # meters
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_encoder = 0.0
        self.last_right_encoder = 0.0
        
        # ROS 2 Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # ROS 2 Publishers
        self.sensor_pub = self.create_publisher(Range, 'distance_sensor', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create timer for sensor publishing
        self.create_timer(0.1, self.publish_sensors)
        
        self.get_logger().info('🤖 Webots Robot Controller initialized')
        self.get_logger().info(f'   Timestep: {self.timestep}ms')
        self.get_logger().info(f'   Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'   Wheel separation: {self.wheel_separation}m')

    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to wheel velocities
        msg.linear.x: forward/backward velocity (m/s)
        msg.angular.z: rotation velocity (rad/s)
        """
        # Differential drive kinematics
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Convert to wheel velocities
        left_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Set motor velocities
        self.left_motor.setVelocity(left_vel)
        self.right_motor.setVelocity(right_vel)
        
        self.get_logger().info(
            f'Motor commands: Left={left_vel:.2f}, Right={right_vel:.2f} rad/s'
        )

    def update_odometry(self):
        """Calculate robot position from wheel encoders"""
        # Get current encoder values
        left_encoder = self.left_encoder.getValue()
        right_encoder = self.right_encoder.getValue()
        
        # Calculate wheel movements
        delta_left = left_encoder - self.last_left_encoder
        delta_right = right_encoder - self.last_right_encoder
        
        # Update last encoder values
        self.last_left_encoder = left_encoder
        self.last_right_encoder = right_encoder
        
        # Calculate linear and angular displacement
        delta_distance = self.wheel_radius * (delta_left + delta_right) / 2.0
        delta_theta = self.wheel_radius * (delta_right - delta_left) / self.wheel_separation
        
        # Update pose
        self.theta += delta_theta
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)

    def publish_sensors(self):
        """Read sensors and publish to ROS 2 topics"""
        
        # Update odometry
        self.update_odometry()
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        self.odom_pub.publish(odom_msg)
        
        # Publish distance sensor
        distance = self.distance_sensor.getValue()
        
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'front_sensor'
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.1
        range_msg.min_range = 0.0
        range_msg.max_range = 1.0
        range_msg.range = min(distance / 1000.0, 1.0)  # Convert to meters
        
        self.sensor_pub.publish(range_msg)
        
        # Log sensor data
        if distance < 500:  # Obstacle detected
            self.get_logger().warn(f'⚠️  Obstacle detected: {distance:.0f}mm')

    def run(self):
        """Main control loop"""
        self.get_logger().info('🚀 Starting robot control loop...')
        
        while self.robot.step(self.timestep) != -1:
            # Process ROS 2 callbacks
            rclpy.spin_once(self, timeout_sec=0.001)

def main(args=None):
    rclpy.init(args=args)
    controller = WebotsRobotController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
**Key Features:**
- ✅ **Differential Drive Kinematics** - Converts Twist to wheel velocities
- ✅ **Odometry** - Calculates position from wheel encoders
- ✅ **Sensor Publishing** - Publishes distance sensor data
- ✅ **ROS 2 Integration** - Standard geometry_msgs/Twist interface

---

## **🎯 Step 5: Create Launch File for Webots Simulation**

### **5.1 Complete Launch File with Timing**

Create `launch/webots_sim_launch.py`:

```bash
cd ~/ros2_ws/src/ce_robot/launch
touch webots_sim_launch.py
chmod +x webots_sim_launch.py
```

Paste this complete launch file:

```python
#!/usr/bin/env python3
"""
Launch file for Webots simulation with ROS 2 integration
Starts Webots simulator and robot controller node
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('ce_robot').find('ce_robot')
    
    # Declare arguments
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, '..', '..', 'src', 'ce_robot', 'worlds', 'my_first_robot.wbt'),
        description='Full path to Webots world file'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Webots mode: realtime, fast, pause'
    )
    
    # Get launch configurations
    world_file = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    
    # Start Webots simulator
    webots_process = ExecuteProcess(
        cmd=['webots', '--mode=realtime', world_file],
        output='screen',
        shell=False
    )
    
    # Start robot controller (delayed to let Webots initialize)
    robot_controller = TimerAction(
        period=3.0,  # Wait 3 seconds for Webots to start
        actions=[
            Node(
                package='ce_robot',
                executable='my_robot_controller',
                name='webots_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': True
                }]
            )
        ]
    )
    
    return LaunchDescription([
        world_file_arg,
        mode_arg,
        webots_process,
        robot_controller,
    ])
```

**Launch File Features:**
- ✅ **Configurable World File** - Pass world file as argument
- ✅ **Simulation Mode** - Choose realtime, fast, or pause
- ✅ **Delayed Start** - Wait for Webots to initialize before starting controller
- ✅ **Sim Time** - Use simulation time instead of wall clock

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
        # Install controller files
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
            # Add controller as executable
            'my_robot_controller = ce_robot.controllers.my_robot_controller:main',
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
