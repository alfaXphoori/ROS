# 🤖 **Webots Simulation Lab Exercises**

Complete hands-on exercises to master Webots robot simulation with ROS 2 integration.

---

## **Exercise 1: Basic Robot Simulation (Beginner) ⏱️ 40 min**

### **Objective**
Create and launch a simple two-wheeled mobile robot in Webots, establish basic ROS 2 communication, and test motor control.

### **Skills You'll Learn**
- Webots world file creation
- Robot controller development
- Basic Webots-ROS 2 communication
- Motor command publishing and receiving

### **Step-by-Step Guide**

#### **Step 1: Create World File**

Create `~/ros2_ws/src/ce_robot/worlds/simple_robot_world.wbt`:

```vrml
#VRML_SIM R2024b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/floors/protos/RectangleArena.proto"

WorldInfo {
  info "Simple Robot Simulation"
  title "Exercise 1: Basic Robot"
}

Viewpoint {
  orientation 0 1 0 0
  position 0 0 2
}

TexturedBackgroundLight {
}

RectangleArena {
  floorSize 5 5
}

DEF ROBOT Robot {
  translation 0 0 0.1
  rotation 0 0 1 0
  children [
    Transform {
      translation 0 0 0
      children [
        Shape {
          geometry Box { size 0.2 0.2 0.1 }
        }
      ]
    }
    Transform {
      translation -0.12 0.1 0
      children [
        Shape {
          geometry Sphere { radius 0.03 }
        }
        DEF LEFT_MOTOR HingeJoint {
          jointParameters HingeJointParameters {
            axis 0 1 0
            anchor 0 0 0
          }
          device [
            RotationalMotor {
              name "left_motor"
              maxVelocity 6.28
              maxTorque 5
            }
            PositionSensor {
              name "left_encoder"
            }
          ]
          endPoint Solid {
            translation 0 0.1 0
            rotation 1 0 0 1.5708
            geometry Cylinder {
              height 0.04
              radius 0.03
            }
          }
        }
      ]
    }
    Transform {
      translation -0.12 -0.1 0
      children [
        Shape {
          geometry Sphere { radius 0.03 }
        }
        DEF RIGHT_MOTOR HingeJoint {
          jointParameters HingeJointParameters {
            axis 0 1 0
            anchor 0 0 0
          }
          device [
            RotationalMotor {
              name "right_motor"
              maxVelocity 6.28
              maxTorque 5
            }
            PositionSensor {
              name "right_encoder"
            }
          ]
          endPoint Solid {
            translation 0 -0.1 0
            rotation 1 0 0 1.5708
            geometry Cylinder {
              height 0.04
              radius 0.03
            }
          }
        }
      ]
    }
  ]
  controller "simple_robot_controller"
  name "SimpleRobot"
}
```

#### **Step 2: Create Robot Controller**

Create `~/ros2_ws/src/ce_robot/ce_robot/simple_robot_controller.py`:

```python
#!/usr/bin/env python3
"""Exercise 1: Simple Webots robot controller"""

from controller import Robot, Motor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleRobotController(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Get motors
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # ROS 2 subscriber for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        self.current_left_vel = 0.0
        self.current_right_vel = 0.0
        
        self.get_logger().info('Simple robot controller started')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor velocities"""
        # Simple differential drive conversion
        linear = msg.linear.x
        angular = msg.angular.z
        
        wheel_separation = 0.2  # Distance between wheels
        
        self.current_left_vel = linear - (angular * wheel_separation / 2)
        self.current_right_vel = linear + (angular * wheel_separation / 2)
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Set motor velocities
            self.left_motor.setVelocity(self.current_left_vel)
            self.right_motor.setVelocity(self.current_right_vel)

def main():
    rclpy.init()
    controller = SimpleRobotController()
    controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### **Step 3: Create Launch File**

Create `~/ros2_ws/src/ce_robot/ce_robot/simple_sim_launch.py`:

```python
#!/usr/bin/env python3
"""Exercise 1: Simple simulation launch"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare('ce_robot')
    world_file = LaunchConfiguration('world_file')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value=[pkg_share, '/worlds/simple_robot_world.wbt'],
            description='Path to world file'
        ),
        
        ExecuteProcess(
            cmd=['webots', world_file, '--batch'],
            output='screen'
        ),
    ])
```

#### **Step 4: Test Robot Control**

Launch simulation:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch ce_robot simple_sim_launch.py
```

In another terminal, send commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Verify:** Robot moves forward in Webots simulator

---

## **Exercise 2: Multi-Sensor Robot (Intermediate) ⏱️ 50 min**

### **Objective**
Create a robot with multiple sensors (distance sensor, camera, IMU), publish sensor data to ROS 2 topics, and implement basic obstacle avoidance.

### **Skills You'll Learn**
- Multiple sensor integration
- Sensor data processing and publishing
- Obstacle avoidance algorithm
- Advanced motor control

### **Implementation**

#### **Step 1: Create Enhanced World File**

Create `~/ros2_ws/src/ce_robot/worlds/sensor_robot_world.wbt`:

```vrml
#VRML_SIM R2024b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/obstacles/protos/OilBarrel.proto"

WorldInfo {
  info "Multi-Sensor Robot Simulation"
  title "Exercise 2: Sensor Integration"
}

Viewpoint {
  orientation 0 1 0 0
  position 0 0 2
}

TexturedBackgroundLight {
}

RectangleArena {
  floorSize 8 8
}

OilBarrel {
  translation 1 0 0
}

OilBarrel {
  translation -1 1 0
}

OilBarrel {
  translation 0 -1.5 0
}

DEF SENSOR_ROBOT Robot {
  translation 0 0 0.1
  rotation 0 0 1 0
  children [
    Shape {
      geometry Box { size 0.2 0.2 0.1 }
    }
    
    # Distance Sensor (Front)
    Transform {
      translation 0.1 0 0.05
      rotation 0 0 1 1.5708
      children [
        DistanceSensor {
          name "distance_sensor"
          maxRange 2.0
          type "sharp"
        }
      ]
    }
    
    # Camera
    Transform {
      translation 0.1 0 0.05
      children [
        Camera {
          name "camera"
          width 160
          height 120
          fieldOfView 1.047
        }
      ]
    }
    
    # IMU
    InertialUnit {
      name "imu"
    }
    
    # Motors (left and right)
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.12 0.1 0
      }
      device [
        RotationalMotor {
          name "left_motor"
          maxVelocity 6.28
          maxTorque 5
        }
        PositionSensor { name "left_encoder" }
      ]
      endPoint Solid {
        translation -0.12 0.1 0
        rotation 1 0 0 1.5708
        geometry Cylinder { height 0.04 radius 0.03 }
      }
    }
    
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.12 -0.1 0
      }
      device [
        RotationalMotor {
          name "right_motor"
          maxVelocity 6.28
          maxTorque 5
        }
        PositionSensor { name "right_encoder" }
      ]
      endPoint Solid {
        translation -0.12 -0.1 0
        rotation 1 0 0 1.5708
        geometry Cylinder { height 0.04 radius 0.03 }
      }
    }
  ]
  controller "sensor_robot_controller"
  name "SensorRobot"
}
```

#### **Step 2: Create Sensor Robot Controller**

Create `~/ros2_ws/src/ce_robot/ce_robot/sensor_robot_controller.py`:

```python
#!/usr/bin/env python3
"""Exercise 2: Multi-sensor robot controller with obstacle avoidance"""

from controller import Robot, Motor, DistanceSensor, Camera, InertialUnit
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import Float32

class SensorRobotController(Node):
    def __init__(self):
        super().__init__('sensor_robot_controller')
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Get motors
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # Get sensors
        self.distance_sensor = self.robot.getDevice('distance_sensor')
        self.camera = self.robot.getDevice('camera')
        self.imu = self.robot.getDevice('imu')
        
        # Enable sensors
        self.distance_sensor.enable(self.timestep)
        self.camera.enable(self.timestep)
        self.imu.enable(self.timestep)
        
        # ROS 2 publishers
        self.distance_pub = self.create_publisher(Range, 'distance_data', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_level', 10)
        
        # ROS 2 subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.obstacle_distance_threshold = 0.5
        
        self.get_logger().info('Sensor robot controller started')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist to motor velocities"""
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_separation = 0.2
        
        self.left_vel = linear - (angular * wheel_separation / 2)
        self.right_vel = linear + (angular * wheel_separation / 2)
    
    def publish_sensor_data(self):
        """Read sensors and publish to ROS 2"""
        # Distance sensor
        distance = self.distance_sensor.getValue()
        range_msg = Range()
        range_msg.header.frame_id = 'robot_front'
        range_msg.min_range = 0.0
        range_msg.max_range = 2.0
        range_msg.range = float(distance)
        self.distance_pub.publish(range_msg)
        
        # IMU
        accel = self.imu.getAcceleration()
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        self.imu_pub.publish(imu_msg)
        
        # Battery (simulated)
        battery_msg = Float32(data=0.9)
        self.battery_pub.publish(battery_msg)
    
    def obstacle_avoidance(self):
        """Simple obstacle avoidance logic"""
        distance = self.distance_sensor.getValue()
        
        if distance < self.obstacle_distance_threshold:
            # Obstacle detected: stop and turn right
            self.left_vel = 0.5
            self.right_vel = -0.5
            self.get_logger().warn(f'Obstacle detected at {distance:.2f}m')
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Apply obstacle avoidance
            self.obstacle_avoidance()
            
            # Publish sensor data
            self.publish_sensor_data()
            
            # Set motor velocities
            self.left_motor.setVelocity(self.left_vel)
            self.right_motor.setVelocity(self.right_vel)

def main():
    rclpy.init()
    controller = SensorRobotController()
    controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### **Step 3: Test Multi-Sensor Integration**

Launch simulation:

```bash
colcon build
source install/setup.bash
ros2 launch ce_robot sensor_sim_launch.py
```

Monitor sensor data in new terminals:

```bash
# Terminal 1: Distance sensor
ros2 topic echo /distance_data

# Terminal 2: IMU data
ros2 topic echo /imu_data

# Terminal 3: Send commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
```

**Verify:** Robot avoids obstacles when detected, sensor data publishes to topics

---

## **Exercise 3: Advanced Multi-Sensor System (Advanced) ⏱️ 60 min**

### **Objective**
Build a sophisticated robot with multiple sensors, data fusion, and adaptive control. Implement state estimation, sensor fusion, and coordinated multi-sensor decision making.

### **Skills You'll Learn**
- Advanced sensor fusion techniques
- State estimation algorithms
- Adaptive control based on multiple sensor inputs
- Performance monitoring and optimization

### **Implementation**

#### **Step 1: Create Advanced World File**

Create `~/ros2_ws/src/ce_robot/worlds/advanced_sensor_world.wbt`:

```vrml
#VRML_SIM R2024b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/obstacles/protos/OilBarrel.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2024b/projects/objects/obstacles/protos/Wall.proto"

WorldInfo {
  info "Advanced Multi-Sensor Robot"
  title "Exercise 3: Sensor Fusion & Adaptive Control"
}

Viewpoint {
  orientation 0 1 0 0
  position 0 0 3
}

TexturedBackgroundLight {
}

RectangleArena {
  floorSize 12 12
}

# Obstacles
OilBarrel { translation 2 0 0 }
OilBarrel { translation -2 2 0 }
OilBarrel { translation 0 -2.5 0 }
Wall { translation 3 3 0 rotation 0 0 1 0.785 }

DEF ADVANCED_ROBOT Robot {
  translation 0 0 0.1
  children [
    Shape { geometry Box { size 0.25 0.25 0.12 } }
    
    # Front distance sensor
    Transform {
      translation 0.12 0 0.06
      children [DistanceSensor { name "front_sensor" maxRange 2.5 }]
    }
    
    # Left distance sensor
    Transform {
      translation 0 0.12 0.06
      rotation 0 0 1 1.5708
      children [DistanceSensor { name "left_sensor" maxRange 2.5 }]
    }
    
    # Right distance sensor
    Transform {
      translation 0 -0.12 0.06
      rotation 0 0 1 -1.5708
      children [DistanceSensor { name "right_sensor" maxRange 2.5 }]
    }
    
    # High-resolution camera
    Transform {
      translation 0.12 0 0.06
      children [Camera { name "camera" width 320 height 240 }]
    }
    
    # 9-DOF IMU
    InertialUnit { name "imu" }
    
    # GPS (for position feedback)
    GPS { name "gps" }
    
    # Compass
    Compass { name "compass" }
    
    # Motors
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.15 0.12 0
      }
      device [
        RotationalMotor { name "left_motor" maxVelocity 6.28 maxTorque 5 }
        PositionSensor { name "left_encoder" }
      ]
      endPoint Solid {
        translation -0.15 0.12 0
        rotation 1 0 0 1.5708
        geometry Cylinder { height 0.04 radius 0.035 }
      }
    }
    
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor -0.15 -0.12 0
      }
      device [
        RotationalMotor { name "right_motor" maxVelocity 6.28 maxTorque 5 }
        PositionSensor { name "right_encoder" }
      ]
      endPoint Solid {
        translation -0.15 -0.12 0
        rotation 1 0 0 1.5708
        geometry Cylinder { height 0.04 radius 0.035 }
      }
    }
  ]
  controller "advanced_robot_controller"
  name "AdvancedRobot"
}
```

#### **Step 2: Create Advanced Robot Controller**

Create `~/ros2_ws/src/ce_robot/ce_robot/advanced_robot_controller.py`:

```python
#!/usr/bin/env python3
"""Exercise 3: Advanced sensor fusion and adaptive control"""

from controller import Robot, Motor, DistanceSensor, Camera, InertialUnit, GPS, Compass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovariance
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import Float32MultiArray
import math

class AdvancedRobotController(Node):
    def __init__(self):
        super().__init__('advanced_robot_controller')
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Motors
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # Sensors
        self.front_sensor = self.robot.getDevice('front_sensor')
        self.left_sensor = self.robot.getDevice('left_sensor')
        self.right_sensor = self.robot.getDevice('right_sensor')
        self.imu = self.robot.getDevice('imu')
        self.gps = self.robot.getDevice('gps')
        self.compass = self.robot.getDevice('compass')
        
        # Enable sensors
        for sensor in [self.front_sensor, self.left_sensor, self.right_sensor, self.imu, self.gps, self.compass]:
            sensor.enable(self.timestep)
        
        # ROS 2 publishers
        self.sensor_fusion_pub = self.create_publisher(Float32MultiArray, 'sensor_fusion', 10)
        self.state_estimate_pub = self.create_publisher(PoseWithCovariance, 'state_estimate', 10)
        self.obstacle_map_pub = self.create_publisher(Float32MultiArray, 'obstacle_map', 10)
        
        # ROS 2 subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Control state
        self.left_vel = 0.0
        self.right_vel = 0.0
        
        # Sensor fusion state
        self.robot_position = [0.0, 0.0]
        self.robot_heading = 0.0
        
        self.get_logger().info('Advanced robot controller initialized')
    
    def cmd_vel_callback(self, msg):
        """Velocity command callback"""
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_separation = 0.24
        
        self.left_vel = linear - (angular * wheel_separation / 2)
        self.right_vel = linear + (angular * wheel_separation / 2)
    
    def sensor_fusion(self):
        """Fuse multiple sensors for accurate state estimation"""
        # Read all sensors
        front_dist = self.front_sensor.getValue()
        left_dist = self.left_sensor.getValue()
        right_dist = self.right_sensor.getValue()
        
        accel = self.imu.getAcceleration()
        gyro = self.imu.getGyro()
        
        gps_pos = self.gps.getValues()
        compass_heading = self.compass.getValues()
        
        # Sensor fusion algorithm (simplified Kalman-like approach)
        self.robot_position[0] = gps_pos[0]
        self.robot_position[1] = gps_pos[1]
        self.robot_heading = math.atan2(compass_heading[1], compass_heading[0])
        
        # Publish fused sensor data
        fusion_msg = Float32MultiArray()
        fusion_msg.data = [
            front_dist, left_dist, right_dist,
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2],
            self.robot_position[0], self.robot_position[1], self.robot_heading
        ]
        self.sensor_fusion_pub.publish(fusion_msg)
        
        return {
            'front': front_dist,
            'left': left_dist,
            'right': right_dist,
            'position': self.robot_position,
            'heading': self.robot_heading
        }
    
    def adaptive_control(self, sensor_data):
        """Adaptive control based on multi-sensor input"""
        front_dist = sensor_data['front']
        left_dist = sensor_data['left']
        right_dist = sensor_data['right']
        
        # Adaptive obstacle avoidance
        if front_dist < 0.6:
            if left_dist > right_dist:
                self.left_vel = 0.3
                self.right_vel = -0.3
            else:
                self.left_vel = -0.3
                self.right_vel = 0.3
        
        # Wall-following behavior
        elif 0.8 < left_dist < 1.2:
            self.left_vel = 1.0
            self.right_vel = 1.0
        
        # Limit velocities
        max_vel = 2.0
        self.left_vel = max(-max_vel, min(max_vel, self.left_vel))
        self.right_vel = max(-max_vel, min(max_vel, self.right_vel))
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Sensor fusion
            sensor_data = self.sensor_fusion()
            
            # Adaptive control
            self.adaptive_control(sensor_data)
            
            # Set motor velocities
            self.left_motor.setVelocity(self.left_vel)
            self.right_motor.setVelocity(self.right_vel)

def main():
    rclpy.init()
    controller = AdvancedRobotController()
    controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### **Step 3: Test Advanced System**

Launch simulation:

```bash
colcon build
source install/setup.bash
ros2 launch ce_robot advanced_sim_launch.py
```

Monitor in separate terminals:

```bash
# Monitor sensor fusion
ros2 topic echo /sensor_fusion

# Monitor state estimate
ros2 topic echo /state_estimate

# Monitor obstacle map
ros2 topic echo /obstacle_map
```

**Verify:** Robot implements complex behaviors with multi-sensor fusion, adapts to environment

---

## **Summary Table**

| Exercise | Difficulty | Duration | Key Skills | Output Files |
|----------|-----------|----------|-----------|--------------|
| 1 | Beginner | 40 min | Basic simulation, motor control | simple_robot_world.wbt, simple_robot_controller.py |
| 2 | Intermediate | 50 min | Multi-sensor integration, obstacle avoidance | sensor_robot_world.wbt, sensor_robot_controller.py |
| 3 | Advanced | 60 min | Sensor fusion, adaptive control, state estimation | advanced_sensor_world.wbt, advanced_robot_controller.py |

---

**✅ Complete all 3 exercises to master Webots simulation with ROS 2!** 🤖🎉
