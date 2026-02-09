#!/usr/bin/env python3
"""
Level 2.1: Touch Sensor / Bumper Controller

Reactive behavior using TouchSensor (bumper type).
Robot explores environment and reacts to physical contact.

Mission: Navigate forward, detect obstacles with bumper sensor,
         and execute escape maneuvers when collision is detected.

This demonstrates:
- TouchSensor API (type "bumper")
- Reactive behaviors (stimulus → response)
- Simple obstacle avoidance
- State machine for behavior control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import math


class TouchSensorController(Node):
    """
    Level 2.1: Touch Sensor Controller
    
    Demonstrates TouchSensor API (type "bumper")
    Detects obstacles through physical contact
    """
    
    # Robot parameters
    WHEEL_RADIUS = 0.08    # meters
    WHEEL_DISTANCE = 0.24  # meters
    
    # Behavior parameters
    FORWARD_SPEED = 0.3    # m/s
    TURN_SPEED = 0.5       # rad/s
    BACKUP_SPEED = -0.2    # m/s
    BACKUP_TIME = 1.0      # seconds
    TURN_TIME = 1.75        # seconds
    
    def __init__(self):
        super().__init__('touch_sensor_controller')
        
        # Initialize Webots Robot
        try:
            from controller import Robot
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
        except ImportError:
            self.get_logger().error('Webots controller module not found!')
            raise
        
        # Get motor devices
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # Set motors to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Get wheel encoders for odometry
        self.left_encoder = self.robot.getDevice('left_wheel_sensor')
        self.right_encoder = self.robot.getDevice('right_wheel_sensor')
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Get bumper TouchSensor
        self.bumper = self.robot.getDevice('bumper')
        self.bumper.enable(self.timestep)
        
        # State machine
        self.state = "EXPLORING"
        self.state_timer = 0.0
        self.collision_count = 0
        
        # ROS2 publishers
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.collision_pub = self.create_publisher(Bool, '/collision_detected', 10)
        
        # Wait one timestep for sensors to initialize
        self.robot.step(self.timestep)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 LEVEL 2.1: TOUCH SENSOR CONTROLLER')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Mission: Explore and avoid obstacles')
        self.get_logger().info('Method: TouchSensor bumper detection')
        self.get_logger().info('Behaviors:')
        self.get_logger().info('  - EXPLORING: Move forward, monitor bumper')
        self.get_logger().info('  - COLLISION: Bumper pressed (getValue() = 1)')
        self.get_logger().info('  - REACTION: Back up and turn away')
        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 Starting exploration...')
        self.get_logger().info('')
        
    def read_collision(self):
        """Read bumper TouchSensor value"""
        # TouchSensor type "bumper" returns 0.0 (no contact) or 1.0 (contact)
        bumper_value = self.bumper.getValue()
        
        return {
            'detected': bumper_value > 0.5,  # True if bumper pressed
            'value': bumper_value
        }
    
    def set_velocity(self, linear, angular):
        """Set robot velocity"""
        # Differential drive kinematics
        left_speed = (linear - angular * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS
        right_speed = (linear + angular * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS
        
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    
    def publish_state(self):
        """Publish current state to ROS2"""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
    
    def handle_collision(self, collision_data):
        """React to collision detection"""
        if not collision_data['detected']:
            return False
        
        self.collision_count += 1
        
        # Always turn right
        turn_direction = -1
        
        self.get_logger().info(f'💥 COLLISION #{self.collision_count} detected!')
        
        # Publish collision event
        collision_msg = Bool()
        collision_msg.data = True
        self.collision_pub.publish(collision_msg)
        
        # Transition to BACKING_UP state
        self.state = "BACKING_UP"
        self.state_timer = 0.0
        self.turn_direction = turn_direction
        
        return True
    
    def update_state_machine(self, dt):
        """Update behavior state machine"""
        collision_data = self.read_collision()
        
        # State: EXPLORING (moving forward)
        if self.state == "EXPLORING":
            # Check for collision FIRST
            if self.handle_collision(collision_data):
                # Immediately start backing up
                self.set_velocity(self.BACKUP_SPEED, 0.0)
                return
            
            # No collision, continue forward
            self.set_velocity(self.FORWARD_SPEED, 0.0)
            
            # Publish state and bumper info
            if self.state_timer % 2.0 < dt:  # Every 2 seconds
                self.get_logger().info(
                    f'🔍 Exploring... (collisions: {self.collision_count}) | '
                    f'Bumper: {collision_data["value"]:.1f}'
                )
        
        # State: BACKING_UP (reverse after collision)
        elif self.state == "BACKING_UP":
            self.set_velocity(self.BACKUP_SPEED, 0.0)
            
            if self.state_timer >= self.BACKUP_TIME:
                self.state = "TURNING"
                self.state_timer = 0.0
                self.get_logger().info(f'🔄 Turning right...')
        
        # State: TURNING (rotate to new direction)
        elif self.state == "TURNING":
            self.set_velocity(0.0, self.turn_direction * self.TURN_SPEED)
            
            if self.state_timer >= self.TURN_TIME:
                self.state = "EXPLORING"
                self.state_timer = 0.0
                self.get_logger().info('✅ Maneuver complete - resuming exploration')
        
        # Update timer
        self.state_timer += dt
        
        # Publish current state
        self.publish_state()
    
    def run_step(self):
        """Execute one control step"""
        dt = self.timestep / 1000.0  # Convert to seconds
        
        # Update state machine
        self.update_state_machine(dt)
        
        # Step simulation
        return self.robot.step(self.timestep) != -1


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        controller = TouchSensorController()
        
        # Main control loop
        while rclpy.ok() and controller.run_step():
            rclpy.spin_once(controller, timeout_sec=0)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
