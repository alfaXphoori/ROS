#!/usr/bin/env python3
"""
Tutorial 00: Simple Robot Controller

Webots controller that runs inside the simulation.
Subscribes to /cmd_vel and controls the motors.

This is the "brain" that runs on the robot itself.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleRobotController(Node):
    """Basic differential drive controller for Tutorial 00"""
    
    # Robot parameters
    WHEEL_RADIUS = 0.08    # meters
    WHEEL_DISTANCE = 0.24  # meters (distance between left and right wheels)
    
    def __init__(self):
        super().__init__('simple_robot_controller_00')
        
        # Import Webots controller
        try:
            from controller import Robot
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
        except ImportError:
            self.get_logger().error(
                'Webots controller module not found! '
                'This controller must run inside Webots.'
            )
            raise
        
        # Get motor devices
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # Set motors to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Simple Robot Controller Started')
        self.get_logger().info('Waiting for /cmd_vel commands...')
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to wheel velocities.
        
        Args:
            msg: Twist message with linear.x and angular.z
        """
        # Extract linear and angular velocities
        linear_vel = msg.linear.x    # m/s
        angular_vel = msg.angular.z  # rad/s
        
        # Differential drive kinematics:
        # v_left = (2*v - w*L) / (2*R)
        # v_right = (2*v + w*L) / (2*R)
        # where:
        #   v = linear velocity (m/s)
        #   w = angular velocity (rad/s)
        #   L = wheel distance (m)
        #   R = wheel radius (m)
        
        left_speed = (linear_vel - angular_vel * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS
        right_speed = (linear_vel + angular_vel * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS
        
        # Set motor velocities
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        
        # Log command (only if not zero)
        if linear_vel != 0.0 or angular_vel != 0.0:
            self.get_logger().info(
                f'Cmd: v={linear_vel:.2f} m/s, ω={angular_vel:.2f} rad/s | '
                f'Wheels: L={left_speed:.2f}, R={right_speed:.2f} rad/s'
            )
    
    def run_step(self):
        """Execute one simulation step"""
        # Let ROS2 process callbacks
        rclpy.spin_once(self, timeout_sec=0)
        
        # Step the simulation
        return self.robot.step(self.timestep) != -1


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = SimpleRobotController()
        
        # Main simulation loop
        while rclpy.ok() and controller.run_step():
            pass
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
