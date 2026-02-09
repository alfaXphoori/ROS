#!/usr/bin/env python3

"""
Level 1.1: Wheel Encoder Mission
Move forward exactly 1 meter and stop using encoder feedback only

Mission:
Command the robot to move forward exactly 1 meter and stop
⚠️ Do not use time.sleep; rely only on odometry feedback

This demonstrates:
- Reading PositionSensor (wheel encoders)
- Converting encoder radians to distance meters
- Closed-loop control based on sensor feedback
- Basic odometry computation

Author: CE Robotics
License: MIT
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from controller import Robot
import math


class WheelEncoderMission(Node):
    """
    Level 1.1: Wheel Encoder Mission Controller
    
    Demonstrates proprioceptive sensing - the robot "knows itself"
    Uses wheel encoders to move exactly 1 meter forward
    """
    
    def __init__(self):
        super().__init__('wheel_encoder_mission')
        
        # Initialize Webots Robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Robot parameters (must match the .wbt file)
        self.WHEEL_RADIUS = 0.08      # meters
        self.WHEEL_DISTANCE = 0.24    # meters (distance between wheels)
        self.MAX_SPEED = 6.0          # rad/s
        
        # Get motor devices
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        
        # Set motors to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Get position sensors (encoders) - REQUIRED for this mission
        self.left_encoder = self.robot.getDevice('left_wheel_sensor')
        self.right_encoder = self.robot.getDevice('right_wheel_sensor')
        
        if self.left_encoder is None or self.right_encoder is None:
            self.get_logger().error('❌ ENCODERS NOT FOUND!')
            self.get_logger().error('This mission requires PositionSensor on wheels.')
            self.get_logger().error('Please use base_robot.wbt with encoders.')
            raise RuntimeError('Missing wheel encoders')
        
        # Enable encoders
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Mission state
        self.mission_active = False
        self.mission_complete = False
        self.target_distance = 5.4  # meters (from y=-2.7 to y=2.7)
        self.distance_traveled = 0.0
        self.speed = 0.2  # m/s (slower speed for better accuracy)
        
        # Encoder tracking
        self.prev_left_encoder = 0.0
        self.prev_right_encoder = 0.0
        self.first_reading = True
        
        # ROS2 publishers
        self.mission_status_pub = self.create_publisher(Bool, '/mission_complete', 10)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 LEVEL 1.1: WHEEL ENCODER MISSION')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Mission: Move forward exactly {self.target_distance}m and stop')
        self.get_logger().info(f'Wheel radius: {self.WHEEL_RADIUS}m')
        self.get_logger().info(f'Speed: {self.speed}m/s')
        self.get_logger().info('⚠️  No time.sleep - using encoder feedback only!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting mission in 3 seconds...')
        self.get_logger().info('')
        
        # Start mission after initialization
        self.create_timer(3.0, self.start_mission)
        
    def start_mission(self):
        """Start the 1-meter mission"""
        if not self.mission_active and not self.mission_complete:
            self.mission_active = True
            self.get_logger().info('🚀 MISSION START - Moving forward...')
            
            # Set motors to move forward
            wheel_speed = self.speed / self.WHEEL_RADIUS
            self.left_motor.setVelocity(wheel_speed)
            self.right_motor.setVelocity(wheel_speed)
    
    def read_encoders_and_update_distance(self):
        """
        Read wheel encoders and calculate distance traveled
        This is the core of Level 1.1 - proprioceptive sensing
        """
        # Read current encoder values (in radians)
        left_pos = self.left_encoder.getValue()
        right_pos = self.right_encoder.getValue()
        
        # Skip first reading (need previous values for delta)
        if self.first_reading:
            self.prev_left_encoder = left_pos
            self.prev_right_encoder = right_pos
            self.first_reading = False
            return
        
        # Calculate change in encoder position (radians)
        delta_left = left_pos - self.prev_left_encoder
        delta_right = right_pos - self.prev_right_encoder
        
        # Convert radians to distance (meters)
        # Distance = angle (radians) × wheel_radius
        dist_left = delta_left * self.WHEEL_RADIUS
        dist_right = delta_right * self.WHEEL_RADIUS
        
        # Average distance from both wheels (differential drive)
        delta_distance = (dist_left + dist_right) / 2.0
        
        # Update total distance traveled
        self.distance_traveled += delta_distance
        
        # Update previous encoder values
        self.prev_left_encoder = left_pos
        self.prev_right_encoder = right_pos
    
    def check_mission_complete(self):
        """Check if we've reached the target distance"""
        if self.mission_active and not self.mission_complete:
            if self.distance_traveled >= self.target_distance:
                # MISSION COMPLETE - STOP THE ROBOT
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                self.mission_active = False
                self.mission_complete = True
                
                # Calculate error
                error = abs(self.distance_traveled - self.target_distance)
                error_percent = (error / self.target_distance) * 100
                
                self.get_logger().info('')
                self.get_logger().info('=' * 60)
                self.get_logger().info('✅ MISSION COMPLETE!')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'Target distance: {self.target_distance:.4f}m')
                self.get_logger().info(f'Actual distance: {self.distance_traveled:.4f}m')
                self.get_logger().info(f'Error: {error:.4f}m ({error_percent:.2f}%)')
                self.get_logger().info('=' * 60)
                
                # Publish mission complete status
                msg = Bool()
                msg.data = True
                self.mission_status_pub.publish(msg)
    
    def run_step(self):
        """Execute one control step"""
        if self.mission_active:
            # Update distance from encoders
            self.read_encoders_and_update_distance()
            
            # Check if mission complete
            self.check_mission_complete()
            
            # Log progress every 0.1m
            if hasattr(self, '_last_log_distance'):
                if self.distance_traveled - self._last_log_distance >= 0.1:
                    remaining = self.target_distance - self.distance_traveled
                    self.get_logger().info(
                        f'📏 Distance: {self.distance_traveled:.3f}m | '
                        f'Remaining: {remaining:.3f}m'
                    )
                    self._last_log_distance = self.distance_traveled
            else:
                self._last_log_distance = 0.0


def main(args=None):
    """Main function to run the wheel encoder mission"""
    rclpy.init(args=args)
    
    controller = WheelEncoderMission()
    
    # Main loop: integrate Webots simulation step with ROS2 spinning
    while rclpy.ok() and controller.robot.step(controller.timestep) != -1:
        controller.run_step()
        rclpy.spin_once(controller, timeout_sec=0)
        
        # Exit after mission complete (optional - can keep running)
        if controller.mission_complete:
            controller.get_logger().info('Mission finished. Press Ctrl+C to exit.')
            # Keep running so you can see the result in Webots
    
    # Cleanup
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
