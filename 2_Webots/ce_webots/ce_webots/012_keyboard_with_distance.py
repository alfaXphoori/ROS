#!/usr/bin/env python3
"""
Level 1.1: Keyboard Control with Distance Tracking

Manual control with real-time encoder feedback showing distance traveled.
Demonstrates how encoders measure movement during manual operation.

Controls:
    W/S - Forward/Backward
    A/D - Turn Left/Right
    X   - Stop
    R   - Reset distance counter
    ESC - Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios


class KeyboardWithDistance(Node):
    """Keyboard teleop with real-time distance tracking from encoders"""
    
    # Robot parameters
    WHEEL_RADIUS = 0.08    # meters
    WHEEL_DISTANCE = 0.24  # meters
    
    def __init__(self):
        super().__init__('keyboard_with_distance')
        
        # Import Webots controller
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
        
        # Get encoders
        self.left_encoder = self.robot.getDevice('left_wheel_sensor')
        self.right_encoder = self.robot.getDevice('right_wheel_sensor')
        
        if self.left_encoder is None or self.right_encoder is None:
            self.get_logger().error('❌ Encoders not found!')
            raise RuntimeError('Missing wheel encoders')
        
        # Enable encoders
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Velocities
        self.linear_vel = 0.5   # m/s
        self.angular_vel = 0.5  # rad/s
        
        # Distance tracking
        self.prev_left_encoder = 0.0
        self.prev_right_encoder = 0.0
        self.first_reading = True
        self.total_distance = 0.0
        self.distance_left = 0.0
        self.distance_right = 0.0
        
        # Current command
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('Keyboard Control with Distance Tracking Started')
        self.print_instructions()
    
    def print_instructions(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("🎮 KEYBOARD CONTROL WITH DISTANCE TRACKING")
        print("="*60)
        print("Controls:")
        print("  W - Forward          A - Turn Left")
        print("  S - Backward         D - Turn Right")
        print("  X - Stop             R - Reset Distance")
        print("  ESC - Quit")
        print("="*60)
        print(f"Speed: {self.linear_vel:.2f} m/s | Turn: {self.angular_vel:.2f} rad/s")
        print("="*60)
        print()
    
    def get_key(self):
        """Read a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def set_velocity(self, linear, angular):
        """Set robot velocity and update motors"""
        self.current_linear = linear
        self.current_angular = angular
        
        # Differential drive kinematics
        left_speed = (linear - angular * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS
        right_speed = (linear + angular * self.WHEEL_DISTANCE / 2.0) / self.WHEEL_RADIUS
        
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    
    def update_distance(self):
        """Read encoders and update distance traveled"""
        # Read current encoder values
        left_pos = self.left_encoder.getValue()
        right_pos = self.right_encoder.getValue()
        
        # Check for valid readings (NaN protection)
        if left_pos is None or right_pos is None:
            return
        if not isinstance(left_pos, (int, float)) or not isinstance(right_pos, (int, float)):
            return
        
        # Skip first few readings to allow encoders to stabilize
        if self.first_reading:
            self.prev_left_encoder = left_pos
            self.prev_right_encoder = right_pos
            self.first_reading = False
            return
        
        # Calculate change in encoder position
        delta_left = left_pos - self.prev_left_encoder
        delta_right = right_pos - self.prev_right_encoder
        
        # Convert to distance (use absolute value for total distance tracking)
        dist_left = abs(delta_left * self.WHEEL_RADIUS)
        dist_right = abs(delta_right * self.WHEEL_RADIUS)
        
        # Sanity check - ignore unrealistic jumps (sensor glitch protection)
        if dist_left < 1.0 and dist_right < 1.0:
            # Update totals
            self.distance_left += dist_left
            self.distance_right += dist_right
            self.total_distance += (dist_left + dist_right) / 2.0
        
        # Update previous values
        self.prev_left_encoder = left_pos
        self.prev_right_encoder = right_pos
    
    def reset_distance(self):
        """Reset distance counter"""
        self.total_distance = 0.0
        self.distance_left = 0.0
        self.distance_right = 0.0
        self.get_logger().info('📏 Distance reset to 0.000m')
    
    def print_status(self):
        """Print current status"""
        cmd_str = ""
        if self.current_linear > 0:
            cmd_str = "↑ Forward"
        elif self.current_linear < 0:
            cmd_str = "↓ Backward"
        elif self.current_angular > 0:
            cmd_str = "↺ Turn Left"
        elif self.current_angular < 0:
            cmd_str = "↻ Turn Right"
        else:
            cmd_str = "⏸ Stopped"
        
        print(f"\r📏 Distance: {self.total_distance:6.3f}m | "
              f"L: {self.distance_left:5.3f}m | R: {self.distance_right:5.3f}m | "
              f"{cmd_str:12s}", end='', flush=True)
    
    def run(self):
        """Main control loop"""
        try:
            # Non-blocking keyboard input handling
            import select
            
            # Wait for first encoder readings to stabilize
            for _ in range(5):
                self.robot.step(self.timestep)
            
            # Initialize encoders
            self.prev_left_encoder = self.left_encoder.getValue()
            self.prev_right_encoder = self.right_encoder.getValue()
            self.first_reading = False
            
            while rclpy.ok():
                # Update distance from encoders
                self.update_distance()
                
                # Print status
                self.print_status()
                
                # Check for keyboard input (non-blocking)
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = self.get_key()
                    
                    # ESC to quit
                    if key == '\x1b':
                        self.get_logger().info('\n\nExiting...')
                        self.set_velocity(0.0, 0.0)
                        break
                    
                    # Movement keys
                    elif key == 'w' or key == 'W':
                        self.set_velocity(self.linear_vel, 0.0)
                    elif key == 's' or key == 'S':
                        self.set_velocity(-self.linear_vel, 0.0)
                    elif key == 'a' or key == 'A':
                        self.set_velocity(0.0, self.angular_vel)
                    elif key == 'd' or key == 'D':
                        self.set_velocity(0.0, -self.angular_vel)
                    elif key == 'x' or key == 'X':
                        self.set_velocity(0.0, 0.0)
                    
                    # Reset distance
                    elif key == 'r' or key == 'R':
                        self.reset_distance()
                    
                    # Help
                    elif key == 'h' or key == 'H':
                        print()  # New line
                        self.print_instructions()
                
                # Step simulation
                if self.robot.step(self.timestep) == -1:
                    break
                
        except KeyboardInterrupt:
            self.get_logger().info('\n\nInterrupted by user')
            self.set_velocity(0.0, 0.0)
        except Exception as e:
            self.get_logger().error(f'\n\nError: {e}')
            self.set_velocity(0.0, 0.0)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = KeyboardWithDistance()
        controller.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
